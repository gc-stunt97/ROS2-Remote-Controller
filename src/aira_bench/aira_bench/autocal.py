"""Autocalibrazione del BUSTO (scheda funzioni della plancia, 23/10/2026).

Automatizza le prove che si facevano a mano con gli slider: il busto si muove da solo, e per ogni
prova si scrivono i numeri utili alla taratura. Gira dentro il giro a 20 Hz della plancia: la
sequenza e' un GENERATORE che a ogni giro guarda la telemetria e rende il messaggio da mandare al
micro (target, duty RAW, arm, modo) oppure None.

Prove (in quest'ordine):
  1. fermo 10 s al centro        -> rumore dei pot, corrente a riposo, quanto sta in zona morta
  2. gradini roll/pitch           -> tempo di arrivo, sovraelongazione, errore residuo, corrente
  3. inversioni piccole +-2 deg   -> ritardo della risposta all'inversione (il gioco), ampiezza vera
  4. sinusoidi a periodo calante  -> fin dove il busto segue: guadagno e ritardo per periodo
  5. (opzionale) RAW              -> duty che sale piano finche' il busto si muove = lo STACCO, per
                                     asse e verso; stop a 2 deg (pitch) / 5 deg (roll, piu' gioco)

File in ~/.aira/autocal/: autocal_<data>.jsonl (risultati + segni di inizio prova) e
autocal_<data>_tel.jsonl (telemetria completa a 50 Hz, la scrive Link). Non si azzerano.
"""

import json
import math
import os
import time

AUTOCAL_DIR = os.path.expanduser("~/.aira/autocal")
APPROACH = 10.0             # deg/s del target per spostarsi fra una prova e l'altra
STEP_HOLD = 4.0             # s per ogni gradino
STEPS = (("roll", (3.0, 10.0)), ("pitch", (3.0, 15.0)))
REV_AMP, REV_N, REV_HOLD = 2.0, 8, 2.0
SINES = (("roll", 8.0), ("pitch", 10.0))
SINE_PERIODS = (8.0, 4.0, 2.0, 1.0)
SINE_CYCLES = 3             # il primo serve ad avviarsi, si analizzano gli altri
RAW_RATE = 0.03             # duty/s della rampa RAW
RAW_MAX = 0.30              # = raw_max del profilo: oltre il micro taglia comunque
RAW_STOP = {"pitch": 2.0, "roll": 5.0}   # deg di moto e si ferma (Giulio 23/10: roll ha piu' gioco)
RAW_MOVE = 0.3              # deg: "si e' mosso" = stacco
MODE_TIMEOUT = 3.0


class Abort(Exception):
    pass


def _approach(cur, goal, step):
    dp, dr = goal[0] - cur[0], goal[1] - cur[1]
    d = math.hypot(dp, dr)
    if d <= step:
        return goal, True
    return (cur[0] + dp / d * step, cur[1] + dr / d * step), False


def _key(axis):
    return "p" if axis == "pitch" else "r"


def _pose(axis, v):
    return (v, 0.0) if axis == "pitch" else (0.0, v)


def _mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def _std(xs):
    if len(xs) < 2:
        return float("nan")
    m = _mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def _r(v, n=2):
    return None if v is None or (isinstance(v, float) and math.isnan(v)) else round(v, n)


# ------------------------------------------------------------------ analisi (funzioni pure)
def rest_metrics(s):
    out = {}
    for k in ("p", "r"):
        out[f"{k}_rumore_std"] = _r(_std([x[k] for x in s]), 3)
    for k in ("ia", "ib"):
        out[f"{k}_media_A"] = _r(_mean([x[k] for x in s]), 3)
        out[f"{k}_std_A"] = _r(_std([x[k] for x in s]), 3)
    hd = [x["hd"] for x in s if x["hd"] is not None]
    if hd:
        out["in_zona_morta_pitch"] = _r(_mean([1.0 if h & 1 else 0.0 for h in hd]))
        out["in_zona_morta_roll"] = _r(_mean([1.0 if h & 2 else 0.0 for h in hd]))
    return out


def step_metrics(s, axis, start, tgt):
    k = _key(axis)
    d = tgt - start
    sign = 1.0 if d >= 0 else -1.0
    t_arr = next((x["t"] for x in s if abs(x[k] - tgt) < 1.0), None)
    over = max(0.0, max((x[k] - tgt) * sign for x in s))
    tail = [x[k] for x in s if x["t"] > s[-1]["t"] - 0.5]
    return {"da": _r(start, 1), "a": tgt, "arrivo_s": _r(t_arr), "sovraelong_deg": _r(over),
            "errore_finale_deg": _r(_mean(tail) - tgt),
            "corrente_picco_A": _r(max(max(abs(x["ia"]), abs(x["ib"])) for x in s))}


def reversal_metrics(s, axis, start, tgt):
    k = _key(axis)
    sign = 1.0 if tgt >= start else -1.0
    t_go = next((x["t"] for x in s if (x[k] - start) * sign > RAW_MOVE), None)
    tail = [x[k] for x in s if x["t"] > s[-1]["t"] - 0.3]
    return {"da": _r(start), "a": tgt, "ritardo_ms": None if t_go is None else round(t_go * 1000),
            "raggiunto_deg": _r(_mean(tail))}


def _fit(ts, ys, w):
    """y ~ a cos(wt) + b sin(wt) + c, minimi quadrati (3x3). Ritorna (ampiezza, fase rad)."""
    rows = [(math.cos(w * t), math.sin(w * t), 1.0) for t in ts]
    m = [[sum(r[i] * r[j] for r in rows) for j in range(3)] for i in range(3)]
    v = [sum(r[i] * y for r, y in zip(rows, ys)) for i in range(3)]
    # Gauss
    for i in range(3):
        piv = max(range(i, 3), key=lambda r: abs(m[r][i]))
        m[i], m[piv], v[i], v[piv] = m[piv], m[i], v[piv], v[i]
        if abs(m[i][i]) < 1e-12:
            return 0.0, 0.0
        for r in range(i + 1, 3):
            f = m[r][i] / m[i][i]
            for c in range(i, 3):
                m[r][c] -= f * m[i][c]
            v[r] -= f * v[i]
    x = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        x[i] = (v[i] - sum(m[i][c] * x[c] for c in range(i + 1, 3))) / m[i][i]
    return math.hypot(x[0], x[1]), math.atan2(x[1], x[0])


def sine_metrics(s, axis, period):
    k = _key(axis)
    w = 2.0 * math.pi / period
    use = [x for x in s if x["t"] >= period]          # il primo ciclo serve ad avviarsi
    ts = [x["t"] for x in use]
    a_t, ph_t = _fit(ts, [x["cmd"] for x in use], w)
    a_m, ph_m = _fit(ts, [x[k] for x in use], w)
    lag = (ph_m - ph_t) % (2.0 * math.pi)             # quanto la misura e' in ritardo
    err = [x[k] - x["cmd"] for x in use]
    return {"periodo_s": period, "ampiezza_chiesta": _r(a_t), "ampiezza_vera": _r(a_m),
            "guadagno": _r(a_m / a_t if a_t else float("nan")),
            "ritardo_ms": round(lag / w * 1000), "errore_rms_deg": _r(math.sqrt(_mean([e * e for e in err])))}


# ------------------------------------------------------------------ sequenza
class Autocal:
    def __init__(self, mix, include_raw, tune, fw, log):
        self.mix = mix
        self.include_raw = include_raw
        self.log = log
        os.makedirs(AUTOCAL_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(AUTOCAL_DIR, f"autocal_{stamp}.jsonl")
        self.tel_path = os.path.join(AUTOCAL_DIR, f"autocal_{stamp}_tel.jsonl")
        self.f = open(self.path, "w", encoding="utf-8")
        self.t_start = time.time()
        self.t_last = self.t_start
        self.dt = 0.05
        self.tel = {}
        self.cur = (0.0, 0.0)
        self.phase = "preparazione"
        self.results = []
        self._write({"t": "autocal_start", "fw": fw, "raw": include_raw, "mix": list(mix), "tune": tune})
        self.gen = self._run()

    # -------------------------------------------------- interfaccia con la plancia
    def step(self, tel):
        now = time.time()
        self.dt = min(0.2, now - self.t_last)
        self.t_last = now
        self.tel = tel
        return next(self.gen)

    def status(self):
        el = int(time.time() - self.t_start)
        return f"autocalibrazione: {self.phase}   {el // 60}:{el % 60:02d}"

    def finish(self, why):
        self._write({"t": "autocal_end", "ok": why is None, "motivo": why,
                     "durata_s": round(time.time() - self.t_start)})
        self.f.close()
        if why is None:
            self.log(f"[autocal] FINITA in {int(time.time() - self.t_start)} s. File: {self.path}")
        self.log("[autocal] riassunto:")
        for line in self.summary():
            self.log("    " + line)

    def summary(self):
        out = []
        by = {}
        for name, m in self.results:
            by.setdefault(name.split(" ")[0], []).append((name, m))
        for name, m in by.get("fermo", []):
            out.append(f"fermo: rumore pot p {m['p_rumore_std']} r {m['r_rumore_std']} deg, "
                       f"corrente A {m['ia_media_A']} B {m['ib_media_A']} A")
        for name, m in by.get("gradino", []):
            out.append(f"{name}: arrivo {m['arrivo_s']} s, sovraelong {m['sovraelong_deg']}, "
                       f"errore {m['errore_finale_deg']} deg, picco {m['corrente_picco_A']} A")
        for axis in ("roll", "pitch"):
            rev = [m for n, m in by.get("inversione", []) if axis in n]
            if rev:
                d = [m["ritardo_ms"] for m in rev if m["ritardo_ms"] is not None]
                out.append(f"inversioni {axis}: ritardo medio {round(_mean(d)) if d else '-'} ms "
                           f"(max {max(d) if d else '-'}), mai partito {sum(1 for m in rev if m['ritardo_ms'] is None)}/{len(rev)}")
        for name, m in by.get("sinusoide", []):
            out.append(f"{name}: guadagno {m['guadagno']}, ritardo {m['ritardo_ms']} ms, "
                       f"errore rms {m['errore_rms_deg']}")
        for name, m in by.get("stacco", []):
            out.append(f"{name}: " + (f"parte a duty {m['duty_stacco']} ({m['corrente_stacco_A']} A), "
                                      f"verso {'OK' if m['verso_ok'] else 'INVERTITO'}"
                                      if m["duty_stacco"] is not None else f"NON parte fino a {RAW_MAX}"))
        return out

    # -------------------------------------------------- utilita'
    def _write(self, obj):
        obj["time"] = round(time.time(), 3)
        self.f.write(json.dumps(obj) + "\n")
        self.f.flush()

    def _result(self, name, metrics):
        self.results.append((name, metrics))
        self._write({"t": "risultato", "prova": name, **metrics})
        self.log(f"[autocal] {name}: " + ", ".join(f"{k} {v}" for k, v in metrics.items()))

    def _begin(self, phase):
        self.phase = phase
        self._write({"t": "prova", "nome": phase})

    def _armed(self):
        if self.tel.get("st") != "armato":
            raise Abort(f"micro {self.tel.get('st')} {self.tel.get('f') or ''}".strip())

    def _sample(self, t0, cmd=None):
        x = self.tel
        return {"t": time.time() - t0, "p": float(x.get("p", 0.0)), "r": float(x.get("r", 0.0)),
                "ia": float(x.get("ia", 0.0)), "ib": float(x.get("ib", 0.0)), "hd": x.get("hd"),
                "cmd": cmd}

    @staticmethod
    def _cmd(p, r):
        return {"t": "cmd", "p": round(p, 2), "r": round(r, 2)}

    def _goto(self, p, r):
        while True:
            self._armed()
            self.cur, done = _approach(self.cur, (p, r), APPROACH * self.dt)
            yield self._cmd(*self.cur)
            if done:
                return

    def _hold(self, p, r, secs):
        self.cur = (p, r)
        t0 = time.time()
        s = []
        while time.time() - t0 < secs:
            self._armed()
            yield self._cmd(p, r)
            s.append(self._sample(t0))
        return s

    def _set_mode(self, mode):
        t0 = time.time()
        while self.tel.get("st") == "armato":
            yield {"t": "arm", "on": False}
            if time.time() - t0 > MODE_TIMEOUT:
                raise Abort("non si disarma")
        while self.tel.get("mode") != mode:
            if self.tel.get("st") == "fault":
                raise Abort(f"fault {self.tel.get('f')}")
            yield {"t": "mode", "m": mode}
            if time.time() - t0 > 2 * MODE_TIMEOUT:
                raise Abort(f"il micro non passa in modo {mode}")
        while self.tel.get("st") != "armato":
            if self.tel.get("st") == "fault":
                raise Abort(f"fault {self.tel.get('f')}")
            yield {"t": "arm", "on": True}
            if time.time() - t0 > 3 * MODE_TIMEOUT:
                raise Abort("il micro non si arma")
        # all'arm il micro riparte dalla posa misurata: il target anche
        self.cur = (float(self.tel.get("p", 0.0)), float(self.tel.get("r", 0.0)))

    def _raw_ramp(self, axis, sgn):
        """Duty che sale piano (pitch: motori insieme; roll: in opposizione) finche' il giunto si
        muove di RAW_STOP gradi. Lo stacco e' il duty a cui si e' mosso di RAW_MOVE."""
        sa, sb, sr = self.mix
        k = _key(axis)
        start = float(self.tel.get(k, 0.0))
        t0 = time.time()
        t_top = None
        got = None
        while True:
            self._armed()
            t = time.time() - t0
            d = min(RAW_MAX, RAW_RATE * t)
            if d >= RAW_MAX and t_top is None:
                t_top = t
            if axis == "pitch":
                a, b = sa * sgn * d, sb * sgn * d
            else:
                a, b = sa * sr * sgn * d, -sb * sr * sgn * d
            yield {"t": "raw", "a": round(a, 3), "b": round(b, 3)}
            moved = float(self.tel.get(k, 0.0)) - start
            if got is None and abs(moved) > RAW_MOVE:
                got = {"duty_stacco": round(d, 3),
                       "corrente_stacco_A": _r(max(abs(float(self.tel.get("ia", 0.0))),
                                                   abs(float(self.tel.get("ib", 0.0)))))}
            if abs(moved) > RAW_STOP[axis] or (t_top is not None and t - t_top > 1.0):
                break
        yield {"t": "raw", "a": 0.0, "b": 0.0}
        res = got or {"duty_stacco": None, "corrente_stacco_A": None}
        res["mosso_deg"] = _r(moved)
        res["verso_ok"] = moved * sgn > 0
        return res

    # -------------------------------------------------- la sequenza
    def _run(self):
        self._armed()
        self.cur = (float(self.tel.get("p", 0.0)), float(self.tel.get("r", 0.0)))

        self._begin("al centro")
        yield from self._goto(0.0, 0.0)
        yield from self._hold(0.0, 0.0, 2.0)

        self._begin("fermo 10 s")
        s = yield from self._hold(0.0, 0.0, 10.0)
        self._result("fermo 10 s", rest_metrics(s))

        for axis, amps in STEPS:
            for amp in amps:
                for tgt in (amp, 0.0, -amp, 0.0):
                    start = float(self.tel.get(_key(axis), 0.0))
                    name = f"gradino {axis} {start:+.0f} -> {tgt:+.0f}"
                    self._begin(name)
                    s = yield from self._hold(*_pose(axis, tgt), STEP_HOLD)
                    self._result(name, step_metrics(s, axis, start, tgt))

        for axis, _amps in STEPS:
            for n in range(REV_N):
                tgt = REV_AMP if n % 2 == 0 else -REV_AMP
                start = float(self.tel.get(_key(axis), 0.0))
                name = f"inversione {axis} {n + 1}/{REV_N}"
                self._begin(name)
                s = yield from self._hold(*_pose(axis, tgt), REV_HOLD)
                self._result(name, reversal_metrics(s, axis, start, tgt))
            yield from self._hold(0.0, 0.0, 1.0)

        for axis, amp in SINES:
            for period in SINE_PERIODS:
                name = f"sinusoide {axis} {amp:.0f} deg periodo {period:g} s"
                self._begin(name)
                yield from self._goto(*_pose(axis, amp))      # coseno: si parte dal picco
                yield from self._hold(*_pose(axis, amp), 1.0)
                t0 = time.time()
                s = []
                w = 2.0 * math.pi / period
                while time.time() - t0 < SINE_CYCLES * period:
                    self._armed()
                    v = amp * math.cos(w * (time.time() - t0))
                    self.cur = _pose(axis, v)
                    yield self._cmd(*self.cur)
                    s.append(self._sample(t0, v))
                self._result(name, sine_metrics(s, axis, period))
            yield from self._goto(0.0, 0.0)

        if self.include_raw:
            for axis in ("pitch", "roll"):
                for sgn in (1.0, -1.0):
                    name = f"stacco {axis} {'+' if sgn > 0 else '-'}"
                    self._begin(name)
                    yield from self._goto(0.0, 0.0)
                    yield from self._hold(0.0, 0.0, 1.0)
                    yield from self._set_mode("raw")
                    res = yield from self._raw_ramp(axis, sgn)
                    self._result(name, res)
                    yield from self._set_mode("pos")
            self._begin("al centro")
            yield from self._goto(0.0, 0.0)
            yield from self._hold(0.0, 0.0, 1.0)
        self.phase = "finita"
