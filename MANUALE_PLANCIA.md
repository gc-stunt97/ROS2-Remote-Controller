# Manuale della plancia di banco (aira_bench)

Guida d'uso, corta. Il *perché* delle scelte sta in `CONTROLLER_HANDBOOK.md` §14 e nell'handbook
del robot (§14.13 per il busto). Questo file si aggiorna ogni volta che la plancia cambia o che
si aggiunge una parte del robot (una sezione per parte, in fondo).

---

## 1. Cos'è e come si avvia

Il telecomando fa la parte del mini PC: si aggancia a un micro (ESP32-P4) via Ethernet, gli manda
il profilo, lo comanda con gli stick e mostra cosa fa. Il micro non sa di essere al banco.

- **Avvio:** icona **"AIRA Banco micro"** sul desktop (oppure `ros2 launch aira_bench bench.launch.py`).
- **Collegamento:** micro sulla porta "solo micro" del RUT241, diretto o tramite lo switch del robot.
- **Chiudi:** pulsante rosso **chiudi** in alto a destra. Disarma prima di uscire.
- Il micro si aggancia **da solo** se in rete ce n'è uno solo che conosciamo; altrimenti si
  sceglie dalla scheda **micro**.

## 2. Lo schermo

### In alto
- **Riga del micro agganciato** (ruolo, MAC, IP, versione firmware). *nessun micro agganciato* = non c'è nessuno.

### Colonna di sinistra (sempre visibile)
| Cosa | A cosa serve |
|---|---|
| **Riquadro di stato** | `ARMATO` verde, `OFF` giallo, `FAULT (motivo)` rosso, `OFFLINE` grigio = niente telemetria da 0,5 s. Accanto: modo (`POS`/`RAW`) e `NON CALIBRATO` se mancano i pot. |
| **ARMA / DISARMA** | Accende/spegne l'anello. Niente conferma. Rifiutato se c'è un fault o il fungo è premuto. |
| **posizione / RAW motori** | Modo. Si cambia **solo da disarmato**: il pallino segue sempre il modo vero del micro. |
| **clear** | Toglie il fault. Poi va **riarmato** a mano. |
| **stacca HB** | Simula il cavo che cade: il micro deve disarmarsi da solo (~300 ms). Diventa **RIATTACCA HB** rosso. |
| **rimanda cfg** | Rimanda tutto il profilo al micro. **Da premere dopo un OTA o un riavvio del micro.** Rimette anche gli slider PID ai valori del profilo. |
| **leggi cfg** | Chiede al micro la config che sta usando (finisce nel log). |
| **Barre dei giunti** | Verde = posizione **misurata**, blu = **target** chiesto, giallo = **setpoint** (la rampa che insegue il target). |
| Testo sotto le barre | `mis` / `target` / `set` in gradi, `vel` (°/s), `u` = spinta dell'anello per quel giunto (duty, prima del mixing), `raw` = lettura grezza del pot. |
| **Barre mot A / mot B** | Duty vero dato a ciascun motore (−1…+1). |
| Testo sotto i motori | Duty, **corrente** (A) e mV grezzi dell'ACS per motore, `saturo` = 1 se il duty è stato tagliato al massimo, **heartbeat visto dal micro** (ms), `⚠ FUNGO` se premuto. |

> La corrente dell'ACS è quella **nel motore**, non quella dell'alimentatore: col PWM l'alimentatore
> legge circa *duty × corrente del motore*, quindi meno. Guadagno tarato sulla carta, mai
> confrontato con uno strumento.

### Schede di destra
- **log** — messaggi della plancia e del micro. Le righe `[PID]` dicono cosa hai cambiato.
- **micro (N)** — tutte le schede che si annunciano. Rosso = ha già un altro capo (il mini PC);
  grigio = assente. **aggancia** per sceglierne una; *AGGANCIATO* = quella in uso.
- **cal / OTA** — calibrazione dei pot e aggiornamento firmware (vedi §4).
- **slider** — comando a mano senza stick (vedi §3).
- **funzioni** — movimenti automatici (Twirl, oscillatori) al posto del joystick (vedi §3).
- **PID** — taratura dal vivo (vedi la sezione della parte, es. §6.3 per il busto).

Ogni slider ha il **nome a sinistra e il valore a destra**, sopra la barra: col dito sulla barra
il numero resta leggibile.

## 3. Come si comanda

- **Stick:** comandano sempre, se pubblicano. La mappatura dipende dalla parte (vedi la sua sezione).
- **Slider (scheda slider):** comandano se il joystick tace da 1 s, oppure **sempre** con la
  casella *comanda con gli slider (stick esclusi)*. Utile per un valore **fisso e ripetibile**
  (prove registrate, tester, misure di corrente). All'arm e al cambio di sorgente gli slider
  tornano a zero. **slider a zero** li azzera a mano.
- **Funzioni (scheda funzioni):** scegli **Manuale (joystick)**, **Twirl**, **Oscillatore pitch** o
  **Oscillatore roll**, poi **AVVIA** (serve il micro armato in modo posizione). Mentre una funzione
  gira stick e slider sono **ignorati**.
  - Avvio: il target parte dalla posa misurata, va **piano** (10°/s) al punto di partenza, poi la
    velocità sale in ~1,5 s. **STOP** (o scegliere Manuale): torna piano al centro, poi comanda di
    nuovo il joystick. Disarmo, fault o fungo la interrompono subito.
  - Si vedono solo gli slider della funzione scelta, e si cambiano anche in corsa:
    - **Twirl:** **periodo s** (secondi per un giro), **swing pitch** e **swing roll** (ampiezza in
      gradi), casella **twirl antiorario** per il verso.
    - **Oscillatore pitch:** periodo, **swing pitch**, **roll fisso** = il roll resta fermo lì mentre
      il pitch oscilla. **Oscillatore roll:** uguale a parti invertite (**pitch fisso**).
  - Mentre una funzione gira non se ne sceglie un'altra: prima **STOP**.
  - Sulle barre il **blu** (target) segue la funzione, il **giallo** (setpoint) è la rampa del micro,
    il **verde** è il busto vero. Se il verde resta indietro, a limitare sono **vel max / acc / PWM
    max** della scheda PID: si possono cambiare mentre la funzione gira, cambiare scheda non ferma
    niente. Avvio e stop finiscono nel registro della telemetria.
- **Autocalibrazione (scheda funzioni):** il busto fa da solo, in ~4-5 minuti, le prove che
  facevamo a mano: **fermo 10 s** (rumore dei pot, corrente a riposo), **gradini** di roll e pitch
  (3° e 10/15°: tempo di arrivo, sovraelongazione, errore residuo), **inversioni** ±2° (ritardo =
  il gioco), **sinusoidi** con periodo 8→4→2→1 s (fin dove segue: guadagno e ritardo) e, con la
  casella **includi RAW (stacco)**, il duty che sale piano finché il busto si muove (stop a 2° sul
  pitch, 5° sul roll) = lo **stacco** per asse e verso. Chiede conferma: busto libero, fungo a portata.
  - Si avvia armato in modo posizione e coi pot calibrati; passa da sola in RAW e ritorno.
  - **STOP**, fault, fungo o telemetria persa: **disarma** e la lascia lì (riarmare a mano).
  - Risultati riga per riga nel **log** e riassunto alla fine; file in `~/.aira/autocal/`
    (`autocal_<data>.jsonl` = risultati, `..._tel.jsonl` = telemetria completa): **non si
    azzerano**, me li copio io per i grafici. I valori PID in uso finiscono nel file.
- **RAW: a fondo = 100% duty** — di norma in RAW il fondo corsa è il 30%. Si accende solo da
  disarmato, chiede conferma. ⚠️ 12 V pieni.
- **RAW: ignora pot e limiti (diagnosi)** — in RAW il micro non guarda più pot e limiti (resta solo
  la sovracorrente). Solo da disarmato, colpi brevi: il busto può arrivare alle battute.

## 4. Procedure

**Aggiornare il firmware (OTA)**
1. Disarma. Il `.bin` nuovo sta in `~/fw_out/` (lo copio io).
2. *cal / OTA* → **OTA: scegli .bin** → il micro scarica, si riavvia, si conferma al primo heartbeat
   (se il firmware nuovo non parla entro ~90 s torna da solo al vecchio).
3. Controlla la versione nella riga in alto o nella scheda *micro*.
4. **rimanda cfg** (oggi la plancia non lo fa da sola dopo un OTA).
- **riavvia micro**: riavvio semplice (es. per rifare lo zero degli ACS, che si prende all'avvio a motori fermi).

**Calibrare i pot** (due pose, angolo letto con livella o telefono)
1. Disarmato, porta il giunto alla posa A, scrivi l'angolo vero nella casella, **cattura A**.
2. Stessa cosa per la posa B (lontana dalla A).
3. **salva calibrazione e spingi**: va nel file locale e al micro. I valori stampati nel log vanno
   **ricopiati nel profilo del repo** (`AIRA_Robot/periph/<parte>.yaml`), se no restano solo lì.

**Tarare dal vivo (scheda PID)**
1. Arma, muovi lo slider: il valore parte **al rilascio** del dito (cambiarli da armato non disarma).
2. Il **micro** li dimentica a **rimanda cfg**, al riaggancio o al riavvio: lì tornano quelli del
   profilo. Quando sei contento premi **scrivi valori nel log**: la riga resta anche nel file della
   sessione (`~/.aira/bench_log/`), e mi dici di ricopiarli nel profilo.
3. **valori del profilo**: torna ai valori del repo (e li rimanda al micro).

**Registrare una prova**
- **Niente si perde chiudendo la plancia:** ogni avvio è una **sessione** con la sua data, e si
  tengono le ultime 20.
  - **Telemetria** da armato a 50 Hz, con una riga per ogni slider PID mosso e per ogni avvio o stop
    di funzione: `~/.aira/bench_tel/tel_<data>.jsonl`. `~/.aira/bench_tel.jsonl` punta sempre
    all'ultima sessione.
  - **Scheda log**, compreso «scrivi valori nel log»: `~/.aira/bench_log/log_<data>.txt`.
  - Prova tipo: 10 s fermo, poi i gradini, poi la spinta a mano (o l'autocalibrazione, che fa
    tutto da sola).

## 5. Problemi noti

| Sintomo | Cosa fare |
|---|---|
| Joystick "morto" dopo aver riaperto la plancia | Seriale contesa col modo mouse: **chiudi, aspetta 3 s, riapri**. |
| Dopo OTA/riavvio il micro si comporta "di default" | **rimanda cfg**. |
| Il modo non cambia | Si cambia solo da disarmato (il log lo dice). |
| `OFFLINE` | Micro spento, cavo, o rete: guarda la scheda *micro*. |
| Finestra "Ubuntu has experienced an internal error" all'avvio | Rapporti di crash vecchi, innocui: si cancellano i file in `/var/crash/`. |

---

## 6. Busto (`torso`) — firmware ≥ 0.3.2

### 6.1 Comandi
- **Posizione:** **stick DX** — avanti/indietro = **pitch** (fondo = ±30°), destra/sinistra = **roll** (±19°).
  Il target viene comunque tagliato ai limiti del profilo.
- **RAW:** stick DX *push-pull* — su/giù = i due motori insieme, destra/sinistra = in opposizione.
  Dagli slider invece **un motore ciascuno** (*pitch / mot A*, *roll / mot B*), per mappare i versi.
- Motore **A = sinistra** (visto dal robot), **B = destra**. Roll + = verso destra, pitch + = avanti.

### 6.2 Fault (riquadro rosso, poi **clear** e riarma)
| Fault | Cosa vuol dire |
|---|---|
| `pot_pitch` / `pot_roll` | Lettura del pot fuori scala: filo staccato o pot rotto. |
| `limite` | Posa oltre i limiti + margine (4°). Busto troppo veloce o spinto oltre. |
| `stallo_a` / `stallo_b` | Quel motore spinge forte ma la sua manovella non si muove. |
| `sovracorrente` | Corrente oltre `cur.max` (12 A) per 300 ms. |
| `segno` | Il giunto scappa dal bersaglio mentre l'anello spinge: segno del mixing sbagliato (o forza insufficiente). |
| `watchdog` | Heartbeat perso: disarmato. Non serve clear, basta riarmare. |

### 6.3 Scheda PID
Duty = frazione di PWM (0,30 = 30% della tensione). Valori di partenza = quelli del profilo.

| Slider | Cosa fa | Se lo alzi |
|---|---|---|
| **pitch / roll kp** | Spinta proporzionale all'errore (duty per grado). | Più rigido e pronto; troppo = oscilla. |
| **pitch / roll ki** | Integrale: cresce finché l'errore resta, vince carichi e attrito. | Tiene meglio la posa; troppo = va oltre e torna (pendola). |
| **pitch / roll kd** | Frena in base alla velocità. | Smorza; troppo = nervoso col rumore del pot. |
| **PWM max pitch / roll** | Tetto della spinta del **giunto** (prima del mixing). 1 = nessun tetto. | — sotto lo stacco il giunto non arriva più al bersaglio. |
| **PWM max motori** | Tetto del duty per **motore**. Di fatto è la velocità massima. | Più veloce e più forte. |
| **vel max** | Velocità massima della rampa del setpoint (°/s). | Al banco 120 = non limita; sul robot 15-20. |
| **acc** | Accelerazione della rampa (°/s²). | Partenze/frenate più secche. |
| **PWM min stacco** | Aggiunto a ogni comando per vincere l'attrito statico. | Basso = correzioni lente "a strappo"; alto = tremola sul bersaglio. Giusto: appena sotto lo stacco del motore. |
| **stallo duty / ms** | Fault `stallo` se un motore supera *duty* per *ms* senza che la sua manovella si muova di 1°. | Ora quasi spento (0,93 / 5000) perché scattava spingendo a mano; **sul robot ~0,45 / 1000-1500**. |
| **cedevolezza 0=rigido** | Da fermo il busto **cede verso chi lo spinge** come una molla (gradi per unità di spinta) e torna quando lasci. | Più morbido. 0 = rigido. |
| **gioco duty 0=off** | Dopo ogni inversione il motore non supera questo duty finché il dente non prende (attraversa il gioco piano). | **Lasciare 0** finché non misuriamo lo stacco a vuoto: un valore piccolo fa **oscillare** il busto (il motore resta fermo, l'integrale si carica, poi parte di colpo). Va messo **appena sopra lo stacco a vuoto**. |
| **gioco presa A** | Corrente che vuol dire "dente in presa": superata, il limite si toglie. | — |
| **gioco max ms** | Tempo massimo col limite del gioco (sicurezza se l'ACS tace). | — |
| **soglia carico PWM** | A bersaglio l'anello si spegne (zona morta) solo se la forza che sta facendo è sotto questa soglia: se qualcuno spinge resta acceso e tiene, senza "molla e riprende". | Bassa = a bersaglio può restare acceso a lungo; alta = tornano i colpetti con spinte leggere. 0 = vecchio comportamento. |

### 6.4 Note
- I tergicristalli sono **reversibili** con molto attrito: da spenti tengono la posa, ma a mano si muovono.
- Il roll ha un gioco di ~2° (vite/ruota): si vede come un piccolo scatto alle inversioni.
- Profilo: `AIRA_Robot/periph/torso.yaml` (sezione `bench:` = mappatura degli stick).

---

## 7. Prossime parti
*(collo, testa, base… una sezione come la §6 per ognuna: comandi, fault, slider.)*
