#include "ArduinoJson.h"

unsigned long currentMillis;
long previousMillisLoop = 0; // set up timers
long previousMillisLed = 0; // set up timers
float loopTime = 20;
float ledBlinkTime = 500;
boolean ledState = LOW;  // Stato attuale del primo LED

// values for reading sticks
// Convenzione (come sulla GUI RobotHex): Y = avanti/indietro, X = destra/sinistra.
// Le etichette qui sotto sono ora COERENTI col cablaggio fisico (vedi setup/loop).

float LX;
float LY;
float LZ;
float RX;
float RY;
float RZ;

// -------------------- Pulsanti (tutti NO -> massa, pull-up software) --------------------
// Indici del vettore buttons[]: usati per pin, debounce e chiavi JSON.
const int BTN_COUNT = 6;
enum { B_BL = 0, B_BR, B_EM, B_B1, B_B2, B_B3 };
// Cablaggio verificato sul telecomando il 2026-07-14: il tastino SINISTRO e' su PB10 e
// il DESTRO su PB15 (l'opposto di quanto assumeva questa tabella prima).
// pin:            BL(L stick)  BR(R stick)  EM stop   Button1  Button2  Button3
const int   btnPin[BTN_COUNT] = { PB10,      PB15,      PB14,    PB11,    PB12,   PB13 };
const char* btnKey[BTN_COUNT] = { "BL",      "BR",      "EM",    "B1",    "B2",   "B3" };

// Stato per il debounce
int  btnLastReading[BTN_COUNT];   // ultima lettura grezza
int  btnStable[BTN_COUNT];        // stato stabile filtrato (HIGH=riposo, LOW=premuto)
long btnLastChange[BTN_COUNT];    // millis dell'ultimo cambio grezzo
const long debounceMs = 30;       // finestra di stabilita' del debounce

void setup()
{

  Serial.begin(57600);

  // ADC a 12 bit: il core ufficiale STM32 di default legge a 10 bit, ma il
  // firmware assume 12 bit (offset -2048). Lo rendiamo esplicito.
  analogReadResolution(12);

  pinMode(PC13, OUTPUT);

  // Pulsanti: tutti Normally-Open verso massa -> pull-up interno.
  // A riposo il pin legge HIGH; premuto legge LOW.
  for (int i = 0; i < BTN_COUNT; i++) {
    pinMode(btnPin[i], INPUT_PULLUP);
    btnLastReading[i] = HIGH;
    btnStable[i]      = HIGH;
    btnLastChange[i]  = 0;
  }

  pinMode(PA0, INPUT);
  pinMode(PA1, INPUT);
  pinMode(PA2, INPUT);
  pinMode(PA3, INPUT);
  pinMode(PA4, INPUT);
  pinMode(PA5, INPUT);
}

// deadzone function for cheap joysticks

float deadzone(float value)
{
  if (value > 50)
  {
    value = value - 70;
  }
  else if (value < -50)
  {
    value = value + 70;
  }
  else
  {
    value = 0;
  }
  value = value / 1950; // scale so that we get 0.0 ~ 1.0 (1843.2)
  return value;
}

// Aggiorna il debounce di un pulsante e ritorna 1 = premuto, 0 = riposo.
// I pulsanti sono attivi-bassi (NO verso massa con pull-up): premuto = LOW.
int readButton(int i)
{
  int reading = digitalRead(btnPin[i]);
  if (reading != btnLastReading[i]) {
    btnLastChange[i]  = currentMillis;   // il segnale si e' mosso: (ri)parte il timer
    btnLastReading[i] = reading;
  }
  if (currentMillis - btnLastChange[i] >= debounceMs) {
    btnStable[i] = reading;              // stabile abbastanza a lungo: lo accettiamo
  }
  return (btnStable[i] == LOW) ? 1 : 0;  // 1 = premuto
}

void loop()
{

  currentMillis = millis();
  if (currentMillis - previousMillisLoop >= loopTime) // run a loop eveRY 20ms
  {
    previousMillisLoop = currentMillis; // reset the clock to time it

    // --- Assi: etichette coerenti col cablaggio (Y=avanti, X=laterale) ---
    LY = analogRead(PA1);   // Left  Y  (avanti/indietro)
    LX = analogRead(PA0);   // Left  X  (destra/sinistra)
    LZ = analogRead(PA2);   // Left  Z  (yaw / rotazione manopola)

    RY = analogRead(PA5);   // Right Y  (avanti/indietro)
    RX = analogRead(PA3);   // Right X  (destra/sinistra)
    RZ = analogRead(PA4);   // Right Z  (yaw / rotazione manopola)

    // remove offsets + deadzone

    LY = LY - 2048;
    LY = deadzone(LY);
    LX = LX - 2048;
    LX = deadzone(LX);
    LZ = LZ - 2048;
    LZ = deadzone(LZ);
    RY = RY - 2048;
    RY = deadzone(RY);
    RX = RX - 2048;
    RX = deadzone(RX);
    RZ = RZ - 2048;
    RZ = deadzone(RZ);

    // Segni: preservano l'orientamento end-to-end usato finora.
    // (se una direzione risultasse invertita, cambia il segno del solo asse interessato)
    LY = LY * 1;   // avanti = +
    LX = LX * -1;  // invert value/direction as required based on wiring
    LZ = LZ * 1;   // yaw normalizzato a ~[-1,1] come gli altri assi (gain 'feel' -> nel teleop)

    RY = RY * 1;   // avanti = +
    RX = RX * -1;  // invert value/direction as required based on wiring
    RZ = RZ * 1;   // yaw normalizzato a ~[-1,1] come gli altri assi (gain 'feel' -> nel teleop)

    // --- Pulsanti (con debounce). 1 = premuto ---
    int btn[BTN_COUNT];
    for (int i = 0; i < BTN_COUNT; i++) {
      btn[i] = readButton(i);
    }
/*
    Serial.print("LY:");
    Serial.print(LY);
    Serial.print(",LX:");
    Serial.print(LX);
    Serial.print(",LZ:");
    Serial.print(LZ);
    Serial.print(",RY:");
    Serial.print(RY);
    Serial.print(",RX:");
    Serial.print(RX);
    Serial.print(",RZ:");
    Serial.println(RZ);
*/
    StaticJsonDocument<512> jsonDocument;  // margine per 6 assi + 6 pulsanti

    char json_buffer[10]; // Dimensione del buffer in base ai valori massimi che vuoi formattare
    jsonDocument["LY"] = dtostrf(LY, 4, 2, json_buffer);
    jsonDocument["LZ"] = dtostrf(LZ, 4, 2, json_buffer);
    jsonDocument["LX"] = dtostrf(LX, 4, 2, json_buffer);
    jsonDocument["RY"] = dtostrf(RY, 4, 2, json_buffer);
    jsonDocument["RZ"] = dtostrf(RZ, 4, 2, json_buffer);
    jsonDocument["RX"] = dtostrf(RX, 4, 2, json_buffer);

    // Pulsanti (interi 0/1, 1 = premuto):
    //  BL/BR = tastini in cima agli stick; EM = fungo d'emergenza; B1..B3 = general purpose.
    for (int i = 0; i < BTN_COUNT; i++) {
      jsonDocument[btnKey[i]] = btn[i];
    }


    String jsonString;
    serializeJson(jsonDocument, jsonString);

Serial.println(jsonString);



  } // end of 20ms loop

    if (currentMillis - previousMillisLed >= ledBlinkTime) {
        previousMillisLed = currentMillis;  // Aggiorna il tempo precedente

      if (ledState == LOW) {
          ledState = HIGH;
        } else {
        ledState = LOW;
       }
      digitalWrite(PC13, ledState);
      }

} // end of main loop
