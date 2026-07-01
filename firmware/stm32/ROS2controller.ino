
#include "ArduinoJson.h"

unsigned long currentMillis;
long previousMillisLoop = 0; // set up timers
long previousMillisLed = 0; // set up timers
float loopTime = 20;
float ledBlinkTime = 500;
boolean ledState = LOW;  // Stato attuale del primo LED

// values for reading sticks

float LX;
float LY;
float LZ;
float RX;
float RY;
float RZ;

int butL;
int butR;

void setup()
{

  Serial.begin(57600);

  pinMode(PC13, OUTPUT);
  
  pinMode(PB0, INPUT_PULLUP);
  pinMode(PB1, INPUT_PULLUP);
  pinMode(PB2, INPUT_PULLUP);
  pinMode(PB3, INPUT_PULLUP);

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

int invButtons(int value)
{

  if (value == 0)
  {
    value = 1;
  }
  else if (value == 1)
  {
    value = 0;
  }
  return value;
}

void loop()
{

  currentMillis = millis();
  if (currentMillis - previousMillisLoop >= loopTime) // run a loop eveRY 10ms
  {
    previousMillisLoop = currentMillis; // reset the clock to time it

    LY = analogRead(PA0);
    LZ = analogRead(PA2);
    LX = analogRead(PA1);
    butL = digitalRead(PB0);
    
    RY = analogRead(PA3);
    RZ = analogRead(PA4);
    RX = analogRead(PA5);
    butR = digitalRead(PB1);

   

    // remove offsets

    LY = LY - 2048;
    LY = deadzone(LY);
    LZ = LZ - 2048;
    LZ = deadzone(LZ);
    LX = LX - 2048;
    LX = deadzone(LX);
    RY = RY - 2048;
    RY = deadzone(RY);
    RZ = RZ - 2048;
    RZ = deadzone(RZ);
    RX = RX - 2048;
    RX = deadzone(RX);

    butL = invButtons(butL);
    butR = invButtons(butR);

    RZ = RZ * 1;  // yaw normalizzato a ~[-1,1] come gli altri assi (gain 'feel' -> nel teleop)
    RX = RX * 1;  // invert value/direction as required based on wiring
    RY = RY * -1; // invert value/direction as required based on wiring

    LZ = LZ * 1;  // yaw normalizzato a ~[-1,1] come gli altri assi (gain 'feel' -> nel teleop)
    LX = LX * 1;  // invert value/direction as required based on wiring
    LY = LY * -1; // invert value/direction as required based on wiring
/*
    Serial.print("LY:");
    Serial.print(LY);
    Serial.print(",LZ:");
    Serial.print(LZ);
    Serial.print(",LX:");
    Serial.print(LX);
    //Serial.print(",butL:");
    //Serial.print(butL);
    Serial.print(",RY:");
    Serial.print(RY);
    Serial.print(",RZ:");
    Serial.print(RZ);
    Serial.print(",RX:");
    Serial.println(RX);
    //Serial.print(",butR:");
    //Serial.println(butR);

*/
    StaticJsonDocument<256> jsonDocument;  // margine per 6 assi + 2 tastini

    char json_buffer[10]; // Dimensione del buffer in base ai valori massimi che vuoi formattare
    jsonDocument["LY"] = dtostrf(LY, 4, 2, json_buffer);
    jsonDocument["LZ"] = dtostrf(LZ, 4, 2, json_buffer);
    jsonDocument["LX"] = dtostrf(LX, 4, 2, json_buffer);
    jsonDocument["RY"] = dtostrf(RY, 4, 2, json_buffer);
    jsonDocument["RZ"] = dtostrf(RZ, 4, 2, json_buffer);
    jsonDocument["RX"] = dtostrf(RX, 4, 2, json_buffer);

    // Tastini in cima ai joystick (interi 0/1). Dopo invButtons(): 1 = premuto.
    jsonDocument["BL"] = butL;
    jsonDocument["BR"] = butR;


    String jsonString;
    serializeJson(jsonDocument, jsonString);

Serial.println(jsonString);


    

  } // end of 10ms loop

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
