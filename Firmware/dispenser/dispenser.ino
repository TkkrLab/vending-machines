/*
 * Vending machine firmware: dispenser unit
 * Can be used as both a candy machine drawer or for driving the can dispenser of the cola machine
 * 
 * MIT License
 *  
 * Copyright (c) 2019 Renze Nicolai
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

//----- CONFIGURATION -----

/* Select one of the following hardware configurations (or add your own down below) */

#define HW_VENDO     //(Vendo 75 5 slots can machine)
//#define HW_CANDY3  //(Candy machine drawer with 3 slots)
//#define HW_CANDY6  //(Candy machine drawer with 6 slots)

/* Configure the hardware ID of this device. The ID must be unique within each vending machine */

#define UNIT_ID 1

//-------------------------

//Do not change anything below this line.

#ifdef HW_CANDY3
  #define NEOPIXELS_PIN 8
  #define NEOPIXELS_AMOUNT 9
  #define MOTORS_AMOUNT 3
  #define MOTOR1_PIN 4
  #define MOTOR2_PIN 3
  #define MOTOR3_PIN 2
  #define SWITCH1_PIN A2
  #define SWITCH2_PIN A1
  #define SWITCH3_PIN A0
  const uint8_t motors[] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN};
  const uint8_t switches[] = {SWITCH1_PIN, SWITCH2_PIN, SWITCH3_PIN};
#endif
#ifdef HW_CANDY6
  #define NEOPIXELS_PIN 8
  #define NEOPIXELS_AMOUNT 9
  #define MOTORS_AMOUNT 6
  #define MOTOR1_PIN 7
  #define MOTOR2_PIN 6
  #define MOTOR3_PIN 5
  #define MOTOR4_PIN 4
  #define MOTOR5_PIN 3
  #define MOTOR6_PIN 2
  #define SWITCH1_PIN A5
  #define SWITCH2_PIN A4
  #define SWITCH3_PIN A3
  #define SWITCH4_PIN A2
  #define SWITCH5_PIN A1
  #define SWITCH6_PIN A0
  const uint8_t motors[] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN, MOTOR5_PIN, MOTOR6_PIN};
  const uint8_t switches[] = {SWITCH1_PIN, SWITCH2_PIN, SWITCH3_PIN, SWITCH4_PIN, SWITCH5_PIN, SWITCH6_PIN};
#endif
#ifdef HW_VENDO
  #define MOTORS_AMOUNT 5
  #define MOTOR1_PIN A1
  #define MOTOR2_PIN A2
  #define MOTOR3_PIN A3
  #define MOTOR4_PIN A4
  #define MOTOR5_PIN A5
  #define SWITCH1_PIN 11
  #define SWITCH2_PIN 10
  #define SWITCH3_PIN 9
  #define SWITCH4_PIN 8
  #define SWITCH5_PIN 7
  #define EMPTY1_PIN 6
  #define EMPTY2_PIN 5
  #define EMPTY3_PIN 4
  #define EMPTY4_PIN 3
  #define EMPTY5_PIN 2
  const uint8_t motors[]   = {MOTOR1_PIN,  MOTOR2_PIN,  MOTOR3_PIN,  MOTOR4_PIN,  MOTOR5_PIN};
  const uint8_t switches[] = {SWITCH1_PIN, SWITCH2_PIN, SWITCH3_PIN, SWITCH4_PIN, SWITCH5_PIN};
  const uint8_t empty[]    = {EMPTY1_PIN,  EMPTY2_PIN,  EMPTY3_PIN,  EMPTY4_PIN,  EMPTY5_PIN};
#endif

#include <Adafruit_NeoPixel.h>

#define STATE_IDLE 0
#define STATE_STARTING 1
#define STATE_WAIT1 2
#define STATE_VENDING 3
#define STATE_WAIT2 4
#define STATE_CORRECTION 5

#define TIMEOUT 15000

unsigned long previousMillis = 0;
const unsigned long interval = 100; //Interval in milliseconds

unsigned long previousEmptyMillis = 0;
const unsigned long emptyInterval = 500;

uint8_t channel = 0;

uint8_t correctioncounter[MOTORS_AMOUNT] = {0};

uint8_t selectedLed = 0;

uint8_t state[MOTORS_AMOUNT] = { 0 };

#ifdef NEOPIXELS_PIN
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXELS_AMOUNT, NEOPIXELS_PIN, NEO_GRBW + NEO_KHZ800);
#endif

unsigned long startTime;

bool moveChannel(uint8_t channel, bool mode = false) {
  digitalWrite(motors[channel], HIGH);
  startTime = millis();
  while(digitalRead(switches[channel]) == mode) {
    unsigned long currentTime = millis();
    if (currentTime - startTime >= TIMEOUT) {
      digitalWrite(motors[channel], LOW);
      return false;
    }
  }
  digitalWrite(motors[channel], LOW);
  return true;
}

void halt() {
  Serial.println("#Halting...");
  for (uint8_t i = 0; i<MOTORS_AMOUNT; i++) {
    digitalWrite(motors[i], LOW);
  }
  Serial.println("state=halted");
  Serial.println("#!!! HALTED !!!");
  #ifdef NEOPIXELS_PIN
  for (uint8_t i = 0; i<NEOPIXELS_AMOUNT; i++) {
    strip.setPixelColor(i, (uint32_t) 0x00FF0000);
  }
  strip.show();
  #endif
  while(1);
}

void setup() {
  for (uint8_t i = 0; i<MOTORS_AMOUNT; i++) {
    pinMode(switches[i], INPUT_PULLUP);
    pinMode(motors[i], OUTPUT);
    #ifdef EMPTY1_PIN
      pinMode(empty[i], INPUT_PULLUP);
    #endif
  }
  
  Serial.begin(115200);
  
  Serial.println("id="+String(UNIT_ID));
  Serial.println("type=drawer");
  Serial.println("state=booting");

  #ifdef NEOPIXELS_PIN
  Serial.println("boot=leds");
  Serial.println("#Configuring LEDs...");
  strip.begin();
  strip.setBrightness(255);
  for (uint8_t i = 0; i<NEOPIXELS_AMOUNT; i++) {
    strip.setPixelColor(i, (uint32_t) 0x00000000);
  }
  strip.setPixelColor((NEOPIXELS_AMOUNT-1)-(UNIT_ID-1), (uint32_t) 0x00006666);
  strip.show();
  #endif
  
  Serial.println("#Aligning motors...");
  for (uint8_t i = 0; i<MOTORS_AMOUNT; i++) {
    Serial.println("boot=aligning,"+String(i));
    if (!moveChannel(i, true)) {
      Serial.println("boot=error");
      Serial.println("state=error");
      Serial.println("error=motor,"+String(i)+",aligning");
      Serial.println("#HARDWARE ERROR: CAN NOT ALIGN CHANNEL "+String(i));
      halt();
    }
    delay(100);
  }
  Serial.println("boot=ok");
  Serial.println("state=ready");
}

void sendEmpty() {
  #ifdef EMPTY1_PIN
  Serial.print("empty=");
  uint8_t emptyValue = 0;
  for (uint8_t i = 0; i<MOTORS_AMOUNT; i++) {
    if (digitalRead(empty[i])) emptyValue += (1<<i);
  }
  Serial.println(emptyValue);
  #endif
}

bool prevBusy = false;
void loop() {
 while (Serial.available()>0) {
      char c = Serial.read();
      switch(c) {
        case 'e': {
          sendEmpty();
          break;
        }
        case 'd': {
          uint8_t motor = Serial.parseInt();
          if (motor>=MOTORS_AMOUNT) {
            Serial.println("error=motor,"+String(motor)+",invalid");
            Serial.println("#Invalid motor");
          } else {
             if (state[motor]==STATE_IDLE) state[motor] = STATE_STARTING;
          }
          break;
        }

        case 'n': {
          uint8_t motor = Serial.parseInt();
          if (motor>=MOTORS_AMOUNT) {
            Serial.println("error=motor,"+String(motor)+",invalid");
            Serial.println("#Invalid motor");
          } else {
             if (state[motor]==STATE_IDLE) state[motor] = STATE_CORRECTION;
          }
          break;
        }
        
        case 'i':
          Serial.println("id="+String(UNIT_ID));
          break;

        case 'm':
          Serial.println("motors="+String(MOTORS_AMOUNT));
          break;

        case 't':
          Serial.println("type=drawer");
          break;

        case 's':
          //Serial.print("?");
          selectedLed = Serial.parseInt();
          #ifdef NEOPIXELS_PIN
          if (selectedLed>=NEOPIXELS_AMOUNT) selectedLed = NEOPIXELS_AMOUNT-1;
          #endif
          //Serial.println(selectedLed);
          break;

        case 'c': {
          #ifdef NEOPIXELS_PIN
          //Serial.print("?");
          uint32_t color = Serial.parseInt();
          color = (color<<8)+Serial.parseInt();
          color = (color<<8)+Serial.parseInt();
          color = (color<<8)+Serial.parseInt();
          //Serial.println(color, HEX);
          strip.setPixelColor((NEOPIXELS_AMOUNT-1)-selectedLed, color);
          #endif
          break;
        }
        case 'u':
          #ifdef NEOPIXELS_PIN
          strip.show();
          #endif
          //Serial.println("!");
          break;
      }
    }
  
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousEmptyMillis > emptyInterval) {
    previousEmptyMillis = currentMillis;
    sendEmpty();
  }
  
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    bool busy = false;
    for (uint8_t i = 0; i<MOTORS_AMOUNT; i++) {
      if (state[i]!=STATE_IDLE) {
        busy = true;
        break;
      }
    }

    if (prevBusy!=busy) {
      prevBusy = busy;
      if (busy) {
        Serial.println("state=busy");
      } else {
        Serial.println("state=ready");
      }
    }

    for (uint8_t i = 0; i<MOTORS_AMOUNT; i++) {
      switch(state[i]) {
        case STATE_STARTING:
          startTime = millis();
          state[i] = STATE_WAIT1;
          digitalWrite(motors[i], HIGH); //Start motor
          Serial.println("#Preparing to vend product "+String(i)+"...");
          break;
        case STATE_WAIT1:
          //Wait until button gets pressed
          if (!digitalRead(switches[i])) {
            state[i] = STATE_VENDING;
            Serial.println("#Vending product "+String(i)+"...");
          } else if (millis() - startTime >= TIMEOUT) {
            Serial.println("state=error");
            Serial.println("error=motor,"+String(i)+",preparing");
            halt();
          }
          break;
        case STATE_VENDING:
          //Wait until button gets released
          if (digitalRead(switches[i])) {
            state[i] = STATE_WAIT2;
            Serial.println("#Finishing vending process for product "+String(i)+"...");
          } else if (millis() - startTime >= TIMEOUT) {
            Serial.println("state=error");
            Serial.println("error=motor,"+String(i)+",vending");
            halt();
          }
          break;
        case STATE_WAIT2:
          //Wait until button gets pressed
          if (!digitalRead(switches[i])) {
            state[i] = STATE_IDLE;
            digitalWrite(motors[i], LOW);
            Serial.println("#Vending product "+String(i)+" completed.");
          }else if (millis() - startTime >= TIMEOUT) {
            Serial.println("state=error");
            Serial.println("error=motor,"+String(i)+",finising");
            halt();
          }
          break;
        case STATE_CORRECTION:
          if (correctioncounter[i]>1) {
            state[i] = STATE_IDLE;
            correctioncounter[i] = 0;
            digitalWrite(motors[i], LOW);
            Serial.println("#Corrected "+String(i)+".");
          } else {
            Serial.println("state=correcting");
            Serial.println("corr="+String(i)+","+String(correctioncounter[i]));
            Serial.println("#Correcting "+String(i)+": "+String(correctioncounter[i])+"....");
            digitalWrite(motors[i], HIGH);
          }
          correctioncounter[i]++;
      }
    }
  }
}
