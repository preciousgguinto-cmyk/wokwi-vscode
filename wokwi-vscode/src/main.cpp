#include <Arduino.h>

/*
  Smart Fire Suppression and Multi-Zone Sprinkler Controller (CMSC 306)
  Wokwi version: gate-built priority encoder + flip-flop flow counter

  Hardware does:  zone priority (NOT/OR/AND gates), valve interlock (AND + NOT-Q3),
                  4-bit flow counter (D flip-flops, flag Q3 = 8th pulse),
                  fail-safe buzzer (OR gate with ANY).
  Software does:  zone decode, valve requests, simulated flow pulses, LCD, LEDs, Serial log.

  Zone code from the gates (active HIGH): 00 = Zone 1, 01 = Zone 2, 10 = Zone 3, 11 = Zone 4
  ANY goes HIGH when at least one zone has fire. Lower zone number wins.
  Fire = DIP switch ON (pulls that zone's sensor line LOW).
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---- Pins ----
const uint8_t PIN_ANY   = 2;              // ANY fire flag (HIGH = fire), INT0
const uint8_t PIN_B0    = 3;              // zone code bit 0
const uint8_t PIN_B1    = 4;              // zone code bit 1
const uint8_t PIN_VALVE[4] = {6, 7, 8, 9};// valve requests, zone 1..4
const uint8_t PIN_PULSE = 10;             // counter clock (counts on rising edge)
const uint8_t PIN_RESET = 11;             // counter synchronous reset (HIGH + one clock)
const uint8_t PIN_LIMIT = 12;             // Q3 (HIGH from the 8th pulse)
const uint8_t PIN_BUZZ  = 13;             // buzzer request
const uint8_t PIN_LED_G = A0;             // normal
const uint8_t PIN_LED_Y = A1;             // flow limit reached
const uint8_t PIN_LED_R = A2;             // active hazard

// ---- Timing and limits ----
const unsigned long PULSE_PERIOD_MS = 500;   // one simulated flow pulse every 0.5 s
const unsigned long MIN_ACTIVE_MS   = 2000;  // keep valve open at least this long
const unsigned long LCD_PERIOD_MS   = 250;
const uint8_t       LIMIT_PULSES    = 8;     // Q3 goes HIGH at count 8

enum State { NORMAL, ACTIVE, LIMIT };
State state = NORMAL;

volatile uint8_t latchedZone = 0;   // set in ISR so a brief trigger is not missed
uint8_t  activeZone    = 0;
uint8_t  pulsesSent    = 0;
unsigned long incidentStart = 0;
unsigned long lastPulse     = 0;
unsigned long lastLcd       = 0;
bool limitHit = false;

uint8_t decodeZone() {              // call only while ANY is HIGH
  uint8_t code = digitalRead(PIN_B0) | (digitalRead(PIN_B1) << 1);
  return code + 1;                  // 0..3 -> zone 1..4
}

uint8_t readZone() {                // 0 = no hazard
  if (digitalRead(PIN_ANY) == LOW) return 0;
  return decodeZone();
}

void onAny() {                      // ANY rising edge: grab the zone immediately
  latchedZone = decodeZone();
}

void stamp() {
  unsigned long s = millis() / 1000;
  char b[16];
  snprintf(b, sizeof(b), "[T+%02lu:%02lu:%02lu] ", s / 3600, (s / 60) % 60, s % 60);
  Serial.print(b);
}

void setValve(uint8_t zone) {       // zone 0 = all closed
  for (uint8_t i = 0; i < 4; i++) digitalWrite(PIN_VALVE[i], (zone == i + 1) ? HIGH : LOW);
}

void clockPulse() {                 // one rising edge on the counter clock
  digitalWrite(PIN_PULSE, HIGH);
  delay(1);
  digitalWrite(PIN_PULSE, LOW);
  delay(1);
}

void resetCounter() {               // synchronous reset: RESET high, then one clock edge
  digitalWrite(PIN_RESET, HIGH);
  delay(1);
  clockPulse();
  digitalWrite(PIN_RESET, LOW);
  pulsesSent = 0;
}

void sendFlowPulse() {
  clockPulse();
  pulsesSent++;
  lastPulse = millis();
}

void startIncident(uint8_t zone) {
  activeZone = zone;
  incidentStart = millis();
  lastPulse = millis();
  limitHit = false;
  resetCounter();
  setValve(zone);
  state = ACTIVE;
  stamp(); Serial.print("INCIDENT START zone="); Serial.println(zone);
}

void endIncident() {
  setValve(0);
  unsigned long dur = millis() - incidentStart;
  stamp();
  Serial.print("INCIDENT END zone="); Serial.print(activeZone);
  Serial.print(" duration="); Serial.print(dur / 1000.0, 1);
  Serial.print("s pulses="); Serial.print(pulsesSent);
  Serial.print(" limit="); Serial.println(limitHit ? "YES" : "NO");
  resetCounter();
  latchedZone = 0;
  activeZone = 0;
  state = NORMAL;
}

void updateLeds() {
  digitalWrite(PIN_LED_G, state == NORMAL);
  digitalWrite(PIN_LED_R, state == ACTIVE);
  digitalWrite(PIN_LED_Y, state == LIMIT);
}

void printLine(uint8_t row, const char* text) {
  char b[17];
  snprintf(b, sizeof(b), "%-16s", text);
  lcd.setCursor(0, row);
  lcd.print(b);
}

void updateLcd() {
  char l1[17], l2[17];
  if (state == NORMAL) {
    snprintf(l1, sizeof(l1), "SYSTEM READY");
    snprintf(l2, sizeof(l2), "All zones clear");
  } else if (state == ACTIVE) {
    snprintf(l1, sizeof(l1), "FIRE ZONE %u", activeZone);
    snprintf(l2, sizeof(l2), "Flow %u/%u V:ON", pulsesSent, LIMIT_PULSES);
  } else {
    snprintf(l1, sizeof(l1), "MAX FLOW Z%u", activeZone);
    snprintf(l2, sizeof(l2), "Flow %u/%u V:OFF", pulsesSent, LIMIT_PULSES);
  }
  printLine(0, l1);
  printLine(1, l2);
}

void setup() {
  Serial.begin(9600);

  pinMode(PIN_ANY, INPUT);
  pinMode(PIN_B0, INPUT);
  pinMode(PIN_B1, INPUT);
  pinMode(PIN_LIMIT, INPUT);
  for (uint8_t i = 0; i < 4; i++) { pinMode(PIN_VALVE[i], OUTPUT); digitalWrite(PIN_VALVE[i], LOW); }
  pinMode(PIN_PULSE, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_BUZZ,  OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);

  lcd.init();
  lcd.backlight();

  resetCounter();                   // flip-flops power up in an unknown state
  attachInterrupt(digitalPinToInterrupt(PIN_ANY), onAny, RISING);

  stamp(); Serial.println("SYSTEM READY");
  updateLeds();
  updateLcd();
}

void loop() {
  uint8_t zone  = readZone();
  bool limitHW  = (digitalRead(PIN_LIMIT) == HIGH);   // counter Q3 flag

  switch (state) {
    case NORMAL: {
      uint8_t z = zone ? zone : latchedZone;
      if (z) startIncident(z);
      break;
    }

    case ACTIVE:
      if (zone && zone != activeZone) {                // different zone now has priority
        activeZone = zone;
        setValve(zone);
        stamp(); Serial.print("PRIORITY CHANGE zone="); Serial.println(zone);
      }
      if (!zone && millis() - incidentStart >= MIN_ACTIVE_MS) {
        endIncident();
        break;
      }
      if (limitHW) {                                   // hardware gate already closed the valve
        limitHit = true;
        setValve(0);
        state = LIMIT;
        stamp(); Serial.println("MAX DISCHARGE REACHED, valves closed by interlock");
      } else if (millis() - lastPulse >= PULSE_PERIOD_MS) {
        sendFlowPulse();
      }
      break;

    case LIMIT:
      if (!zone && millis() - incidentStart >= MIN_ACTIVE_MS) endIncident();
      break;
  }

  digitalWrite(PIN_BUZZ, state != NORMAL);             // OR gate also forces buzzer from ANY
  updateLeds();

  if (millis() - lastLcd >= LCD_PERIOD_MS) {
    lastLcd = millis();
    updateLcd();
  }
}
