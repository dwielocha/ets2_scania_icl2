#include <SPI.h>
#include <DFRobot_MCP2515.h>

// =====================================================================================
// Scania ICL2 instrument cluster bench driver (Arduino Mega + DFRobot CAN shield, MCP2515)
// 250 kbps, 29-bit J1939 IDs.
//
// Wiring (bench): ICL A:19 -> +24 V, A:20 -> 24 V GND, B:04 -> +24 V via switch (ignition),
//                 A:09 / A:18 -> CAN H / CAN L.
//                 INFO button emulation: Mega pin 5 -> A:05, Mega GND -> A:01
//                 (A:05 idles ~4.9 V, pull-up ~490 ohm, short-circuit current 9.8 mA)
//
// Serial keys (115200 baud, line ending can be anything):
//   s / r      : stop / resume all periodic transmitting
//   0          : toggle BWE heartbeat (18FFFAE6)
//   v          : toggle VIS fix (18FFB11E)
//   c          : clear burst (all-FF frames for PGN FF00..FFFF)
//   p<n>       : press INFO input with PWM duty n (0..255), e.g. p100
//   t<ms>      : hold time of a press in ms (default 400), e.g. t1500
//   w          : sweep press duty 245 -> 0 (step 3), results printed as PRESS lines
//   x          : stop sweeps / release button line
//   k<i> <v>   : send 0CFFC127 for 300 ms with byte i = v (decimal), e.g. "k0 254"
//   g<i>       : slow sweep of bit patterns in byte i of 0CFFC127 (3 s per step), e.g. g0
//
// Findings so far (keep):
//  - BWE alert disappears when 18FFFAE6 is present (only the PGN matters, not the source).
//  - VIS error is fixed by 18FFB11E.
//  - INFO input A:05: accepted presses roughly duty 60..135 (~1.2..2.7 V). Edges are fuzzy
//    (45 worked, 50 failed, 140 failed). With 2 alerts active a press steps BWE -> VIS.
//  - 18FEC1EE (distance, source EE) fills the km field.
//  - Do NOT block the main loop for long: gaps in the heartbeat / VIS frames raise alerts.
// =====================================================================================

DFRobot_MCP2515 CAN(10);

bool transmitting = true;
bool bweHeartbeat = true;   // 18FFFAE6: clears the BWE alert
bool visFix = false;         // 18FFB11E: clears the VIS error

const uint8_t SPEED_VALUE = 0;
const uint8_t TEMP_VALUE = 141;  // about 90 C
const uint8_t FUEL_VALUE = 225;  // 90 %

// ---- frames sent all the time ----
uint8_t eec1[8] = {0xFF, 0xFF, 0xFF, 0x00, 0x19, 0x2E, 0x03, 0xFF};
uint8_t speedFrame[8] = {0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00, SPEED_VALUE};
uint8_t coolant[8] = {TEMP_VALUE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t fuel[8] = {0xFF, FUEL_VALUE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t airPressure[8] = {0xFF, 100, 100, 100, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t oilPressure[8] = {0xFF, 0xFF, 0xFF, 100, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t DM1_OK[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t DM1_NA[8] = {0x3C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t DLN2[8] = {0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t EBC1[8] = {0x10, 0xFF, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x10};
uint8_t EBC4[8] = {250, 250, 250, 250, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t cuvInfo[8] = {0x00, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};  // 18FFB11E from VIS (1E)
uint8_t allFF[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// High resolution vehicle distance (PGN FEC1, source EE): 5 m/bit, little endian.
// bytes 1-4 total distance, bytes 5-8 trip distance. 0x0178C200 = 123456 km.
uint8_t distanceFrame[8] = {0x00, 0xC2, 0x78, 0x01, 0x00, 0x00, 0x00, 0x00};

// "Ignition on" frame suggested by the Google text (unverified): PGN FEE4, source 27
uint8_t ignFrame[8] = {0x00, 0x0F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};

// ---- timers ----
unsigned long last20 = 0;
unsigned long last100 = 0;
unsigned long last900 = 0;

// ---- INFO button emulation on A:05 (Mega pin 5 = OC3A, 31 kHz PWM) ----
const int BTN_PIN = 5;
uint16_t holdMs = 400;
const uint16_t GAP_MS = 1500;
bool btnActive = false;
unsigned long btnEnd = 0;

uint8_t lastDuty = 255;          // duty of the most recent press
unsigned long lastPressMs = 0;   // when it started
uint8_t prevDuty = 255;          // the press before that
unsigned long prevPressMs = 0;
uint16_t markCount = 0;

bool sweepOn = false;
int sweepDuty = 245;
int sweepStep = 3;
int sweepEnd = 0;
unsigned long sweepNext = 0;

bool abortFlag = false;           // set when 'x' is received during a blocking test
bool ignOn = true;

const int SENSE_PIN = A0;
bool loopOn = false;
int idleAdc = 0, targetAdc = 0, curDuty = 0;
unsigned long nextLoop = 0;

int avgAdc(uint8_t n) {
  long s = 0;
  for (uint8_t i = 0; i < n; i++) s += analogRead(SENSE_PIN);
  return s / n;
}

// q<permille>: press at permille/1000 of the idle voltage, e.g. q400
void pressRatio(int permille) {
  btnIdle();
  delay(5);                                   // let the line settle at idle
  idleAdc = avgAdc(32);
  targetAdc = (long)idleAdc * permille / 1000;
  curDuty = constrain((long)permille * 255 / 1000, 0, 255);
  analogWrite(BTN_PIN, curDuty);
  pinMode(BTN_PIN, OUTPUT);
  btnActive = true;
  btnEnd = millis() + holdMs;
  loopOn = true;
  Serial.print(F("PRESS ratio ")); Serial.print(permille);
  Serial.print(F("  idleAdc ")); Serial.print(idleAdc);
  Serial.print(F("  target ")); Serial.println(targetAdc);
}

void loopTick() {                             // call from loop()
  if (!loopOn) return;
  if (!btnActive) { loopOn = false; return; }
  if (millis() < nextLoop) return;
  nextLoop = millis() + 2;
  int a = avgAdc(8);
  if (a < targetAdc - 1 && curDuty < 255) curDuty++;
  else if (a > targetAdc + 1 && curDuty > 0) curDuty--;
  analogWrite(BTN_PIN, curDuty);
}

// =====================================================================================
// CAN helpers
// =====================================================================================

void sendFrame(uint32_t id, uint8_t *data) {
  CAN.sendMsgBuf(id, 1, 8, data);
}

void clearBurst(uint8_t rounds) {
  Serial.println(F("Clear burst: all-FF frames for PGN FF00..FFFF"));
  for (uint8_t r = 0; r < rounds; r++) {
    for (uint16_t ps = 0; ps < 256; ps++) {
      sendFrame(0x18FF0000UL | ((uint32_t)ps << 8) | 0xE6, allFF);
      delay(1);
    }
  }
  Serial.println(F("Clear burst done"));
}

// ---- receive logging: print only when the data of an ID changed ----
const uint8_t CACHE_SIZE = 40;
uint32_t cacheId[CACHE_SIZE];
uint8_t cacheData[CACHE_SIZE][8];
uint8_t cacheCount = 0;

bool shouldPrint(uint32_t id, uint8_t *data) {
  for (uint8_t i = 0; i < cacheCount; i++) {
    if (cacheId[i] == id) {
      if (memcmp(cacheData[i], data, 8) == 0) return false;
      memcpy(cacheData[i], data, 8);
      return true;
    }
  }
  if (cacheCount < CACHE_SIZE) {
    cacheId[cacheCount] = id;
    memcpy(cacheData[cacheCount], data, 8);
    cacheCount++;
  }
  return true;
}

void printData(uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void receiveCAN() {
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    uint8_t len = 0;
    uint8_t data[8] = {0};

    CAN.readMsgBuf(&len, data);
    uint32_t id = CAN.getCanId();

    uint8_t pf = (id >> 16) & 0xFF;
    uint8_t ps = (id >> 8) & 0xFF;
    uint8_t sa = id & 0xFF;

    if (shouldPrint(id, data)) {
      Serial.print(F("RX | ID: 0x"));
      Serial.print(id, HEX);
      Serial.print(F(" | DATA: "));
      printData(data, len);
    }

    if ((pf == 0xDA || pf == 0xDB)) {
      Serial.print(F(">>> DIAG to 0x"));
      Serial.print(ps, HEX);
      Serial.print(F(" from 0x"));
      Serial.print(sa, HEX);
      Serial.print(F(" : "));
      printData(data, len);
    }
  }
}

// =====================================================================================
// Periodic transmission (also called from inside blocking tests)
// =====================================================================================

void txPeriodic() {
  if (!transmitting) return;
  unsigned long now = millis();

  // every 20 ms
  if (now - last20 >= 20) {
    last20 = now;
    sendFrame(0x0CF00400, eec1);
    sendFrame(0x0CFE6CEE, speedFrame);
  }

  // every 100 ms
  if (now - last100 >= 100) {
    last100 = now;
    sendFrame(0x0CFEEE00, coolant);
    sendFrame(0x0CFEFC27, fuel);
    sendFrame(0x18FEAE30, airPressure);
    sendFrame(0x18FEEF27, oilPressure);
    if (visFix) sendFrame(0x18FFB11E, cuvInfo);       // fixes the VIS error
    if (bweHeartbeat) sendFrame(0x18FFFAE6, allFF);   // clears the BWE alert
    sendFrame(0x18FEC1EE, distanceFrame);             // odometer / trip km
    if (ignOn) sendFrame(0x18FEE427, ignFrame);
  }

  // every 900 ms: diagnostics
  if (now - last900 >= 900) {
    last900 = now;

    sendFrame(0x18FECA0B, DM1_OK);
    sendFrame(0x18FECA30, DM1_OK);
    sendFrame(0x18FECA43, DM1_NA);
    sendFrame(0x18FECA00, DM1_OK);
    sendFrame(0x18FECA2F, DM1_OK);
    sendFrame(0x18FECA27, DM1_OK);
    sendFrame(0x18FECA10, DM1_OK);
    sendFrame(0x18FECA03, DM1_OK);

    sendFrame(0x0CFF8100, DLN2);
    sendFrame(0x18F0010B, EBC1);
    sendFrame(0x1CFEAC0B, EBC4);
  }
}

// =====================================================================================
// INFO button emulation
// =====================================================================================

void btnIdle() {                  // high-Z = "no button", the cluster pull-up wins
  pinMode(BTN_PIN, INPUT);
  btnActive = false;
}

void btnPress(uint8_t duty) {
  analogWrite(BTN_PIN, duty);
  pinMode(BTN_PIN, OUTPUT);
  
  btnActive = true;
  btnEnd = millis() + holdMs;
  Serial.print(F("PRESS duty "));
  Serial.print(duty);
  Serial.print(F("  (~"));
  Serial.print(duty * 5.0 / 255.0, 2);
  Serial.print(F(" V, hold "));
  Serial.print(holdMs);
  Serial.println(F(" ms)"));
  prevDuty = lastDuty;
  prevPressMs = lastPressMs;
  lastDuty = duty;
  lastPressMs = millis();
}

void buttonTick() {
  unsigned long now = millis();
  if (btnActive && (long)(now - btnEnd) >= 0) btnIdle();
  if (sweepOn && !btnActive && (long)(now - sweepNext) >= 0) {
    btnPress(sweepDuty);
    sweepNext = now + holdMs + GAP_MS;
    sweepDuty -= sweepStep;
    if (sweepDuty < sweepEnd) {
      sweepOn = false;
      Serial.println(F("sweep done"));
    }
  }
}

// =====================================================================================
// Blocking-test helpers: keep everything running, abort on 'x'
// =====================================================================================

void pollAbort() {
  while (Serial.available()) {
    char c = Serial.peek();
    if (c == 'x' || c == 'X') {
      Serial.read();
      abortFlag = true;
      sweepOn = false;
      btnIdle();
    } else {
      Serial.read();              // discard newline / other characters
    }
  }
}

// waits ms while keeping CAN traffic and button timing alive; returns false if aborted
bool waitMs(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    receiveCAN();
    txPeriodic();
    buttonTick();
    pollAbort();
    if (abortFlag) return false;
  }
  return true;
}

// send 0CFFC127 (all FF except byte idx = val) every 20 ms for 300 ms
void sendFfc1(uint8_t idx, uint8_t val) {
  uint8_t f[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  f[idx & 7] = val;
  unsigned long t = millis();
  unsigned long lastTx = 0;
  while (millis() - t < 300) {
    if (millis() - lastTx >= 20) {
      sendFrame(0x0CFFC127, f);
      lastTx = millis();
    }
    receiveCAN();
    txPeriodic();
  }
  Serial.print(F("FFC1 byte"));
  Serial.print(idx);
  Serial.print(F(" = 0x"));
  Serial.println(val, HEX);
}

// =====================================================================================
// Serial commands
// =====================================================================================

void handleSerial() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 's') {
      transmitting = false;
      Serial.println(F("Transmitting stopped"));
    } else if (c == 'r') {
      transmitting = true;
      Serial.println(F("Transmitting resumed"));
    } else if (c == '0') {
      bweHeartbeat = !bweHeartbeat;
      Serial.println(bweHeartbeat ? F("BWE heartbeat 18FFFAE6 ON") : F("BWE heartbeat 18FFFAE6 off"));
    } else if (c == 'v') {
      visFix = !visFix;
      Serial.println(visFix ? F("VIS fix 18FFB11E ON") : F("VIS fix 18FFB11E off"));
    } else if (c == 'c') {
      clearBurst(3);
    } else if (c == 'p') {
      sweepOn = false;
      int d = Serial.parseInt();
      btnPress((uint8_t)constrain(d, 0, 255));
    } else if (c == 't') {
      int t = Serial.parseInt();
      holdMs = (uint16_t)constrain(t, 50, 10000);
      Serial.print(F("hold time = "));
      Serial.print(holdMs);
      Serial.println(F(" ms"));
    } else if (c == 'w') {
      sweepDuty = 245;
      sweepStep = 3; sweepEnd = 0;
      sweepNext = millis();
      sweepOn = true;
      Serial.println(F("sweep start"));
    } else if (c == 'x') {
      sweepOn = false;
      btnIdle();
      Serial.println(F("stopped, button released"));
    } else if (c == 'k') {
      int idx = Serial.parseInt();
      int val = Serial.parseInt();
      sendFfc1((uint8_t)idx, (uint8_t)val);
    } else if (c == 'g') {
      int idx = Serial.parseInt();
      const uint8_t pats[] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F,
                              0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x55, 0x00};
      abortFlag = false;
      Serial.print(F("FFC1 sweep of byte "));
      Serial.print(idx);
      Serial.println(F(" (send x to abort)"));
      for (uint8_t p = 0; p < sizeof(pats); p++) {
        sendFfc1((uint8_t)idx, pats[p]);
        if (!waitMs(3000)) {
          Serial.println(F("aborted"));
          return;
        }
      }
      Serial.println(F("FFC1 sweep done"));
    } else if (c == 'i') {              // toggle the ignition frame
      ignOn = !ignOn;
      Serial.println(ignOn ? F("IGN frame ON") : F("IGN frame off"));
    } else if (c == 'j') {              
      // j<idx> <val> change a byte, e.g. "j1 15" (decimal)
      int idx = Serial.parseInt();
      int val = Serial.parseInt();
      ignFrame[idx & 7] = (uint8_t)val;
      Serial.print(F("IGN byte")); Serial.print(idx & 7);
      Serial.print(F(" = 0x")); Serial.println((uint8_t)val, HEX);
    }  else if (c == 'm') {              // m = mark: "the screen just changed"
      markCount++;
      unsigned long now = millis();
      Serial.print(F(">>> MARK #"));
      Serial.print(markCount);
      Serial.print(F("  last press: duty "));
      Serial.print(lastDuty);
      Serial.print(F(" ("));
      Serial.print((now - lastPressMs) / 1000.0, 1);
      Serial.print(F(" s ago)  | previous press: duty "));
      Serial.print(prevDuty);
      Serial.print(F(" ("));
      Serial.print((now - prevPressMs) / 1000.0, 1);
      Serial.println(F(" s ago)"));
    } else if (c == 'q') { sweepOn = false; pressRatio(Serial.parseInt()); }
     else if (c == 'f') {              // f<start> <end>, step 1, e.g. "f150 40"
      int s = Serial.parseInt();
      int e = Serial.parseInt();
      sweepDuty = constrain(s, 0, 255);
      sweepEnd = constrain(e, 0, 255);
      sweepStep = 1;
      sweepNext = millis();
      sweepOn = true;
      Serial.println(F("fine sweep start"));
    }
  }
}

// =====================================================================================
// setup / loop
// =====================================================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);                 // parseInt must not stall the CAN traffic

  TCCR3B = (TCCR3B & 0xF8) | 0x01;       // ~31 kHz PWM on pin 5 (timer 3)
  btnIdle();

  while (CAN.begin(CAN_250KBPS) != 0) {
    Serial.println(F("CAN init error, retrying..."));
    delay(1000);
  }

  Serial.println(F("CAN OK - 250 kbps"));
  clearBurst(3);
}

void loop() {
  receiveCAN();
  handleSerial();
  buttonTick();
  loopTick();
  txPeriodic();
}