#include <SPI.h>
#include <DFRobot_MCP2515.h>

// Scania ICL2 cluster driven from ETS2 telemetry.
// Arduino Mega + DFRobot CAN shield (MCP2515), 250 kbps, 29-bit J1939 IDs.
//
// Serial input (115200 baud), one line per update:
//   d<speed km/h x10>,<rpm>,<fuel % x10>,<flags>,<odometer km x10>,<water C>,<oil C>
// Example: "d853,1450,725,5,1234567,88,95"
//   = 85.3 km/h, 1450 rpm, 72.5 % fuel, left indicator + high beam,
//     123456.7 km, water 88 C, oil 95 C.
// Only the first 3 fields are required; missing trailing fields are left unchanged.
//
// flags bitmask: 1 left indicator, 2 right indicator, 4 high beam, 8 low beam,
//                16 parking brake, 32 parking lights.  Hazards = 1 + 2.

#define ENABLE_CCVS 1     // extra CCVS frame (parking brake + speed). Set 0 if it upsets the cluster.
#define TEMP_OFFSET 40    // J1939 coolant temp: raw = C + 40. Adjust if the gauge reads off.
#define INVERT_PARKING_LIGHTS 1   // 1: cluster bright when game lights are ON, dim when OFF. 0: the other way round.
#define BLINK_MS 350      // indicator on/off time (the Arduino generates the flashing)

const uint8_t F_LEFT = 1, F_RIGHT = 2, F_HIGH = 4, F_LOW = 8, F_PARKBRAKE = 16, F_PARKLIGHTS = 32;

DFRobot_MCP2515 CAN(10);

// ---- live values, updated from serial ----
uint8_t eec1[8]       = {0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x2E, 0x03, 0xFF};  // rpm: bytes 3..4
uint8_t speedFrame[8] = {0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0x00};  // speed: bytes 6..7
uint8_t fuel[8]       = {0xFF, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   // level: byte 1
uint8_t coolant[8]    = {141, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   // water: byte 0, oil: bytes 2..3
uint8_t cuvInfo[8]    = {0x00, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};  // lights + VIS fix
uint8_t distanceFrame[8] = {0x00, 0xC2, 0x78, 0x01, 0x00, 0x00, 0x00, 0x00}; // odometer: bytes 0..3
#if ENABLE_CCVS
uint8_t ccvs[8]       = {0xF3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // parking brake + speed
#endif

// ---- static frames the cluster needs ----
uint8_t airPressure[8] = {0xFF, 100, 100, 100, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t oilPressure[8] = {0xFF, 0xFF, 0xFF, 100, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t allFF[8]       = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // BWE heartbeat
uint8_t ignFrame[8]    = {0x00, 0x0F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t DM1_OK[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t DM1_NA[8] = {0x3C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t DLN2[8]   = {0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t EBC1[8]   = {0x10, 0xFF, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x10};
uint8_t EBC4[8]   = {250, 250, 250, 250, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned long last20 = 0, last100 = 0, last900 = 0;
unsigned long lastTelemetry = 0;
bool cuvDirty = false;

void sendFrame(uint32_t id, uint8_t *data) {
  CAN.sendMsgBuf(id, 1, 8, data);
}

// ---- value setters ----
void setSpeed(uint16_t kmh10) {            // km/h x10 -> 1/256 km/h per bit
  uint32_t raw = (uint32_t)kmh10 * 256UL / 10UL;
  if (raw > 0xFA00) raw = 0xFA00;
  speedFrame[6] = raw & 0xFF;
  speedFrame[7] = raw >> 8;
#if ENABLE_CCVS
  ccvs[1] = raw & 0xFF;
  ccvs[2] = raw >> 8;
#endif
}

void setRpm(uint16_t rpm) {                // 0.125 rpm per bit
  uint32_t raw = (uint32_t)rpm * 8UL;
  if (raw > 0xFAFF) raw = 0xFAFF;
  eec1[3] = raw & 0xFF;
  eec1[4] = raw >> 8;
}

void setFuel(uint16_t pct10) {             // % x10 -> 0.4 % per bit
  uint16_t raw = pct10 / 4;
  if (raw > 250) raw = 250;
  fuel[1] = raw;
}

void setOdometer(uint32_t km10) {          // km x10 -> 5 m per bit (km x200)
  uint32_t raw = km10 * 20UL;
  distanceFrame[0] = raw & 0xFF;
  distanceFrame[1] = (raw >> 8) & 0xFF;
  distanceFrame[2] = (raw >> 16) & 0xFF;
  distanceFrame[3] = (raw >> 24) & 0xFF;
}

void setWaterTemp(int c) {
  coolant[0] = constrain(c + TEMP_OFFSET, 0, 250);
}

void setOilTemp(int c) {                   // 0.03125 C per bit, offset -273 C
  uint16_t raw = (uint16_t)((constrain(c, -40, 210) + 273) * 32);
  coolant[2] = raw & 0xFF;
  coolant[3] = raw >> 8;
}

uint8_t curFlags = 0;                      // lever/switch state from the PC
bool blinkPhase = false;                   // generated flashing phase
unsigned long lastBlink = 0;

void applyLamps() {
  uint8_t fl = curFlags;
  uint8_t b0 = ((fl & F_HIGH) ? 0x01 : 0) | ((fl & F_LOW) ? 0x04 : 0);
  uint8_t b2 = ((fl & F_LEFT) && blinkPhase ? 0x10 : 0) | ((fl & F_RIGHT) && blinkPhase ? 0x40 : 0);
  bool parkLamp = (fl & F_PARKLIGHTS) != 0;
#if INVERT_PARKING_LIGHTS
  parkLamp = !parkLamp;                    // cluster dims when it sees "parking lights on"
#endif
  uint8_t b7 = 0xCF | (parkLamp ? 0x10 : 0);
  if (b0 != cuvInfo[0] || b2 != cuvInfo[2] || b7 != cuvInfo[7]) {
    cuvInfo[0] = b0;
    cuvInfo[2] = b2;
    cuvInfo[7] = b7;
    cuvDirty = true;                       // send right away, not at the next 100 ms tick
  }
}

void setFlags(uint8_t fl) {
  bool hadBlinker = curFlags & (F_LEFT | F_RIGHT);
  bool hasBlinker = fl & (F_LEFT | F_RIGHT);
  curFlags = fl;
  if (hasBlinker && !hadBlinker) {         // lever just moved: start with the lamp on
    blinkPhase = true;
    lastBlink = millis();
  }
  applyLamps();
#if ENABLE_CCVS
  ccvs[0] = 0xF3 | ((fl & F_PARKBRAKE) ? 0x04 : 0);
#endif
}

void blinkTick() {
  if ((curFlags & (F_LEFT | F_RIGHT)) && millis() - lastBlink >= BLINK_MS) {
    lastBlink = millis();
    blinkPhase = !blinkPhase;
    applyLamps();
  }
}

// ---- serial line parser (non-blocking) ----
void readSerial() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[n] = 0;
      n = 0;
      if (buf[0] != 'd') continue;

      int s = 0, r = 0, f = 0, fl = 0, w = 0, o = 0;
      long odo = 0;
      int cnt = sscanf(buf + 1, "%d,%d,%d,%d,%ld,%d,%d", &s, &r, &f, &fl, &odo, &w, &o);
      if (cnt >= 3) {
        setSpeed(constrain(s, 0, 2500));
        setRpm(constrain(r, 0, 8000));
        setFuel(constrain(f, 0, 1000));
        lastTelemetry = millis();
      }
      if (cnt >= 4) setFlags((uint8_t)fl);
      if (cnt >= 5) setOdometer((uint32_t)constrain(odo, 0L, 20000000L));
      if (cnt >= 6) setWaterTemp(w);
      if (cnt >= 7) setOilTemp(o);
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ---- cyclic CAN transmission ----
void txPeriodic() {
  unsigned long now = millis();

  if (now - last20 >= 20) {
    last20 = now;
    sendFrame(0x0CF00400, eec1);
    sendFrame(0x0CFE6CEE, speedFrame);
    if (cuvDirty) {
      sendFrame(0x18FFB11E, cuvInfo);
      cuvDirty = false;
    }
  }

  if (now - last100 >= 100) {
    last100 = now;
    sendFrame(0x0CFEEE00, coolant);
    sendFrame(0x0CFEFC27, fuel);
    sendFrame(0x18FEAE30, airPressure);
    sendFrame(0x18FEEF27, oilPressure);
    sendFrame(0x18FFB11E, cuvInfo);
    sendFrame(0x18FFFAE6, allFF);
    sendFrame(0x18FEC1EE, distanceFrame);
    sendFrame(0x18FEE427, ignFrame);
#if ENABLE_CCVS
    sendFrame(0x18FEF100, ccvs);
#endif
  }

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

void setup() {
  Serial.begin(115200);

  while (CAN.begin(CAN_250KBPS) != 0) {
    Serial.println(F("CAN init error, retrying..."));
    delay(1000);
  }

  Serial.println(F("Scania ICL bridge ready - CAN 250 kbps, waiting for ETS2 data"));
}

void loop() {
  readSerial();

  // no data from the PC for 2 s -> zero speed, rpm and lights
  if (lastTelemetry && millis() - lastTelemetry > 2000) {
    setSpeed(0);
    setRpm(0);
    setFlags(0);
    lastTelemetry = 0;
  }

  blinkTick();
  txPeriodic();
}
