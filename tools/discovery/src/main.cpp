// Wiring discovery for the XIAO ESP32C3 + STM32 + OLED rig.
//
// Phase A is entirely PASSIVE - every candidate pin is configured as a plain
// input and merely observed. Nothing is driven, so an STM32 output can never
// fight us. Only pins that Phase A proves are idle do we later drive as I2C.

#include <Arduino.h>
#include <Wire.h>

static const uint8_t PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
static const size_t NPINS = sizeof(PINS);

struct PinInfo {
  uint8_t pin;
  uint32_t transitions;
  uint32_t min_pulse_us;
  int idle_level;
};
static PinInfo info[NPINS];

static const uint32_t BAUDS[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800};

static uint32_t snapBaud(uint32_t measured) {
  uint32_t best = 0; float bestErr = 1e9;
  for (uint32_t b : BAUDS) {
    float e = fabsf((float)measured - (float)b) / (float)b;
    if (e < bestErr) { bestErr = e; best = b; }
  }
  return (bestErr < 0.25f) ? best : 0;
}

// Watch one pin for `ms` without driving it.
static PinInfo observe(uint8_t pin, uint32_t ms) {
  PinInfo r{pin, 0, 0xFFFFFFFF, -1};
  pinMode(pin, INPUT);            // no pullup: don't perturb the line
  delayMicroseconds(200);

  int last = digitalRead(pin);
  uint32_t tLast = micros();
  uint32_t high = 0, total = 0;
  uint32_t tEnd = millis() + ms;

  while (millis() < tEnd) {
    int v = digitalRead(pin);
    total++; if (v) high++;
    if (v != last) {
      uint32_t now = micros(), w = now - tLast;
      if (r.transitions > 0 && w < r.min_pulse_us) r.min_pulse_us = w;
      tLast = now; last = v; r.transitions++;
    }
  }
  if (r.transitions == 0) r.min_pulse_us = 0;
  r.idle_level = (total && high > total / 2) ? 1 : 0;
  return r;
}

static void phaseA() {
  Serial.println("\n--- PHASE A: passive pin observation (nothing driven) ---");
  Serial.println("pin  idle  transitions  min_pulse_us  implied_baud");
  for (size_t i = 0; i < NPINS; i++) {
    info[i] = observe(PINS[i], 300);
    uint32_t implied = info[i].min_pulse_us ? 1000000UL / info[i].min_pulse_us : 0;
    Serial.printf("%3u  %4d  %11lu  %12lu  %s%lu\n",
                  info[i].pin, info[i].idle_level,
                  (unsigned long)info[i].transitions,
                  (unsigned long)info[i].min_pulse_us,
                  implied ? "~" : "", (unsigned long)implied);
  }
}

static bool isQuiet(uint8_t pin) {
  for (size_t i = 0; i < NPINS; i++)
    if (info[i].pin == pin) return info[i].transitions == 0 && info[i].idle_level == 1;
  return false;
}

static void scanPair(uint8_t sda, uint8_t scl) {
  if (!isQuiet(sda) || !isQuiet(scl)) {
    Serial.printf("SDA=%2u SCL=%2u : skipped (pin active or idle-low)\n", sda, scl);
    return;
  }
  Wire.end();
  delay(5);
  if (!Wire.begin(sda, scl, 100000)) {
    Serial.printf("SDA=%2u SCL=%2u : Wire.begin failed\n", sda, scl);
    return;
  }
  String found = "";
  for (uint8_t a = 3; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { char b[8]; snprintf(b, 8, " 0x%02X", a); found += b; }
  }
  Wire.end();
  Serial.printf("SDA=%2u SCL=%2u :%s\n", sda, scl, found.length() ? found.c_str() : " -");
}

static void phaseB() {
  Serial.println("\n--- PHASE B: I2C scan (only on pins Phase A proved idle) ---");
  const uint8_t pairs[][2] = {{6,7},{7,6},{8,9},{9,8},{4,5},{5,4},{2,3},{3,2},{20,21},{21,20},{10,20},{6,5}};
  for (auto &p : pairs) scanPair(p[0], p[1]);
}

static void phaseC() {
  Serial.println("\n--- PHASE C: UART decode on active pins ---");
  bool any = false;
  for (size_t i = 0; i < NPINS; i++) {
    if (info[i].transitions < 5) continue;
    any = true;
    uint32_t implied = info[i].min_pulse_us ? 1000000UL / info[i].min_pulse_us : 0;
    uint32_t baud = snapBaud(implied);
    Serial.printf("\nGPIO%u is active (implied ~%lu baud -> trying %lu)\n",
                  info[i].pin, (unsigned long)implied, (unsigned long)baud);
    if (!baud) { Serial.println("  no standard baud matches; skipping decode"); continue; }

    Serial1.begin(baud, SERIAL_8N1, info[i].pin, -1);   // RX only, TX disabled
    delay(50);
    while (Serial1.available()) Serial1.read();

    uint8_t buf[256]; size_t n = 0;
    uint32_t tEnd = millis() + 2000;
    while (millis() < tEnd && n < sizeof(buf))
      if (Serial1.available()) buf[n++] = Serial1.read();
    Serial1.end();

    Serial.printf("  %u bytes in 2s\n", (unsigned)n);
    if (!n) continue;
    size_t pr = 0;
    for (size_t k = 0; k < n; k++)
      if ((buf[k] >= 32 && buf[k] < 127) || buf[k] == 10 || buf[k] == 13) pr++;
    Serial.printf("  printable %u/%u (%u%%)\n", (unsigned)pr, (unsigned)n,
                  (unsigned)(100 * pr / n));
    Serial.print("  hex:");
    for (size_t k = 0; k < n && k < 48; k++) Serial.printf(" %02X", buf[k]);
    Serial.println();
    Serial.print("  txt: ");
    for (size_t k = 0; k < n; k++)
      Serial.print((buf[k] >= 32 && buf[k] < 127) ? (char)buf[k]
                   : (buf[k] == 10 ? '\n' : (buf[k] == 13 ? ' ' : '.')));
    Serial.println();
  }
  if (!any) Serial.println("no pin showed UART-like activity");
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println("\n================ XIAO ESP32C3 wiring discovery ================");
  phaseA();
  phaseB();
  phaseC();
  Serial.println("\n================ discovery complete ================");
}

void loop() {
  delay(5000);
  Serial.println("[idle] re-running phase A/C to catch intermittent traffic");
  phaseA();
  phaseC();
}
