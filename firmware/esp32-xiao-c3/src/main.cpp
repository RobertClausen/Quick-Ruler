// Seeed XIAO ESP32C3 - IR distance sensor readout.
//
//   * reads real samples from an STM32F103 over UART (frames defined below)
//   * shows sample rate / distance / raw voltage on a 128x32 SSD1306
//   * streams every sample to a PC over BLE (Nordic UART Service)
//
// WIRING (verified on hardware, see README):
//   OLED   SDA = GPIO6, SCL = GPIO7, address 0x3C
//   STM32  PA9 (USART1_TX) -> GPIO20 (D7, our RX)
//          PA10 (USART1_RX) <- GPIO21 (D6, our TX)   [command path]
//
// HARDWARE NOTES (do not "simplify" these away):
//   * Do NOT call Wire.begin(): U8g2's HW-I2C constructor initialises the bus
//     itself and doing both deadlocks inside u8g2.begin().
//   * Opening the USB CDC port with DTR asserted can drop the chip into serial
//     download mode. Monitor with DTR and RTS both deasserted.

#include <Arduino.h>
#include <U8g2lib.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

// ---------------- pins ----------------
#define SDA_PIN     6
#define SCL_PIN     7
#define STM32_RX    20
#define STM32_TX    21
#define STM32_BAUD  115200

#define DEVICE_NAME "IR-Distance-Sensor"   // what shows up in a BLE scan

// ---------------- sensor spec ----------------
// Sharp GP2Y0A21-class analog IR: usable 100..800 mm, ~20 Hz.
#define DIST_MIN_MM   100.0f
#define DIST_MAX_MM   800.0f
#define ADC_VREF      3.3f       // STM32 analog reference
#define ADC_COUNTS    4095.0f    // 12-bit
#define SHARP_K       27.86f     // V = K * d_cm ^ -E
#define SHARP_E       1.15f

#define LINK_TIMEOUT_MS 1000     // no frame for this long => link is down

// ---------------- BLE (Nordic UART Service) ----------------
#define NUS_SERVICE "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // PC -> ESP32 (write)
#define NUS_TX      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // ESP32 -> PC (notify)

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);

static BLECharacteristic *txChar = nullptr;
static volatile bool bleConnected = false;

// ---------------- runtime state ----------------
static uint32_t g_rate_hz     = 20;      // rate the STM32 reports it is using
static float    g_meas_hz     = 0.0f;    // rate we actually observe
static uint32_t g_seq         = 0;       // sequence number from the STM32
static uint16_t g_adc         = 0;
static float    g_volts       = 0.0f;
static float    g_dist_mm     = 0.0f;
static bool     g_in_range    = false;
static bool     g_recording   = false;
static float    g_rec_ref_mm  = 0.0f;
static char     g_host[24]    = "";      // name of the connected computer
static uint32_t g_host_ms     = 0;
static uint32_t g_last_frame_ms = 0;
static uint32_t g_bad_frames  = 0;
static bool     g_link        = false;

// ---------------- helpers ----------------
static void bleSend(const char *s) {
  if (!bleConnected || txChar == nullptr) return;
  txChar->setValue((uint8_t *)s, strlen(s));
  txChar->notify();
}

static void bleSendf(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  bleSend(buf);
}

// Nominal (uncalibrated) distance for a measured voltage. Correcting this is
// what the ruler calibration runs are for.
static float voltsToMm(float v) {
  if (v <= 0.01f) return NAN;
  return powf(SHARP_K / v, 1.0f / SHARP_E) * 10.0f;
}

// ---------------- STM32 UART frames ----------------
// $IR,<seq>,<adc>,<rate_hz>*<xor checksum of the payload, 2 hex digits>
static void handleFrame(char *line) {
  char *star = strrchr(line, '*');
  if (line[0] != '$' || !star) { g_bad_frames++; return; }
  *star = '\0';
  char *payload = line + 1;

  uint8_t cs = 0;
  for (const char *p = payload; *p; ++p) cs ^= (uint8_t)*p;
  if (cs != (uint8_t)strtol(star + 1, nullptr, 16)) { g_bad_frames++; return; }

  if (!strncmp(payload, "IR,", 3)) {
    unsigned long seq = 0, rate = 0;
    unsigned adc = 0;
    if (sscanf(payload + 3, "%lu,%u,%lu", &seq, &adc, &rate) != 3) {
      g_bad_frames++;
      return;
    }
    g_seq  = seq;
    g_adc  = (uint16_t)adc;
    if (rate >= 1 && rate <= 200) g_rate_hz = rate;
    g_volts = g_adc * ADC_VREF / ADC_COUNTS;
    g_dist_mm = voltsToMm(g_volts);
    g_in_range = !isnan(g_dist_mm) && g_dist_mm >= DIST_MIN_MM && g_dist_mm <= DIST_MAX_MM;
    g_last_frame_ms = millis();

    bleSendf("D,%lu,%lu,%u,%.4f,%.1f\n", (unsigned long)g_seq,
             (unsigned long)g_last_frame_ms, g_adc, g_volts,
             g_in_range ? g_dist_mm : -1.0f);
  } else {
    Serial.printf("[stm32] %s\n", payload);   // $BOOT / $OK / $ERR
  }
}

static void pollSTM32() {
  static char line[96];
  static uint8_t len = 0;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (len) { line[len] = '\0'; handleFrame(line); len = 0; }
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      len = 0;                 // overlong garbage: resynchronise
      g_bad_frames++;
    }
  }
}

// ---------------- command handling ----------------
static void handleCommand(const char *cmd) {
  if (!strncasecmp(cmd, "RATE", 4)) {
    int hz = atoi(cmd + 4);
    if (hz >= 1 && hz <= 200) {
      Serial1.printf("RATE %d\n", hz);   // the STM32 owns the sample clock
      bleSendf("OK RATE %d (forwarded to STM32)\n", hz);
    } else {
      bleSend("ERR RATE range 1..200\n");
    }
  } else if (!strncasecmp(cmd, "REC", 3)) {
    const char *p = cmd + 3;
    g_recording = (atoi(p) != 0);
    const char *sp = strchr(p + 1, ' ');
    if (sp) g_rec_ref_mm = atof(sp + 1);
    bleSendf("OK REC %d\n", g_recording ? 1 : 0);
  } else if (!strncasecmp(cmd, "HOST", 4)) {
    // The central has no name we can read, so the PC tells us who it is.
    const char *p = cmd + 4;
    while (*p == ' ') p++;
    strncpy(g_host, p, sizeof(g_host) - 1);
    g_host[sizeof(g_host) - 1] = '\0';
    g_host_ms = millis();
    bleSendf("OK HOST %s\n", g_host);
  } else if (!strncasecmp(cmd, "PING", 4)) {
    bleSendf("OK PONG rate=%lu link=%d bad=%lu\n",
             (unsigned long)g_rate_hz, g_link ? 1 : 0,
             (unsigned long)g_bad_frames);
  } else if (!strncasecmp(cmd, "SIM", 3) || !strncasecmp(cmd, "SWEEP", 5)) {
    bleSend("ERR no simulator: reading a real sensor\n");
  } else {
    bleSend("ERR unknown\n");
  }
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.empty()) return;
    char buf[96];
    size_t n = v.size() < sizeof(buf) - 1 ? v.size() : sizeof(buf) - 1;
    memcpy(buf, v.data(), n);
    buf[n] = '\0';
    while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = '\0';
    handleCommand(buf);
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { bleConnected = true; }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    g_recording = false;
    g_host[0] = '\0';
    s->startAdvertising();          // allow the PC to reconnect
  }
};

// ---------------- display ----------------
// Right-align s at baseline y, trimming characters until it fits maxw px.
static void drawRightFit(const char *s, int y, int maxw) {
  char tmp[32];
  strncpy(tmp, s, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';
  while (u8g2.getStrWidth(tmp) > maxw && strlen(tmp) > 1) tmp[strlen(tmp) - 1] = '\0';
  u8g2.drawStr(128 - u8g2.getStrWidth(tmp), y, tmp);
}

static void drawScreen() {
  char buf[24];
  u8g2.clearBuffer();

  // briefly announce which computer just linked up
  if (g_host[0] && millis() - g_host_ms < 2500) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Linked to");
    u8g2.setFont(u8g2_font_7x13B_tf);
    char tmp[24];
    strncpy(tmp, g_host, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';
    while (u8g2.getStrWidth(tmp) > 128 && strlen(tmp) > 1) tmp[strlen(tmp) - 1] = '\0';
    u8g2.drawStr(0, 28, tmp);
    u8g2.sendBuffer();
    return;
  }

  // the sensor board going quiet must never look like a plausible reading
  if (!g_link) {
    u8g2.setFont(u8g2_font_7x13B_tf);
    u8g2.drawStr(0, 13, "NO SENSOR LINK");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 28, "waiting for STM32");
    const char *s = bleConnected ? "BLE" : "---";
    u8g2.drawStr(128 - u8g2.getStrWidth(s), 28, s);
    u8g2.sendBuffer();
    return;
  }

  // top-left: distance, large
  u8g2.setFont(u8g2_font_10x20_tf);
  if (g_in_range) snprintf(buf, sizeof(buf), "%.0f", g_dist_mm);
  else            snprintf(buf, sizeof(buf), "---");
  u8g2.drawStr(0, 16, buf);
  int w = u8g2.getStrWidth(buf);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(w + 3, 16, "mm");

  // top-right: achieved sample rate
  snprintf(buf, sizeof(buf), "%.1fHz", g_meas_hz);
  u8g2.drawStr(128 - u8g2.getStrWidth(buf), 9, buf);

  u8g2.drawHLine(0, 20, 128);

  // bottom-left: raw voltage
  snprintf(buf, sizeof(buf), "%.3f V", g_volts);
  u8g2.drawStr(0, 31, buf);

  // bottom-right: recording state, else who is connected
  if (g_recording) {
    char r[24];
    snprintf(r, sizeof(r), "REC %.0f", g_rec_ref_mm);
    if ((millis() / 400) % 2) u8g2.drawStr(128 - u8g2.getStrWidth(r), 31, r);
  } else if (g_host[0]) {
    drawRightFit(g_host, 31, 128 - u8g2.getStrWidth(buf) - 6);
  } else {
    const char *s = bleConnected ? "BLE" : "---";
    u8g2.drawStr(128 - u8g2.getStrWidth(s), 31, s);
  }

  u8g2.sendBuffer();
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n[boot] XIAO ESP32C3 IR display");

  Serial1.begin(STM32_BAUD, SERIAL_8N1, STM32_RX, STM32_TX);

  u8g2.setBusClock(100000);
  u8g2.begin();
  u8g2.setContrast(255);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(0, 13, "IR sensor");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 28, "starting BLE...");
  u8g2.sendBuffer();

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(247);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *svc = server->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  BLECharacteristic *rxChar = svc->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCallbacks());
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[boot] BLE advertising as '" DEVICE_NAME "'");
}

void loop() {
  static uint32_t win_start_ms = 0;
  static uint32_t win_seq0     = 0;
  static uint32_t last_draw_ms = 0;
  static uint32_t last_stat_ms = 0;

  pollSTM32();

  uint32_t now_ms = millis();
  g_link = (g_last_frame_ms != 0) && (now_ms - g_last_frame_ms < LINK_TIMEOUT_MS);

  // measure the real rate from the STM32's own sequence numbers, so dropped
  // frames show up as a rate below the commanded one
  if (now_ms - win_start_ms >= 1000) {
    if (win_start_ms && g_seq >= win_seq0)
      g_meas_hz = (g_seq - win_seq0) * 1000.0f / (now_ms - win_start_ms);
    if (!g_link) g_meas_hz = 0.0f;
    win_start_ms = now_ms;
    win_seq0     = g_seq;
  }

  if (now_ms - last_draw_ms >= 200) {       // 5 fps is plenty for a 128x32 panel
    last_draw_ms = now_ms;
    drawScreen();
  }

  if (now_ms - last_stat_ms >= 1000) {
    last_stat_ms = now_ms;
    bleSendf("S,%.2f,%.1f,%.4f,%lu,%d\n", g_meas_hz, g_dist_mm, g_volts,
             (unsigned long)g_rate_hz, g_link ? 1 : 0);
    Serial.printf("[%lus] link=%d %.1f Hz  %u adc  %.4f V  %.1f mm  bad=%lu ble=%d\n",
                  (unsigned long)(now_ms / 1000), g_link ? 1 : 0, g_meas_hz,
                  g_adc, g_volts, g_dist_mm, (unsigned long)g_bad_frames,
                  bleConnected ? 1 : 0);
  }
}
