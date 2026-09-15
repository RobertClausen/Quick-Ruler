// STM32F103 -> ESP32 IR distance sensor front end.
//
// Reads a Sharp analog IR distance sensor on PA0 (ADC1_IN0), oversamples it,
// and emits one checksummed ASCII frame per sample on USART1 (PA9 = TX).
//
// Frame format:   $IR,<seq>,<adc>,<rate_hz>*<xor checksum, 2 hex digits>\n
//
// Raw ADC counts are sent rather than millimetres on purpose: the distance
// curve and its calibration live on the ESP32/PC side, so recalibrating never
// means reflashing this board.

#include <Arduino.h>

#define SENSOR_PIN   PA0     // ADC1_IN0, confirmed wired to the Sharp output
#define LED_PIN      PC13    // on-board LED, active low
#define ADC_BITS     12
#define OVERSAMPLE   16      // averaged per reported sample
#define UART_BAUD    115200

// USART1: RX = PA10, TX = PA9. PA9 is the wire that reaches the ESP32.
HardwareSerial SerialIR(PA10, PA9);

static uint32_t g_seq     = 0;
static uint32_t g_rate_hz = 20;      // the sensor is realistically ~20 Hz
static char     g_line[64];
static uint8_t  g_len = 0;

static uint16_t readSensor() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < OVERSAMPLE; i++) acc += analogRead(SENSOR_PIN);
  return (uint16_t)(acc / OVERSAMPLE);
}

static void sendSample(uint16_t adc) {
  char payload[48];
  snprintf(payload, sizeof(payload), "IR,%lu,%u,%lu",
           (unsigned long)g_seq, adc, (unsigned long)g_rate_hz);
  uint8_t cs = 0;
  for (const char *p = payload; *p; ++p) cs ^= (uint8_t)*p;
  char frame[64];
  snprintf(frame, sizeof(frame), "$%s*%02X\n", payload, cs);
  SerialIR.print(frame);
}

// Accepts "RATE <hz>" on PA10 so the PC can retune the rate end to end.
// Harmless if that wire is absent - nothing ever arrives.
static void handleLine(char *s) {
  if (!strncasecmp(s, "RATE", 4)) {
    long hz = atol(s + 4);
    if (hz >= 1 && hz <= 200) {
      g_rate_hz = (uint32_t)hz;
      SerialIR.print("$OK,RATE\n");
    } else {
      SerialIR.print("$ERR,RATE\n");
    }
  } else if (!strncasecmp(s, "PING", 4)) {
    SerialIR.print("$OK,PONG\n");
  }
}

static void pollCommands() {
  while (SerialIR.available()) {
    char c = (char)SerialIR.read();
    if (c == '\n' || c == '\r') {
      if (g_len) { g_line[g_len] = '\0'; handleLine(g_line); g_len = 0; }
    } else if (g_len < sizeof(g_line) - 1) {
      g_line[g_len++] = c;
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);        // active low: start off
  pinMode(SENSOR_PIN, INPUT_ANALOG);
  analogReadResolution(ADC_BITS);
  SerialIR.begin(UART_BAUD);
  delay(50);
  SerialIR.print("$BOOT,stm32f103,ir\n");
}

void loop() {
  static uint32_t next_us = 0;
  uint32_t period_us = 1000000UL / g_rate_hz;
  uint32_t now = micros();

  if ((int32_t)(now - next_us) >= 0) {
    if ((int32_t)(now - next_us) > (int32_t)(period_us * 4)) next_us = now;
    next_us += period_us;

    uint16_t adc = readSensor();
    g_seq++;
    sendSample(adc);
    digitalWrite(LED_PIN, (g_seq % 10 < 1) ? LOW : HIGH);   // brief blink
  }

  pollCommands();
}
