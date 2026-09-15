# Hardware notes

Wiring, bring-up traps and the link protocol for Quick Ruler.
Build and run instructions live in the top-level README.

An STM32 Blue Pill reads the Sharp GP2Y0A21YK0F and streams samples to a
XIAO ESP32-C3 over UART. The ESP32 shows sample rate, distance and raw sensor
voltage on a 128x32 SSD1306, and relays every sample to a PC over BLE, where a
local web UI graphs them live and records fixed-distance runs against a ruler.

## Hardware

| | |
|---|---|
| MCU | Seeed XIAO ESP32C3, 4 MB flash, native USB-Serial/JTAG, `/dev/ttyACM0` |
| OLED | SSD1306 128x32, I2C `0x3C`, **SDA = GPIO6, SCL = GPIO7** |
| Sensor MCU | STM32F103 (DEV_ID `0x410`, 64 KB) via ST-LINK/V2.1, `/dev/ttyACM1` |
| Sensor | Sharp analog IR, 100–800 mm, ~20 Hz, on **PA0 / ADC1_IN0** |
| Sensor link | **STM32 PA9 (USART1_TX) → ESP32 GPIO20 (D7)** @ 115200, verified |
| Command path | ESP32 GPIO21 (D6) → STM32 PA10 (USART1_RX) |
| BLE MAC | advertises as `IR-Distance-Sensor` (the unit used here is `94:A9:90:67:04:FA`) |

The XIAO's I2C pins differ from the older SuperMini bench rig (which used
GPIO8/9). Firmware written for that board will not drive this display until
`SDA_PIN` / `SCL_PIN` are changed.

## Two boards, two projects

```bash
cd firmware/esp32-xiao-c3  && pio run -t upload --upload-port /dev/ttyACM0
cd firmware/stm32-bluepill && pio run -t upload      # over SWD via ST-LINK
```

The STM32 samples PA0 (16x oversampled), and emits one checksummed frame per
sample on USART1:

```
$IR,<seq>,<adc>,<rate_hz>*<xor checksum of payload, 2 hex digits>
```

It accepts `RATE <hz>` and `PING` on PA10. Raw ADC counts are sent rather than
millimetres on purpose — the distance curve and its calibration live on the
ESP32/PC side, so recalibrating never means reflashing the sensor board.

The ESP32 validates the checksum, converts counts to volts (3.3 V ref, 12-bit)
and to millimetres, and shows `NO SENSOR LINK` on the OLED if no valid frame
arrives for a second, so a dead sensor board can never look like a plausible
reading.

**The ESP32-C3 has Bluetooth 5 LE only — no Bluetooth Classic, so no SPP
virtual COM port.** The PC link is BLE using the Nordic UART Service; that is
why a small Python client is required instead of a plain serial terminal.

## Firmware

```bash
cd firmware/esp32-xiao-c3 && pio run -t upload --upload-port /dev/ttyACM0
```

`huge_app.csv` is required: BLE + U8g2 comes to ~1.03 MB and overflows the
1.3 MB default app partition once anything else is added.

### Three traps worth knowing

1. **GPIO9 is the BOOT strapping pin and also our SCL.** On the C3's USB-JTAG
   bridge the host's DTR line drives it, so opening `/dev/ttyACM0` with DTR
   asserted drops the chip into serial download mode (`waiting for download`)
   and your firmware never runs. Always monitor with DTR **and** RTS low:

   ```bash
   python3 -c "import serial,sys;s=serial.Serial();s.port='/dev/ttyACM0';s.baudrate=115200;s.dtr=False;s.rts=False;s.open();[sys.stdout.write(l.decode('utf-8','replace')) for l in iter(s.readline,b'')]"
   ```

   If it is already stuck, get it out with
   `esptool.py --port /dev/ttyACM0 --after hard-reset flash-id`.

2. **Never call `Wire.begin()` alongside U8g2's HW-I2C constructor.** U8g2
   initialises the bus itself; doing both deadlocks inside `u8g2.begin()`.

3. **`Serial.setTxTimeoutMs(0)` silently discards output** when the host has
   not asserted DTR — indistinguishable from a crash. Leave it at the default.

### BLE protocol (Nordic UART Service)

Notifications, newline-terminated (a notification may split mid-line — the
client reassembles on `\n`):

```
D,<seq>,<device_ms>,<adc>,<volts>,<mm>     per sample; mm = -1 when out of range
S,<meas_hz>,<mm>,<volts>,<rate_hz>         once per second
```

Commands written to the RX characteristic:

| Command | Effect |
|---|---|
| `RATE <hz>` | set sample rate, 1–200 |
| `SIM <mm>` | hold the stand-in sensor at a distance |
| `SWEEP` | stand-in sensor sweeps the range |
| `REC <0\|1> [mm]` | drive the on-screen recording indicator |
| `HOST <name>` | tell the device which computer is connected |
| `RATE <hz>` | forwarded on to the STM32, which owns the sample clock |
| `PING` | liveness check |

The simulator is gone — both boards now run against the real sensor.

## Calibration bench

```bash
python3 pc/calib_server.py      # then open http://127.0.0.1:8765
```

Needs `bleak` and `aiohttp`. Scan for the device and click it to connect —
no addresses are hardcoded. The device advertises as `IR-Distance-Sensor`;
the scan list hides unnamed devices by default (a BLE scan is mostly anonymous
phones and laptops), with a checkbox to show them all.

On connect the server sends `HOST <hostname>`. A BLE peripheral cannot read the
central's name, so this is how the OLED knows which machine is driving it — it
shows a `Linked to <name>` banner for 2.5 s, then keeps the name in the
bottom-right corner, truncated to the space left by the voltage readout.
The device name itself is `DEVICE_NAME` at the top of `src/main.cpp`.

There are two capture modes. **Start sweep…** walks the whole range in steps and
puts every sample into a single CSV — see the README. **Record sample…** is the
single-distance spot check: it asks for the sample rate and the real ruler
distance (plus a duration, shown live as a sample count, and an optional note),
commands the device to that rate, captures the run, and writes:

- `pc/data/run_<dist>mm_<timestamp>.csv` — every sample:
  `seq, device_ms, host_time, adc, volts, reported_mm, reference_mm, in_range`
- `pc/data/calibration_summary.csv` — one row per run with mean/sd/min/max
  voltage, mean reported distance, and the error against the ruler.

The calibration chart plots each run's mean voltage against its ruler
reference with ±1σ bars, over the nominal curve the firmware assumes.

### Reading the results

Precision degrades sharply with distance, which is inherent to this sensor
class rather than a bug — from a bench run at 20 Hz:

| Reference | V mean | reported sd |
|---|---|---|
| 100 mm | 1.9729 | 0.2 mm |
| 400 mm | 0.4003 | 6.2 mm |
| 700 mm | 0.2100 | 25.2 mm |

The response is nearly flat past ~600 mm, so a few mV of noise maps to
centimetres of error. Expect to average hard at the far end, and treat the
last 100 mm of range as indicative only. Runs at the extremes also report
fewer in-range samples, because noise pushes the inverse curve past the
100/800 mm limits.


## Debugging the STM32 without a toolchain

The ST-LINK USB node is world-writable, so `pyocd` reaches the target over SWD
with no root. Useful because it inspects a **running** target:

```bash
pyocd cmd --connect attach -c "read32 0x40021018"   # RCC_APB2ENR
```

`tools/adc_probe.py` sweeps every ADC-capable pin (PA0-PA7, PB0, PB1) and prints
volts, which is how the sensor was located on PA0 — a floating STM32 analog pin
settles near mid-rail (~1.9 V), so a pin sitting anywhere else has a real source.
`tools/continuity.py` drives a chosen STM32 pin and reports which ESP32 GPIO
follows it, which is how the PA9 → GPIO20 wire was confirmed.
`tools/discovery/` is a passive pin-observation sketch for the ESP32: it never
drives anything, so it cannot fight an STM32 output.
