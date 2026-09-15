<img width="770" height="770" alt="7d75a078-2ea6-45a3-ad7b-7f2f3f713c77" src="https://github.com/user-attachments/assets/845d6a19-65df-4265-b097-d1be493c4dc9" />

# Quick Ruler
By using the IR sensor, we can measure the distance between the sensor and the object with the range of 10cm to 80cm. That's more than two ruler worth of measurable distance!

This project is made thanks to the my special topic class where to try our best to calibrate a cheap sensor the achieve a fine measurement. In this case, we chosen the IR distance sensor from Sharp that is widely used as a counterpart from the ultrasonic sensor.

<img width="1920" height="770" alt="6ab4340e-c0da-4cd3-a43c-5337176d39e8" src="https://github.com/user-attachments/assets/cfbe9c3a-be00-4992-a514-d6f1c8511aa4" />

# BOM
- STM32 Blue pill
- Seeed Studio ESP32-C3
- 128x32 OLED display
- Sharp GP2Y0A21YKOF IR distance sensor
- M1x8 screws

This project requires soldiering, an open room with good ventilation is advised.

# Code

| Path | What it is |
|---|---|
| `firmware/stm32-bluepill/` | Blue Pill: samples the Sharp sensor, streams frames over UART |
| `firmware/esp32-xiao-c3/` | XIAO ESP32-C3: OLED readout + BLE bridge to the PC |
| `pc/` | Calibration bench — live graphs, ruler-referenced capture, CSV export |
| `tools/` | Bring-up helpers used to find the wiring (see `docs/HARDWARE.md`) |
| `docs/HARDWARE.md` | Verified pinout, the link protocol, and the traps worth knowing |

## Wiring

| From | To |
|---|---|
| Sharp sensor output | STM32 **PA0** (ADC1_IN0) |
| STM32 **PA9** (USART1_TX) | ESP32 **GPIO20** (D7) |
| STM32 **PA10** (USART1_RX) | ESP32 **GPIO21** (D6) |
| OLED SDA / SCL | ESP32 **GPIO6** / **GPIO7**, address `0x3C` |

The Sharp sensor needs 5 V; the STM32 and ESP32 are 3.3 V parts, but the
sensor's analog output stays inside the ADC range so no divider is required.

## Build and flash

Both boards are [PlatformIO](https://platformio.org/) projects — it fetches the
ARM and Xtensa toolchains itself, so there is nothing to install by hand.

```bash
cd firmware/stm32-bluepill && pio run -t upload      # over SWD via ST-LINK
```

```bash
cd firmware/esp32-xiao-c3 && pio run -t upload --upload-port /dev/ttyACM0
```

## Calibration bench

```bash
pip install -r pc/requirements.txt
python3 pc/calib_server.py     # then open http://127.0.0.1:8765
```

Scan for the sensor, click it to connect, then **Record sample…** asks for a
sample rate and the real distance you measured with a ruler. Each run is written
to `pc/data/run_<distance>mm_<timestamp>.csv` with every raw sample, and
appended to `pc/data/calibration_summary.csv` with mean/σ voltage and the error
against your ruler. Captured runs are gitignored — they belong to your bench,
not the repo.

The STM32 sends **raw ADC counts, never millimetres**: the distance curve lives
on the ESP32/PC side, so recalibrating never means reflashing the sensor board.
