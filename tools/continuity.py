#!/usr/bin/env python3
"""
Continuity test for the STM32 -> ESP32 UART wire.

Drives a candidate STM32 TX pin up and down over SWD while the ESP32 runs its
passive pin-observation firmware. Whichever ESP32 GPIO reports transitions is
the pin the wire actually lands on. Nothing is flashed on either side.
"""
import threading, time, sys
import serial
from pyocd.core.helpers import ConnectHelper

RCC_APB2ENR = 0x40021018
GPIOA_CRL, GPIOA_CRH, GPIOA_BSRR = 0x40010800, 0x40010804, 0x40010810

# (name, port bit) - PA13/PA14 are SWD, PA0 is the sensor: never drive those.
CANDIDATES = [("PA9 (USART1_TX)", 9), ("PA2 (USART2_TX)", 2)]

stop = threading.Event()
lines = []


def reader():
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = "/dev/ttyACM0", 115200, 1
    s.dtr = False; s.rts = False
    s.open()
    while not stop.is_set():
        l = s.readline()
        if l:
            txt = l.decode("utf-8", "replace").rstrip()
            lines.append((time.time(), txt))
            print("   ESP32| " + txt); sys.stdout.flush()
    s.close()


def set_output(t, bit):
    """Configure PAx as push-pull output, 50 MHz (CNF=00, MODE=11 -> 0x3)."""
    reg = GPIOA_CRL if bit < 8 else GPIOA_CRH
    shift = (bit % 8) * 4
    v = t.read32(reg)
    t.write32(reg, (v & ~(0xF << shift)) | (0x3 << shift))


def main():
    th = threading.Thread(target=reader, daemon=True); th.start()
    session = ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "resume_on_disconnect": True})
    with session:
        t = session.target
        t.halt()
        t.write32(RCC_APB2ENR, t.read32(RCC_APB2ENR) | (1 << 2))
        for name, bit in CANDIDATES:
            print(f"\n=== toggling {name} for 22 s "
                  f"- watch for an ESP32 pin with non-zero transitions ===")
            set_output(t, bit)
            end = time.time() + 22
            while time.time() < end:
                t.write32(GPIOA_BSRR, 1 << bit)          # set
                t.write32(GPIOA_BSRR, 1 << (bit + 16))   # reset
            print(f"=== done toggling {name} ===")
        t.reset()
    stop.set(); time.sleep(1.2)


if __name__ == "__main__":
    main()
