#!/usr/bin/env python3
"""
Read every ADC-capable pin on the STM32F103 over SWD, without flashing anything.

Only peripheral registers are touched (clock enables, GPIO modes, ADC1). Nothing
is written to flash, so a power cycle or reset restores the board completely.
Use it to find which pin the Sharp IR sensor is actually wired to: sweep your
hand in front of the sensor and watch which channel moves.
"""
import sys, time
from pyocd.core.helpers import ConnectHelper

RCC_CFGR, RCC_APB2ENR = 0x40021004, 0x40021018
GPIOA_CRL, GPIOB_CRL = 0x40010800, 0x40010C00
ADC1 = 0x40012400
SR, CR1, CR2, SMPR1, SMPR2, SQR1, SQR3, DR = (ADC1 + o for o in
    (0x00, 0x04, 0x08, 0x0C, 0x10, 0x2C, 0x34, 0x4C))

CHANNELS = [(0, "PA0"), (1, "PA1"), (2, "PA2"), (3, "PA3"), (4, "PA4"),
            (5, "PA5"), (6, "PA6"), (7, "PA7"), (8, "PB0"), (9, "PB1")]


def main():
    session = ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "resume_on_disconnect": True})
    with session:
        t = session.target
        t.halt()

        # ADC clock must be <=14 MHz; PCLK2 is 72 MHz so use /6.
        cfgr = t.read32(RCC_CFGR)
        t.write32(RCC_CFGR, (cfgr & ~(3 << 14)) | (2 << 14))
        # clock GPIOA, GPIOB and ADC1
        t.write32(RCC_APB2ENR, t.read32(RCC_APB2ENR) | (1 << 2) | (1 << 3) | (1 << 9))
        # analog input mode (0b0000) for PA0-PA7 and PB0/PB1
        t.write32(GPIOA_CRL, 0x00000000)
        t.write32(GPIOB_CRL, t.read32(GPIOB_CRL) & 0xFFFFFF00)

        # longest sample time (239.5 cyc) suits a high-impedance sensor output
        t.write32(SMPR2, 0o7777777777 & 0x3FFFFFFF)
        t.write32(SMPR1, 0x00FFFFFF)
        t.write32(SQR1, 0)
        t.write32(CR1, 0)

        t.write32(CR2, 1)                       # ADON: wake the ADC
        time.sleep(0.01)
        t.write32(CR2, t.read32(CR2) | (1 << 3))   # RSTCAL
        while t.read32(CR2) & (1 << 3): pass
        t.write32(CR2, t.read32(CR2) | (1 << 2))   # CAL
        while t.read32(CR2) & (1 << 2): pass

        def sample(ch):
            t.write32(SQR3, ch)
            # EXTSEL=SWSTART(0b111), EXTTRIG, ADON
            t.write32(CR2, (7 << 17) | (1 << 20) | 1)
            t.write32(CR2, t.read32(CR2) | (1 << 22))   # SWSTART
            for _ in range(2000):
                if t.read32(SR) & (1 << 1):
                    return t.read32(DR) & 0xFFF
            return None

        print("STM32F103 ADC sweep — 3.3 V reference, 12-bit\n")
        print("      " + "".join(f"{n:>9}" for _, n in CHANNELS))
        prev = {}
        moved = set()
        for row in range(12):
            vals = []
            for ch, name in CHANNELS:
                raw = sample(ch)
                v = None if raw is None else raw * 3.3 / 4095
                vals.append(v)
                if name in prev and v is not None and abs(v - prev[name]) > 0.08:
                    moved.add(name)
                prev[name] = v
            print(f"{row:>4}  " + "".join(
                f"{'  err  ':>9}" if v is None else f"{v:>8.3f}V" for v in vals))
            sys.stdout.flush()
            time.sleep(0.7)

        print("\nchannels that moved >80 mV during the sweep:",
              ", ".join(sorted(moved)) if moved else "none")
        t.reset()          # hand the board back exactly as we found it
        print("target reset")


if __name__ == "__main__":
    main()
