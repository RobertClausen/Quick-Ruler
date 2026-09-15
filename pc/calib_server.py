#!/usr/bin/env python3
"""
Calibration bench for the ESP32-C3 IR distance sensor.

Bridges the ESP32's BLE Nordic-UART stream to a local web UI that graphs the
readings live and records fixed-distance runs against a ruler reference.
Each run is written to its own CSV, and every run is also appended to a
rolling calibration summary.

    python3 calib_server.py          # then open http://127.0.0.1:8765
"""

import asyncio
import csv
import datetime as dt
import json
import logging
import math
import pathlib
import socket
import statistics
import sys

from aiohttp import web, WSMsgType
from bleak import BleakClient, BleakScanner

NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # PC -> ESP32
NUS_TX      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # ESP32 -> PC

HERE     = pathlib.Path(__file__).resolve().parent
DATA_DIR = HERE / "data"
SUMMARY  = DATA_DIR / "calibration_summary.csv"
HOST, PORT = "127.0.0.1", 8765

logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("calib")


class Recording:
    """One fixed-distance capture run."""

    def __init__(self, ref_mm: float, rate_hz: int, duration_s: float, note: str = ""):
        self.ref_mm = ref_mm
        self.rate_hz = rate_hz
        self.duration_s = duration_s
        self.note = note
        self.target_n = max(1, int(round(rate_hz * duration_s)))
        self.rows: list[dict] = []
        self.started = dt.datetime.now()

    def add(self, s: dict) -> None:
        self.rows.append({
            "seq": s["seq"],
            "device_ms": s["ms"],
            "host_time": dt.datetime.now().isoformat(timespec="milliseconds"),
            "adc": s["adc"],
            "volts": s["v"],
            "reported_mm": s["mm"] if s["mm"] is not None else "",
            "reference_mm": self.ref_mm,
            "in_range": int(s["mm"] is not None),
        })

    @property
    def done(self) -> bool:
        return len(self.rows) >= self.target_n

    def stats(self) -> dict:
        volts = [r["volts"] for r in self.rows]
        mms = [r["reported_mm"] for r in self.rows if r["reported_mm"] != ""]

        def agg(xs):
            if not xs:
                return dict(n=0, mean=None, sd=None, min=None, max=None)
            return dict(n=len(xs), mean=statistics.fmean(xs),
                        sd=statistics.stdev(xs) if len(xs) > 1 else 0.0,
                        min=min(xs), max=max(xs))

        v, m = agg(volts), agg(mms)
        err = (m["mean"] - self.ref_mm) if m["mean"] is not None else None
        return {"volts": v, "mm": m, "error_mm": err,
                "n_total": len(self.rows), "n_in_range": m["n"]}

    def write_csv(self) -> pathlib.Path:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        stamp = self.started.strftime("%Y%m%d_%H%M%S")
        path = DATA_DIR / f"run_{int(round(self.ref_mm)):04d}mm_{stamp}.csv"
        fields = ["seq", "device_ms", "host_time", "adc", "volts",
                  "reported_mm", "reference_mm", "in_range"]
        with path.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            w.writerows(self.rows)
        return path

    def append_summary(self, run_csv: pathlib.Path) -> None:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        st = self.stats()
        fields = ["timestamp", "reference_mm", "rate_hz", "duration_s",
                  "n_samples", "n_in_range", "volts_mean", "volts_sd",
                  "volts_min", "volts_max", "reported_mm_mean", "reported_mm_sd",
                  "error_mm", "note", "run_file"]
        new = not SUMMARY.exists()
        with SUMMARY.open("a", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            if new:
                w.writeheader()

            def r(x, n=4):
                return "" if x is None else round(x, n)

            w.writerow({
                "timestamp": self.started.isoformat(timespec="seconds"),
                "reference_mm": self.ref_mm, "rate_hz": self.rate_hz,
                "duration_s": self.duration_s, "n_samples": st["n_total"],
                "n_in_range": st["n_in_range"],
                "volts_mean": r(st["volts"]["mean"]), "volts_sd": r(st["volts"]["sd"]),
                "volts_min": r(st["volts"]["min"]), "volts_max": r(st["volts"]["max"]),
                "reported_mm_mean": r(st["mm"]["mean"], 2),
                "reported_mm_sd": r(st["mm"]["sd"], 2),
                "error_mm": r(st["error_mm"], 2), "note": self.note,
                "run_file": run_csv.name,
            })


class Bench:
    def __init__(self):
        self.client: BleakClient | None = None
        self.address: str | None = None
        self.name: str | None = None
        self.rate_hz = 20
        self.meas_hz = 0.0
        self.link = False           # STM32 -> ESP32 UART link alive?
        self.rec: Recording | None = None
        self.clients: set[web.WebSocketResponse] = set()
        self._buf = ""
        self._loop: asyncio.AbstractEventLoop | None = None

    # ---------- websocket plumbing ----------
    async def broadcast(self, msg: dict) -> None:
        if not self.clients:
            return
        payload = json.dumps(msg)
        for ws in list(self.clients):
            try:
                await ws.send_str(payload)
            except Exception:
                self.clients.discard(ws)

    def status(self) -> dict:
        return {"type": "status",
                "connected": bool(self.client and self.client.is_connected),
                "address": self.address, "name": self.name,
                "rate_hz": self.rate_hz, "meas_hz": self.meas_hz,
                "link": self.link,
                "recording": bool(self.rec),
                "rec_ref_mm": self.rec.ref_mm if self.rec else None,
                "rec_have": len(self.rec.rows) if self.rec else 0,
                "rec_target": self.rec.target_n if self.rec else 0}

    async def push_status(self) -> None:
        await self.broadcast(self.status())

    async def log(self, msg: str, level: str = "info") -> None:
        log.info(msg)
        await self.broadcast({"type": "log", "msg": msg, "level": level})

    # ---------- BLE ----------
    async def scan(self, seconds: float = 6.0) -> None:
        await self.log(f"scanning {seconds:.0f}s for BLE devices...")
        found = await BleakScanner.discover(timeout=seconds, return_adv=True)
        devices = []
        for addr, (d, adv) in found.items():
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            name = adv.local_name or d.name or ""
            is_nus = NUS_SERVICE in uuids or "ir-dist" in name.lower()
            devices.append({"address": addr, "name": name or "(unnamed)",
                            "rssi": adv.rssi, "is_sensor": is_nus})
        # most likely candidates first, then by signal strength
        devices.sort(key=lambda x: (not x["is_sensor"], -(x["rssi"] or -999)))
        await self.broadcast({"type": "scan_result", "devices": devices})
        n = sum(1 for d in devices if d["is_sensor"])
        await self.log(f"scan done: {len(devices)} device(s), {n} matching sensor")

    def _on_notify(self, _handle, data: bytearray) -> None:
        # BLE notifications split arbitrarily; reassemble on newlines.
        self._buf += bytes(data).decode("utf-8", "replace")
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if line:
                asyncio.create_task(self._handle_line(line))

    async def _handle_line(self, line: str) -> None:
        if line.startswith("D,"):
            p = line.split(",")
            if len(p) != 6:
                return
            try:
                mm = float(p[5])
                sample = {"seq": int(p[1]), "ms": int(p[2]), "adc": int(p[3]),
                          "v": float(p[4]), "mm": None if mm < 0 else mm}
            except ValueError:
                return
            await self.broadcast({"type": "sample", **sample})
            if self.rec is not None:
                self.rec.add(sample)
                if len(self.rec.rows) % 5 == 0 or self.rec.done:
                    await self.push_status()
                if self.rec.done:
                    await self.finish_record()
        elif line.startswith("S,"):
            p = line.split(",")
            if len(p) >= 5:
                try:
                    self.meas_hz = float(p[1])
                    self.rate_hz = int(p[4])
                    if len(p) >= 6:
                        self.link = p[5].strip() == "1"
                except ValueError:
                    pass
                await self.push_status()
        else:
            await self.log(f"device: {line}")

    async def connect(self, address: str, name: str = "") -> None:
        await self.disconnect()
        await self.log(f"connecting to {address} ...")
        client = BleakClient(address, disconnected_callback=self._on_disconnect)
        await client.connect()
        self.client, self.address, self.name = client, address, name
        await client.start_notify(NUS_TX, self._on_notify)
        await self.send_cmd(f"RATE {self.rate_hz}")
        # A BLE peripheral cannot read the central's name, so tell it who we are;
        # the OLED shows this so you can see which machine grabbed the sensor.
        await self.send_cmd(f"HOST {socket.gethostname()[:20]}")
        await self.log(f"connected to {name or address}")
        await self.push_status()

    def _on_disconnect(self, _client) -> None:
        loop = self._loop
        if loop:
            asyncio.run_coroutine_threadsafe(self._after_disconnect(), loop)

    async def _after_disconnect(self) -> None:
        await self.log("device disconnected", "warn")
        self.client = None
        self.link = False
        if self.rec is not None:
            await self.log("recording aborted by disconnect", "warn")
            self.rec = None
        await self.push_status()

    async def disconnect(self) -> None:
        if self.client and self.client.is_connected:
            try:
                await self.client.disconnect()
            except Exception:
                pass
        self.client = None
        await self.push_status()

    async def send_cmd(self, cmd: str) -> None:
        if not (self.client and self.client.is_connected):
            await self.log(f"not connected, dropped '{cmd}'", "warn")
            return
        await self.client.write_gatt_char(NUS_RX, (cmd + "\n").encode(), response=False)

    # ---------- recording ----------
    async def start_record(self, ref_mm: float, rate_hz: int, duration_s: float,
                           note: str) -> None:
        if self.rec is not None:
            await self.log("already recording", "warn")
            return
        if not (self.client and self.client.is_connected):
            await self.log("connect to the sensor first", "error")
            return

        if not self.link:
            await self.log("no STM32 sensor link - refusing to record", "error")
            return
        await self.send_cmd(f"RATE {int(rate_hz)}")
        await asyncio.sleep(0.6)          # let the rate change settle

        self.rec = Recording(ref_mm, int(rate_hz), duration_s, note)
        self.rate_hz = int(rate_hz)
        await self.send_cmd(f"REC 1 {ref_mm:.0f}")
        await self.log(f"recording {self.rec.target_n} samples @ {rate_hz} Hz "
                       f"at reference {ref_mm:.1f} mm")
        await self.push_status()

    async def finish_record(self, aborted: bool = False) -> None:
        rec, self.rec = self.rec, None
        if rec is None:
            return
        await self.send_cmd("REC 0")
        if not rec.rows:
            await self.log("no samples captured", "warn")
            await self.push_status()
            return
        path = rec.write_csv()
        rec.append_summary(path)
        st = rec.stats()
        await self.log(
            f"saved {len(rec.rows)} samples -> {path.name} | "
            f"V={st['volts']['mean']:.4f}±{st['volts']['sd']:.4f} | "
            f"error={st['error_mm']:+.1f} mm" if st["error_mm"] is not None
            else f"saved {len(rec.rows)} samples -> {path.name}")
        await self.broadcast({"type": "record_done", "file": path.name,
                              "ref_mm": rec.ref_mm, "rate_hz": rec.rate_hz,
                              "aborted": aborted, "stats": st})
        await self.push_status()
        await self.send_runs()

    async def send_runs(self) -> None:
        runs = []
        if SUMMARY.exists():
            with SUMMARY.open(newline="") as fh:
                runs = list(csv.DictReader(fh))
        await self.broadcast({"type": "runs", "runs": runs})


bench = Bench()


async def ws_handler(request: web.Request) -> web.WebSocketResponse:
    # No heartbeat: some embedded browser views don't answer WS pings,
    # and aiohttp then tears down a perfectly healthy connection.
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    bench.clients.add(ws)
    await ws.send_str(json.dumps(bench.status()))
    await bench.send_runs()
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                m = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            cmd = m.get("cmd")
            try:
                if cmd == "scan":
                    await bench.scan(float(m.get("seconds", 6)))
                elif cmd == "connect":
                    await bench.connect(m["address"], m.get("name", ""))
                elif cmd == "disconnect":
                    await bench.disconnect()
                elif cmd == "start_record":
                    await bench.start_record(float(m["ref_mm"]), int(m["rate_hz"]),
                                             float(m["duration_s"]),
                                             m.get("note", ""))
                elif cmd == "stop_record":
                    await bench.finish_record(aborted=True)
                elif cmd == "rate":
                    await bench.send_cmd(f"RATE {int(m['hz'])}")
            except Exception as exc:                       # keep the UI alive
                await bench.log(f"{cmd} failed: {exc}", "error")
    finally:
        bench.clients.discard(ws)
    return ws


async def index(_request):
    return web.FileResponse(HERE / "static" / "index.html")


async def on_start(app):
    bench._loop = asyncio.get_running_loop()
    log.info(f"UI at http://{HOST}:{PORT}   CSVs in {DATA_DIR}")


def main():
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/data/", DATA_DIR, show_index=True)
    app.router.add_static("/static/", HERE / "static")
    app.on_startup.append(on_start)
    web.run_app(app, host=HOST, port=PORT, print=None)


if __name__ == "__main__":
    main()
