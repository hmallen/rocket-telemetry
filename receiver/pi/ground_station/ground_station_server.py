import copy
import json
import math
import mimetypes
import queue
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))

try:
    from lora_receive_pi import LED, parse_payload, safe_ascii, setup_radio
except ImportError as exc:  # pylint: disable=import-error
    raise SystemExit(
        "Failed to import LoRa receiver module. Run from receiver/pi on the Raspberry Pi."
    ) from exc

HOST = "0.0.0.0"
PORT = 8000

STATIC_DIR = Path(__file__).resolve().parent / "static"
TILE_CACHE_DIR = STATIC_DIR / "tiles"
TILE_SOURCE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
TILE_USER_AGENT = "RocketTelemetryGroundStation/1.0"
TILE_REQUEST_TIMEOUT = 10
MAX_TILE_REQUEST = 50000

mimetypes.add_type("text/css", ".css")
mimetypes.add_type("text/javascript", ".js")
mimetypes.add_type("image/png", ".png")

BAT_STATE_LABELS = {
    0: "OK",
    1: "WARN",
    2: "SHED",
    3: "CUTOFF",
}


class TelemetryStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = self._blank_state()

    def _blank_state(self):
        return {
            "timestamp": None,
            "packet_count": 0,
            "callsign": None,
            "radio": {
                "rssi_dbm": None,
                "snr_db": None,
                "crc_ok": None,
                "payload_len": None,
                "raw_hex": None,
                "raw_ascii": None,
            },
            "gps": {
                "t_ms": None,
                "lat_e7": None,
                "lon_e7": None,
                "height_mm": None,
                "lat_deg": None,
                "lon_deg": None,
                "alt_m": None,
            },
            "alt": {
                "t_ms": None,
                "press_pa_x10": None,
                "temp_c_x100": None,
                "press_pa": None,
                "press_kpa": None,
                "temp_c": None,
            },
            "imu": {
                "t_ms": None,
                "gx": None,
                "gy": None,
                "gz": None,
                "ax": None,
                "ay": None,
                "az": None,
                "gx_dps": None,
                "gy_dps": None,
                "gz_dps": None,
                "ax_g": None,
                "ay_g": None,
                "az_g": None,
            },
            "battery": {
                "t_ms": None,
                "vbat_mv": None,
                "vbat_v": None,
                "bat_state": None,
                "bat_state_label": None,
            },
        }

    def update(self, parsed, radio_info, raw_info):
        with self._lock:
            self._state["timestamp"] = time.time()
            self._state["packet_count"] += 1

            radio_state = self._state["radio"]
            radio_state.update(radio_info)
            if raw_info is not None:
                radio_state["raw_hex"] = raw_info.get("hex")
                radio_state["raw_ascii"] = raw_info.get("ascii")

            if parsed:
                payload_type = parsed.get("type")
                if payload_type == "id":
                    self._state["callsign"] = parsed.get("callsign")
                elif payload_type == "gps":
                    lat_e7 = parsed.get("lat_e7")
                    lon_e7 = parsed.get("lon_e7")
                    height_mm = parsed.get("height_mm")
                    lat_deg = lat_e7 / 1e7 if lat_e7 is not None else None
                    lon_deg = lon_e7 / 1e7 if lon_e7 is not None else None
                    alt_m = height_mm / 1000.0 if height_mm is not None else None
                    self._state["gps"].update({
                        "t_ms": parsed.get("t_ms"),
                        "lat_e7": lat_e7,
                        "lon_e7": lon_e7,
                        "height_mm": height_mm,
                        "lat_deg": lat_deg,
                        "lon_deg": lon_deg,
                        "alt_m": alt_m,
                    })
                elif payload_type == "alt":
                    press_pa_x10 = parsed.get("press_pa_x10")
                    temp_c_x100 = parsed.get("temp_c_x100")
                    press_pa = press_pa_x10 / 10.0 if press_pa_x10 is not None else None
                    press_kpa = press_pa / 1000.0 if press_pa is not None else None
                    temp_c = temp_c_x100 / 100.0 if temp_c_x100 is not None else None
                    self._state["alt"].update({
                        "t_ms": parsed.get("t_ms"),
                        "press_pa_x10": press_pa_x10,
                        "temp_c_x100": temp_c_x100,
                        "press_pa": press_pa,
                        "press_kpa": press_kpa,
                        "temp_c": temp_c,
                    })
                elif payload_type == "imu":
                    gx = parsed.get("gx")
                    gy = parsed.get("gy")
                    gz = parsed.get("gz")
                    ax = parsed.get("ax")
                    ay = parsed.get("ay")
                    az = parsed.get("az")
                    self._state["imu"].update({
                        "t_ms": parsed.get("t_ms"),
                        "gx": gx,
                        "gy": gy,
                        "gz": gz,
                        "ax": ax,
                        "ay": ay,
                        "az": az,
                        "gx_dps": gx / 10.0 if gx is not None else None,
                        "gy_dps": gy / 10.0 if gy is not None else None,
                        "gz_dps": gz / 10.0 if gz is not None else None,
                        "ax_g": ax / 1000.0 if ax is not None else None,
                        "ay_g": ay / 1000.0 if ay is not None else None,
                        "az_g": az / 1000.0 if az is not None else None,
                    })
                elif payload_type == "bat":
                    vbat_mv = parsed.get("vbat_mv")
                    bat_state = parsed.get("bat_state")
                    self._state["battery"].update({
                        "t_ms": parsed.get("t_ms"),
                        "vbat_mv": vbat_mv,
                        "vbat_v": vbat_mv / 1000.0 if vbat_mv is not None else None,
                        "bat_state": bat_state,
                        "bat_state_label": BAT_STATE_LABELS.get(bat_state, "UNKNOWN"),
                    })

            return copy.deepcopy(self._state)

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self._state)


class MapDownloadManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._state = {
            "running": False,
            "total": 0,
            "completed": 0,
            "failed": 0,
            "last_error": None,
            "bounds": None,
            "min_zoom": None,
            "max_zoom": None,
            "started_at": None,
            "finished_at": None,
            "cached_tiles": self._count_cached_tiles(),
            "canceled": False,
        }

    def _count_cached_tiles(self):
        if not TILE_CACHE_DIR.exists():
            return 0
        return sum(1 for _ in TILE_CACHE_DIR.rglob("*.png"))

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self._state)

    def cancel(self):
        with self._lock:
            if not self._state["running"]:
                return False
            self._stop_event.set()
            self._state["canceled"] = True
        return True

    def start(self, payload):
        if not payload:
            return False, "Missing request payload."

        bounds = payload.get("bounds")
        try:
            min_zoom = int(payload.get("min_zoom"))
            max_zoom = int(payload.get("max_zoom"))
        except (TypeError, ValueError):
            return False, "Zoom levels must be integers."

        if bounds is None:
            return False, "Bounds are required."

        try:
            south = float(bounds.get("south"))
            west = float(bounds.get("west"))
            north = float(bounds.get("north"))
            east = float(bounds.get("east"))
        except (TypeError, ValueError):
            return False, "Bounds must be numeric."

        if min_zoom < 0 or max_zoom < 0 or min_zoom > max_zoom:
            return False, "Invalid zoom range."

        if south >= north or west >= east:
            return False, "Bounds must define a valid rectangle."

        tile_plan = self._calculate_tile_plan(south, west, north, east, min_zoom, max_zoom)
        total_tiles = tile_plan["total"]
        if total_tiles == 0:
            return False, "Selection produced no tiles."
        if total_tiles > MAX_TILE_REQUEST:
            return False, "Tile request too large. Reduce area or zoom range."

        with self._lock:
            if self._state["running"]:
                return False, "Download already running."
            self._stop_event.clear()
            self._state.update({
                "running": True,
                "total": total_tiles,
                "completed": 0,
                "failed": 0,
                "last_error": None,
                "bounds": {
                    "south": south,
                    "west": west,
                    "north": north,
                    "east": east,
                },
                "min_zoom": min_zoom,
                "max_zoom": max_zoom,
                "started_at": time.time(),
                "finished_at": None,
                "cached_tiles": self._count_cached_tiles(),
                "canceled": False,
            })

        self._thread = threading.Thread(
            target=self._download_tiles,
            args=(south, west, north, east, min_zoom, max_zoom, tile_plan["ranges"],),
            daemon=True,
        )
        self._thread.start()
        return True, None

    def _download_tiles(self, south, west, north, east, min_zoom, max_zoom, ranges):
        TILE_CACHE_DIR.mkdir(parents=True, exist_ok=True)
        for zoom in range(min_zoom, max_zoom + 1):
            x_min, x_max, y_min, y_max = ranges[zoom]
            for x in range(x_min, x_max + 1):
                for y in range(y_min, y_max + 1):
                    if self._stop_event.is_set():
                        self._finalize(cancelled=True)
                        return

                    file_path = TILE_CACHE_DIR / str(zoom) / str(x) / f"{y}.png"
                    if file_path.exists():
                        self._increment_progress()
                        continue

                    file_path.parent.mkdir(parents=True, exist_ok=True)
                    try:
                        request = urllib.request.Request(
                            TILE_SOURCE_URL.format(z=zoom, x=x, y=y),
                            headers={"User-Agent": TILE_USER_AGENT},
                        )
                        with urllib.request.urlopen(request, timeout=TILE_REQUEST_TIMEOUT) as response:
                            if response.status != 200:
                                raise urllib.error.HTTPError(
                                    request.full_url, response.status, response.reason, response.headers, None
                                )
                            data = response.read()
                        file_path.write_bytes(data)
                        self._increment_progress(cached_inc=1)
                    except (urllib.error.URLError, urllib.error.HTTPError, OSError) as exc:
                        self._increment_progress(failed_inc=1, error=str(exc))

        self._finalize(cancelled=False)

    def _finalize(self, cancelled):
        with self._lock:
            self._state["running"] = False
            self._state["finished_at"] = time.time()
            self._state["canceled"] = cancelled

    def _increment_progress(self, failed_inc=0, cached_inc=0, error=None):
        with self._lock:
            self._state["completed"] += 1
            if failed_inc:
                self._state["failed"] += failed_inc
                if error:
                    self._state["last_error"] = error
            if cached_inc:
                self._state["cached_tiles"] += cached_inc

    def _calculate_tile_plan(self, south, west, north, east, min_zoom, max_zoom):
        ranges = {}
        total = 0
        for zoom in range(min_zoom, max_zoom + 1):
            x_min, y_min = self._latlon_to_tile(north, west, zoom)
            x_max, y_max = self._latlon_to_tile(south, east, zoom)
            x_min, x_max = sorted((x_min, x_max))
            y_min, y_max = sorted((y_min, y_max))
            ranges[zoom] = (x_min, x_max, y_min, y_max)
            total += (x_max - x_min + 1) * (y_max - y_min + 1)
        return {"ranges": ranges, "total": total}

    def _latlon_to_tile(self, lat, lon, zoom):
        lat = max(min(lat, 85.0511), -85.0511)
        lon = max(min(lon, 180.0), -180.0)
        n = 2 ** zoom
        x = int((lon + 180.0) / 360.0 * n)
        lat_rad = math.radians(lat)
        y = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        x = max(0, min(x, n - 1))
        y = max(0, min(y, n - 1))
        return x, y


TELEMETRY = TelemetryStore()
MAP_DOWNLOADS = MapDownloadManager()
CLIENTS = []
CLIENTS_LOCK = threading.Lock()
STOP_EVENT = threading.Event()


def broadcast(snapshot):
    data = json.dumps({"state": snapshot})
    with CLIENTS_LOCK:
        for client in list(CLIENTS):
            try:
                client.put_nowait(data)
            except queue.Full:
                try:
                    client.get_nowait()
                except queue.Empty:
                    pass
                try:
                    client.put_nowait(data)
                except queue.Full:
                    pass


def lora_worker():
    spi = None
    radio = None
    try:
        spi, radio = setup_radio()
        while not STOP_EVENT.is_set():
            pkt = radio.poll_packet()
            if not pkt:
                time.sleep(0.01)
                continue

            payload, crc_ok, rssi, snr = pkt

            if crc_ok:
                LED.on()
            else:
                LED.off()

            parsed = parse_payload(payload) if crc_ok else None
            raw_info = {
                "hex": payload.hex(),
                "ascii": safe_ascii(payload),
            }
            radio_info = {
                "rssi_dbm": rssi,
                "snr_db": snr,
                "crc_ok": bool(crc_ok),
                "payload_len": len(payload),
            }
            snapshot = TELEMETRY.update(parsed, radio_info, raw_info)
            broadcast(snapshot)
    except Exception as exc:  # pylint: disable=broad-except
        print("LoRa worker stopped:", exc)
    finally:
        try:
            LED.off()
        except OSError:
            pass

        if radio is not None:
            try:
                radio.cs(1)
            except OSError:
                pass

        if spi is not None:
            try:
                spi.close()
            except OSError:
                pass

        try:
            import RPi.GPIO as GPIO  # pylint: disable=import-error

            GPIO.cleanup()
        except (ImportError, RuntimeError):
            pass


class GroundStationHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path

        if path in ("/", "/index.html"):
            return self._serve_static("index.html")
        if path in ("/map-setup", "/map-setup.html"):
            return self._serve_static("map_setup.html")
        if path == "/style.css":
            return self._serve_static("style.css")
        if path == "/map-setup.css":
            return self._serve_static("map-setup.css")
        if path == "/app.js":
            return self._serve_static("app.js")
        if path == "/map-setup.js":
            return self._serve_static("map-setup.js")
        if path == "/api/state":
            return self._send_json({"state": TELEMETRY.snapshot()})
        if path == "/api/map/status":
            return self._send_json({"status": MAP_DOWNLOADS.snapshot()})
        if path == "/events":
            return self._serve_events()
        if path.startswith("/tiles/"):
            return self._serve_tiles(path)
        if path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return

        self.send_error(404, "Not Found")

    def do_POST(self):
        path = urlparse(self.path).path

        if path == "/api/map/download":
            payload = self._read_json()
            ok, error = MAP_DOWNLOADS.start(payload)
            if not ok:
                return self._send_json({"ok": False, "error": error}, status=400)
            return self._send_json({"ok": True})

        if path == "/api/map/cancel":
            MAP_DOWNLOADS.cancel()
            return self._send_json({"ok": True})

        self.send_error(404, "Not Found")

    def _serve_static(self, filename):
        file_path = STATIC_DIR / filename
        return self._serve_file(file_path)

    def _serve_tiles(self, path):
        parts = path.strip("/").split("/")
        if len(parts) != 4:
            self.send_error(404, "Not Found")
            return

        _, zoom, x, y_file = parts
        if not (zoom.isdigit() and x.isdigit() and y_file.endswith(".png")):
            self.send_error(404, "Not Found")
            return

        y = y_file[:-4]
        if not y.isdigit():
            self.send_error(404, "Not Found")
            return

        file_path = TILE_CACHE_DIR / zoom / x / y_file
        return self._serve_file(file_path)

    def _serve_file(self, file_path):
        if not file_path.exists():
            self.send_error(404, "Not Found")
            return

        mime_type = mimetypes.guess_type(file_path)[0] or "application/octet-stream"
        data = file_path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", mime_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, payload, status=200):
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return None
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None

    def _serve_events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        client_queue = queue.Queue(maxsize=1)
        with CLIENTS_LOCK:
            CLIENTS.append(client_queue)

        try:
            snapshot = TELEMETRY.snapshot()
            self.wfile.write(f"data: {json.dumps({'state': snapshot})}\n\n".encode("utf-8"))
            self.wfile.flush()

            last_keepalive = time.time()
            while True:
                try:
                    data = client_queue.get(timeout=1.0)
                except queue.Empty:
                    if time.time() - last_keepalive >= 10:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                        last_keepalive = time.time()
                    continue

                self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with CLIENTS_LOCK:
                if client_queue in CLIENTS:
                    CLIENTS.remove(client_queue)

    def log_message(self, fmt, *args):  # pylint: disable=arguments-differ
        print("%s - - [%s] %s" % (self.client_address[0], self.log_date_time_string(), fmt % args))


def main():
    if not STATIC_DIR.exists():
        raise SystemExit("Static assets not found at %s" % STATIC_DIR)

    TILE_CACHE_DIR.mkdir(parents=True, exist_ok=True)

    worker = threading.Thread(target=lora_worker, daemon=True)
    worker.start()

    server = ThreadingHTTPServer((HOST, PORT), GroundStationHandler)
    print("Ground station running at http://%s:%d" % (HOST, PORT))

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STOP_EVENT.set()
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
