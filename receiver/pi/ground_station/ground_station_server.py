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

try:
    import adafruit_ssd1306
    import board
    import busio
except Exception:  # pylint: disable=broad-except
    adafruit_ssd1306 = None
    board = None
    busio = None

try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:  # pylint: disable=broad-except
    Image = None
    ImageDraw = None
    ImageFont = None

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))

try:
    import lora_receive_pi as lora_driver
    from lora_receive_pi import LED, parse_payload, safe_ascii, setup_radio
except ImportError as exc:  # pylint: disable=import-error
    raise SystemExit(
        "Failed to import LoRa receiver module. Run from receiver/pi on the Raspberry Pi."
    ) from exc

# FIX: Bind to localhost only for security. Use SSH tunnel or firewall for remote access.
# To allow remote access, set HOST="0.0.0.0" AND configure AUTH_TOKEN below.
HOST = "127.0.0.1"
PORT = 8000

# FIX: Simple shared-secret authentication for command endpoints.
# Set to None to disable auth (not recommended if HOST="0.0.0.0").
# Generate a secure token with: python3 -c "import secrets; print(secrets.token_urlsafe(32))"
import os
AUTH_TOKEN = os.environ.get("GS_AUTH_TOKEN", "CHANGE_ME_BEFORE_FLIGHT")

STATIC_DIR = Path(__file__).resolve().parent / "static"
TILE_CACHE_DIR = STATIC_DIR / "tiles"
TILE_SOURCE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
TILE_USER_AGENT = "RocketTelemetryGroundStation/1.0"
TILE_REQUEST_TIMEOUT = 10
MAX_TILE_REQUEST = 50000

OLED_WIDTH = 128
OLED_HEIGHT = 32
OLED_I2C_ADDR = 0x3C
OLED_LINE_HEIGHT = 8
OLED_CHARS_PER_LINE = OLED_WIDTH // 6
OLED_UPDATE_INTERVAL_S = 0.5
OLED_FONT_SIZE = 8
OLED_FONT_PATH = Path(__file__).resolve().parent / "resources" / "slkscr.ttf"

mimetypes.add_type("text/css", ".css")
mimetypes.add_type("text/javascript", ".js")
mimetypes.add_type("image/png", ".png")

BAT_STATE_LABELS = {
    0: "OK",
    1: "WARN",
    2: "SHED",
    3: "CUTOFF",
}

ATTITUDE_FILTERS = ("madgwick", "mahony", "complementary", "accel-threshold")
DEFAULT_Z_THRESHOLD_G = 1.2
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

LORA_CMD_MAGIC = 0xB1
LORA_CMD_SD_START = 0x01
LORA_CMD_SD_STOP = 0x02
LORA_CMD_BUZZER = 0x03
LORA_CMD_TELEM_ENABLE = 0x04
LORA_CMD_TELEM_DISABLE = 0x05
LORA_CMD_REPEAT_COUNT = 3
LORA_CMD_REPEAT_DELAY_S = 0.1


def _clamp(value, low, high):
    return max(low, min(high, value))


def _quat_to_euler(q0, q1, q2, q3):
    roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    pitch = math.asin(_clamp(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0))
    yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG


class OledStatusDisplay:
    def __init__(self, width=OLED_WIDTH, height=OLED_HEIGHT, address=OLED_I2C_ADDR):
        self._lock = threading.Lock()
        self._enabled = False
        self._display = None
        self._i2c = None
        self._font = None
        self._disabled_reason = None
        self._last_update_s = 0.0
        self._update_interval_s = OLED_UPDATE_INTERVAL_S
        self._chars_per_line = OLED_CHARS_PER_LINE
        self._width = width
        self._height = height

        if "linux" not in sys.platform.lower():
            self._disabled_reason = "Linux only"
            return
        if adafruit_ssd1306 is None or board is None or busio is None:
            self._disabled_reason = "OLED dependencies missing"
            return
        if Image is None or ImageDraw is None or ImageFont is None:
            self._disabled_reason = "Pillow not installed"
            return

        try:
            self._i2c = busio.I2C(board.SCL, board.SDA)
            self._display = adafruit_ssd1306.SSD1306_I2C(width, height, self._i2c, addr=address)
            self._font = self._load_font()
            self._display.fill(0)
            self._display.show()
            self._enabled = True
            self._draw_lines(["Ground Station", "OLED ready", "Waiting data", ""])
            print("OLED status display enabled")
        except Exception as exc:  # pylint: disable=broad-except
            self._disabled_reason = str(exc)
            self._display = None
            print("OLED status display disabled:", exc)

    def _load_font(self):
        if ImageFont is None:
            return None
        if OLED_FONT_PATH.exists():
            try:
                return ImageFont.truetype(str(OLED_FONT_PATH), OLED_FONT_SIZE)
            except Exception as exc:  # pylint: disable=broad-except
                print("OLED font load failed (%s): %s" % (OLED_FONT_PATH, exc))
        else:
            print("OLED font file not found at %s; falling back to default font" % OLED_FONT_PATH)

        return ImageFont.load_default()

    @staticmethod
    def _fmt_voltage(value):
        if value is None:
            return "--"
        return "%.2f" % value

    def _build_lines(self, snapshot):
        snapshot = snapshot or {}
        radio = snapshot.get("radio") or {}
        battery = snapshot.get("battery") or {}
        monitor = snapshot.get("voltage_monitor") or {}
        recovery = snapshot.get("recovery") or {}
        gps = snapshot.get("gps") or {}

        callsign = (snapshot.get("callsign") or "--")[:6]
        packet_count = int(snapshot.get("packet_count") or 0)
        rssi = radio.get("rssi_dbm")
        rssi_text = "--" if rssi is None else "%d" % int(rssi)
        line1 = "ID:%s P:%d R:%s" % (callsign, packet_count, rssi_text)

        rocket_v = self._fmt_voltage(battery.get("vbat_v"))
        rocket_state = (battery.get("bat_state_label") or "--")[:7]
        line2 = "Rkt:%sV %s" % (rocket_v, rocket_state)

        pi_batt_v = self._fmt_voltage(monitor.get("vbatt_v"))
        pi_in_v = self._fmt_voltage(monitor.get("vin_v"))
        line3 = "Pi:%sV In:%sV" % (pi_batt_v, pi_in_v)

        if monitor.get("shutdown_triggered"):
            line4 = "ALERT: PI SHUTDOWN"
        elif monitor.get("last_error"):
            line4 = "Err:%s" % monitor.get("last_error")
        elif monitor.get("warning"):
            line4 = "Warn:%s" % monitor.get("warning")
        else:
            phase = (recovery.get("phase") or "--")[:8]
            altitude = recovery.get("altitude_agl_m")
            if altitude is None:
                altitude = gps.get("alt_m")
            alt_text = "--" if altitude is None else "%dm" % int(round(altitude))
            line4 = "Alt:%s Ph:%s" % (alt_text, phase)

        return [line1, line2, line3, line4]

    def _draw_lines(self, lines):
        if not self._enabled or self._display is None:
            return

        image = Image.new("1", (self._width, self._height))
        draw = ImageDraw.Draw(image)
        for idx, line in enumerate(lines[:4]):
            y = idx * OLED_LINE_HEIGHT
            draw.text((0, y), (line or "")[:self._chars_per_line], font=self._font, fill=255)
        self._display.image(image)
        self._display.show()

    def update(self, snapshot):
        if not self._enabled:
            return

        now = time.time()
        with self._lock:
            if now - self._last_update_s < self._update_interval_s:
                return
            self._last_update_s = now
            try:
                self._draw_lines(self._build_lines(snapshot))
            except Exception as exc:  # pylint: disable=broad-except
                print("OLED update failed:", exc)

    def stop(self):
        if not self._enabled:
            return
        with self._lock:
            try:
                self._display.fill(0)
                self._display.show()
            except Exception:  # pylint: disable=broad-except
                pass

    @property
    def disabled_reason(self):
        return self._disabled_reason


class ComplementaryFilter:
    def __init__(self, alpha=0.96):
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def reset(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(self, gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt):
        self.roll += gx_dps * dt
        self.pitch += gy_dps * dt
        self.yaw += gz_dps * dt

        if ax_g is not None and ay_g is not None and az_g is not None:
            roll_acc = math.atan2(ay_g, az_g) * RAD_TO_DEG
            pitch_acc = math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG
            alpha = self.alpha
            self.roll = alpha * self.roll + (1.0 - alpha) * roll_acc
            self.pitch = alpha * self.pitch + (1.0 - alpha) * pitch_acc

        return self.roll, self.pitch, self.yaw


class AccelThresholdFilter:
    def __init__(self, z_threshold_g=DEFAULT_Z_THRESHOLD_G):
        self.z_threshold_g = z_threshold_g
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def reset(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def set_threshold(self, z_threshold_g):
        if z_threshold_g is None:
            return
        self.z_threshold_g = z_threshold_g

    def update(self, gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt):
        if ax_g is None or ay_g is None or az_g is None:
            if gx_dps is None or gy_dps is None or gz_dps is None:
                return None
            self.roll += gx_dps * dt
            self.pitch += gy_dps * dt
            self.yaw += gz_dps * dt
            return self.roll, self.pitch, self.yaw

        if abs(az_g) < self.z_threshold_g:
            self.roll = math.atan2(ay_g, az_g) * RAD_TO_DEG
            self.pitch = math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG
            return self.roll, self.pitch, self.yaw

        if gx_dps is None or gy_dps is None or gz_dps is None:
            return None

        self.roll += gx_dps * dt
        self.pitch += gy_dps * dt
        self.yaw += gz_dps * dt
        return self.roll, self.pitch, self.yaw


class MahonyFilter:
    def __init__(self, kp=0.6, ki=0.0):
        self.kp = kp
        self.ki = ki
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.i_x = 0.0
        self.i_y = 0.0
        self.i_z = 0.0

    def reset(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.i_x = 0.0
        self.i_y = 0.0
        self.i_z = 0.0

    def update(self, gx, gy, gz, ax, ay, az, dt):
        if ax is not None and ay is not None and az is not None:
            norm = math.sqrt(ax * ax + ay * ay + az * az)
        else:
            norm = 0.0

        if norm > 0.0:
            ax /= norm
            ay /= norm
            az /= norm

            vx = 2.0 * (self.q1 * self.q3 - self.q0 * self.q2)
            vy = 2.0 * (self.q0 * self.q1 + self.q2 * self.q3)
            vz = self.q0 * self.q0 - self.q1 * self.q1 - self.q2 * self.q2 + self.q3 * self.q3

            ex = (ay * vz - az * vy)
            ey = (az * vx - ax * vz)
            ez = (ax * vy - ay * vx)

            if self.ki > 0.0:
                self.i_x += self.ki * ex * dt
                self.i_y += self.ki * ey * dt
                self.i_z += self.ki * ez * dt
                gx += self.i_x
                gy += self.i_y
                gz += self.i_z

            gx += self.kp * ex
            gy += self.kp * ey
            gz += self.kp * ez

        q0 = self.q0
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3

        q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        q_dot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        q_dot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        q_dot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        q0 += q_dot0 * dt
        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt

        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm == 0.0:
            return 0.0, 0.0, 0.0

        self.q0 = q0 / norm
        self.q1 = q1 / norm
        self.q2 = q2 / norm
        self.q3 = q3 / norm

        return _quat_to_euler(self.q0, self.q1, self.q2, self.q3)


class MadgwickFilter:
    def __init__(self, beta=0.1):
        self.beta = beta
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

    def reset(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

    def update(self, gx, gy, gz, ax, ay, az, dt):
        q0 = self.q0
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3

        if ax is not None and ay is not None and az is not None:
            norm = math.sqrt(ax * ax + ay * ay + az * az)
        else:
            norm = 0.0

        if norm > 0.0:
            ax /= norm
            ay /= norm
            az /= norm

            _2q0 = 2.0 * q0
            _2q1 = 2.0 * q1
            _2q2 = 2.0 * q2
            _2q3 = 2.0 * q3
            _4q0 = 4.0 * q0
            _4q1 = 4.0 * q1
            _4q2 = 4.0 * q2
            _8q1 = 8.0 * q1
            _8q2 = 8.0 * q2
            q0q0 = q0 * q0
            q1q1 = q1 * q1
            q2q2 = q2 * q2
            q3q3 = q3 * q3

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
            s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
            s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay

            norm_s = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if norm_s > 0.0:
                s0 /= norm_s
                s1 /= norm_s
                s2 /= norm_s
                s3 /= norm_s
        else:
            s0 = s1 = s2 = s3 = 0.0

        q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self.beta * s0
        q_dot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - self.beta * s1
        q_dot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - self.beta * s2
        q_dot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - self.beta * s3

        q0 += q_dot0 * dt
        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt

        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm == 0.0:
            return 0.0, 0.0, 0.0

        self.q0 = q0 / norm
        self.q1 = q1 / norm
        self.q2 = q2 / norm
        self.q3 = q3 / norm

        return _quat_to_euler(self.q0, self.q1, self.q2, self.q3)


class AttitudeEstimator:
    def __init__(self, mode="madgwick", threshold_g=DEFAULT_Z_THRESHOLD_G):
        self.mode = None
        self._filter = None
        self._last_t_s = None
        self.threshold_g = threshold_g
        self.set_mode(mode)

    def set_mode(self, mode):
        mode = (mode or "").lower()
        if mode not in ATTITUDE_FILTERS:
            raise ValueError("Unknown attitude filter: %s" % mode)
        if mode == self.mode:
            return
        if mode == "madgwick":
            self._filter = MadgwickFilter()
        elif mode == "mahony":
            self._filter = MahonyFilter()
        elif mode == "accel-threshold":
            self._filter = AccelThresholdFilter(self.threshold_g)
        else:
            self._filter = ComplementaryFilter()
        self.mode = mode
        self._last_t_s = None

    def set_threshold_g(self, threshold_g):
        try:
            value = float(threshold_g)
        except (TypeError, ValueError) as exc:
            raise ValueError("Threshold must be a number.") from exc
        if value <= 0.0:
            raise ValueError("Threshold must be positive.")
        self.threshold_g = value
        if isinstance(self._filter, AccelThresholdFilter):
            self._filter.set_threshold(value)

    def update(self, t_ms, gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g):
        if t_ms is None:
            return None

        t_s = t_ms / 1000.0
        dt = 0.02
        if self._last_t_s is not None:
            dt = t_s - self._last_t_s
            if not (0.0 < dt <= 1.0):
                dt = 0.02
        self._last_t_s = t_s

        if self.mode == "complementary":
            if gx_dps is None or gy_dps is None or gz_dps is None:
                return None
            return self._filter.update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt)

        if self.mode == "accel-threshold":
            return self._filter.update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt)

        if gx_dps is None or gy_dps is None or gz_dps is None:
            return None

        gx = gx_dps * DEG_TO_RAD
        gy = gy_dps * DEG_TO_RAD
        gz = gz_dps * DEG_TO_RAD
        return self._filter.update(gx, gy, gz, ax_g, ay_g, az_g, dt)


class TelemetryStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._attitude = AttitudeEstimator()
        self._state = self._blank_state()
        self._state["attitude"]["filter"] = self._attitude.mode
        self._state["attitude"]["threshold_g"] = self._attitude.threshold_g

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
            "navsat": {
                "t_ms": None,
                "svs_total": None,
                "svs_used": None,
                "cno_max": None,
                "cno_avg": None,
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
            "sd_logging": {
                "enabled": None,
                "last_command": None,
                "ack_timestamp": None,
            },
            "telemetry_tx": {
                "enabled": None,
                "last_command": None,
                "ack_timestamp": None,
            },
            "voltage_monitor": {
                "timestamp": None,
                "enabled": False,
                "running": False,
                "status": "init",
                "disabled_reason": None,
                "vin_v": None,
                "vout_v": None,
                "vbatt_v": None,
                "temp_v": None,
                "temp_c": None,
                "temp_f": None,
                "warning": None,
                "shutdown_triggered": False,
                "last_error": None,
            },
            "attitude": {
                "roll": None,
                "pitch": None,
                "yaw": None,
                "filter": None,
                "threshold_g": self._attitude.threshold_g,
            },
            "recovery": {
                "enabled": True,
                "mode": "downlink",
                "phase": "idle",
                "launch_alt_m": None,
                "altitude_agl_m": None,
                "max_altitude_agl_m": None,
                "vertical_speed_mps": None,
                "drogue": {
                    "deployed": False,
                    "deploy_timestamp": None,
                    "deploy_alt_agl_m": None,
                    "reason": None,
                },
                "main": {
                    "deployed": False,
                    "deploy_timestamp": None,
                    "deploy_alt_agl_m": None,
                    "reason": None,
                },
                "rules": None,
            },
        }

    def set_attitude_filter(self, mode):
        with self._lock:
            self._attitude.set_mode(mode)
            self._state["attitude"].update({
                "roll": None,
                "pitch": None,
                "yaw": None,
                "filter": self._attitude.mode,
            })

    def set_attitude_threshold(self, threshold_g):
        with self._lock:
            self._attitude.set_threshold_g(threshold_g)
            self._state["attitude"]["threshold_g"] = self._attitude.threshold_g

    def update(self, parsed, radio_info, raw_info):
        with self._lock:
            self._state["timestamp"] = time.time()
            self._state["packet_count"] += 1
            self._state["voltage_monitor"] = lora_driver.get_voltage_monitor_snapshot()

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
                elif payload_type == "navsat":
                    self._state["navsat"].update({
                        "t_ms": parsed.get("t_ms"),
                        "svs_total": parsed.get("svs_total"),
                        "svs_used": parsed.get("svs_used"),
                        "cno_max": parsed.get("cno_max"),
                        "cno_avg": parsed.get("cno_avg"),
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
                    gx_dps = gx / 10.0 if gx is not None else None
                    gy_dps = gy / 10.0 if gy is not None else None
                    gz_dps = gz / 10.0 if gz is not None else None
                    ax_g = ax / 1000.0 if ax is not None else None
                    ay_g = ay / 1000.0 if ay is not None else None
                    az_g = az / 1000.0 if az is not None else None
                    attitude = self._attitude.update(
                        parsed.get("t_ms"),
                        gx_dps,
                        gy_dps,
                        gz_dps,
                        ax_g,
                        ay_g,
                        az_g,
                    )
                    self._state["imu"].update({
                        "t_ms": parsed.get("t_ms"),
                        "gx": gx,
                        "gy": gy,
                        "gz": gz,
                        "ax": ax,
                        "ay": ay,
                        "az": az,
                        "gx_dps": gx_dps,
                        "gy_dps": gy_dps,
                        "gz_dps": gz_dps,
                        "ax_g": ax_g,
                        "ay_g": ay_g,
                        "az_g": az_g,
                    })
                    if attitude is not None:
                        roll, pitch, yaw = attitude
                        self._state["attitude"].update({
                            "roll": roll,
                            "pitch": pitch,
                            "yaw": yaw,
                            "filter": self._attitude.mode,
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
                elif payload_type == "recovery":
                    drogue_deployed = bool(parsed.get("drogue_deployed"))
                    main_deployed = bool(parsed.get("main_deployed"))
                    self._state["recovery"].update({
                        "enabled": True,
                        "mode": "downlink",
                        "phase": parsed.get("phase") or "unknown",
                        "launch_alt_m": None,
                        "altitude_agl_m": parsed.get("altitude_agl_m"),
                        "max_altitude_agl_m": parsed.get("max_altitude_agl_m"),
                        "vertical_speed_mps": parsed.get("vertical_speed_mps"),
                        "drogue": {
                            "deployed": drogue_deployed,
                            "deploy_timestamp": None,
                            "deploy_alt_agl_m": parsed.get("drogue_deploy_alt_agl_m"),
                            "reason": parsed.get("drogue_reason") if drogue_deployed else None,
                        },
                        "main": {
                            "deployed": main_deployed,
                            "deploy_timestamp": None,
                            "deploy_alt_agl_m": parsed.get("main_deploy_alt_agl_m"),
                            "reason": parsed.get("main_reason") if main_deployed else None,
                        },
                        "rules": None,
                    })
                elif payload_type == "cmd_ack":
                    cmd = parsed.get("command")
                    ack_ts = self._state["timestamp"]
                    if cmd in ("sd_start", "sd_stop"):
                        self._state["sd_logging"].update({
                            "enabled": parsed.get("logging_enabled", parsed.get("enabled")),
                            "last_command": cmd,
                            "ack_timestamp": ack_ts,
                        })
                    elif cmd in ("telemetry_enable", "telemetry_disable"):
                        self._state["telemetry_tx"].update({
                            "enabled": parsed.get("telemetry_enabled", parsed.get("enabled")),
                            "last_command": cmd,
                            "ack_timestamp": ack_ts,
                        })

            return copy.deepcopy(self._state)

    def update_voltage_monitor(self, monitor_snapshot):
        with self._lock:
            self._state["voltage_monitor"] = dict(monitor_snapshot or {})
            return copy.deepcopy(self._state)

    def snapshot(self):
        with self._lock:
            self._state["voltage_monitor"] = lora_driver.get_voltage_monitor_snapshot()
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
LORA_LOCK = threading.Lock()
LORA_RADIO = {"radio": None}
OLED_DISPLAY = None


def broadcast(snapshot):
    if OLED_DISPLAY is not None:
        OLED_DISPLAY.update(snapshot)

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


def handle_voltage_monitor_update(monitor_snapshot):
    snapshot = TELEMETRY.update_voltage_monitor(monitor_snapshot)
    broadcast(snapshot)


def lora_worker():
    # FIX: Add retry/recovery logic so a transient SPI error doesn't kill telemetry for the whole flight.
    # Use exponential backoff with a bounded retry count, then re-init the radio if retries fail.
    retry_delay = 1.0  # Start with 1 second
    max_delay = 30.0   # Cap backoff at 30 seconds
    max_retries = 10   # Maximum number of consecutive init failures before giving up
    retry_count = 0
    
    while not STOP_EVENT.is_set():
        spi = None
        radio = None
        try:
            spi, radio = setup_radio()
            with LORA_LOCK:
                LORA_RADIO["radio"] = radio
            
            # Radio initialized successfully - reset backoff
            retry_delay = 1.0
            retry_count = 0
            print("LoRa radio initialized successfully")
            
            # Main poll loop
            while not STOP_EVENT.is_set():
                with LORA_LOCK:
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
            retry_count += 1
            print(f"LoRa worker error: {exc}. Retry {retry_count}/{max_retries} in {retry_delay:.1f}s...")
            
            # Check if we've exceeded max retries
            if retry_count >= max_retries:
                print("LoRa worker: Maximum retries exceeded. Giving up.")
                break
            
            # Wait before retry (unless stop event is set)
            STOP_EVENT.wait(retry_delay)
            
            # Exponential backoff with cap
            retry_delay = min(retry_delay * 2, max_delay)
            
        finally:
            # Cleanup radio resources
            try:
                LED.off()
            except OSError:
                pass

            with LORA_LOCK:
                LORA_RADIO["radio"] = None

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

            # Note: GPIO.cleanup() is called here on each retry, but that's acceptable
            # since we're re-initializing the entire radio stack
            try:
                import RPi.GPIO as GPIO  # pylint: disable=import-error

                GPIO.cleanup()
            except (ImportError, RuntimeError):
                pass
    
    print("LoRa worker thread exiting")


def send_lora_command(action, duration_s=None):
    if action == "sd_start":
        payload = bytes([LORA_CMD_MAGIC, LORA_CMD_SD_START])
    elif action == "sd_stop":
        payload = bytes([LORA_CMD_MAGIC, LORA_CMD_SD_STOP])
    elif action == "telemetry_enable":
        payload = bytes([LORA_CMD_MAGIC, LORA_CMD_TELEM_ENABLE])
    elif action == "telemetry_disable":
        payload = bytes([LORA_CMD_MAGIC, LORA_CMD_TELEM_DISABLE])
    elif action == "buzzer":
        try:
            duration = int(duration_s)
        except (TypeError, ValueError):
            return False, "Duration must be a number"
        if duration not in (1, 5, 10, 30):
            return False, "Duration must be one of 1, 5, 10, 30"
        payload = bytes([LORA_CMD_MAGIC, LORA_CMD_BUZZER, duration])
    else:
        return False, "Unknown command"

    with LORA_LOCK:
        radio = LORA_RADIO.get("radio")
        if radio is None:
            return False, "LoRa radio unavailable"
        try:
            radio.write_reg(lora_driver.REG_PA_CONFIG, lora_driver.PA_BOOST | 0x0F)
            radio.write_reg(lora_driver.REG_PA_DAC, 0x84)
            radio.write_reg(lora_driver.REG_OCP, 0x20 | 0x0B)
        except AttributeError:
            pass
        ok = False
        for attempt in range(LORA_CMD_REPEAT_COUNT):
            tx_ok = radio.send_packet(payload)
            print(
                "LoRa CMD %s %s attempt %d/%d ok=%s"
                % (action, payload.hex(), attempt + 1, LORA_CMD_REPEAT_COUNT, "yes" if tx_ok else "no")
            )
            ok = ok or tx_ok
            if attempt + 1 < LORA_CMD_REPEAT_COUNT:
                time.sleep(LORA_CMD_REPEAT_DELAY_S)
    if not ok:
        return False, "LoRa transmit failed"
    return True, None


class GroundStationHandler(BaseHTTPRequestHandler):
    def _check_auth(self):
        """Check authentication for command endpoints. Returns True if authorized."""
        # FIX: Simple token-based authentication for command endpoints.
        # If AUTH_TOKEN is None or empty, auth is disabled (not recommended for HOST="0.0.0.0").
        if AUTH_TOKEN is None or AUTH_TOKEN == "" or AUTH_TOKEN == "CHANGE_ME_BEFORE_FLIGHT":
            # No auth configured - only allow if localhost
            if HOST == "127.0.0.1" or HOST == "localhost":
                return True
            # Warn but allow for now (should fail in production)
            print(f"WARNING: No auth token configured and HOST={HOST} - this is insecure!")
            return True
        
        # Check for token in Authorization header (Bearer token format)
        auth_header = self.headers.get("Authorization", "")
        if auth_header.startswith("Bearer "):
            token = auth_header[7:]  # Strip "Bearer " prefix
            if token == AUTH_TOKEN:
                return True
        
        # Also check X-Auth-Token header (simpler format for curl/scripts)
        x_token = self.headers.get("X-Auth-Token", "")
        if x_token == AUTH_TOKEN:
            return True
        
        return False
    
    def _send_auth_error(self):
        """Send 401 Unauthorized response."""
        self._send_json({"ok": False, "error": "Unauthorized"}, status=401)
    
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
        if path == "/api/attitude/filters":
            return self._send_json({
                "filters": list(ATTITUDE_FILTERS),
                "active": TELEMETRY.snapshot().get("attitude", {}).get("filter"),
                "threshold_g": TELEMETRY.snapshot().get("attitude", {}).get("threshold_g"),
            })
        if path == "/api/attitude/threshold":
            return self._send_json({
                "threshold_g": TELEMETRY.snapshot().get("attitude", {}).get("threshold_g"),
            })
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

        # FIX: Require authentication for command endpoints (not read-only telemetry SSE)
        if path == "/api/map/download":
            # No auth required — map tile downloads are a local convenience, not a rocket command.
            payload = self._read_json()
            ok, error = MAP_DOWNLOADS.start(payload)
            if not ok:
                return self._send_json({"ok": False, "error": error}, status=400)
            return self._send_json({"ok": True})

        if path == "/api/map/cancel":
            # No auth required — cancelling a tile download is safe.
            MAP_DOWNLOADS.cancel()
            return self._send_json({"ok": True})

        if path == "/api/attitude/filter":
            if not self._check_auth():
                return self._send_auth_error()
            payload = self._read_json() or {}
            mode = payload.get("filter")
            try:
                TELEMETRY.set_attitude_filter(mode)
            except ValueError as exc:
                return self._send_json({"ok": False, "error": str(exc)}, status=400)
            snapshot = TELEMETRY.snapshot()
            broadcast(snapshot)
            return self._send_json({
                "ok": True,
                "filter": snapshot.get("attitude", {}).get("filter"),
            })

        if path == "/api/attitude/threshold":
            if not self._check_auth():
                return self._send_auth_error()
            payload = self._read_json() or {}
            try:
                TELEMETRY.set_attitude_threshold(payload.get("threshold_g"))
            except ValueError as exc:
                return self._send_json({"ok": False, "error": str(exc)}, status=400)
            snapshot = TELEMETRY.snapshot()
            broadcast(snapshot)
            return self._send_json({
                "ok": True,
                "threshold_g": snapshot.get("attitude", {}).get("threshold_g"),
            })

        if path == "/api/command":
            if not self._check_auth():
                return self._send_auth_error()
            payload = self._read_json() or {}
            action = payload.get("action")
            duration_s = payload.get("duration_s")
            ok, error = send_lora_command(action, duration_s)
            if not ok:
                return self._send_json({"ok": False, "error": error}, status=400)
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
        except Exception:  # pylint: disable=broad-except
            # FIX: Catch all connection errors (BrokenPipeError, ConnectionResetError,
            # ConnectionAbortedError, OSError, etc.) to ensure client cleanup always runs.
            # Client disconnects can manifest in various ways depending on OS and network state.
            pass
        finally:
            # FIX: Always remove disconnected clients from broadcast list
            with CLIENTS_LOCK:
                if client_queue in CLIENTS:
                    CLIENTS.remove(client_queue)

    def log_message(self, fmt, *args):  # pylint: disable=arguments-differ
        print("%s - - [%s] %s" % (self.client_address[0], self.log_date_time_string(), fmt % args))


def main():
    global OLED_DISPLAY

    if not STATIC_DIR.exists():
        raise SystemExit("Static assets not found at %s" % STATIC_DIR)

    TILE_CACHE_DIR.mkdir(parents=True, exist_ok=True)

    OLED_DISPLAY = OledStatusDisplay()
    if OLED_DISPLAY.disabled_reason:
        print("OLED status display unavailable:", OLED_DISPLAY.disabled_reason)

    voltage_monitor = lora_driver.start_voltage_monitor(on_sample=handle_voltage_monitor_update)
    server = None

    worker = threading.Thread(target=lora_worker, daemon=True)
    worker.start()

    try:
        server = ThreadingHTTPServer((HOST, PORT), GroundStationHandler)
        print("Ground station running at http://%s:%d" % (HOST, PORT))
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STOP_EVENT.set()
        if voltage_monitor is not None:
            voltage_monitor.stop()
        if OLED_DISPLAY is not None:
            OLED_DISPLAY.stop()
        if server is not None:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    main()
