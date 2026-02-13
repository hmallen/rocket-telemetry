import math
import time

# pylint: disable=import-error
try:
    from machine import SPI, Pin
except ImportError:  # pragma: no cover - desktop lint/testing only
    SPI = None  # type: ignore[assignment]
    Pin = None  # type: ignore[assignment]

# Pico <-> SX1278 pins
PIN_SCK = 18
PIN_MOSI = 19
PIN_MISO = 16
PIN_NSS = 17
PIN_RST = 20
PIN_DIO0 = 21

# Protocol/config mirrors src/cfg.h + src/lora_link.cpp
PROTO_MAGIC = 0xA1
CMD_MAGIC = 0xB1
CMD_ACK = 0x10
CMD_SD_START = 0x01
CMD_SD_STOP = 0x02
CMD_BUZZER = 0x03

LORA_CALLSIGN = "CALLSIGN"

LORA_FREQ_HZ = 433_000_000
LORA_MIN_TX_INTERVAL_MS = 200
LORA_HEARTBEAT_MS = 30_000
LORA_GPS_INTERVAL_MS = 1000
LORA_ALT_INTERVAL_MS = 200
LORA_IMU_INTERVAL_MS = 200
LORA_BAT_INTERVAL_MS = 1000
LORA_NAVSAT_INTERVAL_MS = 2000
LORA_RECOVERY_INTERVAL_MS = 500
LORA_MAX_MISSION_TX_MS = 20 * 60 * 1000

LORA_ACK_REPEAT_COUNT = 3
LORA_ACK_REPEAT_MS = 400

GPS_LOCK_DELAY_MS = 1500
START_TX_ENABLED = True

VBAT_WARN_V = 7.10
VBAT_SHED_V = 6.90
VBAT_CUTOFF_V = 6.60

RECOVERY_MIN_ASCENT_AGL_MM = 90_000
RECOVERY_LIFTOFF_CONFIRM_AGL_MM = 5_000
RECOVERY_LAUNCH_VSPEED_CMS = 500
RECOVERY_APOGEE_DROP_MM = 12_000
RECOVERY_APOGEE_NEG_VSPEED_CMS = -100
RECOVERY_APOGEE_PRESS_RISE_PA_X10 = 50
RECOVERY_APOGEE_VOTE_MIN = 2
RECOVERY_MAIN_DEPLOY_AGL_MM = 300_000
RECOVERY_MAIN_BACKUP_AGL_MM = 220_000
RECOVERY_MAIN_DROP_AFTER_DROGUE_MM = 30_000
RECOVERY_MAIN_FAST_DESCENT_CMS = -2500
RECOVERY_MAIN_FAST_DESCENT_MAX_AGL_MM = 450_000
RECOVERY_DESCENT_CONFIRM_MM = 2_000
RECOVERY_LANDED_AGL_MM = 20_000

RECOVERY_PHASE_IDLE = 0
RECOVERY_PHASE_ASCENT = 1
RECOVERY_PHASE_DESCENT = 2
RECOVERY_PHASE_LANDED = 3
RECOVERY_REASON_NONE = 0
RECOVERY_DROGUE_REASON_APOGEE_VOTE = 1
RECOVERY_MAIN_REASON_PRIMARY_ALTITUDE = 1
RECOVERY_MAIN_REASON_BACKUP_ALTITUDE = 2
RECOVERY_MAIN_REASON_FAST_DESCENT = 3
RECOVERY_MAIN_REASON_BACKUP_NO_DROGUE = 4

# SX127x registers
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_OCP = 0x0B
REG_LNA = 0x0C
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE_ADDR = 0x0E
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_PKT_SNR_VALUE = 0x19
REG_PKT_RSSI_VALUE = 0x1A
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_PREAMBLE_MSB = 0x20
REG_PREAMBLE_LSB = 0x21
REG_PAYLOAD_LENGTH = 0x22
REG_MODEM_CONFIG_3 = 0x26
REG_SYNC_WORD = 0x39
REG_DIO_MAPPING_1 = 0x40
REG_VERSION = 0x42
REG_PA_DAC = 0x4D

LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_RX_CONT = 0x05

IRQ_RX_DONE = 0x40
IRQ_CRC_ERR = 0x20
IRQ_TX_DONE = 0x08

DIO0_RX_DONE = 0x00
DIO0_TX_DONE = 0x40
PA_BOOST = 0x80
SYNCWORD_LORA_PUBLIC = 0x34


def sleep_ms(ms):
    fn = getattr(time, "sleep_ms", None)
    if not callable(fn):
        time.sleep(ms / 1000.0)
    else:
        fn(ms)  # pylint: disable=not-callable


def ticks_ms():
    fn = getattr(time, "ticks_ms", None)
    if not callable(fn):
        return int(time.time() * 1000)
    return fn()  # pylint: disable=not-callable


def ticks_diff(newer, older):
    fn = getattr(time, "ticks_diff", None)
    if not callable(fn):
        return newer - older
    return fn(newer, older)  # pylint: disable=not-callable


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def clamp_i(v, lo, hi):
    v = int(v)
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def append_u16_le(buf, v):
    v = int(v) & 0xFFFF
    buf.append(v & 0xFF)
    buf.append((v >> 8) & 0xFF)


def append_i16_le(buf, v):
    append_u16_le(buf, clamp_i(v, -32768, 32767))


def append_u32_le(buf, v):
    v = int(v) & 0xFFFFFFFF
    buf.append(v & 0xFF)
    buf.append((v >> 8) & 0xFF)
    buf.append((v >> 16) & 0xFF)
    buf.append((v >> 24) & 0xFF)


def append_i32_le(buf, v):
    append_u32_le(buf, clamp_i(v, -2147483648, 2147483647))


def pressure_pa_from_alt_m(alt_m):
    x = 1.0 - (2.25577e-5 * alt_m)
    if x < 0.05:
        x = 0.05
    return 101325.0 * (x ** 5.25588)


class Lcg:
    def __init__(self, seed=0xA53C9E21):
        self.state = seed & 0xFFFFFFFF
        if self.state == 0:
            self.state = 1

    def rand_u32(self):
        self.state = (1664525 * self.state + 1013904223) & 0xFFFFFFFF
        return self.state

    def uniform(self, lo, hi):
        return lo + (hi - lo) * (self.rand_u32() / 4294967295.0)

    def centered(self, span):
        return self.uniform(-span, span)


class SX1278:
    def __init__(self, spi, nss, rst, dio0):
        self.spi = spi
        self.nss = nss
        self.rst = rst
        self.dio0 = dio0
        self.nss.init(Pin.OUT, value=1)
        self.rst.init(Pin.OUT, value=1)
        self.dio0.init(Pin.IN)

    def _cs(self, level):
        self.nss.value(level)

    def reset(self):
        self.rst.value(0)
        sleep_ms(50)
        self.rst.value(1)
        sleep_ms(50)

    def read_reg(self, addr):
        self._cs(0)
        self.spi.write(bytearray([addr & 0x7F]))
        v = self.spi.read(1, 0x00)[0]
        self._cs(1)
        return v

    def write_reg(self, addr, val):
        self._cs(0)
        self.spi.write(bytearray([addr | 0x80, val & 0xFF]))
        self._cs(1)

    def read_fifo(self, n):
        self._cs(0)
        self.spi.write(bytearray([REG_FIFO & 0x7F]))
        data = self.spi.read(n, 0x00)
        self._cs(1)
        return data

    def write_fifo(self, data):
        self._cs(0)
        self.spi.write(bytearray([REG_FIFO | 0x80]))
        self.spi.write(data)
        self._cs(1)

    def set_mode(self, mode):
        self.write_reg(REG_OP_MODE, LONG_RANGE_MODE | (mode & 0x07))
        sleep_ms(5)

    def set_frequency(self, freq_hz):
        frf = int((freq_hz << 19) // 32_000_000)
        self.write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self.write_reg(REG_FRF_MID, (frf >> 8) & 0xFF)
        self.write_reg(REG_FRF_LSB, frf & 0xFF)

    def clear_irqs(self):
        self.write_reg(REG_IRQ_FLAGS, 0xFF)

    def init_lora_433(self):
        self.reset()
        ver = self.read_reg(REG_VERSION)
        print("SX127x RegVersion: 0x%02X" % ver)

        self.set_mode(MODE_SLEEP)
        self.set_mode(MODE_STDBY)
        self.write_reg(REG_FIFO_TX_BASE_ADDR, 0x00)
        self.write_reg(REG_FIFO_RX_BASE_ADDR, 0x00)
        self.set_frequency(LORA_FREQ_HZ)

        # Match receiver defaults used in this project.
        self.write_reg(REG_LNA, self.read_reg(REG_LNA) | 0x03)
        self.write_reg(REG_MODEM_CONFIG_1, 0x72)  # BW125 / CR4/5
        self.write_reg(REG_MODEM_CONFIG_2, 0x74)  # SF7 / CRC on
        self.write_reg(REG_MODEM_CONFIG_3, 0x04)  # AGC on
        self.write_reg(REG_PREAMBLE_MSB, 0x00)
        self.write_reg(REG_PREAMBLE_LSB, 0x08)
        self.write_reg(REG_SYNC_WORD, SYNCWORD_LORA_PUBLIC)
        self.write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE)

        self.write_reg(REG_PA_CONFIG, PA_BOOST | 0x08)  # ~10 dBm to match cfg.h
        self.write_reg(REG_PA_DAC, 0x84)
        self.write_reg(REG_OCP, 0x20 | 0x0B)

        self.clear_irqs()
        self.set_mode(MODE_RX_CONT)
        print("LoRa RX/TX ready @ 433 MHz")

    def poll_packet(self):
        irq = self.read_reg(REG_IRQ_FLAGS)
        if not (irq & IRQ_RX_DONE):
            return None

        crc_ok = not (irq & IRQ_CRC_ERR)
        n = self.read_reg(REG_RX_NB_BYTES)
        addr = self.read_reg(REG_FIFO_RX_CURRENT_ADDR)
        self.write_reg(REG_FIFO_ADDR_PTR, addr)
        payload = self.read_fifo(n)

        snr_raw = self.read_reg(REG_PKT_SNR_VALUE)
        snr_db = (snr_raw - 256 if snr_raw > 127 else snr_raw) / 4.0
        rssi_dbm = -157 + self.read_reg(REG_PKT_RSSI_VALUE)

        self.clear_irqs()
        return payload, crc_ok, rssi_dbm, snr_db

    def send_packet(self, payload, timeout_ms=2000):
        if not payload:
            return False
        self.set_mode(MODE_STDBY)
        self.clear_irqs()
        self.write_reg(REG_FIFO_TX_BASE_ADDR, 0x00)
        self.write_reg(REG_FIFO_ADDR_PTR, 0x00)
        self.write_reg(REG_PAYLOAD_LENGTH, len(payload))
        self.write_reg(REG_DIO_MAPPING_1, DIO0_TX_DONE)
        self.write_fifo(payload)
        self.set_mode(MODE_TX)

        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < timeout_ms:
            if self.read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE:
                self.clear_irqs()
                self.write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE)
                self.set_mode(MODE_RX_CONT)
                return True
            sleep_ms(2)

        self.clear_irqs()
        self.write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE)
        self.set_mode(MODE_RX_CONT)
        return False


class RecoveryState:
    def __init__(self):
        self.initialized = False
        self.launch_alt_mm = 0
        self.last_alt_mm = 0
        self.last_t_ms = 0
        self.agl_mm = 0
        self.max_agl_mm = 0
        self.vspeed_cms = 0
        self.phase = RECOVERY_PHASE_IDLE
        self.launch_armed = False
        self.liftoff_detected = False
        self.have_min_press = False
        self.min_press_pa_x10 = 0
        self.drogue_deployed = False
        self.main_deployed = False
        self.drogue_reason = RECOVERY_REASON_NONE
        self.main_reason = RECOVERY_REASON_NONE
        self.drogue_deploy_agl_mm = -1
        self.main_deploy_agl_mm = -1

    def update(self, now_ms, alt_mm, press_pa_x10):
        if not self.initialized:
            self.initialized = True
            self.launch_alt_mm = alt_mm
            self.last_alt_mm = alt_mm
            self.last_t_ms = now_ms
            return

        agl_raw = alt_mm - self.launch_alt_mm
        self.agl_mm = agl_raw if agl_raw > 0 else 0
        if self.agl_mm > self.max_agl_mm:
            self.max_agl_mm = self.agl_mm

        self.vspeed_cms = 0
        dt_ms = now_ms - self.last_t_ms
        if dt_ms > 0 and dt_ms <= 10_000:
            self.vspeed_cms = clamp_i((alt_mm - self.last_alt_mm) * 100 / dt_ms, -32768, 32767)

        if (not self.launch_armed) and self.max_agl_mm >= RECOVERY_MIN_ASCENT_AGL_MM:
            self.launch_armed = True
        if (not self.liftoff_detected) and (
            self.agl_mm >= RECOVERY_LIFTOFF_CONFIRM_AGL_MM
            or self.vspeed_cms >= RECOVERY_LAUNCH_VSPEED_CMS
        ):
            self.liftoff_detected = True

        if press_pa_x10 > 0:
            if not self.have_min_press:
                self.have_min_press = True
                self.min_press_pa_x10 = press_pa_x10
            elif (not self.drogue_deployed) and press_pa_x10 < self.min_press_pa_x10:
                self.min_press_pa_x10 = press_pa_x10

        flight_enabled = self.launch_armed and self.liftoff_detected
        altitude_drop = self.agl_mm <= (self.max_agl_mm - RECOVERY_APOGEE_DROP_MM)
        descending = (
            self.agl_mm <= (self.max_agl_mm - RECOVERY_DESCENT_CONFIRM_MM)
            or self.vspeed_cms < 0
        )

        if flight_enabled:
            self.phase = RECOVERY_PHASE_DESCENT if descending else RECOVERY_PHASE_ASCENT
        else:
            self.phase = RECOVERY_PHASE_IDLE

        if flight_enabled and (not self.drogue_deployed):
            neg_vspeed = self.vspeed_cms <= RECOVERY_APOGEE_NEG_VSPEED_CMS
            pressure_rise = self.have_min_press and (
                press_pa_x10 >= (self.min_press_pa_x10 + RECOVERY_APOGEE_PRESS_RISE_PA_X10)
            )
            votes = 0
            if altitude_drop:
                votes += 1
            if neg_vspeed:
                votes += 1
            if pressure_rise:
                votes += 1
            if votes >= RECOVERY_APOGEE_VOTE_MIN:
                self.drogue_deployed = True
                self.drogue_reason = RECOVERY_DROGUE_REASON_APOGEE_VOTE
                self.drogue_deploy_agl_mm = self.agl_mm

        if not self.main_deployed:
            fast_descent = self.vspeed_cms <= RECOVERY_MAIN_FAST_DESCENT_CMS
            low_enough_for_fast = self.agl_mm <= RECOVERY_MAIN_FAST_DESCENT_MAX_AGL_MM
            backup_alt_trigger = self.agl_mm <= RECOVERY_MAIN_BACKUP_AGL_MM

            if self.drogue_deployed:
                enough_drop = (
                    self.drogue_deploy_agl_mm < 0
                    or self.agl_mm <= (self.drogue_deploy_agl_mm - RECOVERY_MAIN_DROP_AFTER_DROGUE_MM)
                )
                primary_main = descending and enough_drop and self.agl_mm <= RECOVERY_MAIN_DEPLOY_AGL_MM
                backup_fast = descending and fast_descent and low_enough_for_fast

                reason = RECOVERY_REASON_NONE
                if primary_main:
                    reason = RECOVERY_MAIN_REASON_PRIMARY_ALTITUDE
                elif backup_fast:
                    reason = RECOVERY_MAIN_REASON_FAST_DESCENT
                elif backup_alt_trigger:
                    reason = RECOVERY_MAIN_REASON_BACKUP_ALTITUDE

                if reason != RECOVERY_REASON_NONE:
                    self.main_deployed = True
                    self.main_reason = reason
                    self.main_deploy_agl_mm = self.agl_mm
            elif flight_enabled and descending and backup_alt_trigger:
                self.main_deployed = True
                self.main_reason = RECOVERY_MAIN_REASON_BACKUP_NO_DROGUE
                self.main_deploy_agl_mm = self.agl_mm

        if self.main_deployed and self.agl_mm <= RECOVERY_LANDED_AGL_MM:
            self.phase = RECOVERY_PHASE_LANDED

        self.last_alt_mm = alt_mm
        self.last_t_ms = now_ms


class FlightProfile:
    def __init__(self, rng):
        self.rng = rng
        self.base_lat = 37.1672
        self.base_lon = -106.4976
        self.site_alt_m = 2400.0
        self.launch_delay_s = 5.0
        self.east_m = 0.0
        self.north_m = 0.0
        self.last_ms = None

    def _launch_t(self, t_s):
        return t_s - self.launch_delay_s

    def _phase(self, lt):
        if lt < 0:
            return "pad"
        if lt < 4:
            return "boost"
        if lt < 20:
            return "coast"
        if lt < 78:
            return "drogue"
        if lt < 126:
            return "main"
        if lt < 140:
            return "flare"
        return "landed"

    def _agl_vspeed(self, lt):
        if lt < 0:
            return 0.0, 0.0
        if lt < 4.0:
            return 20.0 * lt * lt, 40.0 * lt
        if lt < 16.0:
            d = lt - 4.0
            return 320.0 + 160.0 * d - 4.75 * d * d, 160.0 - 9.5 * d
        if lt < 20.0:
            d = lt - 16.0
            return 1556.0 + 46.0 * d - 7.5 * d * d, 46.0 - 15.0 * d
        if lt < 78.0:
            d = lt - 20.0
            return 1620.0 - 22.0 * d, -22.0
        if lt < 126.0:
            d = lt - 78.0
            return 344.0 - 6.5 * d, -6.5
        if lt < 140.0:
            d = lt - 126.0
            e = math.exp(-d / 4.0)
            return 32.0 * e, -8.0 * e
        return 0.0, 0.0

    def _wind(self, lt):
        if lt < 0:
            return 0.1, 0.0
        if lt < 20:
            return 6.5, 2.0
        if lt < 78:
            return 11.0, -1.5
        if lt < 126:
            return 4.5, -0.6
        return 0.3, 0.1

    def sample(self, now_ms):
        if self.last_ms is None:
            self.last_ms = now_ms
        dt = clamp((now_ms - self.last_ms) / 1000.0, 0.0, 0.5)
        self.last_ms = now_ms

        t_s = now_ms / 1000.0
        lt = self._launch_t(t_s)
        phase = self._phase(lt)
        agl_m, vs_mps = self._agl_vspeed(lt)

        east_v, north_v = self._wind(lt)
        self.east_m += east_v * dt + self.rng.centered(0.08)
        self.north_m += north_v * dt + self.rng.centered(0.08)

        abs_alt_m = self.site_alt_m + agl_m
        press_pa = pressure_pa_from_alt_m(abs_alt_m) + self.rng.centered(7.0)
        temp_c = 19.0 - 0.0060 * agl_m + self.rng.centered(0.12)

        gps_valid = now_ms >= GPS_LOCK_DELAY_MS
        cos_lat = math.cos(self.base_lat * math.pi / 180.0)
        lat_deg = self.base_lat + (self.north_m / 111320.0)
        lon_deg = self.base_lon + (self.east_m / (111320.0 * cos_lat))
        if gps_valid:
            lat_deg += self.rng.centered(1.5) / 111320.0
            lon_deg += self.rng.centered(1.5) / (111320.0 * cos_lat)

        if phase == "pad":
            ax_g, ay_g, az_g = 0.01, -0.01, 1.00
            gx_dps, gy_dps, gz_dps = 1.0, -0.5, 0.3
        elif phase == "boost":
            ax_g = 0.30 * math.sin(lt * 12.0)
            ay_g = 0.24 * math.cos(lt * 11.0)
            az_g = 5.1 + 0.45 * math.sin(lt * 20.0)
            gx_dps = 30.0 * math.sin(lt * 5.0)
            gy_dps = 25.0 * math.cos(lt * 5.5)
            gz_dps = 85.0 + 18.0 * math.sin(lt * 3.0)
        elif phase == "coast":
            ax_g = 0.10 * math.sin(lt * 2.5)
            ay_g = 0.12 * math.cos(lt * 2.0)
            az_g = 0.25 + 0.10 * math.sin(lt * 3.5)
            gx_dps = 12.0 + 7.0 * math.sin(lt * 1.8)
            gy_dps = 10.0 * math.cos(lt * 1.9)
            gz_dps = 30.0 + 9.0 * math.sin(lt * 1.4)
        elif phase == "drogue":
            ax_g = 0.55 * math.sin(lt * 1.3)
            ay_g = 0.65 * math.cos(lt * 1.1)
            az_g = 1.35 + 0.35 * math.sin(lt * 2.1)
            gx_dps = 55.0 * math.sin(lt * 1.2)
            gy_dps = 62.0 * math.cos(lt * 1.1)
            gz_dps = 18.0 * math.sin(lt * 1.6)
        elif phase in ("main", "flare"):
            ax_g = 0.17 * math.sin(lt * 1.0)
            ay_g = 0.20 * math.cos(lt * 0.9)
            az_g = 1.05 + 0.10 * math.sin(lt * 1.3)
            gx_dps = 15.0 * math.sin(lt * 0.8)
            gy_dps = 18.0 * math.cos(lt * 0.9)
            gz_dps = 8.0 * math.sin(lt * 1.1)
        else:
            ax_g, ay_g, az_g = 0.01, -0.01, 1.00
            gx_dps, gy_dps, gz_dps = 0.6, -0.5, 0.3

        ax_g += self.rng.centered(0.03)
        ay_g += self.rng.centered(0.03)
        az_g += self.rng.centered(0.05)
        gx_dps += self.rng.centered(1.2)
        gy_dps += self.rng.centered(1.2)
        gz_dps += self.rng.centered(1.5)

        vbat_v = 7.95 - 0.0075 * t_s + 0.04 * math.sin(t_s * 0.06) + self.rng.centered(0.015)
        vbat_v = clamp(vbat_v, 6.55, 8.35)
        if vbat_v < VBAT_CUTOFF_V:
            bat_state = 3
        elif vbat_v < VBAT_SHED_V:
            bat_state = 2
        elif vbat_v < VBAT_WARN_V:
            bat_state = 1
        else:
            bat_state = 0

        dyn = 2 if phase == "boost" else (1 if phase == "coast" else 0)
        svs_total = clamp_i(13 + int(2 * math.sin(t_s * 0.15)) + int(self.rng.centered(1.2)), 9, 18)
        svs_used = clamp_i(svs_total - 3 - dyn + int(self.rng.centered(1.2)), 4, svs_total)
        cno_max = clamp_i(47 - 2 * dyn + int(4 * math.sin(t_s * 0.11)) + int(self.rng.centered(2.0)), 18, 55)
        cno_avg = clamp_i(cno_max - 9 + int(self.rng.centered(2.5)), 12, cno_max)

        return {
            "phase": phase,
            "gps_valid": gps_valid,
            "press_pa_x10": clamp_i(round(press_pa * 10.0), -2147483648, 2147483647),
            "temp_c_x100": clamp_i(round(temp_c * 100.0), -32768, 32767),
            "lat_e7": clamp_i(round(lat_deg * 1e7), -2147483648, 2147483647),
            "lon_e7": clamp_i(round(lon_deg * 1e7), -2147483648, 2147483647),
            "height_mm": clamp_i(round((abs_alt_m + self.rng.centered(1.5)) * 1000.0), -2147483648, 2147483647),
            "gx": clamp_i(round(gx_dps * 10.0), -32768, 32767),
            "gy": clamp_i(round(gy_dps * 10.0), -32768, 32767),
            "gz": clamp_i(round(gz_dps * 10.0), -32768, 32767),
            "ax": clamp_i(round(ax_g * 1000.0), -32768, 32767),
            "ay": clamp_i(round(ay_g * 1000.0), -32768, 32767),
            "az": clamp_i(round(az_g * 1000.0), -32768, 32767),
            "vbat_mv": clamp_i(round(vbat_v * 1000.0), 0, 65535),
            "bat_state": bat_state,
            "svs_total": svs_total,
            "svs_used": svs_used,
            "cno_max": cno_max,
            "cno_avg": cno_avg,
            "vbat_v": vbat_v,
            "agl_m": agl_m,
            "vs_mps": vs_mps,
        }


class FlightEmulator:
    def __init__(self, radio):
        self.radio = radio
        self.rng = Lcg()
        self.profile = FlightProfile(self.rng)
        self.recovery = RecoveryState()

        self.tx_enabled = START_TX_ENABLED
        self.logging_enabled = START_TX_ENABLED
        self.shutdown = False

        self.last_tx_ms = -LORA_MIN_TX_INTERVAL_MS
        self.next_tx_ms = 0

        self.last_id_tx_ms = 0
        self.last_gps_tx_ms = 0
        self.last_alt_tx_ms = 0
        self.last_imu_tx_ms = 0
        self.last_bat_tx_ms = 0
        self.last_navsat_tx_ms = 0
        self.last_recovery_tx_ms = 0

        self.ack_pending = False
        self.ack_retry_after_ms = 0
        self.ack_retries_left = 0
        self.ack_buf = bytearray()
        self.next_status_ms = 0

    def _can_tx(self, now_ms):
        if now_ms < self.next_tx_ms:
            return False
        return (now_ms - self.last_tx_ms) >= LORA_MIN_TX_INTERVAL_MS

    def _record_tx(self, now_ms):
        self.last_tx_ms = now_ms
        self.next_tx_ms = now_ms + LORA_MIN_TX_INTERVAL_MS

    def _queue_ack(self, cmd, now_ms):
        self.ack_buf = bytearray([CMD_MAGIC, CMD_ACK, cmd & 0xFF, 1 if self.logging_enabled else 0])
        self.ack_pending = True
        self.ack_retries_left = LORA_ACK_REPEAT_COUNT
        self.ack_retry_after_ms = now_ms + LORA_ACK_REPEAT_MS

    def _schedule_ack_retry(self, now_ms):
        if self.ack_retries_left == 0:
            self.ack_pending = False
            self.ack_retry_after_ms = 0
            return
        self.ack_retries_left -= 1
        if self.ack_retries_left == 0:
            self.ack_pending = False
            self.ack_retry_after_ms = 0
            return
        self.ack_retry_after_ms = now_ms + LORA_ACK_REPEAT_MS

    def _maybe_send_ack(self, now_ms):
        if not self.ack_pending:
            return False
        if now_ms < self.ack_retry_after_ms:
            return False
        if not self._can_tx(now_ms):
            return False

        ok = self.radio.send_packet(self.ack_buf)
        if ok:
            self._record_tx(now_ms)
        self._schedule_ack_retry(now_ms)
        return True

    def _handle_command(self, payload, now_ms):
        if len(payload) < 2:
            return
        if payload[0] != CMD_MAGIC:
            return

        cmd = payload[1]
        if cmd == CMD_BUZZER:
            if len(payload) < 3:
                return
            print("CMD buzzer duration=%ds" % int(payload[2]))
            return

        if cmd == CMD_SD_START:
            self.logging_enabled = True
            if not self.shutdown:
                self.tx_enabled = True
            self._queue_ack(CMD_SD_START, now_ms)
            print("CMD sd_start")
        elif cmd == CMD_SD_STOP:
            self.logging_enabled = False
            self.tx_enabled = False
            self._queue_ack(CMD_SD_STOP, now_ms)
            print("CMD sd_stop")

    def _build_id(self):
        c = LORA_CALLSIGN.encode("ascii")
        return bytearray([PROTO_MAGIC, 1, len(c)]) + c

    def _build_alt(self, now_ms, s):
        b = bytearray([PROTO_MAGIC, 0])
        append_u32_le(b, now_ms)
        append_i32_le(b, s["press_pa_x10"])
        append_i16_le(b, s["temp_c_x100"])
        return b

    def _build_gps(self, now_ms, s):
        b = bytearray([PROTO_MAGIC, 2])
        append_u32_le(b, now_ms)
        append_i32_le(b, s["lat_e7"])
        append_i32_le(b, s["lon_e7"])
        append_i32_le(b, s["height_mm"])
        return b

    def _build_imu(self, now_ms, s):
        b = bytearray([PROTO_MAGIC, 3])
        append_u32_le(b, now_ms)
        append_i16_le(b, s["gx"])
        append_i16_le(b, s["gy"])
        append_i16_le(b, s["gz"])
        append_i16_le(b, s["ax"])
        append_i16_le(b, s["ay"])
        append_i16_le(b, s["az"])
        return b

    def _build_bat(self, now_ms, s):
        b = bytearray([PROTO_MAGIC, 4])
        append_u32_le(b, now_ms)
        append_u16_le(b, s["vbat_mv"])
        b.append(s["bat_state"] & 0xFF)
        return b

    def _build_navsat(self, now_ms, s):
        b = bytearray([PROTO_MAGIC, 5])
        append_u32_le(b, now_ms)
        b.append(s["svs_total"] & 0xFF)
        b.append(s["svs_used"] & 0xFF)
        b.append(s["cno_max"] & 0xFF)
        b.append(s["cno_avg"] & 0xFF)
        return b

    def _build_recovery(self, now_ms):
        r = self.recovery
        flags = (1 if r.drogue_deployed else 0) | (2 if r.main_deployed else 0)
        b = bytearray([PROTO_MAGIC, 6])
        append_u32_le(b, now_ms)
        b.append(r.phase & 0xFF)
        b.append(flags & 0xFF)
        append_i32_le(b, r.agl_mm)
        append_i32_le(b, r.max_agl_mm)
        append_i16_le(b, r.vspeed_cms)
        append_i32_le(b, r.drogue_deploy_agl_mm)
        append_i32_le(b, r.main_deploy_agl_mm)
        b.append(r.drogue_reason & 0xFF)
        b.append(r.main_reason & 0xFF)
        return b

    def _late(self, now_ms, last_ms, interval_ms):
        if interval_ms == 0:
            return -2147483647
        if last_ms == 0:
            return now_ms
        return (now_ms - last_ms) - interval_ms

    def _pick_type(self, now_ms, s):
        gps_late = self._late(now_ms, self.last_gps_tx_ms, LORA_GPS_INTERVAL_MS) if s["gps_valid"] else -2147483647
        alt_late = self._late(now_ms, self.last_alt_tx_ms, LORA_ALT_INTERVAL_MS)
        imu_late = self._late(now_ms, self.last_imu_tx_ms, LORA_IMU_INTERVAL_MS)
        bat_late = self._late(now_ms, self.last_bat_tx_ms, LORA_BAT_INTERVAL_MS)
        nav_late = self._late(now_ms, self.last_navsat_tx_ms, LORA_NAVSAT_INTERVAL_MS) if s["gps_valid"] else -2147483647
        rec_late = self._late(now_ms, self.last_recovery_tx_ms, LORA_RECOVERY_INTERVAL_MS) if self.recovery.initialized else -2147483647

        pick = 0xFF
        best = -2147483647

        if gps_late >= 0 and (best < 0 or gps_late > best):
            pick = 2
            best = gps_late
        if nav_late >= 0 and (best < 0 or nav_late > best):
            pick = 5
            best = nav_late
        if rec_late >= 0 and (best < 0 or rec_late > best):
            pick = 6
            best = rec_late
        if alt_late >= 0 and (best < 0 or alt_late > best):
            pick = 0
            best = alt_late
        if imu_late >= 0 and (best < 0 or imu_late > best):
            pick = 3
            best = imu_late
        if bat_late >= 0 and (best < 0 or bat_late > best):
            pick = 4

        return pick

    def step(self, now_ms):
        for _ in range(3):
            pkt = self.radio.poll_packet()
            if not pkt:
                break
            payload, crc_ok, _rssi, _snr = pkt
            if crc_ok:
                self._handle_command(payload, now_ms)

        sample = self.profile.sample(now_ms)
        if sample["gps_valid"]:
            self.recovery.update(now_ms, sample["height_mm"], sample["press_pa_x10"])

        if self.shutdown:
            return

        if now_ms >= LORA_MAX_MISSION_TX_MS:
            self.shutdown = True
            self.tx_enabled = False
            self.ack_pending = False
            print("Mission timeout reached; LoRa TX shut down")
            return

        if self._maybe_send_ack(now_ms):
            return

        if not self.tx_enabled:
            return
        if not self._can_tx(now_ms):
            return

        if self.last_id_tx_ms == 0 or (now_ms - self.last_id_tx_ms) >= LORA_HEARTBEAT_MS:
            if self.radio.send_packet(self._build_id()):
                self.last_id_tx_ms = now_ms
                self._record_tx(now_ms)
            return

        ptype = self._pick_type(now_ms, sample)
        if ptype == 0xFF:
            return

        payload = None
        if ptype == 0:
            payload = self._build_alt(now_ms, sample)
        elif ptype == 2:
            payload = self._build_gps(now_ms, sample)
        elif ptype == 3:
            payload = self._build_imu(now_ms, sample)
        elif ptype == 4:
            payload = self._build_bat(now_ms, sample)
        elif ptype == 5:
            payload = self._build_navsat(now_ms, sample)
        elif ptype == 6:
            payload = self._build_recovery(now_ms)

        if payload and self.radio.send_packet(payload):
            self._record_tx(now_ms)
            if ptype == 0:
                self.last_alt_tx_ms = now_ms
            elif ptype == 2:
                self.last_gps_tx_ms = now_ms
            elif ptype == 3:
                self.last_imu_tx_ms = now_ms
            elif ptype == 4:
                self.last_bat_tx_ms = now_ms
            elif ptype == 5:
                self.last_navsat_tx_ms = now_ms
            elif ptype == 6:
                self.last_recovery_tx_ms = now_ms

        if now_ms >= self.next_status_ms:
            self.next_status_ms = now_ms + 2000
            print(
                "t=%5.1fs tx=%s phase=%s agl=%6.1fm vb=%.2fV"
                % (now_ms / 1000.0, "on" if self.tx_enabled else "off", sample["phase"], sample["agl_m"], sample["vbat_v"])
            )


def main():
    if SPI is None or Pin is None:
        raise RuntimeError("Run this script on MicroPython with machine module available")

    spi = SPI(
        0,
        baudrate=1_000_000,
        polarity=0,
        phase=0,
        sck=Pin(PIN_SCK),
        mosi=Pin(PIN_MOSI),
        miso=Pin(PIN_MISO),
    )

    radio = SX1278(
        spi=spi,
        nss=Pin(PIN_NSS, Pin.OUT),
        rst=Pin(PIN_RST, Pin.OUT),
        dio0=Pin(PIN_DIO0, Pin.IN),
    )
    radio.init_lora_433()

    emu = FlightEmulator(radio)
    start_ticks = ticks_ms()
    print("LoRa flight emulator started")

    while True:
        now_ms = ticks_diff(ticks_ms(), start_ticks)
        emu.step(now_ms)
        sleep_ms(5)


if __name__ == "__main__":
    main()
