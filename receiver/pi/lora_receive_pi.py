import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass

try:
    import spidev
    import RPi.GPIO as GPIO
except ImportError as e:
    raise SystemExit(
        "Missing Raspberry Pi dependencies. Install with:\n"
        "  sudo apt-get install -y python3-spidev python3-rpi.gpio\n"
        "and enable SPI (raspi-config) if needed."
    ) from e

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None


if not hasattr(time, "sleep_ms"):
    def sleep_ms(ms):
        time.sleep(ms / 1000.0)
    time.sleep_ms = sleep_ms


@dataclass(frozen=True)
class VoltageMonitorConfig:
    i2c_bus: int = 1
    address: int = 0x48

    ch_vin: int = 0b11000001
    ch_vout: int = 0b11010001
    ch_vbatt: int = 0b11100001
    ch_tempv: int = 0b11110001

    vref: float = 6.144
    max_reading: float = 2047.0

    group_sleep_s: float = 2.0
    channel_sleep_s: float = 0.1

    vbatt_min: float = 3.1
    vin_min: float = 3.8
    temp_min_c: float = 5.0
    temp_max_c: float = 60.0

    shutdown_cmd: str = "shutdown -h 2 &"
    temp_profile: str = "piz_uptime_2"


VOLTAGE_MONITOR_CFG = VoltageMonitorConfig()

_VOLTAGE_STATE_LOCK = threading.Lock()
_VOLTAGE_STATE = {
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
}


def _update_voltage_state(**changes):
    with _VOLTAGE_STATE_LOCK:
        _VOLTAGE_STATE.update(changes)


def get_voltage_monitor_snapshot():
    with _VOLTAGE_STATE_LOCK:
        return dict(_VOLTAGE_STATE)


def _read_adc_voltage(bus, address, channel_cfg, cfg):
    bus.write_i2c_block_data(address, 0x01, [0x85, 0x83])
    bus.write_i2c_block_data(address, 0x00, [0x00, 0x00])
    time.sleep(cfg.channel_sleep_s)

    bus.write_i2c_block_data(address, 0x01, [channel_cfg, 0x43])
    time.sleep(cfg.channel_sleep_s)

    raw = bus.read_word_data(address, 0x00)
    value = (((raw & 0xFF) << 8) | ((int(raw) & 0xFFF0) >> 8))
    value >>= 4
    return (value / cfg.max_reading) * cfg.vref


def _temp_c_from_tempv(temp_v, profile):
    if profile == "piz_uptime_2":
        return (4.0 - temp_v) / 0.0432
    if profile == "pi_uptime_ups_2":
        return (4.236 - temp_v) / 0.0408
    raise ValueError("Unknown temp_profile: %r" % profile)


class VoltageMonitor:
    def __init__(self, cfg=None, on_sample=None):
        self._cfg = cfg or VOLTAGE_MONITOR_CFG
        self._on_sample = on_sample
        self._stop_event = threading.Event()
        self._thread = None

        if "linux" not in sys.platform.lower():
            self._enabled = False
            self._disabled_reason = "Linux only"
        elif SMBus is None:
            self._enabled = False
            self._disabled_reason = "smbus2 not installed"
        else:
            self._enabled = True
            self._disabled_reason = None

        _update_voltage_state(
            enabled=self._enabled,
            running=False,
            status="idle" if self._enabled else "disabled",
            disabled_reason=self._disabled_reason,
            warning=None,
            last_error=None,
        )

    def _emit_update(self):
        if self._on_sample is None:
            return
        try:
            self._on_sample(get_voltage_monitor_snapshot())
        except Exception as exc:  # pylint: disable=broad-except
            print("Voltage monitor callback error:", exc)

    def start(self):
        if not self._enabled:
            print("Voltage monitor disabled:", self._disabled_reason)
            self._emit_update()
            return False
        if self._thread is not None and self._thread.is_alive():
            return True

        self._stop_event.clear()
        _update_voltage_state(
            running=True,
            status="running",
            shutdown_triggered=False,
            last_error=None,
        )
        self._emit_update()

        self._thread = threading.Thread(target=self._run, daemon=True, name="uptime-voltage-monitor")
        self._thread.start()
        return True

    def stop(self):
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        state = get_voltage_monitor_snapshot()
        if state.get("status") == "running":
            _update_voltage_state(status="stopped", running=False)
        else:
            _update_voltage_state(running=False)
        self._emit_update()

    def _run(self):
        cfg = self._cfg
        print("Date & Time               Vin   Vout  Batt-V  Board Temperature")
        sys.stdout.flush()

        try:
            with SMBus(cfg.i2c_bus) as bus:
                while not self._stop_event.is_set():
                    vin = _read_adc_voltage(bus, cfg.address, cfg.ch_vin, cfg)
                    vbatt = _read_adc_voltage(bus, cfg.address, cfg.ch_vbatt, cfg)
                    vout = _read_adc_voltage(bus, cfg.address, cfg.ch_vout, cfg)
                    temp_v = _read_adc_voltage(bus, cfg.address, cfg.ch_tempv, cfg)

                    temp_c = _temp_c_from_tempv(temp_v, cfg.temp_profile)
                    temp_f = temp_c * 1.8 + 32.0
                    warning = None

                    if vin > cfg.vin_min:
                        if temp_c < cfg.temp_min_c:
                            warning = "Temperature too cold for charging"
                        elif temp_c > cfg.temp_max_c:
                            warning = "Temperature too hot for charging"

                    _update_voltage_state(
                        timestamp=time.time(),
                        running=True,
                        status="running",
                        vin_v=vin,
                        vout_v=vout,
                        vbatt_v=vbatt,
                        temp_v=temp_v,
                        temp_c=temp_c,
                        temp_f=temp_f,
                        warning=warning,
                        last_error=None,
                    )
                    self._emit_update()

                    print(
                        "%s %5.2f %5.2f %5.2f %8.2fC %6.2fF"
                        % (time.ctime(), vin, vout, vbatt, temp_c, temp_f)
                    )
                    sys.stdout.flush()

                    if vin < cfg.vin_min and vbatt < cfg.vbatt_min:
                        _update_voltage_state(
                            running=False,
                            status="shutdown-triggered",
                            shutdown_triggered=True,
                            warning="Vin and battery below limits",
                        )
                        self._emit_update()
                        print("Shutdown initiated at %s" % time.ctime())
                        print(
                            "At %s, Vin = %4.2f, Vout = %4.2f, Vbattery = %4.2f, Temperature = %5.2fC %5.2fF"
                            % (time.ctime(), vin, vout, vbatt, temp_c, temp_f)
                        )
                        sys.stdout.flush()
                        subprocess.call(cfg.shutdown_cmd, shell=True)
                        time.sleep(2)
                        return

                    if vin > cfg.vin_min:
                        if temp_c < cfg.temp_min_c:
                            print(
                                "Temperature is too cold for battery charging - at %s, Temperature is %5.2f"
                                % (time.ctime(), temp_c)
                            )
                        elif temp_c > cfg.temp_max_c:
                            print(
                                "Temperature is too hot for battery charging - at %s, Temperature is %5.2f"
                                % (time.ctime(), temp_c)
                            )
                        sys.stdout.flush()

                    time.sleep(cfg.group_sleep_s)
        except Exception as exc:  # pylint: disable=broad-except
            _update_voltage_state(running=False, status="error", last_error=str(exc))
            self._emit_update()
            print("Voltage monitor stopped:", exc)
        finally:
            state = get_voltage_monitor_snapshot()
            if state.get("status") == "running":
                _update_voltage_state(running=False, status="stopped")
                self._emit_update()


def start_voltage_monitor(on_sample=None):
    monitor = VoltageMonitor(on_sample=on_sample)
    monitor.start()
    return monitor


_GPIO_INITIALIZED = False


class Pin:
    IN = 0
    OUT = 1

    def __init__(self, pin_id, mode=None, value=None):
        self._pin_id = pin_id
        self._is_led0 = (pin_id == "LED")
        self._led_brightness_path = None
        self._led_trigger_path = None
        self._gpio_pin = None
        self._mode = None

        if self._is_led0:
            base = "/sys/class/leds"
            for name in ("led0", "ACT"):
                brightness = os.path.join(base, name, "brightness")
                trigger = os.path.join(base, name, "trigger")
                if os.path.exists(brightness):
                    self._led_brightness_path = brightness
                    self._led_trigger_path = trigger
                    break
        else:
            self._gpio_pin = int(pin_id)

        if mode is not None:
            self.init(mode, value=value if value is not None else 0)

    def init(self, mode, value=0):
        self._mode = mode
        if self._is_led0:
            if self._led_trigger_path is not None:
                try:
                    with open(self._led_trigger_path, "w", encoding="utf-8") as f:
                        f.write("none")
                except OSError:
                    pass
            if mode == Pin.OUT:
                self.value(value)
            return

        global _GPIO_INITIALIZED
        if not _GPIO_INITIALIZED:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            _GPIO_INITIALIZED = True

        if mode == Pin.OUT:
            GPIO.setup(self._gpio_pin, GPIO.OUT, initial=GPIO.HIGH if value else GPIO.LOW)
        else:
            GPIO.setup(self._gpio_pin, GPIO.IN)

    def value(self, v=None):
        if self._is_led0:
            if self._led_brightness_path is None:
                return 0
            if v is None:
                try:
                    with open(self._led_brightness_path, "r", encoding="utf-8") as f:
                        return 1 if f.read().strip() != "0" else 0
                except OSError:
                    return 0
            try:
                with open(self._led_brightness_path, "w", encoding="utf-8") as f:
                    f.write("1" if v else "0")
            except OSError:
                pass
            return

        if v is None:
            return GPIO.input(self._gpio_pin)
        GPIO.output(self._gpio_pin, GPIO.HIGH if v else GPIO.LOW)

    def on(self):
        self.value(1)

    def off(self):
        self.value(0)


class SPI:
    def __init__(
        self,
        spi_id,
        baudrate=1_000_000,
        polarity=0,
        phase=0,
        no_cs=False,
    ):
        self._spi = spidev.SpiDev()
        self._spi.open(0, int(spi_id))
        self._spi.max_speed_hz = int(baudrate)
        self._spi.mode = ((1 if polarity else 0) << 1) | (1 if phase else 0)
        try:
            self._spi.no_cs = bool(no_cs)
        except Exception:
            pass

    def write(self, buf):
        self._spi.writebytes(list(buf))

    def xfer(self, buf):
        return bytes(self._spi.xfer2(list(buf)))

    def read(self, n, write=0x00):
        return bytes(self._spi.xfer2([write] * int(n)))

    def close(self):
        self._spi.close()

 
PIN_NSS   = 8
PIN_RST   = 25
PIN_DIO0  = 22

USE_HW_CS = True
SPI_BAUDRATE = 100_000

LED = Pin("LED", Pin.OUT)
LED.off()

# -------- SX127x register map (subset) --------
REG_FIFO                    = 0x00
REG_OP_MODE                 = 0x01
REG_FRF_MSB                 = 0x06
REG_FRF_MID                 = 0x07
REG_FRF_LSB                 = 0x08
REG_PA_CONFIG               = 0x09
REG_PA_RAMP                 = 0x0A
REG_OCP                     = 0x0B
REG_LNA                     = 0x0C
REG_FIFO_ADDR_PTR           = 0x0D
REG_FIFO_TX_BASE_ADDR       = 0x0E
REG_FIFO_RX_BASE_ADDR       = 0x0F
REG_FIFO_RX_CURRENT_ADDR    = 0x10
REG_IRQ_FLAGS               = 0x12
REG_RX_NB_BYTES             = 0x13
REG_PKT_SNR_VALUE           = 0x19
REG_PKT_RSSI_VALUE          = 0x1A
REG_MODEM_CONFIG_1          = 0x1D
REG_MODEM_CONFIG_2          = 0x1E
REG_PREAMBLE_MSB            = 0x20
REG_PREAMBLE_LSB            = 0x21
REG_MODEM_CONFIG_3          = 0x26
REG_SYNC_WORD               = 0x39
REG_DIO_MAPPING_1           = 0x40
REG_VERSION                 = 0x42
REG_PAYLOAD_LENGTH          = 0x22
REG_PA_DAC                  = 0x4D

# -------- Modes / bits --------
LONG_RANGE_MODE = 0x80
MODE_SLEEP      = 0x00
MODE_STDBY      = 0x01
MODE_RX_CONT    = 0x05
MODE_TX         = 0x03

PA_BOOST        = 0x80

IRQ_RX_DONE     = 0x40
IRQ_CRC_ERR     = 0x20
IRQ_TX_DONE     = 0x08

DIO0_RX_DONE    = 0x00
DIO0_TX_DONE    = 0x40

SYNCWORD_LORA_PUBLIC = 0x34

CMD_MAGIC = 0xB1
CMD_ACK = 0x10
CMD_SD_START = 0x01
CMD_SD_STOP = 0x02
CMD_BUZZER = 0x03

class SX1278:
    def __init__(self, spi, nss, rst, dio0):
        self.spi = spi
        self.nss = nss
        self.rst = rst
        self.dio0 = dio0

        if not USE_HW_CS:
            self.nss.init(Pin.OUT, value=1)
        self.rst.init(Pin.OUT, value=1)
        self.dio0.init(Pin.IN)

    def cs(self, level):
        if USE_HW_CS:
            return
        self.nss.value(level)

    def reset(self):
        self.rst.value(0)
        time.sleep_ms(50)
        self.rst.value(1)
        time.sleep_ms(50)

    def read_reg(self, addr):
        if USE_HW_CS:
            return self.spi.xfer(bytearray([addr & 0x7F, 0x00]))[1]
        self.cs(0)
        val = self.spi.xfer(bytearray([addr & 0x7F, 0x00]))[1]
        self.cs(1)
        return val

    def write_reg(self, addr, val):
        if USE_HW_CS:
            self.spi.xfer(bytearray([addr | 0x80, val & 0xFF]))
            return
        self.cs(0)
        self.spi.xfer(bytearray([addr | 0x80, val & 0xFF]))
        self.cs(1)

    def read_fifo(self, n):
        if USE_HW_CS:
            return self.spi.xfer(bytearray([REG_FIFO & 0x7F] + ([0x00] * int(n))))[1:]
        self.cs(0)
        data = self.spi.xfer(bytearray([REG_FIFO & 0x7F] + ([0x00] * int(n))))[1:]
        self.cs(1)
        return data

    def write_fifo(self, data):
        if not data:
            return
        payload = bytearray([REG_FIFO | 0x80]) + bytearray(data)
        if USE_HW_CS:
            self.spi.xfer(payload)
            return
        self.cs(0)
        self.spi.xfer(payload)
        self.cs(1)

    def set_mode(self, mode):
        self.write_reg(REG_OP_MODE, LONG_RANGE_MODE | (mode & 0x07))
        time.sleep_ms(5)

    def set_frequency(self, freq_hz):
        frf = int((freq_hz << 19) // 32_000_000)
        self.write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self.write_reg(REG_FRF_MID, (frf >> 8) & 0xFF)
        self.write_reg(REG_FRF_LSB, frf & 0xFF)

    def clear_irqs(self):
        self.write_reg(REG_IRQ_FLAGS, 0xFF)

    def init_rx_433(self):
        self.reset()

        ver = self.read_reg(REG_VERSION)
        print("RegVersion: 0x%02X" % ver)

        self.set_mode(MODE_SLEEP)
        self.set_mode(MODE_STDBY)

        self.write_reg(REG_FIFO_TX_BASE_ADDR, 0x00)
        self.write_reg(REG_FIFO_RX_BASE_ADDR, 0x00)

        self.set_frequency(433_000_000)

        self.write_reg(REG_LNA, self.read_reg(REG_LNA) | 0x03)
        self.write_reg(REG_MODEM_CONFIG_1, 0x72)  # BW125 / CR4/5
        self.write_reg(REG_MODEM_CONFIG_2, 0x74)  # SF7 / CRC on
        self.write_reg(REG_MODEM_CONFIG_3, 0x04)  # AGC on
        self.write_reg(REG_PREAMBLE_MSB, 0x00)
        self.write_reg(REG_PREAMBLE_LSB, 0x08)
        self.write_reg(REG_SYNC_WORD, SYNCWORD_LORA_PUBLIC)
        self.write_reg(REG_DIO_MAPPING_1, 0x00)

        # Ensure PA_BOOST is enabled for TX on common RFM9x modules.
        self.write_reg(REG_PA_CONFIG, PA_BOOST | 0x0F)
        self.write_reg(REG_PA_DAC, 0x84)
        self.write_reg(REG_OCP, 0x20 | 0x0B)

        self.clear_irqs()
        self.set_mode(MODE_RX_CONT)

        print("LoRa RX running on 433 MHz")

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
        snr = (snr_raw - 256 if snr_raw > 127 else snr_raw) / 4.0
        rssi = -157 + self.read_reg(REG_PKT_RSSI_VALUE)

        self.clear_irqs()

        return payload, crc_ok, rssi, snr

    def send_packet(self, payload, timeout_ms=2000):
        if not payload:
            return False
        data = bytes(payload)

        print("LoRa TX start len=%d hex=%s" % (len(data), data.hex()))

        self.set_mode(MODE_STDBY)
        self.clear_irqs()
        self.write_reg(REG_FIFO_TX_BASE_ADDR, 0x00)
        self.write_reg(REG_FIFO_ADDR_PTR, 0x00)
        self.write_reg(REG_PAYLOAD_LENGTH, len(data))
        self.write_reg(REG_DIO_MAPPING_1, DIO0_TX_DONE)
        self.write_fifo(data)
        self.set_mode(MODE_TX)

        start = time.time()
        tx_done = False
        while (time.time() - start) * 1000.0 < timeout_ms:
            irq = self.read_reg(REG_IRQ_FLAGS)
            if irq & IRQ_TX_DONE:
                tx_done = True
                break
            time.sleep_ms(2)

        self.clear_irqs()
        self.write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE)
        self.set_mode(MODE_RX_CONT)
        return tx_done

def safe_ascii(b):
    return "".join(chr(x) if 32 <= x <= 126 else "." for x in b)

def decode_payload(payload):
    if len(payload) < 2:
        return None
    if payload[0] == CMD_MAGIC:
        if len(payload) < 4:
            return "CMD ACK (short)"
        if payload[1] != CMD_ACK:
            return "CMD type=0x%02X" % payload[1]
        cmd = payload[2]
        enabled = "on" if payload[3] else "off"
        if cmd == CMD_SD_START:
            cmd_label = "sd_start"
        elif cmd == CMD_SD_STOP:
            cmd_label = "sd_stop"
        else:
            cmd_label = "0x%02X" % cmd
        return "ACK %s logging=%s" % (cmd_label, enabled)
    if payload[0] != 0xA1:
        return None
    typ = payload[1]
    if typ == 0:
        if len(payload) < 12:
            return "PROTO A1 DATA (short)"
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        press_pa_x10 = int.from_bytes(payload[6:10], "little", signed=True)
        temp_c_x100 = int.from_bytes(payload[10:12], "little", signed=True)
        return "DATA t_ms=%d press_pa_x10=%d temp_c_x100=%d" % (t_ms, press_pa_x10, temp_c_x100)
    if typ == 1:
        if len(payload) < 3:
            return "PROTO A1 ID (short)"
        n = payload[2]
        if len(payload) < 3 + n:
            return "PROTO A1 ID (short)"
        callsign = payload[3:3 + n].decode("ascii", errors="replace")
        return "ID callsign=%s" % callsign
    if typ == 2:
        if len(payload) < 18:
            return "PROTO A1 GPS (short)"
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        lat_e7 = int.from_bytes(payload[6:10], "little", signed=True)
        lon_e7 = int.from_bytes(payload[10:14], "little", signed=True)
        height_mm = int.from_bytes(payload[14:18], "little", signed=True)
        return "GPS t_ms=%d lat=%.7f lon=%.7f alt_m=%.3f" % (
            t_ms,
            lat_e7 / 1e7,
            lon_e7 / 1e7,
            height_mm / 1000.0,
        )
    if typ == 3:
        if len(payload) < 18:
            return "PROTO A1 IMU (short)"
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        gx = int.from_bytes(payload[6:8], "little", signed=True)
        gy = int.from_bytes(payload[8:10], "little", signed=True)
        gz = int.from_bytes(payload[10:12], "little", signed=True)
        ax = int.from_bytes(payload[12:14], "little", signed=True)
        ay = int.from_bytes(payload[14:16], "little", signed=True)
        az = int.from_bytes(payload[16:18], "little", signed=True)
        return "IMU t_ms=%d gx=%d gy=%d gz=%d ax=%d ay=%d az=%d" % (
            t_ms, gx, gy, gz, ax, ay, az
        )
    if typ == 4:
        if len(payload) < 9:
            return "PROTO A1 BAT (short)"
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        vbat_mv = int.from_bytes(payload[6:8], "little", signed=False)
        bat_state = int(payload[8])
        return "BAT t_ms=%d vbat_v=%.3f state=%d" % (t_ms, vbat_mv / 1000.0, bat_state)
    if typ == 5:
        if len(payload) < 10:
            return "PROTO A1 NAVSAT (short)"
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        svs_total = int(payload[6])
        svs_used = int(payload[7])
        cno_max = int(payload[8])
        cno_avg = int(payload[9])
        return "NAVSAT t_ms=%d svs=%d/%d cno_max=%d cno_avg=%d" % (
            t_ms,
            svs_used,
            svs_total,
            cno_max,
            cno_avg,
        )
    return "PROTO A1 type=%d" % typ


def parse_payload(payload):
    if len(payload) < 2:
        return None
    if payload[0] == CMD_MAGIC:
        if payload[1] != CMD_ACK:
            return None
        if len(payload) < 4:
            return None
        cmd = payload[2]
        if cmd == CMD_SD_START:
            cmd_label = "sd_start"
        elif cmd == CMD_SD_STOP:
            cmd_label = "sd_stop"
        elif cmd == CMD_BUZZER:
            cmd_label = "buzzer"
        else:
            cmd_label = "unknown"
        return {
            "type": "cmd_ack",
            "command": cmd_label,
            "logging_enabled": bool(payload[3]),
        }
    if payload[0] != 0xA1:
        return None
    typ = payload[1]
    if typ == 0:
        if len(payload) < 12:
            return None
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        press_pa_x10 = int.from_bytes(payload[6:10], "little", signed=True)
        temp_c_x100 = int.from_bytes(payload[10:12], "little", signed=True)
        return {
            "type": "alt",
            "t_ms": t_ms,
            "press_pa_x10": press_pa_x10,
            "temp_c_x100": temp_c_x100,
        }
    if typ == 1:
        if len(payload) < 3:
            return None
        n = payload[2]
        if len(payload) < 3 + n:
            return None
        callsign = payload[3:3 + n].decode("ascii", errors="replace")
        return {"type": "id", "callsign": callsign}
    if typ == 2:
        if len(payload) < 18:
            return None
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        lat_e7 = int.from_bytes(payload[6:10], "little", signed=True)
        lon_e7 = int.from_bytes(payload[10:14], "little", signed=True)
        height_mm = int.from_bytes(payload[14:18], "little", signed=True)
        return {
            "type": "gps",
            "t_ms": t_ms,
            "lat_e7": lat_e7,
            "lon_e7": lon_e7,
            "height_mm": height_mm,
        }
    if typ == 3:
        if len(payload) < 18:
            return None
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        gx = int.from_bytes(payload[6:8], "little", signed=True)
        gy = int.from_bytes(payload[8:10], "little", signed=True)
        gz = int.from_bytes(payload[10:12], "little", signed=True)
        ax = int.from_bytes(payload[12:14], "little", signed=True)
        ay = int.from_bytes(payload[14:16], "little", signed=True)
        az = int.from_bytes(payload[16:18], "little", signed=True)
        return {
            "type": "imu",
            "t_ms": t_ms,
            "gx": gx,
            "gy": gy,
            "gz": gz,
            "ax": ax,
            "ay": ay,
            "az": az,
        }
    if typ == 4:
        if len(payload) < 9:
            return None
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        vbat_mv = int.from_bytes(payload[6:8], "little", signed=False)
        bat_state = int(payload[8])
        return {
            "type": "bat",
            "t_ms": t_ms,
            "vbat_mv": vbat_mv,
            "bat_state": bat_state,
        }
    if typ == 5:
        if len(payload) < 10:
            return None
        t_ms = int.from_bytes(payload[2:6], "little", signed=False)
        return {
            "type": "navsat",
            "t_ms": t_ms,
            "svs_total": int(payload[6]),
            "svs_used": int(payload[7]),
            "cno_max": int(payload[8]),
            "cno_avg": int(payload[9]),
        }
    return {"type": "unknown", "type_id": typ}

def init_radio(use_hw_cs):
    # If hardware CS is unreliable, we can disable kernel-driven chip select and
    # drive GPIO8 manually as a normal output (software CS on the same pin).
    spi = SPI(
        0,
        baudrate=SPI_BAUDRATE,
        polarity=0,
        phase=0,
        no_cs=(not use_hw_cs),
    )

    nss = Pin(PIN_NSS) if use_hw_cs else Pin(PIN_NSS, Pin.OUT, value=1)
    radio = SX1278(
        spi=spi,
        nss=nss,
        rst=Pin(PIN_RST, Pin.OUT),
        dio0=Pin(PIN_DIO0, Pin.IN),
    )
    return spi, radio


def setup_radio():
    global USE_HW_CS
    use_hw_cs = USE_HW_CS
    spi, radio = init_radio(use_hw_cs)

    radio.reset()
    raw = radio.spi.xfer(bytearray([REG_VERSION & 0x7F, 0x00]))
    ver = raw[1]
    print("RegVersion raw:", raw.hex())
    print("RegVersion: 0x%02X" % ver)

    if use_hw_cs and ver not in (0x12,):
        print("RegVersion unexpected; retrying with software CS on GPIO%d" % PIN_NSS)
        try:
            spi.close()
        except Exception:
            pass
        USE_HW_CS = False
        use_hw_cs = USE_HW_CS
        spi, radio = init_radio(use_hw_cs)
        radio.reset()
        raw = radio.spi.xfer(bytearray([REG_VERSION & 0x7F, 0x00]))
        ver = raw[1]
        print("RegVersion raw:", raw.hex())
        print("RegVersion: 0x%02X" % ver)

    radio.init_rx_433()
    return spi, radio


def main():
    spi = None
    radio = None
    voltage_monitor = start_voltage_monitor()

    try:
        spi, radio = setup_radio()

        while True:
            pkt = radio.poll_packet()
            if pkt:
                payload, crc_ok, rssi, snr = pkt

                if crc_ok:
                    LED.on()
                else:
                    LED.off()

                print("RX %d bytes | CRC_OK=%s | RSSI=%d dBm | SNR=%.2f dB"
                      % (len(payload), "YES" if crc_ok else "NO", rssi, snr))
                dec = decode_payload(payload)
                if dec:
                    print("DECODE:", dec)
                print("ASCII:", safe_ascii(payload))
                print("HEX:  ", payload.hex())
                print("-" * 50)

                # LED on briefly, then off
                if crc_ok:
                    time.sleep_ms(100)
                    LED.off()

            time.sleep_ms(10)
    except KeyboardInterrupt:
        pass
    finally:
        if voltage_monitor is not None:
            voltage_monitor.stop()

        try:
            LED.off()
        except Exception:
            pass

        if radio is not None:
            try:
                radio.cs(1)
            except Exception:
                pass

        if spi is not None:
            try:
                spi.close()
            except Exception:
                pass

        try:
            GPIO.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()
