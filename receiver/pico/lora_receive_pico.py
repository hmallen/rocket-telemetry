# MicroPython (Raspberry Pi Pico / RP2040) SX1278 (Ra-02 style) RX script
# Lights the Pico on-board LED whenever a packet is received successfully (CRC OK).
#
# Wiring:
#   Ra-02 VCC   -> Pico 3V3(OUT)
#   Ra-02 GND   -> Pico GND
#   Ra-02 SCK   -> GP18
#   Ra-02 MOSI  -> GP19
#   Ra-02 MISO  -> GP16
#   Ra-02 NSS   -> GP17
#   Ra-02 RESET-> GP20
#   Ra-02 DIO0  -> GP21

from machine import SPI, Pin
import time

# -------- Pico pin mapping --------
PIN_SCK   = 18
PIN_MOSI  = 19
PIN_MISO  = 16
PIN_NSS   = 17
PIN_RST   = 20
PIN_DIO0  = 21

# Pico on-board LED
LED = Pin("LED", Pin.OUT)
LED.off()

# -------- SX127x register map (subset) --------
REG_FIFO                    = 0x00
REG_OP_MODE                 = 0x01
REG_FRF_MSB                 = 0x06
REG_FRF_MID                 = 0x07
REG_FRF_LSB                 = 0x08
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

# -------- Modes / bits --------
LONG_RANGE_MODE = 0x80
MODE_SLEEP      = 0x00
MODE_STDBY      = 0x01
MODE_RX_CONT    = 0x05

IRQ_RX_DONE     = 0x40
IRQ_CRC_ERR     = 0x20

SYNCWORD_LORA_PUBLIC = 0x34

class SX1278:
    def __init__(self, spi, nss, rst, dio0):
        self.spi = spi
        self.nss = nss
        self.rst = rst
        self.dio0 = dio0

        self.nss.init(Pin.OUT, value=1)
        self.rst.init(Pin.OUT, value=1)
        self.dio0.init(Pin.IN)

    def cs(self, level):
        self.nss.value(level)

    def reset(self):
        self.rst.value(0)
        time.sleep_ms(50)
        self.rst.value(1)
        time.sleep_ms(50)

    def read_reg(self, addr):
        self.cs(0)
        self.spi.write(bytearray([addr & 0x7F]))
        val = self.spi.read(1, 0x00)[0]
        self.cs(1)
        return val

    def write_reg(self, addr, val):
        self.cs(0)
        self.spi.write(bytearray([addr | 0x80, val & 0xFF]))
        self.cs(1)

    def read_fifo(self, n):
        self.cs(0)
        self.spi.write(bytearray([REG_FIFO & 0x7F]))
        data = self.spi.read(n, 0x00)
        self.cs(1)
        return data

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

def safe_ascii(b):
    return "".join(chr(x) if 32 <= x <= 126 else "." for x in b)

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

radio.init_rx_433()

# -------- main loop --------
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
        print("ASCII:", safe_ascii(payload))
        print("HEX:  ", payload.hex())
        print("-" * 50)

        # LED on briefly, then off
        if crc_ok:
            time.sleep_ms(100)
            LED.off()

    time.sleep_ms(10)
