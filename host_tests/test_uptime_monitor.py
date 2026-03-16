#!/usr/bin/env python3
import time, os, signal, sys, subprocess
from smbus2 import SMBus

BUS = 1
ADDR = 0x49  # set to the responding address from probe

# Set these based on probe results:
CFG_VIN  = 0b11000001  # replace with CH* cfg that tracks input
CFG_VOUT = 0b11010001  # replace with CH* cfg that sits near ~5V
CFG_VBAT = 0b11100001  # replace with CH* cfg that sits 3.0–4.2V
CFG_NTC  = 0b11110001  # replace with CH* cfg that behaves like thermistor

VREF = 6.144
MAX_READING = 2047.0
TIEMPO = 0.10
PERIOD = 2.0

VIN_MIN  = 3.8
VBAT_MIN = 3.1  # shutdown threshold (battery), matches manufacturer intent :contentReference[oaicite:5]{index=5}

def read_volts(bus, addr, cfg):
    bus.write_i2c_block_data(addr, 0x01, [0x85, 0x83])
    bus.write_i2c_block_data(addr, 0x00, [0x00, 0x00])
    time.sleep(TIEMPO)
    bus.write_i2c_block_data(addr, 0x01, [cfg, 0x43])
    time.sleep(TIEMPO)
    reading = bus.read_word_data(addr, 0x00)
    valor = (((reading & 0xFF) << 8) | ((int(reading) & 0xFFF0) >> 8))
    valor >>= 4
    return (valor / MAX_READING) * VREF

def handle_exit(*_):
    raise SystemExit(0)

signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

def main():
    with SMBus(BUS) as bus:
        while True:
            vin  = read_volts(bus, ADDR, CFG_VIN)
            vout = read_volts(bus, ADDR, CFG_VOUT)
            vbat = read_volts(bus, ADDR, CFG_VBAT)
            ntc  = read_volts(bus, ADDR, CFG_NTC)

            # If this is PiZ-UpTime, their line is TempC=(4.0-TempV)/0.0432 :contentReference[oaicite:6]{index=6}
            temp_c = (4.0 - ntc) / 0.0432
            temp_f = temp_c * 1.8 + 32.0

            print(f"{time.ctime()}  Vin={vin:5.2f}V  Vout={vout:5.2f}V  Vbat={vbat:5.2f}V  Temp={temp_c:6.2f}C {temp_f:6.2f}F")
            sys.stdout.flush()

            # Correct shutdown logic: input failed AND battery is low. :contentReference[oaicite:7]{index=7}
            if vin < VIN_MIN and vbat < VBAT_MIN:
                subprocess.call("shutdown -h now &", shell=True)
                time.sleep(2)
                raise SystemExit(0)

            time.sleep(PERIOD)

if __name__ == "__main__":
    main()
