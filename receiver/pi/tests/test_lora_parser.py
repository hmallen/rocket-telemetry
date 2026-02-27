import sys
import unittest
from unittest.mock import MagicMock

# Mock hardware dependencies before importing the module under test
sys.modules["spidev"] = MagicMock()
sys.modules["RPi"] = MagicMock()
sys.modules["RPi.GPIO"] = MagicMock()
sys.modules["smbus2"] = MagicMock()

# Add the parent directory to sys.path to import lora_receive_pi
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))

from lora_receive_pi import parse_payload, CMD_MAGIC, CMD_ACK, CMD_SD_START, CMD_SD_STOP, CMD_TELEM_ENABLE, CMD_TELEM_DISABLE, CMD_ALT_CALIBRATE, CMD_IMU_CALIBRATE, CMD_SET_TX_POWER

class TestLoraParser(unittest.TestCase):

    def test_short_payload(self):
        self.assertIsNone(parse_payload(b""))
        self.assertIsNone(parse_payload(b"\x00"))

    def test_cmd_ack_valid(self):
        # CMD_MAGIC + CMD_ACK + CMD_SD_START + Enabled(1)
        payload = bytes([CMD_MAGIC, CMD_ACK, CMD_SD_START, 1])
        result = parse_payload(payload)
        self.assertEqual(result, {
            "type": "cmd_ack",
            "command": "sd_start",
            "enabled": True,
            "logging_enabled": True
        })

    def test_cmd_ack_invalid_magic(self):
        payload = bytes([0x00, CMD_ACK, CMD_SD_START, 1])
        # Should fall through to other checks, likely returning None or unknown type
        self.assertIsNone(parse_payload(payload))

    def test_cmd_ack_invalid_ack(self):
        payload = bytes([CMD_MAGIC, 0x00, CMD_SD_START, 1])
        self.assertIsNone(parse_payload(payload))

    def test_cmd_ack_short(self):
        payload = bytes([CMD_MAGIC, CMD_ACK, CMD_SD_START])
        self.assertIsNone(parse_payload(payload))

    def test_cmd_tx_power(self):
        # CMD_MAGIC + CMD_ACK + CMD_SET_TX_POWER + Enabled(1) + Power(20)
        payload = bytes([CMD_MAGIC, CMD_ACK, CMD_SET_TX_POWER, 1, 20])
        result = parse_payload(payload)
        self.assertEqual(result, {
            "type": "cmd_ack",
            "command": "telemetry_tx_power",
            "enabled": True,
            "tx_power_dbm": 20
        })

    def test_type_0_environmental(self):
        # Type 0: [0xA1, 0x00, t_ms(4), press(4), temp(2)]
        t_ms = 1000
        press_pa_x10 = 1013250
        temp_c_x100 = 2500

        payload = (
            b'\xA1\x00' +
            t_ms.to_bytes(4, 'little', signed=False) +
            press_pa_x10.to_bytes(4, 'little', signed=True) +
            temp_c_x100.to_bytes(2, 'little', signed=True)
        )

        result = parse_payload(payload)
        self.assertEqual(result["type"], "alt")
        self.assertEqual(result["t_ms"], t_ms)
        self.assertEqual(result["press_pa_x10"], press_pa_x10)
        self.assertEqual(result["temp_c_x100"], temp_c_x100)

    def test_type_1_identity(self):
        # Type 1: [0xA1, 0x01, len(1), 'A', vbat(2), tx_power(1)]
        callsign = "A"
        vbat_mv = 4200
        tx_power = 17

        payload = (
            b'\xA1\x01' +
            len(callsign).to_bytes(1, 'little') +
            callsign.encode('ascii') +
            vbat_mv.to_bytes(2, 'little', signed=False) +
            tx_power.to_bytes(1, 'little')
        )

        result = parse_payload(payload)
        self.assertEqual(result["type"], "id")
        self.assertEqual(result["callsign"], callsign)
        self.assertEqual(result["vbat_mv"], vbat_mv)
        self.assertAlmostEqual(result["vbat_v"], 4.2)
        self.assertEqual(result["tx_power_dbm"], tx_power)

    def test_type_2_gps(self):
        # Type 2: [0xA1, 0x02, t_ms(4), lat(4), lon(4), alt(4)]
        t_ms = 1000
        lat_e7 = 377749000
        lon_e7 = -1224194000
        height_mm = 100000

        payload = (
            b'\xA1\x02' +
            t_ms.to_bytes(4, 'little', signed=False) +
            lat_e7.to_bytes(4, 'little', signed=True) +
            lon_e7.to_bytes(4, 'little', signed=True) +
            height_mm.to_bytes(4, 'little', signed=True)
        )

        result = parse_payload(payload)
        self.assertEqual(result["type"], "gps")
        self.assertAlmostEqual(result["lat_e7"], lat_e7)
        self.assertAlmostEqual(result["lon_e7"], lon_e7)
        self.assertEqual(result["height_mm"], height_mm)

    def test_type_3_imu(self):
        # Type 3: [0xA1, 0x03, t_ms(4), gx(2), gy(2), gz(2), ax(2), ay(2), az(2)]
        t_ms = 1000
        gx, gy, gz = 1, 2, 3
        ax, ay, az = 4, 5, 6

        payload = (
            b'\xA1\x03' +
            t_ms.to_bytes(4, 'little', signed=False) +
            gx.to_bytes(2, 'little', signed=True) +
            gy.to_bytes(2, 'little', signed=True) +
            gz.to_bytes(2, 'little', signed=True) +
            ax.to_bytes(2, 'little', signed=True) +
            ay.to_bytes(2, 'little', signed=True) +
            az.to_bytes(2, 'little', signed=True)
        )

        result = parse_payload(payload)
        self.assertEqual(result["type"], "imu")
        self.assertEqual(result["gx"], gx)
        self.assertEqual(result["gy"], gy)
        self.assertEqual(result["gz"], gz)
        self.assertEqual(result["ax"], ax)
        self.assertEqual(result["ay"], ay)
        self.assertEqual(result["az"], az)

    def test_type_4_battery(self):
        # Type 4: [0xA1, 0x04, t_ms(4), vbat(2), state(1)]
        t_ms = 1000
        vbat_mv = 4200
        bat_state = 1

        payload = (
            b'\xA1\x04' +
            t_ms.to_bytes(4, 'little', signed=False) +
            vbat_mv.to_bytes(2, 'little', signed=False) +
            bat_state.to_bytes(1, 'little')
        )

        result = parse_payload(payload)
        self.assertEqual(result["type"], "bat")
        self.assertEqual(result["vbat_mv"], vbat_mv)
        self.assertEqual(result["bat_state"], bat_state)

    def test_type_5_navsat(self):
        # Type 5: [0xA1, 0x05, t_ms(4), total(1), used(1), max(1), avg(1)]
        t_ms = 1000
        svs_total = 12
        svs_used = 8
        cno_max = 32
        cno_avg = 30

        payload = (
            b'\xA1\x05' +
            t_ms.to_bytes(4, 'little', signed=False) +
            svs_total.to_bytes(1, 'little') +
            svs_used.to_bytes(1, 'little') +
            cno_max.to_bytes(1, 'little') +
            cno_avg.to_bytes(1, 'little')
        )

        result = parse_payload(payload)
        self.assertEqual(result["type"], "navsat")
        self.assertEqual(result["svs_total"], svs_total)
        self.assertEqual(result["svs_used"], svs_used)
        self.assertEqual(result["cno_max"], cno_max)
        self.assertEqual(result["cno_avg"], cno_avg)

    def test_type_6_recovery(self):
        # Type 6: [0xA1, 0x06, t_ms(4), phase(1), flags(1), agl(4), max_agl(4), vspeed(2), drogue_agl(4), main_agl(4), drogue_r(1), main_r(1)]
        t_ms = 1000
        phase = 1
        flags = 0
        agl_mm = 50000
        max_agl_mm = 100000
        vspeed_cms = 1000
        drogue_agl_mm = -1
        main_agl_mm = -1
        drogue_reason = 0
        main_reason = 0

        payload = (
            b'\xA1\x06' +
            t_ms.to_bytes(4, 'little', signed=False) +
            phase.to_bytes(1, 'little') +
            flags.to_bytes(1, 'little') +
            agl_mm.to_bytes(4, 'little', signed=True) +
            max_agl_mm.to_bytes(4, 'little', signed=True) +
            vspeed_cms.to_bytes(2, 'little', signed=True) +
            drogue_agl_mm.to_bytes(4, 'little', signed=True) +
            main_agl_mm.to_bytes(4, 'little', signed=True) +
            drogue_reason.to_bytes(1, 'little') +
            main_reason.to_bytes(1, 'little')
        )

        result = parse_payload(payload)
        self.assertEqual(result["type"], "recovery")
        self.assertEqual(result["phase_code"], 1)
        self.assertEqual(result["phase"], "ascent")
        self.assertEqual(result["altitude_agl_m"], 50.0)
        self.assertEqual(result["max_altitude_agl_m"], 100.0)
        self.assertEqual(result["vertical_speed_mps"], 10.0)
        self.assertIsNone(result["drogue_deploy_alt_agl_m"])
        self.assertIsNone(result["main_deploy_alt_agl_m"])

if __name__ == '__main__':
    unittest.main()
