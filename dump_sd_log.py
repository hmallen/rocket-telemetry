#!/usr/bin/env python3
"""Dump Rocket Telemetry SD log blocks and records to a text file.

Usage examples:
  python dump_sd_log.py --file E:\\LOG00012.BIN
  python dump_sd_log.py --file E:\\ --output flight.txt
  python dump_sd_log.py --file E:\\ --stdout    # also echo to terminal
"""

from __future__ import annotations

import argparse
import binascii
import struct
import sys
from pathlib import Path

BLOCK_MAGIC = 0x424D4C54  # 'TLMB'
BLOCK_HDR_FMT = "<IHHIIII"
BLOCK_HDR_SIZE = struct.calcsize(BLOCK_HDR_FMT)

REC_IMU_FAST = 0x01
REC_BARO = 0x02
REC_GNSS_CHUNK = 0x03
REC_TIME_ANCHOR = 0x04
REC_EVENT = 0x05
REC_STATS = 0x06
REC_BARO2 = 0x07

REC_TYPE_NAMES = {
    REC_IMU_FAST: "REC_IMU_FAST",
    REC_BARO: "REC_BARO",
    REC_GNSS_CHUNK: "REC_GNSS_CHUNK",
    REC_TIME_ANCHOR: "REC_TIME_ANCHOR",
    REC_EVENT: "REC_EVENT",
    REC_STATS: "REC_STATS",
    REC_BARO2: "REC_BARO2",
}


def _safe_ascii(data: bytes) -> str:
    return "".join(chr(b) if 32 <= b <= 126 else "." for b in data)


def _pick_log_file(path: Path) -> Path:
    if path.is_file():
        return path
    if not path.is_dir():
        raise FileNotFoundError(f"Path not found: {path}")

    logs = sorted(path.glob("LOG*.BIN"))
    if not logs:
        raise FileNotFoundError(f"No LOG*.BIN files found in {path}")
    return logs[-1]


def _read_block_header(handle):
    hdr_bytes = handle.read(BLOCK_HDR_SIZE)
    if len(hdr_bytes) == 0:
        return None
    if len(hdr_bytes) < BLOCK_HDR_SIZE:
        raise EOFError("Truncated block header")

    magic, ver, hdr_len, seq, t_start_us, payload_len, crc32 = struct.unpack(
        BLOCK_HDR_FMT, hdr_bytes
    )
    if hdr_len < BLOCK_HDR_SIZE:
        raise ValueError(f"Invalid block header length: {hdr_len}")
    if hdr_len > BLOCK_HDR_SIZE:
        extra = handle.read(hdr_len - BLOCK_HDR_SIZE)
        if len(extra) < hdr_len - BLOCK_HDR_SIZE:
            raise EOFError("Truncated block header padding")
    return {
        "magic": magic,
        "ver": ver,
        "hdr_len": hdr_len,
        "seq": seq,
        "t_start_us": t_start_us,
        "payload_len": payload_len,
        "crc32": crc32,
    }


def _decode_record(rec_type, rec_ver, rec_len, rec_payload):
    name = REC_TYPE_NAMES.get(rec_type, f"UNKNOWN_0x{rec_type:02X}")
    base = {"type": name, "ver": rec_ver, "len": rec_len}

    try:
        if rec_type == REC_IMU_FAST and rec_len >= 24:
            t_us, ax, ay, az, gx, gy, gz, temp, status = struct.unpack_from(
                "<IhhhhhhhH", rec_payload, 0
            )
            base.update(
                {
                    "t_us": t_us,
                    "ax": ax,
                    "ay": ay,
                    "az": az,
                    "gx": gx,
                    "gy": gy,
                    "gz": gz,
                    "temp": temp,
                    "status": status,
                }
            )
        elif rec_type == REC_BARO and rec_len >= 16:
            t_us, press_pa_x10, temp_c_x100, status = struct.unpack_from(
                "<IihH", rec_payload, 0
            )
            base.update(
                {
                    "t_us": t_us,
                    "press_pa_x10": press_pa_x10,
                    "temp_c_x100": temp_c_x100,
                    "status": status,
                }
            )
        elif rec_type == REC_BARO2 and rec_len >= 16:
            t_us, press_pa_x10, temp_c_x100, status = struct.unpack_from(
                "<IihH", rec_payload, 0
            )
            base.update(
                {
                    "t_us": t_us,
                    "press_pa_x10": press_pa_x10,
                    "temp_c_x100": temp_c_x100,
                    "status": status,
                }
            )
        elif rec_type == REC_GNSS_CHUNK and rec_len >= 10:
            t_us, n = struct.unpack_from("<IH", rec_payload, 0)
            data = rec_payload[6:6 + n]
            base.update(
                {
                    "t_us": t_us,
                    "n": n,
                    "ascii": _safe_ascii(data),
                    "hex": data.hex(),
                }
            )
        elif rec_type == REC_TIME_ANCHOR and rec_len >= 18:
            t_us_at_pps, gps_week, tow_ms, fix_ok = struct.unpack_from(
                "<IHIb", rec_payload, 0
            )
            base.update(
                {
                    "t_us_at_pps": t_us_at_pps,
                    "gps_week": gps_week,
                    "tow_ms": tow_ms,
                    "fix_ok": fix_ok,
                }
            )
        elif rec_type == REC_EVENT and rec_len >= 12:
            t_us, event_id, value = struct.unpack_from("<IHh", rec_payload, 0)
            base.update({"t_us": t_us, "event_id": event_id, "value": value})
        elif rec_type == REC_STATS and rec_len >= 24:
            t_us, ring_drops, spool_drops, sd_write_errs, vbat_mv, bat_state, _ = struct.unpack_from(
                "<IIIIHBB", rec_payload, 0
            )
            base.update(
                {
                    "t_us": t_us,
                    "ring_drops": ring_drops,
                    "spool_drops": spool_drops,
                    "sd_write_errs": sd_write_errs,
                    "vbat_mv": vbat_mv,
                    "bat_state": bat_state,
                }
            )
    except struct.error:
        base["decode_error"] = True

    return base


def _dump_log(
    path: Path,
    output_handle,
    max_blocks: int | None,
    show_unknown: bool,
    also_stdout: bool,
) -> None:
    def emit(line: str = "") -> None:
        output_handle.write(f"{line}\n")
        if also_stdout:
            print(line)

    with path.open("rb") as handle:
        block_index = 0
        while True:
            header = _read_block_header(handle)
            if header is None:
                break

            if header["magic"] != BLOCK_MAGIC:
                if header["magic"] == 0:
                    emit("Reached empty preallocated region; stopping.")
                    break
                raise ValueError(
                    f"Invalid block magic 0x{header['magic']:08X} at block {block_index}"
                )

            payload = handle.read(header["payload_len"])
            if len(payload) < header["payload_len"]:
                raise EOFError("Truncated block payload")

            computed_crc = binascii.crc32(payload) & 0xFFFFFFFF
            crc_ok = computed_crc == header["crc32"]

            emit(
                "Block seq={seq} t_start_us={t_start_us} payload_len={payload_len} crc_ok={crc_ok}".format(
                    seq=header["seq"],
                    t_start_us=header["t_start_us"],
                    payload_len=header["payload_len"],
                    crc_ok="YES" if crc_ok else "NO",
                )
            )

            offset = 0
            rec_index = 0
            while offset + 4 <= len(payload):
                rec_type, rec_ver, rec_len = struct.unpack_from("<BBH", payload, offset)
                if rec_len < 4 or offset + rec_len > len(payload):
                    emit(f"  Invalid record length at offset {offset}; stopping block")
                    break

                rec_payload = payload[offset + 4 : offset + rec_len]
                record = _decode_record(rec_type, rec_ver, rec_len, rec_payload)
                name = record.get("type", "UNKNOWN")

                emit(f"  [{rec_index:04d}] {name} ver={rec_ver} len={rec_len}")
                for key, value in record.items():
                    if key in {"type", "ver", "len"}:
                        continue
                    emit(f"        {key}: {value}")

                if name.startswith("UNKNOWN") and show_unknown:
                    preview = rec_payload[:32]
                    emit(f"        raw_hex: {preview.hex()}")

                offset += rec_len
                rec_index += 1

            block_index += 1
            if max_blocks is not None and block_index >= max_blocks:
                emit("Max block limit reached; stopping.")
                break


def _output_path_for(log_path: Path, output_arg: str | None) -> Path:
    if output_arg:
        return Path(output_arg)
    return log_path.with_suffix(".txt")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Parse SD telemetry log records and write a human-readable dump."
    )
    parser.add_argument(
        "--file",
        "-f",
        default=".",
        help="Path to LOGxxxxx.BIN or directory containing logs (default: current directory).",
    )
    parser.add_argument(
        "--max-blocks",
        type=int,
        default=None,
        help="Stop after dumping this many blocks.",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output text file path (default: same name as log with .txt extension).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite output file if it already exists.",
    )
    parser.add_argument(
        "--stdout",
        action="store_true",
        help="Also echo the dump to stdout while writing the file.",
    )
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help="Do not wait for Enter before starting.",
    )
    parser.add_argument(
        "--show-unknown",
        action="store_true",
        help="Include a raw hex preview for unknown record types.",
    )

    args = parser.parse_args()
    try:
        target = _pick_log_file(Path(args.file))
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 2

    output_path = _output_path_for(target, args.output)
    if output_path.exists() and not args.overwrite:
        print(f"Output file already exists: {output_path}", file=sys.stderr)
        print("Use --overwrite to replace it.", file=sys.stderr)
        return 2

    if not args.no_wait:
        try:
            input("Insert SD card and press Enter to dump telemetry log... ")
        except KeyboardInterrupt:
            print("\nCanceled.")
            return 1

    print(f"Dumping: {target}")
    try:
        with output_path.open("w", encoding="utf-8") as output_handle:
            _dump_log(target, output_handle, args.max_blocks, args.show_unknown, args.stdout)
    except (EOFError, ValueError, struct.error) as exc:
        print(f"Error while parsing log: {exc}", file=sys.stderr)
        return 2

    print(f"Wrote dump to: {output_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
