import argparse
import importlib
import time


def _get_board_pin(board_module, pin_name: str):
    try:
        return getattr(board_module, pin_name)
    except AttributeError as e:
        raise SystemExit(f"Unknown pin '{pin_name}'. Try values like CE0, CE1, D25, D22, etc.") from e


def main() -> int:
    parser = argparse.ArgumentParser(description="Receive LoRa packets and print them.")
    parser.add_argument("--freq-mhz", type=float, default=433.0)
    parser.add_argument("--cs", default="CE1")
    parser.add_argument("--reset", default="D25")
    parser.add_argument("--sf", type=int, default=9)
    parser.add_argument("--bw-khz", type=float, default=125.0)
    parser.add_argument("--cr", type=int, default=5)
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--print-hex", action="store_true")
    args = parser.parse_args()

    try:
        board = importlib.import_module("board")
        busio = importlib.import_module("busio")
        digitalio = importlib.import_module("digitalio")
        adafruit_rfm9x = importlib.import_module("adafruit_rfm9x")
    except ImportError as e:
        raise SystemExit(
            "Missing Raspberry Pi dependencies. Install with:\n"
            "  python3 -m pip install -r receiver/pi/requirements.txt\n"
            "Then enable SPI and reboot if needed."
        ) from e

    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

    cs = digitalio.DigitalInOut(_get_board_pin(board, args.cs))
    reset = digitalio.DigitalInOut(_get_board_pin(board, args.reset))

    rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, args.freq_mhz)
    rfm9x.enable_crc = True
    rfm9x.spreading_factor = args.sf
    rfm9x.signal_bandwidth = int(args.bw_khz * 1000)
    rfm9x.coding_rate = args.cr

    print(
        "listening: "
        f"freq={args.freq_mhz:.3f}MHz sf={args.sf} bw={args.bw_khz:.1f}kHz cr=4/{args.cr} crc=on "
        f"cs={args.cs} reset={args.reset}"
    )

    while True:
        pkt = rfm9x.receive(timeout=args.timeout_s)
        if pkt is None:
            continue

        ts = time.time()
        rssi = rfm9x.last_rssi

        try:
            pkt_bytes = bytes(pkt)
            text = pkt_bytes.decode("ascii", errors="replace")
        except (TypeError, UnicodeDecodeError):
            text = ""

        header = f"[{ts:.3f}] rssi={rssi} len={len(pkt)}"
        if text:
            print(f"{header} text={text.rstrip()}" )
        else:
            print(header)

        if args.print_hex:
            print(pkt.hex())


if __name__ == "__main__":
    raise SystemExit(main())
