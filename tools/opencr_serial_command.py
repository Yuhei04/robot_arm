#!/usr/bin/env python3
"""Send one debug-serial command to the OpenCR check sketch and print the response."""

import argparse
import time

import serial


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send one OpenCR debug command and read the response.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port, for example /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baudrate")
    parser.add_argument("--command", required=True, help="Command to send, for example s, r, a, l")
    parser.add_argument("--startup-wait", type=float, default=2.5, help="Wait after opening serial [s]")
    parser.add_argument("--read-seconds", type=float, default=3.0, help="How long to read output [s]")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    command = args.command.strip()
    if not command:
        raise ValueError("--command must not be empty")

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        time.sleep(args.startup_wait)
        ser.reset_input_buffer()
        ser.write((command + "\n").encode("ascii"))
        ser.flush()
        deadline = time.time() + args.read_seconds
        chunks = []
        while time.time() < deadline:
            data = ser.read(4096)
            if data:
                chunks.append(data)
            else:
                time.sleep(0.05)

    output = b"".join(chunks).decode("utf-8", errors="replace")
    print(output.rstrip())


if __name__ == "__main__":
    main()
