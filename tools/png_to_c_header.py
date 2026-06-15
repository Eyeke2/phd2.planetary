#!/usr/bin/env python3
#
# Convert a PNG image to the C byte-array header format used by PHD2 icons.
#

import argparse
import re
from pathlib import Path


def symbol_name(path: Path) -> str:
    name = re.sub(r"\W+", "_", path.stem).strip("_")
    if not name:
        name = "image"
    if name[0].isdigit():
        name = "_" + name
    return name


def write_header(input_path: Path, output_path: Path, symbol: str) -> None:
    data = input_path.read_bytes()

    with output_path.open("w", encoding="ascii", newline="\n") as output:
        output.write(f"/* {input_path.name} - {len(data)} bytes */\n")
        output.write(f"static const unsigned char {symbol}_png[] = {{\n")

        for offset in range(0, len(data), 8):
            chunk = data[offset : offset + 8]
            values = ", ".join(f"0x{byte:02x}" for byte in chunk)
            suffix = "," if offset + 8 < len(data) else ""
            output.write(f"  {values}{suffix}\n")

        output.write("};\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert a PNG file to a PHD2 C byte-array header.")
    parser.add_argument("png", type=Path, help="input PNG file")
    parser.add_argument("-o", "--output", type=Path, help="output header path; defaults to <png>.h")
    parser.add_argument("-n", "--name", help="base C symbol name; defaults to the PNG file stem")
    args = parser.parse_args()

    input_path = args.png
    if input_path.suffix.lower() != ".png":
        parser.error("input file must have a .png extension")
    if not input_path.is_file():
        parser.error(f"input file not found: {input_path}")

    output_path = args.output or input_path.with_suffix(input_path.suffix + ".h")
    symbol = args.name or symbol_name(input_path)
    write_header(input_path, output_path, symbol)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
