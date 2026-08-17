"""Create package LVIM resources from the RFP200 SquareLine C image arrays."""

from __future__ import annotations

import argparse
import re
import struct
from pathlib import Path


IMAGE_SOURCES = {
    "battery": "ui_img_battery_png.c",
    "controller": "ui_img_controller_png.c",
    "motor": "ui_img_motor_png.c",
    "eco": "ui_img_eco_icon_png.c",
    "race": "ui_img_race_icon_png.c",
}


def parse_image(source: Path) -> tuple[int, int, bytes]:
    text = source.read_text(encoding="utf-8")
    array_match = re.search(r"_data\[\]\s*=\s*\{(.*?)\n\};", text, re.S)
    width_match = re.search(r"\.header\.w\s*=\s*(\d+)", text)
    height_match = re.search(r"\.header\.h\s*=\s*(\d+)", text)
    if not array_match or not width_match or not height_match:
        raise ValueError(f"Could not parse {source}")
    pixels = bytes(int(value, 16) for value in re.findall(
        r"0x([0-9a-fA-F]{1,2})", array_match.group(1)))
    width = int(width_match.group(1))
    height = int(height_match.group(1))
    expected = width * height * 3
    if len(pixels) != expected:
        raise ValueError(
            f"{source.name}: got {len(pixels)} bytes, expected {expected}")
    return width, height, pixels


def write_lvim(destination: Path, width: int, height: int,
               pixels: bytes) -> None:
    # 0..3 magic, 4 version, 5 format (1 = RGB565A8), 6..7 reserved,
    # 8..9 width, 10..11 height, 12..15 payload size; all integers LE.
    header = struct.pack("<4sBBHHHI", b"LVIM", 1, 1, 0,
                         width, height, len(pixels))
    destination.write_bytes(header + pixels)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rfp200", type=Path, default=Path(r"C:\esp\RFP200"))
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parents[1] / "assets")
    args = parser.parse_args()

    source_dir = (args.rfp200 / "components" / "ui" / "src" /
                  "assets" / "images")
    args.output.mkdir(parents=True, exist_ok=True)
    for output_name, source_name in IMAGE_SOURCES.items():
        width, height, pixels = parse_image(source_dir / source_name)
        write_lvim(args.output / f"{output_name}.lvim",
                   width, height, pixels)
        print(f"{output_name}.lvim: {width}x{height}, {len(pixels)} bytes")

    # Transparent source used to hide a previously-created profile icon while
    # controller synchronization is unverified.
    write_lvim(args.output / "blank.lvim", 1, 1, b"\x00\x00\x00")
    print("blank.lvim: 1x1, 3 bytes")


if __name__ == "__main__":
    main()
