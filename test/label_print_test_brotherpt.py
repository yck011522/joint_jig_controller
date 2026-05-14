"""Diagnostic print test for Brother PT-P750W using brother_pt library.

This test prints simple diagnostic patterns (checkerboard, grid) to verify
the printable area, tape width, and tape length settings. Use this to test
different tape configurations before deploying label rendering.

Hardware setup: WinUSB driver via Zadig, USB cable, Editor Lite LED off.
"""

from __future__ import annotations

import os
import platform
import sys
from pathlib import Path


# --------------------------------------------------------------------- #
# libusb backend bootstrap (Windows)                                    #
# --------------------------------------------------------------------- #

def _bootstrap_libusb_backend():
    """Locate the libusb DLL (Windows) and register it with PyUSB."""
    if sys.platform != "win32":
        try:
            import usb.backend.libusb1  # noqa: F401
            import usb.core  # noqa: F401
            return None
        except Exception:
            return None

    try:
        import libusb  # type: ignore
    except Exception:
        return None

    arch = "x86_64" if platform.machine().endswith("64") else "x86"
    dll_dir = Path(libusb.__file__).parent / "_platform" / "windows" / arch
    dll_path = dll_dir / "libusb-1.0.dll"
    if not dll_path.exists():
        return None

    try:
        os.add_dll_directory(str(dll_dir))
    except (AttributeError, FileNotFoundError, OSError):
        pass
    os.environ["PATH"] = str(dll_dir) + os.pathsep + os.environ.get("PATH", "")

    try:
        import usb.backend.libusb1
        import usb.core
    except Exception:
        return None

    backend = usb.backend.libusb1.get_backend(
        find_library=lambda _: str(dll_path)
    )
    if backend is None:
        return None

    _orig_find = usb.core.find

    def _find_with_backend(*args, **kwargs):
        kwargs.setdefault("backend", backend)
        return _orig_find(*args, **kwargs)

    usb.core.find = _find_with_backend  # type: ignore[assignment]
    usb.core.Device.is_kernel_driver_active = lambda self, intf: False  # type: ignore
    usb.core.Device.detach_kernel_driver = lambda self, intf: None  # type: ignore
    return backend


_bootstrap_libusb_backend()

try:
    from PIL import Image, ImageDraw  # type: ignore
except Exception as exc:
    print(f"ERROR: Pillow not available: {exc}", file=sys.stderr)
    raise SystemExit(1)

try:
    from brother_pt.printer import BrotherPt  # noqa: E402
    from brother_pt.cmd import MediaWidthToTapeMargin  # noqa: E402
except Exception as exc:
    print(f"ERROR: brother_pt not available: {exc}", file=sys.stderr)
    raise SystemExit(1)


# --------------------------------------------------------------------- #
# Diagnostic pattern generation                                         #
# --------------------------------------------------------------------- #

def _mm_to_dots(mm: float, dpi: int = 180) -> int:
    """Convert millimeters to dots at 180 DPI."""
    return int(round(mm * dpi / 25.4))


def checkerboard(width_px: int, height_px: int, square_size: int = 5) -> Image.Image:
    """Generate a checkerboard pattern for printable area verification."""
    img = Image.new("L", (width_px, height_px), 255)
    draw = ImageDraw.Draw(img)
    for y in range(0, height_px, square_size):
        for x in range(0, width_px, square_size):
            if ((x // square_size) + (y // square_size)) % 2 == 0:
                draw.rectangle([x, y, x + square_size - 1, y + square_size - 1], fill=0)
    return img


def grid(width_px: int, height_px: int, grid_spacing: int = 10) -> Image.Image:
    """Generate a grid pattern with labels for dimension reference."""
    img = Image.new("L", (width_px, height_px), 255)
    draw = ImageDraw.Draw(img)
    # Vertical lines
    for x in range(0, width_px, grid_spacing):
        draw.line([(x, 0), (x, height_px)], fill=0, width=1)
    # Horizontal lines
    for y in range(0, height_px, grid_spacing):
        draw.line([(0, y), (width_px, y)], fill=0, width=1)
    return img


def solid_black(width_px: int, height_px: int) -> Image.Image:
    """Generate a solid black rectangle to test full coverage."""
    return Image.new("L", (width_px, height_px), 0)


def border(width_px: int, height_px: int, border_width: int = 5) -> Image.Image:
    """Generate a border pattern to verify edges."""
    img = Image.new("L", (width_px, height_px), 255)
    draw = ImageDraw.Draw(img)
    draw.rectangle(
        [(0, 0), (width_px - 1, height_px - 1)],
        outline=0, width=border_width
    )
    return img


# --------------------------------------------------------------------- #
# Configuration (tunable for different tape sizes)                      #
# --------------------------------------------------------------------- #

# Tape geometry
TAPE_WIDTH_MM = 12.0  # Change to 9, 18, 24 for different tape sizes
TAPE_LENGTH_MM = 75.0  # Tunable: longer/shorter test strips

# Printing constants
DPI = 180
CUT_MARGIN_DOTS = 14  # ~2 mm trailing margin


# --------------------------------------------------------------------- #
# Main                                                                  #
# --------------------------------------------------------------------- #

def main() -> int:
    """Print a diagnostic pattern to the Brother PT printer."""
    print("=" * 60)
    print("Brother PT-P750W Diagnostic Pattern Test")
    print("=" * 60)
    print()

    print("Connecting to printer...")
    try:
        printer = BrotherPt()
    except Exception as exc:
        print(f"ERROR: Failed to connect: {exc}", file=sys.stderr)
        return 1

    print(f"Connected successfully.")
    print(f"  Media width: {printer.media_width} mm")
    print(f"  Media type: {getattr(printer.media_type, 'name', '?')}")
    print(f"  Tape color: {getattr(printer.tape_color, 'name', '?')}")
    print()

    # Get printable height based on detected tape width
    if printer.media_width not in MediaWidthToTapeMargin.margin:
        print(
            f"ERROR: Tape width {printer.media_width} mm not supported by brother_pt.",
            file=sys.stderr
        )
        return 1

    print_height_px = int(MediaWidthToTapeMargin.to_print_width(printer.media_width))
    print_width_px = _mm_to_dots(TAPE_LENGTH_MM)

    print(f"Tape configuration:")
    print(f"  Configured width: {TAPE_WIDTH_MM} mm")
    print(f"  Detected width:   {printer.media_width} mm")
    print(f"  Printable height: {print_height_px} px")
    print(f"  Print length:     {TAPE_LENGTH_MM} mm ({print_width_px} px @ {DPI} dpi)")
    print()

    # Ask which pattern to print
    print("Available patterns:")
    print("  1) Checkerboard (5x5 px squares)")
    print("  2) Grid (10 px spacing)")
    print("  3) Solid black")
    print("  4) Border")
    print()

    try:
        choice = input("Select pattern [1-4, default 1]: ").strip()
    except EOFError:
        choice = "1"

    if not choice:
        choice = "1"

    pattern_map = {
        "1": ("checkerboard", checkerboard),
        "2": ("grid", grid),
        "3": ("solid", solid_black),
        "4": ("border", border),
    }

    if choice not in pattern_map:
        print(f"ERROR: Invalid choice: {choice}", file=sys.stderr)
        return 1

    pattern_name, pattern_func = pattern_map[choice]
    print()
    print(f"Generating {pattern_name} pattern...")
    img = pattern_func(print_width_px, print_height_px)

    # Save preview
    preview_path = Path(__file__).with_name("diagnostic_preview.png")
    img.save(preview_path)
    print(f"Preview saved to {preview_path}")
    print()

    # Ask to print
    try:
        confirm = input("Print to device now? [y/N]: ").strip().lower()
    except EOFError:
        confirm = ""

    if confirm not in {"y", "yes"}:
        print("Skipping print.")
        return 0

    print("Sending to printer...")
    try:
        printer.print_image(img, margin_px=CUT_MARGIN_DOTS)
    except Exception as exc:
        print(f"ERROR: Print failed: {exc}", file=sys.stderr)
        return 1

    print("Print job complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
