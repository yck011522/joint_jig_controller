"""Test print using the ``brother_pt`` library (treideme/brother_pt).

This is an alternative to ``label_print_test.py`` (which uses the
``ptouch`` library). ``brother_pt`` is image-only: it expects a
pre-rendered PIL image whose **height** equals the printable pin count
for the loaded tape, and a width that determines how long the printed
label will be.

Why try this library?
---------------------
The previous attempt (``ptouch``) accepted the print job over USB but
the printer just blinked red and produced nothing. ``brother_pt``
explicitly queries the printer status before and after the job and
decodes the printer's error bits (no media / cutter jam / cover open /
wrong media / overheating / ...). If the hardware is unhappy we will at
least get a useful error message instead of silent failure.

Tape geometry (constants come from ``brother_pt/cmd.py``)
---------------------------------------------------------
PRINT_HEAD_PINS = 128 at 180 dpi. Per-tape margins (unprintable pins on
each edge):

    9 mm  -> margin 39  -> usable height = 128 - 78 = 50 px
    12 mm -> margin 29  -> usable height = 128 - 58 = 70 px

For a 20 mm diameter tube (circumference ~62.8 mm) plus 15 mm overlap
the label needs to be about 77.8 mm long. At 180 dpi that's
~551 horizontal pixels.

Hardware setup is the same as for ``label_print_test.py``: WinUSB
driver bound to the printer via Zadig, USB cable connected, Editor Lite
LED off, mode switch on USB.
"""

from __future__ import annotations

import math
import os
import platform
import sys
from pathlib import Path


# --------------------------------------------------------------------- #
# libusb backend bootstrap (Windows)                                    #
# --------------------------------------------------------------------- #
# Same trick as in label_print_test.py: locate the DLL bundled by the
# ``libusb`` pip package, register it as PyUSB's backend, and stub the
# kernel-driver methods that don't exist on Windows.
def _bootstrap_libusb_backend():
    if sys.platform != "win32":
        return None
    import libusb  # type: ignore

    arch = "x86_64" if platform.machine().endswith("64") else "x86"
    dll_dir = Path(libusb.__file__).parent / "_platform" / "windows" / arch
    dll_path = dll_dir / "libusb-1.0.dll"
    if not dll_path.exists():
        raise RuntimeError(f"libusb DLL not found at {dll_path}")

    os.add_dll_directory(str(dll_dir))
    os.environ["PATH"] = str(dll_dir) + os.pathsep + os.environ.get("PATH", "")

    import usb.backend.libusb1
    import usb.core

    backend = usb.backend.libusb1.get_backend(find_library=lambda _: str(dll_path))
    if backend is None:
        raise RuntimeError(
            "Failed to load libusb backend. Make sure WinUSB is bound to "
            "the printer (use Zadig)."
        )

    _orig_find = usb.core.find

    def _find_with_backend(*args, **kwargs):
        kwargs.setdefault("backend", backend)
        return _orig_find(*args, **kwargs)

    usb.core.find = _find_with_backend

    # WinUSB has no kernel driver to detach; stub these to no-ops so
    # brother_pt's __initialize() does not blow up.
    usb.core.Device.is_kernel_driver_active = lambda self, intf: False
    usb.core.Device.detach_kernel_driver = lambda self, intf: None
    return backend


_bootstrap_libusb_backend()

from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from brother_pt.cmd import MediaWidthToTapeMargin  # noqa: E402
from brother_pt.printer import BrotherPt  # noqa: E402


# --------------------------------------------------------------------- #
# Configuration                                                         #
# --------------------------------------------------------------------- #

TUBE_DIAMETER_MM = 20.0

# Target overall printed strip length (what you measure with a ruler).
TAPE_LENGTH_MM = 75.0

# Empirical length correction. The printer feeds some extra tape past
# the rendered image before the cutter fires (in addition to
# CUT_MARGIN_DOTS). Subtract that here so that the final tape length
# matches TAPE_LENGTH_MM. Increase this if the print comes out too
# long, decrease if too short.
LENGTH_CALIBRATION_MM = 4.0

# Vertical gap (px) between the two text rows.
LAYOUT_ROW_GAP_PX = 0

DPI = 180

# Margin (in raster columns / dots) the printer feeds before cutting.
# 14 dots ~= 2 mm at 180 dpi. Do NOT lower this -- if you do, the
# unprinted leftover stays inside the machine and the printed margin
# gets cropped on the next job.
CUT_MARGIN_DOTS = 14

# ----------------------- TUNING KNOBS ------------------------------- #
# Font size (in pixels @ 180 dpi) used for both rows. Tune this until
# the text comfortably fits on the tape width you actually load.
# Reference: 12 mm laminated tape -> printable height = 70 px, so each
# row gets ~33 px after subtracting LAYOUT_ROW_GAP_PX.
FONT_SIZE_PX = 36

# Box style for numeric length text like "1500".
LENGTH_BOX_PAD_X_PX = 6
LENGTH_BOX_PAD_Y_PX = 4
LENGTH_BOX_STROKE_PX = 1
# Negative values tighten spacing between characters.
CHAR_TRACKING_PX = -1.0

# ---- Grid layout (anchor-based) ----
# Number of anchor columns. Set to 5 (etc.) to add more anchors.
LAYOUT_COLUMNS = 4
# Left-anchored placement controls (in pixels).
# Anchor X for column i is: LAYOUT_COLUMN_START_PX + i * LAYOUT_COLUMN_SPACING_PX
LAYOUT_COLUMN_START_PX = 62.0
LAYOUT_COLUMN_SPACING_PX = 105.0
# Keep a little white space at the far right side of tape.
LAYOUT_RIGHT_PADDING_PX = 10
# Safety mode: if True, clamp/skip anchors that intrude into right padding.
LAYOUT_ENFORCE_RIGHT_PADDING = True
# Pattern cycle: piece kind for column index i in row r is
#   LAYOUT_PATTERN[(i + r) % len(LAYOUT_PATTERN)]
LAYOUT_PATTERN = ("bar", "seq", "len")
# -------------------------------------------------------------------- #

# What we want to print on the tape. Short format:
#   "#<seq> <bar_id> <length>"   e.g.  "#3 B017 500"
BAR_ID = "B017"
BAR_LENGTH_MM = 1500
ASSEMBLY_SEQ = 122

# BAR_ID = "B1"
# BAR_LENGTH_MM = 150
# ASSEMBLY_SEQ = 1

# Windows monospace fonts so padded fields align visually.
# Prefer Courier New first, then fall back to Consolas.
REGULAR_FONT_CANDIDATES = [
    r"C:\Windows\Fonts\cour.ttf",      # Courier New Regular
    r"C:\Windows\Fonts\COUR.TTF",
    r"C:\Windows\Fonts\consola.ttf",   # Consolas Regular fallback
    r"C:\Windows\Fonts\CONSOLA.TTF",
]

BOLD_FONT_CANDIDATES = [
    r"C:\Windows\Fonts\courbd.ttf",    # Courier New Bold
    r"C:\Windows\Fonts\COURBD.TTF",
    r"C:\Windows\Fonts\consolab.ttf",  # Consolas Bold fallback
    r"C:\Windows\Fonts\CONSOLAB.TTF",
]


# --------------------------------------------------------------------- #
# Helpers                                                               #
# --------------------------------------------------------------------- #

def mm_to_dots(mm: float) -> int:
    return int(round(mm * DPI / 25.4))


def _measure(text: str, font: ImageFont.FreeTypeFont) -> tuple[int, int, int, int]:
    """Return (x0, y0, x1, y1) bounding box of ``text`` for ``font``."""
    return ImageDraw.Draw(Image.new("L", (1, 1))).textbbox((0, 0), text, font=font)


def _text_advance(
    draw: ImageDraw.ImageDraw,
    text: str,
    font: ImageFont.FreeTypeFont,
) -> float:
    """Measure horizontal advance with configurable char tracking."""
    if not text:
        return 0.0
    total = 0.0
    for idx, ch in enumerate(text):
        total += float(draw.textlength(ch, font=font))
        if idx < len(text) - 1:
            total += CHAR_TRACKING_PX
    return total


def _draw_text_with_tracking(
    draw: ImageDraw.ImageDraw,
    x: float,
    y: int,
    text: str,
    font: ImageFont.FreeTypeFont,
    fill: int = 0,
) -> float:
    """Draw text char-by-char and return the rendered advance width."""
    cursor = float(x)
    for idx, ch in enumerate(text):
        draw.text((cursor, y), ch, font=font, fill=fill)
        cursor += float(draw.textlength(ch, font=font))
        if idx < len(text) - 1:
            cursor += CHAR_TRACKING_PX
    return cursor - x


def _pick_font_path(candidates: list[str], label: str) -> str:
    font_path = next((p for p in candidates if Path(p).exists()), None)
    if font_path is None:
        raise RuntimeError(f"No {label} TrueType font found in expected locations.")
    return font_path


def _render_piece(
    kind: str,
    value,
    draw: ImageDraw.ImageDraw,
    x_center: float,
    y_center: float,
    regular_font: ImageFont.FreeTypeFont,
    bold_font: ImageFont.FreeTypeFont,
) -> None:
    """Render one piece centered at (x_center, y_center).

    Supported ``kind`` values:
      - "bar":  bar_id rendered in bold font
      - "seq":  ``#<seq>`` rendered in regular font
      - "len":  length integer rendered in regular font, inside a box
    """
    if kind == "bar":
        text = str(value)
        font = bold_font
        w = _text_advance(draw, text, font)
        x0, y0, x1, y1 = _measure(text, font)
        h = y1 - y0
        x_left = x_center - w / 2.0
        y_top = y_center - h / 2.0 - y0
        _draw_text_with_tracking(draw, x_left, int(round(y_top)),
                                 text, font, fill=0)
        return

    if kind == "seq":
        text = f"#{int(value)}"
        font = regular_font
        w = _text_advance(draw, text, font)
        x0, y0, x1, y1 = _measure(text, font)
        h = y1 - y0
        x_left = x_center - w / 2.0
        y_top = y_center - h / 2.0 - y0
        _draw_text_with_tracking(draw, x_left, int(round(y_top)),
                                 text, font, fill=0)
        return

    if kind == "len":
        digits = str(int(value))
        font = regular_font
        num_w = _text_advance(draw, digits, font)
        nx0, ny0, nx1, ny1 = _measure(digits, font)
        text_h = ny1 - ny0
        box_w = num_w + 2 * LENGTH_BOX_PAD_X_PX
        box_h = text_h + 2 * LENGTH_BOX_PAD_Y_PX
        rect_left = x_center - box_w / 2.0
        rect_top = y_center - box_h / 2.0
        rect_right = rect_left + box_w - 1
        rect_bottom = rect_top + box_h - 1
        draw.rectangle(
            [(rect_left, rect_top), (rect_right, rect_bottom)],
            outline=0, width=LENGTH_BOX_STROKE_PX,
        )
        text_x = rect_left + LENGTH_BOX_PAD_X_PX - nx0
        text_y = rect_top + LENGTH_BOX_PAD_Y_PX - ny0
        _draw_text_with_tracking(draw, text_x, int(round(text_y)),
                                 digits, font, fill=0)
        return

    raise ValueError(f"Unknown piece kind: {kind!r}")


def _draw_grid_row(
    img: Image.Image,
    row_index: int,
    bar_id: str,
    length_mm: int,
    assembly_seq: int,
    regular_font: ImageFont.FreeTypeFont,
    bold_font: ImageFont.FreeTypeFont,
    row_top_px: float,
    row_height_px: float,
) -> None:
    """Draw one row of left-anchored column centers."""
    draw = ImageDraw.Draw(img)
    width = img.size[0]
    n = max(1, int(LAYOUT_COLUMNS))
    y_center = row_top_px + row_height_px / 2.0
    x_limit = width - LAYOUT_RIGHT_PADDING_PX

    for i in range(n):
        kind = LAYOUT_PATTERN[(i + row_index) % len(LAYOUT_PATTERN)]
        x_center = LAYOUT_COLUMN_START_PX + i * LAYOUT_COLUMN_SPACING_PX
        if LAYOUT_ENFORCE_RIGHT_PADDING and x_center > x_limit:
            continue
        if kind == "bar":
            value = bar_id
        elif kind == "seq":
            value = assembly_seq
        elif kind == "len":
            value = length_mm
        else:
            continue
        _render_piece(kind, value, draw, x_center, y_center,
                      regular_font, bold_font)


def render_label_image(
    bar_id: str, length_mm: int, assembly_seq: int,
    height_px: int, tape_width_px: int,
) -> Image.Image:
    """Render the two-row grid label using ``FONT_SIZE_PX`` for both rows."""
    regular_font_path = _pick_font_path(REGULAR_FONT_CANDIDATES, "regular")
    bold_font_path = _pick_font_path(BOLD_FONT_CANDIDATES, "bold")
    regular_font = ImageFont.truetype(regular_font_path, FONT_SIZE_PX)
    bold_font = ImageFont.truetype(bold_font_path, FONT_SIZE_PX)

    img = Image.new("L", (tape_width_px, height_px), 255)
    row_height = (height_px - LAYOUT_ROW_GAP_PX) / 2.0
    _draw_grid_row(img, row_index=0, bar_id=str(bar_id),
                   length_mm=int(length_mm),
                   assembly_seq=int(assembly_seq),
                   regular_font=regular_font, bold_font=bold_font,
                   row_top_px=0.0, row_height_px=row_height)
    _draw_grid_row(img, row_index=1, bar_id=str(bar_id),
                   length_mm=int(length_mm),
                   assembly_seq=int(assembly_seq),
                   regular_font=regular_font, bold_font=bold_font,
                   row_top_px=row_height + LAYOUT_ROW_GAP_PX,
                   row_height_px=row_height)
    return img


# --------------------------------------------------------------------- #
# Main                                                                  #
# --------------------------------------------------------------------- #

def main() -> int:
    print("=" * 60)
    print("Brother PT-P750W label print test (brother_pt library)")
    print("=" * 60)

    # 1. Connect. ``BrotherPt()`` discovers the printer over USB
    #    *and* runs update_status() — so any "no media" / "cover open"
    #    / "wrong media" condition will surface here as a RuntimeError.
    #    Note: don't call find_printers() ourselves before this; on
    #    Windows the open USB handle would block the second discovery
    #    inside BrotherPt and cause "device has no langid" errors.
    print("Connecting and querying printer status ...")
    printer = BrotherPt()
    print("Connected to Brother PT printer.")

    print(f"Media width : {printer.media_width} mm")
    print(f"Media type  : {printer.media_type.name}")
    print(f"Tape color  : {printer.tape_color.name}")
    print(f"Text color  : {printer.text_color.name}")

    # 3. Compute geometry
    if printer.media_width not in MediaWidthToTapeMargin.margin:
        print(f"ERROR: tape width {printer.media_width} mm is not "
              f"supported by brother_pt.", file=sys.stderr)
        return 1
    print_height_px = MediaWidthToTapeMargin.to_print_width(printer.media_width)

    # Render width = target tape length minus the empirically-measured
    # extra feed (LENGTH_CALIBRATION_MM). CUT_MARGIN_DOTS is left alone
    # so the printer doesn't keep unprinted leftover inside the head.
    image_len_mm = TAPE_LENGTH_MM - LENGTH_CALIBRATION_MM
    label_len_px = mm_to_dots(image_len_mm)

    print()
    print(f"Tube dia.   : {TUBE_DIAMETER_MM} mm  (circumf. "
          f"{math.pi * TUBE_DIAMETER_MM:.1f} mm)")
    print(f"Tape len.   : {TAPE_LENGTH_MM:.1f} mm target  "
          f"(image {image_len_mm:.1f} mm = {label_len_px} dots @ {DPI} dpi)")
    print(f"Strip h.    : {print_height_px} px")
    print(f"Font size   : {FONT_SIZE_PX} px")
    print(f"Layout      : 2 rows x {LAYOUT_COLUMNS} cols, pattern={LAYOUT_PATTERN}")
    print(f"Bar id      : {BAR_ID}")
    print(f"Length      : {BAR_LENGTH_MM} mm")
    print(f"Seq         : #{ASSEMBLY_SEQ}")
    print()

    # 4. Render
    img = render_label_image(
        BAR_ID, BAR_LENGTH_MM, ASSEMBLY_SEQ,
        print_height_px, label_len_px,
    )

    # Save a copy next to this script so we can inspect what was sent.
    preview_path = Path(__file__).with_name("label_preview.png")
    img.save(preview_path)
    print(f"Preview saved to {preview_path}")

    # 5. Print. If anything goes wrong on the printer side
    #    (no media / cutter jam / cover open / overheating ...)
    #    print_image() will raise a RuntimeError with a decoded message.
    print("Sending to printer ...")
    # printer.print_image(img, margin_px=CUT_MARGIN_DOTS)
    print("Print job complete - the cutter should have fired.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
