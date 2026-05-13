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
import re
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
ROW_GAP_PX = 4

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
# row gets ~33 px after subtracting ROW_GAP_PX.
FONT_SIZE_PX = 28

# Box style for numeric length text like "1500".
LENGTH_BOX_PAD_X_PX = 2
LENGTH_BOX_PAD_Y_PX = 3
LENGTH_BOX_STROKE_PX = 1
# -------------------------------------------------------------------- #

# What we want to print on the tape. Short format:
#   "#<seq> <bar_id> <length>"   e.g.  "#3 B017 500"
BAR_ID = "B017"
BAR_LENGTH_MM = 1500
ASSEMBLY_SEQ = 122

REGULAR_FONT_CANDIDATES = [
    r"C:\Windows\Fonts\arial.ttf",
    r"C:\Windows\Fonts\ARIAL.TTF",
]

BOLD_FONT_CANDIDATES = [
    r"C:\Windows\Fonts\arialbd.ttf",
    r"C:\Windows\Fonts\ARIALBD.TTF",
]


# --------------------------------------------------------------------- #
# Helpers                                                               #
# --------------------------------------------------------------------- #

def build_row_texts(bar_id: str, length_mm: int, seq: int) -> tuple[str, str]:
    """Build the two text rows.

    Row 1: ``#<seq> <bar_id> <len> #<seq> <bar_id> <len>``
    Row 2: ``<len> #<seq> <bar_id> <len> #<seq> <bar_id>``

    The half-block shift in row 2 means that whichever side of the
    wrapped tube is visible, at least one full ``#<seq> <bar_id>
    <len>`` reads cleanly.
    """
    block = f"#{seq} {bar_id} {length_mm}"
    tail = f"{length_mm}"
    head = f"#{seq} {bar_id}"
    row1 = f"{block} {block}"
    row2 = f"{tail} {block} {head}"
    return row1, row2


def mm_to_dots(mm: float) -> int:
    return int(round(mm * DPI / 25.4))


def _measure(text: str, font: ImageFont.FreeTypeFont) -> tuple[int, int, int, int]:
    """Return (x0, y0, x1, y1) bounding box of ``text`` for ``font``."""
    return ImageDraw.Draw(Image.new("L", (1, 1))).textbbox((0, 0), text, font=font)


def _pick_font_path(candidates: list[str], label: str) -> str:
    font_path = next((p for p in candidates if Path(p).exists()), None)
    if font_path is None:
        raise RuntimeError(f"No {label} TrueType font found in expected locations.")
    return font_path


def _tokenize_with_spaces(text: str) -> list[str]:
    """Split by spaces but keep spaces as explicit tokens."""
    words = text.split(" ")
    tokens: list[str] = []
    for idx, word in enumerate(words):
        tokens.append(word)
        if idx < len(words) - 1:
            tokens.append(" ")
    return tokens


def _split_length_token(token: str) -> str | None:
    """Return numeric length text for tokens like '1500' or '1500mm'."""
    match = re.fullmatch(r"(\d+)(?:mm)?", token)
    if match is None:
        return None
    return match.group(1)


def _segment_metrics(
    token: str,
    regular_font: ImageFont.FreeTypeFont,
    bold_font: ImageFont.FreeTypeFont,
    bar_id: str,
    draw: ImageDraw.ImageDraw,
) -> tuple[float, int, int]:
    """Return (advance_width, y_min, y_max) for one token segment."""
    if token == bar_id:
        x0, y0, x1, y1 = _measure(token, bold_font)
        return float(draw.textlength(token, font=bold_font)), y0, y1

    length_digits = _split_length_token(token)
    if length_digits is None:
        x0, y0, x1, y1 = _measure(token, regular_font)
        return float(draw.textlength(token, font=regular_font)), y0, y1

    nx0, ny0, nx1, ny1 = _measure(length_digits, regular_font)
    num_w = float(draw.textlength(length_digits, font=regular_font))
    box_w = num_w + (2 * LENGTH_BOX_PAD_X_PX)
    advance = box_w
    y_min = ny0 - LENGTH_BOX_PAD_Y_PX
    y_max = ny1 + LENGTH_BOX_PAD_Y_PX
    return advance, y_min, y_max


def _draw_segment(
    token: str,
    draw: ImageDraw.ImageDraw,
    cursor_x: float,
    baseline_y: int,
    regular_font: ImageFont.FreeTypeFont,
    bold_font: ImageFont.FreeTypeFont,
    bar_id: str,
) -> float:
    """Draw one token segment and return its horizontal advance."""
    if token == bar_id:
        draw.text((cursor_x, baseline_y), token, font=bold_font, fill=0)
        return float(draw.textlength(token, font=bold_font))

    length_digits = _split_length_token(token)
    if length_digits is None:
        draw.text((cursor_x, baseline_y), token, font=regular_font, fill=0)
        return float(draw.textlength(token, font=regular_font))

    nx0, ny0, _nx1, ny1 = _measure(length_digits, regular_font)

    num_w = float(draw.textlength(length_digits, font=regular_font))
    box_w = num_w + (2 * LENGTH_BOX_PAD_X_PX)

    rect_left = cursor_x
    rect_top = baseline_y + ny0 - LENGTH_BOX_PAD_Y_PX
    rect_right = cursor_x + box_w - 1
    rect_bottom = baseline_y + ny1 + LENGTH_BOX_PAD_Y_PX - 1
    draw.rectangle(
        [(rect_left, rect_top), (rect_right, rect_bottom)],
        outline=0,
        width=LENGTH_BOX_STROKE_PX,
    )

    draw.text(
        (cursor_x + LENGTH_BOX_PAD_X_PX - nx0, baseline_y),
        length_digits,
        font=regular_font,
        fill=0,
    )
    return box_w


def _draw_row_left_aligned(
    img: Image.Image,
    text: str,
    regular_font: ImageFont.FreeTypeFont,
    bold_font: ImageFont.FreeTypeFont,
    bar_id: str,
    row_top_px: int,
    row_height_px: int,
) -> None:
    draw = ImageDraw.Draw(img)

    tokens = _tokenize_with_spaces(text)

    # Compute global vertical bounds across mixed regular/bold segments
    # so the whole row remains vertically centered.
    min_y = None
    max_y = None
    first_x0 = 0
    for idx, token in enumerate(tokens):
        advance, y0, y1 = _segment_metrics(
            token, regular_font, bold_font, bar_id, draw,
        )
        if idx == 0:
            first_x0 = 0
        min_y = y0 if min_y is None else min(min_y, y0)
        max_y = y1 if max_y is None else max(max_y, y1)

    if min_y is None or max_y is None:
        return

    text_h = max_y - min_y
    # Left-align text so any unused width remains on the right side.
    x = -first_x0
    y = row_top_px + (row_height_px - text_h) // 2 - min_y

    cursor_x = float(x)
    for token in tokens:
        cursor_x += _draw_segment(
            token, draw, cursor_x, y, regular_font, bold_font, bar_id,
        )


def render_label_image(
    row1: str, row2: str, height_px: int, tape_width_px: int,
) -> Image.Image:
    """Render the two-row label using ``FONT_SIZE_PX`` for both rows."""
    regular_font_path = _pick_font_path(REGULAR_FONT_CANDIDATES, "regular")
    bold_font_path = _pick_font_path(BOLD_FONT_CANDIDATES, "bold")
    regular_font = ImageFont.truetype(regular_font_path, FONT_SIZE_PX)
    bold_font = ImageFont.truetype(bold_font_path, FONT_SIZE_PX)

    img = Image.new("L", (tape_width_px, height_px), 255)
    row_height = (height_px - ROW_GAP_PX) // 2
    _draw_row_left_aligned(
        img, row1, regular_font, bold_font, BAR_ID, 0, row_height,
    )
    _draw_row_left_aligned(
        img, row2, regular_font, bold_font, BAR_ID,
        row_height + ROW_GAP_PX, row_height,
    )
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
    row1, row2 = build_row_texts(BAR_ID, BAR_LENGTH_MM, ASSEMBLY_SEQ)

    print()
    print(f"Tube dia.   : {TUBE_DIAMETER_MM} mm  (circumf. "
          f"{math.pi * TUBE_DIAMETER_MM:.1f} mm)")
    print(f"Tape len.   : {TAPE_LENGTH_MM:.1f} mm target  "
          f"(image {image_len_mm:.1f} mm = {label_len_px} dots @ {DPI} dpi)")
    print(f"Strip h.    : {print_height_px} px")
    print(f"Font size   : {FONT_SIZE_PX} px")
    print(f"Row 1       : {row1!r}")
    print(f"Row 2       : {row2!r}")
    print()

    # 4. Render
    img = render_label_image(row1, row2, print_height_px, label_len_px)

    # Save a copy next to this script so we can inspect what was sent.
    preview_path = Path(__file__).with_name("label_preview.png")
    img.save(preview_path)
    print(f"Preview saved to {preview_path}")

    # 5. Print. If anything goes wrong on the printer side
    #    (no media / cutter jam / cover open / overheating ...)
    #    print_image() will raise a RuntimeError with a decoded message.
    print("Sending to printer ...")
    printer.print_image(img, margin_px=CUT_MARGIN_DOTS)
    print("Print job complete - the cutter should have fired.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
