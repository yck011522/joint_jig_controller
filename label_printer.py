"""Brother PT-P750W label-printer integration for the joint jig.

This module wraps the rendering + printing logic that was tuned in
``test/label_print_test_brotherpt.py`` and exposes a small, safe API
for the GUI to call:

    BAR_PRINTER_VID, BAR_PRINTER_PID    -- USB IDs (Brother PT-P750W)
    is_dependencies_available()         -- True if pyusb/libusb/brother_pt
                                           imported OK at module load.
    is_printer_present()                -- Quick USB scan, safe to call
                                           every couple seconds for
                                           hot-plug detection. Does NOT
                                           open / claim the device.
    print_bar_label(bar_id, length_mm,  -- Render and print one label.
                    assembly_seq)         Raises RuntimeError on any
                                           failure (no device, cover
                                           open, wrong media, ...).

The module is import-safe even if libusb / brother_pt are missing or
the USB DLL isn't bound: it just reports the printer as unavailable.
"""

from __future__ import annotations

import os
import platform
import sys
import threading
from pathlib import Path
from typing import Optional

# Brother PT-P750W
BAR_PRINTER_VID = 0x04F9
BAR_PRINTER_PID = 0x2062

# --------------------------------------------------------------------- #
# Optional dependencies                                                 #
# --------------------------------------------------------------------- #
# We import these lazily/defensively so the GUI keeps working even if
# the print stack is not installed. Failures are recorded in
# ``_IMPORT_ERROR`` and surfaced via ``is_dependencies_available()``.

_IMPORT_ERROR: Optional[str] = None
_BACKEND = None

try:
    from PIL import Image, ImageDraw, ImageFont  # type: ignore
except Exception as exc:  # pragma: no cover - PIL is required by GUI too
    _IMPORT_ERROR = f"Pillow import failed: {exc}"
    Image = ImageDraw = ImageFont = None  # type: ignore


def _bootstrap_libusb_backend():
    """Locate the libusb DLL (Windows) and register it with PyUSB.

    Returns the backend on success, or ``None`` on any failure.
    """
    global _IMPORT_ERROR
    if sys.platform != "win32":
        # Other platforms: rely on the system libusb.
        try:
            import usb.backend.libusb1  # noqa: F401
            import usb.core  # noqa: F401
            return None
        except Exception as exc:
            _IMPORT_ERROR = f"pyusb import failed: {exc}"
            return None

    try:
        import libusb  # type: ignore
    except Exception as exc:
        _IMPORT_ERROR = f"libusb pip package not installed: {exc}"
        return None

    arch = "x86_64" if platform.machine().endswith("64") else "x86"
    dll_dir = Path(libusb.__file__).parent / "_platform" / "windows" / arch
    dll_path = dll_dir / "libusb-1.0.dll"
    if not dll_path.exists():
        _IMPORT_ERROR = f"libusb DLL not found at {dll_path}"
        return None

    try:
        os.add_dll_directory(str(dll_dir))
    except (AttributeError, FileNotFoundError, OSError):
        pass
    os.environ["PATH"] = str(dll_dir) + os.pathsep + os.environ.get("PATH", "")

    try:
        import usb.backend.libusb1
        import usb.core
    except Exception as exc:
        _IMPORT_ERROR = f"pyusb import failed: {exc}"
        return None

    backend = usb.backend.libusb1.get_backend(
        find_library=lambda _: str(dll_path)
    )
    if backend is None:
        _IMPORT_ERROR = (
            "Failed to load libusb backend. Make sure WinUSB is bound to "
            "the printer (use Zadig)."
        )
        return None

    _orig_find = usb.core.find

    def _find_with_backend(*args, **kwargs):
        kwargs.setdefault("backend", backend)
        return _orig_find(*args, **kwargs)

    usb.core.find = _find_with_backend  # type: ignore[assignment]

    # WinUSB has no kernel driver to detach.
    usb.core.Device.is_kernel_driver_active = (  # type: ignore[assignment]
        lambda self, intf: False
    )
    usb.core.Device.detach_kernel_driver = (  # type: ignore[assignment]
        lambda self, intf: None
    )
    return backend


_BACKEND = _bootstrap_libusb_backend()

try:
    import usb.core  # type: ignore
except Exception as exc:
    if _IMPORT_ERROR is None:
        _IMPORT_ERROR = f"pyusb import failed: {exc}"
    usb = None  # type: ignore

try:
    from brother_pt.cmd import MediaWidthToTapeMargin  # type: ignore
    from brother_pt.printer import BrotherPt  # type: ignore
except Exception as exc:
    if _IMPORT_ERROR is None:
        _IMPORT_ERROR = f"brother_pt import failed: {exc}"
    MediaWidthToTapeMargin = None  # type: ignore
    BrotherPt = None  # type: ignore


# --------------------------------------------------------------------- #
# Layout / rendering constants (mirrored from the test script)          #
# --------------------------------------------------------------------- #

DPI = 180
TAPE_LENGTH_MM = 75.0
LENGTH_CALIBRATION_MM = 4.0   # extra tape feed before cut
CUT_MARGIN_DOTS = 14          # ~2 mm trailing margin (do NOT lower)

FONT_SIZE_PX = 36
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
# Vertical gap between row 1 and row 2.
LAYOUT_ROW_GAP_PX = 0
# Safety mode: if True, clamp/skip anchors that intrude into right padding.
LAYOUT_ENFORCE_RIGHT_PADDING = True

# Pattern cycle: piece kind for column index i in row r is
#   LAYOUT_PATTERN[(i + r) % len(LAYOUT_PATTERN)]
#   "bar" -> bar_id (bold)
#   "seq" -> "#<assembly_seq>" (regular)
#   "len" -> length_mm in a rectangular box (regular)
LAYOUT_PATTERN = ("bar", "seq", "len")

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
# Public availability helpers                                           #
# --------------------------------------------------------------------- #

def is_dependencies_available() -> bool:
    """Return True iff the print stack (pyusb / libusb / brother_pt) loaded."""
    return _IMPORT_ERROR is None and BrotherPt is not None


def import_error_message() -> Optional[str]:
    """Return the import-time error message, or None if everything loaded."""
    return _IMPORT_ERROR


def is_printer_present() -> bool:
    """Quick USB scan for the Brother PT printer.

    Does NOT open or claim the device, so it is safe to call from a
    periodic poller for hot-plug detection.
    """
    if not is_dependencies_available() or usb is None:
        return False
    try:
        dev = usb.core.find(idVendor=BAR_PRINTER_VID, idProduct=BAR_PRINTER_PID)
        return dev is not None
    except Exception:
        return False


# --------------------------------------------------------------------- #
# Rendering                                                             #
# --------------------------------------------------------------------- #

def _mm_to_dots(mm: float) -> int:
    return int(round(mm * DPI / 25.4))


def _measure(text, font):
    return ImageDraw.Draw(Image.new("L", (1, 1))).textbbox(
        (0, 0), text, font=font
    )


def _text_advance(draw, text: str, font) -> float:
    """Measure horizontal advance with configurable char tracking."""
    if not text:
        return 0.0
    total = 0.0
    for idx, ch in enumerate(text):
        total += float(draw.textlength(ch, font=font))
        if idx < len(text) - 1:
            total += CHAR_TRACKING_PX
    return total


def _draw_text_with_tracking(draw, x: float, y: int, text: str, font,
                             fill: int = 0) -> float:
    """Draw text char-by-char and return the rendered advance width."""
    cursor = float(x)
    for idx, ch in enumerate(text):
        draw.text((cursor, y), ch, font=font, fill=fill)
        cursor += float(draw.textlength(ch, font=font))
        if idx < len(text) - 1:
            cursor += CHAR_TRACKING_PX
    return cursor - x


def _pick_font_path(candidates, label: str) -> str:
    font_path = next((p for p in candidates if Path(p).exists()), None)
    if font_path is None:
        raise RuntimeError(
            f"No {label} TrueType font found in expected locations."
        )
    return font_path


def _render_piece(kind: str, value, draw, x_center: float, y_center: float,
                  regular_font, bold_font) -> None:
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
        # textbbox y0 may be negative; subtract to align glyph top with cell.
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


def _draw_grid_row(img, row_index: int, bar_id: str, length_mm: int,
                   assembly_seq: int, regular_font, bold_font,
                   row_top_px: float, row_height_px: float) -> None:
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


def render_label_image(bar_id: str, length_mm: int, assembly_seq: int,
                       printable_height_px: int):
    """Render the two-row grid label as a PIL grayscale image."""
    if Image is None:
        raise RuntimeError("Pillow is not available.")
    regular_font_path = _pick_font_path(REGULAR_FONT_CANDIDATES, "regular")
    bold_font_path = _pick_font_path(BOLD_FONT_CANDIDATES, "bold")
    regular_font = ImageFont.truetype(regular_font_path, FONT_SIZE_PX)
    bold_font = ImageFont.truetype(bold_font_path, FONT_SIZE_PX)

    image_len_mm = TAPE_LENGTH_MM - LENGTH_CALIBRATION_MM
    tape_width_px = _mm_to_dots(image_len_mm)

    img = Image.new("L", (tape_width_px, printable_height_px), 255)
    row_height = (printable_height_px - LAYOUT_ROW_GAP_PX) / 2.0
    _draw_grid_row(img, row_index=0, bar_id=str(bar_id),
                   length_mm=int(length_mm), assembly_seq=int(assembly_seq),
                   regular_font=regular_font, bold_font=bold_font,
                   row_top_px=0.0, row_height_px=row_height)
    _draw_grid_row(img, row_index=1, bar_id=str(bar_id),
                   length_mm=int(length_mm), assembly_seq=int(assembly_seq),
                   regular_font=regular_font, bold_font=bold_font,
                   row_top_px=row_height + LAYOUT_ROW_GAP_PX,
                   row_height_px=row_height)
    return img


# --------------------------------------------------------------------- #
# Printing                                                              #
# --------------------------------------------------------------------- #

# Serialize all printer access -- two threads must never open BrotherPt
# at the same time.
_PRINT_LOCK = threading.Lock()


def print_bar_label(bar_id: str, length_mm: int, assembly_seq: int) -> dict:
    """Render and print a single bar label.

    Raises ``RuntimeError`` on any failure (no device, cover open, wrong
    media, ...). On success returns a dict with media info for logging.
    """
    if not is_dependencies_available():
        raise RuntimeError(
            f"Label printer support not available: {_IMPORT_ERROR}"
        )
    if not is_printer_present():
        raise RuntimeError(
            "Brother label printer not detected. Plug it in via USB and "
            "make sure the mode switch is set to USB (Editor Lite LED off)."
        )

    with _PRINT_LOCK:
        try:
            printer = BrotherPt()
        except Exception as exc:
            raise RuntimeError(
                f"Failed to open Brother label printer: {exc}"
            ) from exc

        try:
            media_width = int(printer.media_width)
            if MediaWidthToTapeMargin is None:
                raise RuntimeError("brother_pt MediaWidthToTapeMargin missing.")
            if media_width not in MediaWidthToTapeMargin.margin:
                raise RuntimeError(
                    f"Unsupported tape width: {media_width} mm. "
                    "Load 9 mm or 12 mm TZe tape."
                )
            printable_h = MediaWidthToTapeMargin.to_print_width(media_width)

            img = render_label_image(
                bar_id=str(bar_id),
                length_mm=int(round(float(length_mm))),
                assembly_seq=int(assembly_seq),
                printable_height_px=printable_h,
            )

            try:
                printer.print_image(img, margin_px=CUT_MARGIN_DOTS)
            except Exception as exc:
                raise RuntimeError(
                    f"Print job failed: {exc}"
                ) from exc

            return {
                "media_width_mm": media_width,
                "tape_color": getattr(printer.tape_color, "name", "?"),
                "text_color": getattr(printer.text_color, "name", "?"),
                "media_type": getattr(printer.media_type, "name", "?"),
            }
        finally:
            try:
                # brother_pt's BrotherPt has no explicit close; rely on
                # USB resource release when ``printer`` goes out of scope.
                del printer
            except Exception:
                pass


def _preview_printable_height_px() -> int:
    """Best-effort printable height for preview rendering.

    Defaults to 12 mm tape geometry (70 px) when printer deps are not
    available. If ``brother_pt`` is importable, resolve from its table.
    """
    if MediaWidthToTapeMargin is None:
        return 70
    try:
        return int(MediaWidthToTapeMargin.to_print_width(12))
    except Exception:
        return 70


def generate_preview_images() -> tuple[Path, Path]:
    """Generate shortest/longest label previews in the test folder."""
    base_dir = Path(__file__).resolve().parent
    out_dir = base_dir / "test"
    out_dir.mkdir(parents=True, exist_ok=True)

    h_px = _preview_printable_height_px()

    shortest = render_label_image(
        bar_id="B1",       # short bar ID case
        length_mm=100,      # shortest expected 3-digit length
        assembly_seq=1,     # shortest expected sequence
        printable_height_px=h_px,
    )
    longest = render_label_image(
        bar_id="B999",     # long bar ID case
        length_mm=9999,     # longest expected 4-digit length
        assembly_seq=999,   # longest expected sequence
        printable_height_px=h_px,
    )

    short_path = out_dir / "label_preview_shortest.png"
    long_path = out_dir / "label_preview_longest.png"
    shortest.save(short_path)
    longest.save(long_path)
    return short_path, long_path


def _ask_yes_no(prompt: str) -> bool:
    """Return True for yes-like answers, False for no-like answers."""
    while True:
        answer = input(prompt).strip().lower()
        if answer in {"y", "yes"}:
            return True
        if answer in {"n", "no", ""}:
            return False
        print("Please answer 'y' or 'n'.")


def main() -> int:
    """CLI entry point for preview generation and optional real printing."""
    short_path, long_path = generate_preview_images()
    print("Saved label preview images:")
    print(f"  shortest: {short_path}")
    print(f"  longest : {long_path}")

    if not _ask_yes_no("Print both test labels on the real printer now? [y/N]: "):
        print("Skipping printer test.")
        return 0

    test_cases = [
        ("B1", 100, 1),
        ("B999", 9999, 999),
    ]

    for idx, (bar_id, length_mm, seq) in enumerate(test_cases, start=1):
        print(f"Printing test label {idx}/2: bar={bar_id}, len={length_mm}, seq={seq}")
        try:
            print_bar_label(bar_id=bar_id, length_mm=length_mm, assembly_seq=seq)
        except Exception as exc:
            print(f"Failed while printing test label {idx}/2: {exc}")
            return 1

    print("Finished printing both test labels.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
