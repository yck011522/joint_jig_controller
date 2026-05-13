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
import re
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
ROW_GAP_PX = 0
CUT_MARGIN_DOTS = 14          # ~2 mm trailing margin (do NOT lower)

FONT_SIZE_PX = 26
LENGTH_BOX_PAD_X_PX = 3
LENGTH_BOX_PAD_Y_PX = 4
LENGTH_BOX_STROKE_PX = 1
# Negative values tighten spacing between characters.
CHAR_TRACKING_PX = -1.0

# Fixed field widths (in characters) so short values don't collapse to
# the left when compared to max-length values.
SEQ_FIELD_CHARS = 5      # e.g. "#5   " .. "#122 "
BAR_ID_FIELD_CHARS = 5   # e.g. "B5   " .. "B123 "
LENGTH_FIELD_CHARS = 5   # e.g. "500  " .. "1500 "

# Extra manual spacing between info chunks (in pixels). These are in
# addition to the fixed field widths above.
SEQ_TO_BAR_GAP_PX = -6.0
BAR_TO_LENGTH_GAP_PX = -6.0

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

def _build_row_texts(bar_id: str, length_mm: int, seq: int) -> tuple[str, str]:
    seq_field = f"#{seq}".ljust(SEQ_FIELD_CHARS)
    bar_field = str(bar_id).ljust(BAR_ID_FIELD_CHARS)
    len_field = str(length_mm).ljust(LENGTH_FIELD_CHARS)

    # Keep the same semantic pattern as before, but in fixed-width
    # chunks so each block occupies a stable width across bars.
    block = f"{seq_field}{bar_field}{len_field}"
    tail = len_field
    head = f"{seq_field}{bar_field}"
    row1 = f"{block}{block}"
    row2 = f"{tail}{block}{head}"
    return row1, row2


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


def _tokenize_with_spaces(text: str) -> list[str]:
    # Preserve runs of spaces as tokens so field padding survives.
    return re.findall(r"\S+|\s+", text)


def _split_length_token(token: str) -> Optional[str]:
    match = re.fullmatch(r"(\d+)(?:mm)?", token)
    return match.group(1) if match else None


def _token_kind(token: str, bar_id: str) -> Optional[str]:
    """Classify semantic token kind for inter-field spacing control."""
    stripped = token.strip()
    if not stripped:
        return None
    if stripped == bar_id:
        return "bar"
    if re.fullmatch(r"#\d+", stripped):
        return "seq"
    if _split_length_token(stripped) is not None:
        return "len"
    return None


def _segment_metrics(token, regular_font, bold_font, bar_id, draw):
    if token == bar_id:
        x0, y0, x1, y1 = _measure(token, bold_font)
        return _text_advance(draw, token, bold_font), y0, y1
    digits = _split_length_token(token)
    if digits is None:
        x0, y0, x1, y1 = _measure(token, regular_font)
        return _text_advance(draw, token, regular_font), y0, y1
    nx0, ny0, nx1, ny1 = _measure(digits, regular_font)
    num_w = _text_advance(draw, digits, regular_font)
    box_w = num_w + (2 * LENGTH_BOX_PAD_X_PX)
    return box_w, ny0 - LENGTH_BOX_PAD_Y_PX, ny1 + LENGTH_BOX_PAD_Y_PX


def _draw_segment(token, draw, cursor_x, baseline_y,
                  regular_font, bold_font, bar_id):
    if token == bar_id:
        return _draw_text_with_tracking(
            draw, cursor_x, baseline_y, token, bold_font, fill=0
        )
    digits = _split_length_token(token)
    if digits is None:
        return _draw_text_with_tracking(
            draw, cursor_x, baseline_y, token, regular_font, fill=0
        )
    nx0, ny0, _nx1, ny1 = _measure(digits, regular_font)
    num_w = _text_advance(draw, digits, regular_font)
    box_w = num_w + (2 * LENGTH_BOX_PAD_X_PX)
    rect_left = cursor_x
    rect_top = baseline_y + ny0 - LENGTH_BOX_PAD_Y_PX
    rect_right = cursor_x + box_w - 1
    rect_bottom = baseline_y + ny1 + LENGTH_BOX_PAD_Y_PX - 1
    draw.rectangle(
        [(rect_left, rect_top), (rect_right, rect_bottom)],
        outline=0, width=LENGTH_BOX_STROKE_PX,
    )
    _draw_text_with_tracking(
        draw,
        cursor_x + LENGTH_BOX_PAD_X_PX - nx0,
        baseline_y,
        digits,
        regular_font,
        fill=0,
    )
    return box_w


def _draw_row_left_aligned(img, text, regular_font, bold_font, bar_id,
                           row_top_px, row_height_px):
    draw = ImageDraw.Draw(img)
    tokens = _tokenize_with_spaces(text)
    min_y = max_y = None
    for token in tokens:
        _adv, y0, y1 = _segment_metrics(
            token, regular_font, bold_font, bar_id, draw
        )
        min_y = y0 if min_y is None else min(min_y, y0)
        max_y = y1 if max_y is None else max(max_y, y1)
    if min_y is None or max_y is None:
        return
    text_h = max_y - min_y
    y = row_top_px + (row_height_px - text_h) // 2 - min_y
    cursor_x = 0.0
    prev_kind: Optional[str] = None
    for token in tokens:
        kind = _token_kind(token, bar_id)
        if prev_kind == "seq" and kind == "bar":
            cursor_x += SEQ_TO_BAR_GAP_PX
        elif prev_kind == "bar" and kind == "len":
            cursor_x += BAR_TO_LENGTH_GAP_PX
        cursor_x += _draw_segment(
            token, draw, cursor_x, y, regular_font, bold_font, bar_id,
        )
        if kind is not None:
            prev_kind = kind


def render_label_image(bar_id: str, length_mm: int, assembly_seq: int,
                       printable_height_px: int):
    """Render the two-row label as a PIL grayscale image."""
    if Image is None:
        raise RuntimeError("Pillow is not available.")
    regular_font_path = _pick_font_path(REGULAR_FONT_CANDIDATES, "regular")
    bold_font_path = _pick_font_path(BOLD_FONT_CANDIDATES, "bold")
    regular_font = ImageFont.truetype(regular_font_path, FONT_SIZE_PX)
    bold_font = ImageFont.truetype(bold_font_path, FONT_SIZE_PX)

    image_len_mm = TAPE_LENGTH_MM - LENGTH_CALIBRATION_MM
    tape_width_px = _mm_to_dots(image_len_mm)

    row1, row2 = _build_row_texts(bar_id, length_mm, assembly_seq)
    img = Image.new("L", (tape_width_px, printable_height_px), 255)
    row_height = (printable_height_px - ROW_GAP_PX) // 2
    _draw_row_left_aligned(
        img, row1, regular_font, bold_font, bar_id, 0, row_height,
    )
    _draw_row_left_aligned(
        img, row2, regular_font, bold_font, bar_id,
        row_height + ROW_GAP_PX, row_height,
    )
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


def main() -> int:
    """CLI entry point: save two preview images for quick visual checks."""
    short_path, long_path = generate_preview_images()
    print("Saved label preview images:")
    print(f"  shortest: {short_path}")
    print(f"  longest : {long_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
