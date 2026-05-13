"""Quick interactive test for Brother PT label-printer hot-plug detection.

Polls ``label_printer.is_printer_present()`` once per second and prints
a state-change line when the device appears or disappears. Run it,
then physically unplug/replug the printer to verify auto-detection.

Usage:
    python test\\label_printer_hotplug_test.py

Press Ctrl-C to stop.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Allow running this file directly from the repo root or test/ folder.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import label_printer  # noqa: E402


def main() -> int:
    print("Brother PT-P750W hot-plug detector")
    print("=" * 40)
    print(f"Dependencies OK : {label_printer.is_dependencies_available()}")
    if not label_printer.is_dependencies_available():
        print(f"  reason       : {label_printer.import_error_message()}")
        return 1
    print(
        f"Watching VID=0x{label_printer.BAR_PRINTER_VID:04X} "
        f"PID=0x{label_printer.BAR_PRINTER_PID:04X}"
    )
    print("Unplug / replug the USB cable to verify detection.")
    print("Press Ctrl-C to stop.\n")

    last = None
    try:
        while True:
            now_present = label_printer.is_printer_present()
            if now_present != last:
                ts = time.strftime("%H:%M:%S")
                state = "CONNECTED" if now_present else "NOT DETECTED"
                print(f"[{ts}] state changed: {state}")
                last = now_present
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
