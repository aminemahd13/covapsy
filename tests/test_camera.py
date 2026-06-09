"""Smoke test for the RealSense RGB wrong-way detector.

Exits 0 with a note if OpenCV (cv2) is missing (camera is advisory). Otherwise
opens the RGB node for ~5 s and prints each border colour's side and the
wrong_way flag.
"""

import os
import sys
import time

# Pi drivers are in ../pi
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pi"))

from camera import HAVE_CV, Camera


def main() -> int:
    if not HAVE_CV:
        print("opencv (cv2) not available - camera deferred")
        return 0

    print("RGB camera smoke test -- ~5 s. Point at the track borders.")
    try:
        cam = Camera().start()
    except RuntimeError as e:
        print(f" camera unavailable: {e}")
        return 0

    try:
        print(f" using {cam.device}")
        end = time.time() + 5.0
        while time.time() < end:
            f = cam.read()
            print(
                f" ok={f['ok']!s:5} red={f['red_side']!s:5} green={f['green_side']!s:5}"
                f"  wrong_way={f['wrong_way']}"
                f"  (n_red={f['n_red']} n_green={f['n_green']})"
            )
            time.sleep(0.25)
    finally:
        cam.stop()
    print(" done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
