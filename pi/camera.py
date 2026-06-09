"""RealSense D435i RGB via OpenCV (UVC) for wrong-way detection.

D435i colour sensor is a UVC node (/dev/video4, YUYV; video0=depth, video2=IR);
read as BGR with OpenCV. Track borders: green on the RIGHT, red on the LEFT in
the correct travel direction, so a forward frame swapped to red-right/green-left
means wrong way. Advisory telemetry only, not wired into steering.
"""

try:
    import cv2
    import numpy as np
    HAVE_CV = True
except Exception:
    HAVE_CV = False

# D435i colour node first, other RealSense nodes as fallback probes.
DEFAULT_DEVICE = "/dev/video4"
PROBE_NODES = ["/dev/video4", "/dev/video0", "/dev/video1",
               "/dev/video2", "/dev/video3", "/dev/video5"]

_MIN_COLOR_PIXELS = 150   # ignore stray pixels below this count
_ROI_TOP_FRAC = 0.45      # use only the lower part of the frame (the track)


class Camera:
    def __init__(self, device: str = DEFAULT_DEVICE, width: int = 640, height: int = 480):
        self.device = device
        self.width = width
        self.height = height
        self.cap = None

    def start(self) -> "Camera":
        if not HAVE_CV:
            raise RuntimeError("opencv (cv2) not available -- `sudo apt install python3-opencv`")
        # Probe preferred device then the rest for a 3-channel colour frame
        # (skips depth/IR nodes).
        candidates = [self.device] + [n for n in PROBE_NODES if n != self.device]
        for dev in candidates:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            ok, frame = cap.read()
            if ok and frame is not None and frame.ndim == 3 and frame.shape[2] == 3:
                self.cap = cap
                self.device = dev
                return self
            cap.release()
        raise RuntimeError("no usable RGB camera node found among " + ", ".join(candidates))

    def read(self) -> dict:
        out = {"ok": False, "red_side": None, "green_side": None,
               "wrong_way": False, "n_red": 0, "n_green": 0, "device": self.device}
        if self.cap is None:
            return out
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return out

        h, w = frame.shape[:2]
        roi = frame[int(h * _ROI_TOP_FRAC):, :, :]
        b = roi[:, :, 0].astype("int16")
        g = roi[:, :, 1].astype("int16")
        r = roi[:, :, 2].astype("int16")

        red_mask = (r > 110) & (r - g > 45) & (r - b > 45)
        green_mask = (g > 90) & (g - r > 35) & (g - b > 35)

        mid = roi.shape[1] // 2
        red_l = int(red_mask[:, :mid].sum())
        red_r = int(red_mask[:, mid:].sum())
        green_l = int(green_mask[:, :mid].sum())
        green_r = int(green_mask[:, mid:].sum())

        red_side = _dominant(red_l, red_r)
        green_side = _dominant(green_l, green_r)

        # Wrong way: red on the RIGHT and green on the LEFT (swapped).
        wrong_way = (red_side == "right" and green_side == "left")

        out.update({
            "ok": True,
            "red_side": red_side,
            "green_side": green_side,
            "wrong_way": wrong_way,
            "n_red": red_l + red_r,
            "n_green": green_l + green_r,
        })
        return out

    def stop(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self) -> "Camera":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()


def _dominant(left_count: int, right_count: int):
    """Return which half a colour is on, or None if too few pixels or ambiguous."""
    if max(left_count, right_count) < _MIN_COLOR_PIXELS:
        return None
    if left_count > 2 * right_count:
        return "left"
    if right_count > 2 * left_count:
        return "right"
    return None
