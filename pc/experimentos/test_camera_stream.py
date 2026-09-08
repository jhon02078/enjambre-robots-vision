import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


PC_DIR = Path(__file__).resolve().parents[1]
if str(PC_DIR) not in sys.path:
    sys.path.insert(0, str(PC_DIR))

from camera_stream import MjpegFrameBuffer, reconnect_delay


def jpeg_with_level(level):
    image = np.full((16, 24, 3), int(level), dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    if not ok:
        raise RuntimeError("No se pudo crear JPEG sintetico")
    return encoded.tobytes()


class MjpegFrameBufferTests(unittest.TestCase):
    def test_reassembles_fragmented_jpeg(self):
        jpeg = jpeg_with_level(80)
        parser = MjpegFrameBuffer()
        self.assertIsNone(parser.feed(b"--frame\r\n" + jpeg[:17]))
        payload = parser.feed(jpeg[17:] + b"\r\n")
        self.assertEqual(payload, jpeg)

    def test_returns_latest_complete_frame_to_reduce_latency(self):
        first = jpeg_with_level(30)
        latest = jpeg_with_level(210)
        parser = MjpegFrameBuffer()
        payload = parser.feed(first + b"\r\n--frame\r\n" + latest)
        self.assertEqual(payload, latest)

    def test_reconnect_delay_is_bounded(self):
        delay = None
        values = []
        for _ in range(10):
            delay = reconnect_delay(delay)
            values.append(delay)
        self.assertEqual(values[0], 0.4)
        self.assertLessEqual(max(values), 5.0)
        self.assertEqual(values[-1], 5.0)


if __name__ == "__main__":
    unittest.main()
