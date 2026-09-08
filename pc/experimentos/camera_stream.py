from urllib.parse import urlparse

import cv2
import numpy as np
import requests


class MjpegFrameBuffer:
    """Extrae el JPEG completo mas reciente de un flujo MJPEG fragmentado."""

    SOI = b"\xff\xd8"
    EOI = b"\xff\xd9"

    def __init__(self, max_buffer_bytes=8 * 1024 * 1024):
        self.buffer = bytearray()
        self.max_buffer_bytes = max(1024, int(max_buffer_bytes))

    def feed(self, chunk):
        if chunk:
            self.buffer.extend(chunk)
        payload = self._extract_latest_complete()
        if payload is None:
            self._trim_incomplete_buffer()
        return payload

    def _extract_latest_complete(self):
        end = self.buffer.rfind(self.EOI)
        if end < 0:
            return None
        start = self.buffer.rfind(self.SOI, 0, end)
        if start < 0:
            del self.buffer[:end + len(self.EOI)]
            return None
        payload = bytes(self.buffer[start:end + len(self.EOI)])
        del self.buffer[:end + len(self.EOI)]
        return payload

    def _trim_incomplete_buffer(self):
        if len(self.buffer) <= self.max_buffer_bytes:
            return
        start = self.buffer.rfind(self.SOI)
        if start >= 0 and len(self.buffer) - start <= self.max_buffer_bytes:
            del self.buffer[:start]
        else:
            self.buffer.clear()


class MjpegHttpCamera:
    """Lector HTTP MJPEG con timeout, buffer limitado y baja latencia."""

    def __init__(self, url, connect_timeout_s=3.0, read_timeout_s=3.0):
        self.url = str(url)
        self.session = requests.Session()
        self.response = None
        self.chunks = None
        self.parser = MjpegFrameBuffer()
        self.decode_failures = 0
        try:
            self.response = self.session.get(
                self.url,
                stream=True,
                timeout=(float(connect_timeout_s), float(read_timeout_s)),
                headers={
                    "Accept": "multipart/x-mixed-replace,image/jpeg,*/*",
                    "Cache-Control": "no-cache",
                    "Connection": "keep-alive",
                },
            )
            self.response.raise_for_status()
            self.chunks = self.response.iter_content(chunk_size=32 * 1024)
        except Exception:
            self.release()
            raise

    def read(self):
        if self.chunks is None:
            raise ConnectionError("El stream MJPEG no esta abierto")
        for chunk in self.chunks:
            payload = self.parser.feed(chunk)
            if payload is None:
                continue
            frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                self.decode_failures = 0
                return frame
            self.decode_failures += 1
            if self.decode_failures >= 5:
                raise ConnectionError("Se recibieron cinco JPEG invalidos consecutivos")
        raise ConnectionError("El servidor cerro el stream MJPEG")

    def release(self):
        response, self.response = self.response, None
        self.chunks = None
        if response is not None:
            try:
                response.close()
            except Exception:
                pass
        try:
            self.session.close()
        except Exception:
            pass


class OpenCvCamera:
    """Adaptador para camaras locales o fuentes no HTTP."""

    def __init__(self, source, open_timeout_ms=3000, read_timeout_ms=3000):
        self.capture = cv2.VideoCapture()
        for prop_name, value in (
            ("CAP_PROP_OPEN_TIMEOUT_MSEC", open_timeout_ms),
            ("CAP_PROP_READ_TIMEOUT_MSEC", read_timeout_ms),
        ):
            prop = getattr(cv2, prop_name, None)
            if prop is not None:
                try:
                    self.capture.set(prop, value)
                except Exception:
                    pass
        if not self.capture.open(source):
            self.release()
            raise ConnectionError(f"No se pudo abrir la fuente de video: {source}")
        try:
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

    def read(self):
        try:
            ok, frame = self.capture.read()
        except cv2.error as exc:
            raise ConnectionError(f"OpenCV fallo al leer la camara: {exc}") from exc
        if not ok or frame is None:
            raise ConnectionError("La camara no entrego un frame")
        return frame

    def release(self):
        capture, self.capture = getattr(self, "capture", None), None
        if capture is not None:
            try:
                capture.release()
            except Exception:
                pass


def open_camera_stream(source):
    text = str(source).strip()
    scheme = urlparse(text).scheme.lower()
    if scheme in {"http", "https"}:
        return MjpegHttpCamera(text)
    try:
        local_source = int(text)
    except ValueError:
        local_source = text
    return OpenCvCamera(local_source)


def reconnect_delay(previous_s, minimum_s=0.4, maximum_s=5.0):
    """Espera exponencial acotada para reconexiones sucesivas."""
    if previous_s is None or previous_s <= 0:
        return float(minimum_s)
    return min(float(maximum_s), max(float(minimum_s), float(previous_s) * 1.8))
