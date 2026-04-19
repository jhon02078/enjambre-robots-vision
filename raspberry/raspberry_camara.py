!/usr/bin/env python3
import io
import atexit
from threading import Condition

from flask import Flask, Response
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder, Quality
from picamera2.outputs import FileOutput
from libcamera import Transform


WIDTH = 1280
HEIGHT = 720
FPS = 30

ENC_QUALITY = Quality.LOW

BUFFER_COUNT = 3
QUEUE = False

HOST = "0.0.0.0"
PORT = 5000

app = Flask(__name__)

picam2 = None


class StreamingOutput(io.BufferedIOBase):

    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


output = StreamingOutput()


def iniciar_camara():
    global picam2

    picam2 = Picamera2()

    video_config = picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT)},
        controls={"FrameRate": FPS},
        buffer_count=BUFFER_COUNT,
        queue=QUEUE,
        transform=Transform(rotation=0)
    )

    picam2.align_configuration(video_config)
    picam2.configure(video_config)

    encoder = MJPEGEncoder()
    picam2.start_recording(encoder, FileOutput(output), ENC_QUALITY)

    print(f"Streaming MJPEG: {WIDTH}x{HEIGHT} @ {FPS} FPS | {ENC_QUALITY} | buffer_count={BUFFER_COUNT}, queue={QUEUE}")


def parar_camara():
    global picam2
    try:
        if picam2 is not None:
            picam2.stop_recording()
            picam2.close()
            picam2 = None
    except Exception:
        pass


atexit.register(parar_camara)


def mjpeg_generador():

    while True:
        with output.condition:
            output.condition.wait()
            frame = output.frame

        if frame is None:
            continue

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n" +
               frame + b"\r\n")


@app.route("/video")
def video_feed():
    return Response(mjpeg_generador(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/")
def index():
    return (
        "<h2>RPi Camera Stream</h2>"
        "<p><a href='/video'>Abrir stream MJPEG</a></p>"
    )


if name == "__main__":
    iniciar_camara()
    app.run(host=HOST, port=PORT, threaded=True, debug=False, use_reloader=False)