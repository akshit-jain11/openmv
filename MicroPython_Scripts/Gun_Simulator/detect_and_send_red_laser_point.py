import sensor
import time
import image
import pyb
import struct

# ── Camera setup ──────────────────────────────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(False)
sensor.set_hmirror(False)
sensor.skip_frames(time=2000)

clock = time.clock()

RED_THRESHOLDS = [
    (30, 100, 27, 127, -44, 127)
]

MIN_PIXELS = 8
MIN_AREA   = 1
MAX_AREA   = 100
Target_Frame_x = [4, 313]
Target_Frame_y  = [36, 203]

def Check_blob_in_frame(blob):
    if Target_Frame_x[0] <= blob.cx() <= Target_Frame_x[1]:
        if Target_Frame_y[0] <= blob.cy() <= Target_Frame_y[1]:
            return blob
    return False

vcp = pyb.USB_VCP()

FRAME_MAGIC = b'\xDE\xAD\xBE\xEF'   # frame packet marker
BLOB_MAGIC  = b'\xAB\xCD\xEF\x01'   # blob packet marker

# def send_frame(img):
#     data = img.bytearray()
#     vcp.write(FRAME_MAGIC)
#     vcp.write(len(data).to_bytes(4, 'little'))
#     vcp.write(data)

def send_blobs(blobs):
    # Packet: BLOB_MAGIC | count (1 byte) | per blob: cx(2) cy(2) roundness(4f)
    # count = len(blobs)
    payload = bytearray()
    # payload.append(count)
    # for b in blobs:
    #     payload += struct.pack('<HHf', b.cx(), b.cy(), b.roundness())
    payload = struct.pack('<HHf', blobs.cx(), blobs.cy(), blobs.roundness())
    vcp.write(BLOB_MAGIC)
    vcp.write(len(payload).to_bytes(4, 'little'))
    vcp.write(payload)

while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(
        RED_THRESHOLDS,
        pixels_threshold=1,
        area_threshold=1,
        merge=True,
        margin=10,
    )

    blobs_in_frame = []
    for blob in blobs:
        blb = Check_blob_in_frame(blob)
        if blb is not False:
            blobs_in_frame.append(blb)

    filtered = [b for b in blobs_in_frame if MIN_AREA <= b.pixels() <= MAX_AREA]
    detected = [b for b in filtered if b.roundness() > 0.4]

    for blob in detected:
        img.draw_circle(blob.cx(), blob.cy(), 5, color=(255, 0, 0), fill=True)
        send_blobs(blob)

    img.lens_corr(1.8)

    # send_frame(img)
    # send_blobs(detected)   # send blob data as separate packet (no print!)
