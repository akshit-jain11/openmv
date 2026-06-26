"""
Red LED Detector — OpenMV H7 R2
Detects red LED blobs in the camera frame and draws a rectangle around them.

Tuning tips:
  - Adjust RED_THRESHOLDS if your LED isn't detected well. Use the
    OpenMV IDE's "Threshold Editor" (Tools → Machine Vision → Threshold Editor)
    to find the right LAB values for your specific LED and lighting.
  - Increase MIN_PIXELS if you're getting false positives from background red.
  - Lower MIN_PIXELS if the LED blob is small / far away.
"""

import sensor
import time
import rpc
import image
import pyb

# ── Camera setup ──────────────────────────────────────────────────────────────
sensor.reset()
sensor.set_pixformat(sensor.RGB565)   # Colour mode
sensor.set_framesize(sensor.QVGA)     # 320×240 — good balance of speed/detail
sensor.set_vflip(False)               # Flip vertically if your mount is upside-down
sensor.set_hmirror(False)             # Mirror horizontally if needed
sensor.skip_frames(time=2000)         # Let auto-exposure settle

clock = time.clock()

# ── Red colour threshold (LAB colour space) ───────────────────────────────────
# Format: (L_min, L_max, A_min, A_max, B_min, B_max)
# High A+ values = red. Tweak L range to handle bright LED saturation.
# RED_THRESHOLDS = [
#     (10, 100, 30, 127, -40, 50),   # General red / deep red
# ]
# RED_THRESHOLDS = [
#     (10, 80, 40, 127, -20, 50),
# ]


# RED_THRESHOLDS = [
#     (10, 100, 10, 127, -20, 50),   # Wide L range, moderate A
# ]

# RED_THRESHOLDS = [
#     (30, 100, 27, 127, 0, 127)
# ]

RED_THRESHOLDS = [
    (30, 100, 27, 127, -44, 127)
]


# ── Detection parameters ───────────────────────────────────────────────────────
MIN_PIXELS = 8     # Ignore blobs smaller than this (noise filter)
MIN_AREA = 1
MAX_AREA = 100
Target_Frame_x = [4, 313]
Target_Frame_y = [36, 203]

MAX_OVERLAP = 0.5           # Merge blobs that overlap by more than this fraction
RECT_COLOR  = (255, 0, 0)   # Rectangle colour: red (R, G, B)
RECT_THICK  = 2             # Border thickness in pixels
LABEL_COLOR = (255, 255, 0) # Label text colour: yellow


def Check_blob_in_frame(blob):
    if Target_Frame_x[0] <= blob.cx() <= Target_Frame_x[1]:
        if Target_Frame_y[0] <= blob.cy() <= Target_Frame_y[1]:
            return blob

    return False

# ── Main loop ─────────────────────────────────────────────────────────────────


while True:

    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(
         RED_THRESHOLDS,
         pixels_threshold=1,    # Get everything, filter below
         area_threshold=1,
         merge=True,
         margin=10,
     )

    bloba_in_frame = []

    for blob in blobs:
        blb = Check_blob_in_frame(blob)
        if blb is not False:
            bloba_in_frame.append(blb)

    filtered = [b for b in bloba_in_frame if MIN_AREA <= b.pixels() <= MAX_AREA]

    if filtered:
        # Optional: pick only the largest blob (most likely the actual LED)
        # largest = max(blobs, key=lambda b: b.pixels())

        # Draw rectangle around every detected red blob
        for blob in filtered:

            if blob.roundness() > 0.4:

                print("Red Point cx -> %d, cy -> %d, %f\n" % (blob.cx(), blob.cy(), blob.roundness()))

                img.draw_circle(
                    blob.cx(),
                    blob.cy(),
                    5,                      # radius (adjust as needed)
                    color=(255, 0, 0),      # red color
                    fill=True               # makes it solid
                )

    img.lens_corr(1.8)  # dewarp the image
