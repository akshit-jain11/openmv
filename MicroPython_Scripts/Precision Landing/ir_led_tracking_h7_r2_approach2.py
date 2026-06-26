# IR LED Tracking Script for OpenMV Cam H7 R2 (MT9M114 Sensor)
# ---------------------------------------------------------------
# Board   : OpenMV Cam H7 R2
# Sensor  : MT9M114 (640x480, max ~80 FPS at low resolution)
# Purpose : Detect and track a bright IR LED source using blob detection,
#           and compute horizontal / vertical bearing angles to the blob.
#
# ANGLE METHOD: atan2-based pinhole camera model
#   Uses focal length derived from FOV — same approach as ArUco/PnP scripts.
#   Outputs angles in RADIANS directly (no conversion needed before MAVLink).
#
#   angle_x = atan2( (cx - cx_center) / Fx,  1.0 )
#   angle_y = atan2( (cy - cy_center) / Fy,  1.0 )
#
#   Where:
#     Fx = (width  / 2) / tan(H_FOV_rad / 2)   ← focal length in pixels (horizontal)
#     Fy = (height / 2) / tan(V_FOV_rad / 2)   ← focal length in pixels (vertical)
#     cx_center = width  / 2                    ← principal point x
#     cy_center = height / 2                    ← principal point y
#
# WHY atan2 and not linear scaling:
#   Linear:  angle = (cx/w - 0.5) * FOV   — approximation, breaks at wide angles
#   atan2:   angle = atan2((cx - cx_c)/Fx, 1.0) — exact pinhole model, accurate
#            at all angles, consistent with ArUco/PnP/MAVLink convention.
#
# NOTE: The MT9M114 has a built-in 650nm IR cut filter on the stock lens.
# For best results with 850nm/940nm IR LEDs, physically remove the IR cut
# filter from your lens module, or replace the lens with an IR-pass lens.
#
# HOW IT WORKS:
#   Phase 1 (SEARCHING): Captures at QVGA (320x240) to scan the full FOV
#                        for a bright blob that could be an IR LED.
#   Phase 2 (TRACKING): Once a blob is found, switches to QQVGA (160x120)
#                        for maximum FPS and tracks the blob continuously.
#                        If the blob is lost, falls back to Phase 1.
#
# Sign convention (matches MAVLink / ArduPilot LANDING_TARGET):
#   angle_x  negative → beacon is LEFT  of boresight
#   angle_x  positive → beacon is RIGHT of boresight
#   angle_y  negative → beacon is ABOVE boresight
#   angle_y  positive → beacon is BELOW boresight
# ---------------------------------------------------------------

import sensor
import image
import time
import math
import machine
import struct
import os
import pyb

# ==============================================================
# CONFIGURATION
# ==============================================================

# Exposure in microseconds. Keep LOW to darken scene so only
# bright IR LEDs remain visible.
EXPOSURE_MICROSECONDS = 900

# Grayscale threshold — IR LED will be near 255
TRACKING_THRESHOLDS = [(240, 255)]

# --- Lens FOV (degrees) for OpenMV H7 R2 stock M12 lens ---
# These are the ACTUAL measured values for the MT9M114 sensor.
# Update if you replace the lens.
H_FOV = 47.5    # Horizontal field of view in degrees
V_FOV = 36.6    # Vertical   field of view in degrees

# Deadband — angles smaller than this (radians) are treated as
# "centred" and reported as 0.0. Prevents jitter from noisy
# centroid estimates.
# 1.5 degrees = 0.0262 radians
ANGLE_DEADBAND_RAD = math.radians(1.5)

# --- Searching Phase (QVGA = 320x240) ---
SEARCHING_FRAMESIZE       = sensor.QVGA
SEARCHING_AREA_THRESHOLD  = 4
SEARCHING_PIXEL_THRESHOLD = 4

# Fraction of half-width/height at which blob is flagged near edge
TRACKING_EDGE_TOLERANCE = 0.20

# --- MAVLink / UART setup ---
UART_BAUDRATE    = 115200
MAV_system_id    = 1
MAV_component_id = 0x54
packet_sequence  = 0

uart = machine.UART(3, UART_BAUDRATE, timeout_char=1000)

MAV_LANDING_TARGET_message_id = 149
MAV_LANDING_TARGET_frame      = 12
MAV_LANDING_TARGET_extra_crc  = 200
FIXED_DISTANCE_M              = 0.0   # Replace with rangefinder value if available

# --- Data logging ---
FILE_NAME = "PL_data.txt"

def find_file(file_name):
    if file_name in os.listdir():
        print("file exists")
        return 1
    else:
        print("File does not exist")
        return 0

def remove_file(file_name):
    if file_name in os.listdir():
        os.remove(file_name)
        print("Old file deleted")
    else:
        print("File does not exist")

def write_to_file(file_name, buffer):
    f = open(file_name, 'a')
    f.write(buffer)
    f.close()

# ==============================================================
# MAVLINK HELPERS
# ==============================================================

def checksum(data, extra):
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output

def send_ir_landing_target(angle_x_rad, angle_y_rad):
    """
    Send LANDING_TARGET MAVLink message.
    Accepts angles already in RADIANS — no conversion needed.
    """
    global packet_sequence

    temp = struct.pack(
        "<qfffffbb",
        0,                          # time_usec
        angle_x_rad,                # angle_x in radians
        angle_y_rad,                # angle_y in radians
        FIXED_DISTANCE_M,           # distance in metres
        0.0,                        # size_x
        0.0,                        # size_y
        0,                          # target_num
        MAV_LANDING_TARGET_frame,
    )
    temp = struct.pack(
        "<bbbbb30s",
        30,
        packet_sequence & 0xFF,
        MAV_system_id,
        MAV_component_id,
        MAV_LANDING_TARGET_message_id,
        temp,
    )
    temp = struct.pack(
        "<b35sh", 0xFE, temp, checksum(temp, MAV_LANDING_TARGET_extra_crc)
    )
    print(", packet seq %d" % packet_sequence)
    packet_sequence += 1
    uart.write(temp)

# ==============================================================
# FOCAL LENGTH — derived once at startup from FOV + resolution
# ==============================================================
# These are recomputed whenever the frame size changes.
# Call recompute_intrinsics() after any sensor.set_framesize().

def recompute_intrinsics():
    """
    Compute pinhole camera intrinsics from current sensor frame size
    and configured FOV values.

    Returns
    -------
    Fx, Fy         : focal lengths in pixels
    cx_c, cy_c     : principal point (frame centre)

    Formula:
        Fx = (width  / 2) / tan(H_FOV_rad / 2)
        Fy = (height / 2) / tan(V_FOV_rad / 2)

    This is the inverse of the FOV formula:
        H_FOV = 2 * atan( (width/2) / Fx )
    """
    w = sensor.width()
    h = sensor.height()

    Fx   = (w / 2.0) / math.tan(math.radians(H_FOV) / 2.0)
    Fy   = (h / 2.0) / math.tan(math.radians(V_FOV) / 2.0)
    cx_c = w / 2.0
    cy_c = h / 2.0

    print("Intrinsics | w=%d h=%d | Fx=%.1f Fy=%.1f | cx=%.1f cy=%.1f"
          % (w, h, Fx, Fy, cx_c, cy_c))
    return Fx, Fy, cx_c, cy_c

# ==============================================================
# SENSOR INITIALISATION
# ==============================================================

sensor.reset()
green_led = pyb.LED(2)
green_led.on()
time.sleep_ms(250)
green_led.off()
time.sleep_ms(250)
green_led.on()
time.sleep_ms(250)
green_led.off()
time.sleep_ms(250)
green_led.on()
time.sleep_ms(250)
green_led.off()
time.sleep_ms(250)
green_led.on()
time.sleep_ms(250)
green_led.off()
time.sleep_ms(250)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(SEARCHING_FRAMESIZE)      # QVGA = 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_MICROSECONDS)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=2000)

# Compute intrinsics for initial frame size (QVGA)
Fx, Fy, cx_c, cy_c = recompute_intrinsics()

clock = time.clock()

MIN_VALID_AREA = 81   # reject blobs outside this area
MAX_VALID_AREA = 240
MAX_BLOB_PERSISTANT_TIME = 500
# ==============================================================
# ANGLE CALCULATION  (atan2 pinhole model)
# ==============================================================
def compute_angles(blob_cx, blob_cy):
    """
    Convert blob centroid (blob_cx, blob_cy) in pixels to bearing
    angles in RADIANS using the pinhole camera model.

    Uses module-level Fx, Fy, cx_c, cy_c (recomputed on framesize change).

    Parameters
    ----------
    blob_cx : int   Blob centroid x pixel  (from blob.cx())
    blob_cy : int   Blob centroid y pixel  (from blob.cy())

    Returns
    -------
    angle_x : float   Horizontal bearing in radians
                      negative = LEFT,  positive = RIGHT
    angle_y : float   Vertical   bearing in radians
                      negative = ABOVE, positive = BELOW

    How it works
    ------------
    Step 1 — offset from principal point:
                dx = blob_cx - cx_c       (pixels, horizontal)
                dy = blob_cy - cy_c       (pixels, vertical)

    Step 2 — normalise by focal length:
                norm_x = dx / Fx          (unit-less, = tan of angle)
                norm_y = dy / Fy

    Step 3 — recover angle:
                angle_x = atan2(norm_x, 1.0)
                angle_y = atan2(norm_y, 1.0)

              The 1.0 is the normalised depth — after dividing by Fx
              the image plane sits exactly 1.0 unit from the optical
              centre, so atan2(offset, depth=1.0) gives the true angle.

    Example (QVGA, Fx=225):
        Blob at cx=240, cx_c=160  →  dx=80
        norm_x = 80/225 = 0.356
        angle_x = atan2(0.356, 1.0) = 0.342 rad = 19.6°  (RIGHT)
    """
    dx = blob_cx - cx_c
    dy = blob_cy - cy_c

    angle_x = math.atan2(dx / Fx, 1.0)
    angle_y = math.atan2(dy / Fy, 1.0)

    # Deadband — snap near-zero angles to 0 to suppress jitter
    if abs(angle_x) < ANGLE_DEADBAND_RAD:
        angle_x = 0.0
    if abs(angle_y) < ANGLE_DEADBAND_RAD:
        angle_y = 0.0

    return angle_x, angle_y


def angle_to_direction(angle_x, angle_y):
    """Human-readable direction string for console output."""
    h_dir = "LEFT"   if angle_x < -ANGLE_DEADBAND_RAD else \
            "RIGHT"  if angle_x >  ANGLE_DEADBAND_RAD else "CENTRE"
    v_dir = "ABOVE"  if angle_y < -ANGLE_DEADBAND_RAD else \
            "BELOW"  if angle_y >  ANGLE_DEADBAND_RAD else "CENTRE"
    return h_dir, v_dir

# ==============================================================
# HELPER FUNCTIONS
# ==============================================================

def get_blob_centroid_pixels(blob):
    """Returns blob centroid (cx, cy) in current frame pixel space."""
    return (blob.cx(), blob.cy())

def get_blob_width_height(blob):
    return (blob.w(), blob.h())

def get_blob_area(blob):
    return (blob.w() * blob.h())

def log_edge_warning(blob):
    """
    Prints a warning when the blob drifts close to the frame edge.
    Returns (x_err, y_err) in pixels for gimbal / servo use.
    """
    w_half = sensor.width()  / 2.0
    h_half = sensor.height() / 2.0

    x_err = blob.cx() - w_half
    y_err = blob.cy() - h_half

    w_thr = w_half * TRACKING_EDGE_TOLERANCE
    h_thr = h_half * TRACKING_EDGE_TOLERANCE

    near_edge = False
    if x_err < -w_thr:
        print("  <- Blob near LEFT edge  | x_err=%.1f" % x_err, end="  ")
        near_edge = True
    elif x_err > w_thr:
        print("  -> Blob near RIGHT edge | x_err=%.1f" % x_err, end="  ")
        near_edge = True
    if y_err < -h_thr:
        print("  ^ Blob near TOP edge    | y_err=%.1f" % y_err, end="  ")
        near_edge = True
    elif y_err > h_thr:
        print("  v Blob near BOTTOM edge | y_err=%.1f" % y_err, end="  ")
        near_edge = True
    if near_edge:
        print()

    return x_err, y_err

# ==============================================================
# MAIN LOOP
# ==============================================================
def is_valid_blob(blob):

    area = get_blob_area(blob)

    # area filter
    if area < MIN_VALID_AREA: #or area > MAX_VALID_AREA:
        return False

    return True

last_ms          = 0
last_snapshot_ms = 0
valid_blob_last_ms = 0
PRINT_INTERVAL_MS = 50   # 20 Hz = every 50 ms

print("IR LED Tracker — OpenMV H7 R2 / MT9M114  [atan2 mode]")
print("H_FOV=%.1f deg  V_FOV=%.1f deg  Deadband=+/-%.1f deg"
      % (H_FOV, V_FOV, math.degrees(ANGLE_DEADBAND_RAD)))
print("Searching for IR LED...")
# remove_file(FILE_NAME)
# img_id = 0

while True:
    clock.tick()

    img = sensor.snapshot()
    w   = sensor.width()
    h   = sensor.height()

    blobs = img.find_blobs(
        TRACKING_THRESHOLDS,
        area_threshold   = SEARCHING_AREA_THRESHOLD,
        pixels_threshold = SEARCHING_PIXEL_THRESHOLD,
        merge            = True,
    )

    # Filter out rangefinder blobs before picking the best one
    valid_blobs = [b for b in blobs if is_valid_blob(b)]

    if valid_blobs:

        if time.ticks_diff(time.ticks_ms(), valid_blob_last_ms) > MAX_BLOB_PERSISTANT_TIME:

            best = max(valid_blobs, key=lambda b: b.density())
            img.draw_rectangle(best.rect(), color=255)
            img.draw_cross(best.cx(), best.cy(), color=255)

            blob_cx, blob_cy = get_blob_centroid_pixels(best)
            blob_width, blob_height = get_blob_width_height(best)
            # ── Angle calculation (atan2 pinhole model) ──────────────
            angle_x_rad, angle_y_rad = compute_angles(blob_cx, blob_cy)
            h_dir, v_dir = angle_to_direction(angle_x_rad, angle_y_rad)

            if time.ticks_diff(time.ticks_ms(), last_ms) >= PRINT_INTERVAL_MS:
                print("TRACKING | FPS: %.1f | w:%d h:%d | "
                      "blob cx=%d cy=%d | width %d height %d | "
                      "density =%f | roundness =%f |"
                      "angle_x=%+.1f deg (%s)  angle_y=%+.1f deg (%s) | "
                      "dt=%d ms"
                      % (clock.fps(), w, h,
                         blob_cx, blob_cy,
                         blob_width, blob_height,
                         best.density(), best.roundness(),
                         math.degrees(angle_x_rad), h_dir,
                         math.degrees(angle_y_rad), v_dir,
                         time.ticks_diff(time.ticks_ms(), last_ms)))

                # Send MAVLink — already in radians, no conversion needed
                send_ir_landing_target(angle_x_rad, angle_y_rad)
                last_ms = time.ticks_ms()

                # save snap to sd card
                # if time.ticks_diff(time.ticks_ms(), last_snapshot_ms) >= 1000:
                #     # filename = "sd.jpg" % img_id
                #     filename = "debug.jpg"
                #     img.save(filename)
                #     img_id += 1  # Increment the counter
                #     last_snapshot_ms = time.ticks_ms()

    else:
        if time.ticks_diff(time.ticks_ms(), last_ms) >= PRINT_INTERVAL_MS:
            print("SEARCHING | FPS: %.1f | No blob" % clock.fps())
            last_ms = time.ticks_ms()
        valid_blob_last_ms = time.ticks_ms()
