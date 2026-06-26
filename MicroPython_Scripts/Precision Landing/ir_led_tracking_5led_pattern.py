import machine
import sensor
import image
import time
import math
import machine
import struct
import os
import pyb

EXPOSURE_MICROSECONDS = 600
# sensor.reset()
# sensor.set_pixformat(sensor.GRAYSCALE)
# sensor.set_framesize(sensor.QVGA)
# # sensor.set_windowing((240, 240))  # 240x240 center pixels of VGA
# sensor.skip_frames(time=2000)
# sensor.set_auto_gain(False)  # must be turned off for color tracking
# sensor.set_auto_whitebal(False)  # must be turned off for color tracking
# sensor.set_auto_exposure(False, exposure_us=EXPOSURE_MICROSECONDS)
# clock = time.clock()

SEARCHING_FRAMESIZE       = sensor.QVGA
SEARCHING_AREA_THRESHOLD  = 2
SEARCHING_PIXEL_THRESHOLD = 2
TRACKING_THRESHOLDS = [(210, 255)]
# Fraction of half-width/height at which blob is flagged near edge

sensor.reset()
green_led = pyb.LED(2)
red_led = pyb.LED(1)
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


# Macros and varibales
IR_THRESHOLD = (220, 255)
MY_CC = MY_UL = MY_UR = MY_LL = MY_LR = None
Centre_Ir        = None
pattern_found    = False
valid_pattern_last_ms          = 0
debug_last_ms    = 0
packet_send_last_ms = 0
H_FOV = 47.5    # Horizontal field of view in degrees
V_FOV = 36.6    # Vertical   field of view in degrees
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
ANGLE_DEADBAND_RAD = math.radians(1.5)
MAVLINK_SEND_INTERVAL_MS = 50
MAX_PATTERN_PERSISTANT_TIME = 100
Debug_code = 1
# Function definations
clock = time.clock()

def distance(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx*dx + dy*dy)

def midpoint(x1,y1,x2,y2):
    return ((x1+x2)/2, (y1+y2)/2)

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

# 5 ir led pattern detection
def find_ir_led_pattern2(blobs):

    var1 = var2 = var3 = var4 = var5 = 0
    d = 0
    UL = UR = LL = LR = None
    UL_d = UR_d = LL_d = LR_d = 99999

    for center_blob in blobs:

        cx = center_blob.cx()
        cy = center_blob.cy()

        UL = UR = LL = LR = None
        UL_d = UR_d = LL_d = LR_d = 99999
        var2 = var3 = var4 = var5 = 0

        for b in blobs:

            if b == center_blob:
                continue

            x = b.cx()
            y = b.cy()
            d = distance(cx, cy, x, y)
            if x <= cx and y < cy:
                if d <= UL_d:
                    UL_d = d
                    UL = b
                else:
                    var2 = 1

            elif x >= cx and y < cy:
                if d <= UR_d:
                    UR_d = d
                    UR = b
                else:
                    var3 = 1

            elif x < cx and y >= cy:
                if d <= LL_d:
                    LL_d = d
                    LL = b
                else:
                    var4 = 1

            elif x > cx and y >= cy:
                if d <= LR_d:
                    LR_d = d
                    LR = b
                else:
                    var5 = 1

        # check corners exist
        if not (UL and UR and LL and LR):
            print(" %d, %d, %d, %d, %d, %d, %d, %d, %d \n" % (var2,var3,var4,var5,d,UL_d,UR_d,LL_d,LR_d))
            continue

        # symmetry check
        d1 = distance(cx, cy, UL.cx(), UL.cy())
        d2 = distance(cx, cy, UR.cx(), UR.cy())
        d3 = distance(cx, cy, LR.cx(), LR.cy())
        d4 = distance(cx, cy, LL.cx(), LL.cy())

        if min(d1,d2,d3,d4) == 0:
            continue

        ratio = max(d1,d2,d3,d4) / min(d1,d2,d3,d4)

        if ratio > 1.8:
            continue

        # diagonal check
        m1 = midpoint(UL.cx(), UL.cy(), LR.cx(), LR.cy())
        m2 = midpoint(UR.cx(), UR.cy(), LL.cx(), LL.cy())

        if distance(cx, cy, m1[0], m1[1]) > 10:
            continue

        if distance(cx, cy, m2[0], m2[1]) > 10:
            continue
        # print(" %d, %d, %d, %d\n" % (var2,var3,var4,var5))
        var1 = var2 = var3 = var4 = var5 = 0
        return True, center_blob, UL, UR, LL, LR
    # print(" %d, %d, %d, %d\n" % (var2,var3,var4,var5))
    return False, None, None, None, None, None

def find_ir_led_pattern(blobs):
    # print("!!!!! FIND PATTERN !!!!!\n")
    for center_blob in blobs:

        cx = center_blob.cx()
        cy = center_blob.cy()

        UL = UR = LL = LR = None
        UL_d = UR_d = LL_d = LR_d = 99999

        # Find closest blob in each quadrant
        for b in blobs:

            if b == center_blob:
                continue

            x = b.cx()
            y = b.cy()
            d = distance(cx, cy, x, y)

            if x <= cx and y < cy:
                if d < UL_d:
                    UL_d = d
                    UL = b

            elif x > cx and y <= cy:
                if d < UR_d:
                    UR_d = d
                    UR = b

            elif x < cx and y >= cy:
                if d < LL_d:
                    LL_d = d
                    LL = b

            elif x >= cx and y > cy:
                if d < LR_d:
                    LR_d = d
                    LR = b

        # Must have all 4 corners
        if not (UL and UR and LL and LR):
            #print("no corner\n")
            continue

        # symmetry check
        d1 = distance(cx, cy, UL.cx(), UL.cy())
        d2 = distance(cx, cy, UR.cx(), UR.cy())
        d3 = distance(cx, cy, LR.cx(), LR.cy())
        d4 = distance(cx, cy, LL.cx(), LL.cy())

        if min(d1,d2,d3,d4) == 0:
            continue

        ratio = max(d1,d2,d3,d4) / min(d1,d2,d3,d4)

        # if ratio > 1.8:
        #     continue

        # print("ratio ",ratio)

        # Calculate mid point
        mid_point1_x, mid_point1_y = midpoint(UL.cx(), UL.cy(), LR.cx(), LR.cy())
        mid_point2_x, mid_point2_y = midpoint(UR.cx(), UR.cy(), LL.cx(), LL.cy())

        mid_point_avg_x = (mid_point1_x + mid_point2_x)/2
        mid_point_avg_y = (mid_point1_y + mid_point2_y)/2

        # print("avg_mid_point ",mid_point_avg_x,mid_point_avg_y)

        mid_distance = distance(cx, cy, mid_point_avg_x, mid_point_avg_y)
        # if mid_distance > 10:
        #     continue

        # print("Valid Pattern Found ",mid_distance)
        return True,center_blob,UL,UR,LL,LR

    # print("No Valid Pattern Found\n")
    return False,None,None,None,None,None

# currently using
def find_ir_led_pattern3(blobs):

    if len(blobs) < 5:
        return False, None, None, None, None, None, None, None

    for center_blob in blobs:

        cx = center_blob.cx()
        cy = center_blob.cy()

        UL = UR = LL = LR = None
        UL_d = UR_d = LL_d = LR_d = 99999

        for b in blobs:

            if b is center_blob:
                continue

            x = b.cx()
            y = b.cy()
            d = distance(cx, cy, x, y)
            # print("x -> %d, y -> %d, cx -> %d, cy -> %d\n" % (x,y,cx,cy))
            if   x < cx and y <= cy and d < UL_d:
                UL_d, UL = d, b
            elif x > cx and y <= cy and d < UR_d:
                UR_d, UR = d, b
            elif x <= cx and y > cy and d < LL_d:
                LL_d, LL = d, b
            elif x >= cx and y > cy and d < LR_d:
                LR_d, LR = d, b

        if not (UL and UR and LL and LR):
            continue

        d1 = distance(cx, cy, UL.cx(), UL.cy())
        d2 = distance(cx, cy, UR.cx(), UR.cy())
        d3 = distance(cx, cy, LR.cx(), LR.cy())
        d4 = distance(cx, cy, LL.cx(), LL.cy())

        if min(d1, d2, d3, d4) == 0:
            continue

        ratio = max(d1, d2, d3, d4) / min(d1, d2, d3, d4)
        if ratio > 1.8:
            continue

        mid1_x, mid1_y = midpoint(UL.cx(), UL.cy(), LR.cx(), LR.cy())
        mid2_x, mid2_y = midpoint(UR.cx(), UR.cy(), LL.cx(), LL.cy())

        avg_mid_x = (mid1_x + mid2_x) / 2.0
        avg_mid_y = (mid1_y + mid2_y) / 2.0

        mid_distance = distance(cx, cy, avg_mid_x, avg_mid_y)
        if mid_distance > 10:
            continue

        # Valid pattern found
        return True, center_blob, UL, UR, LL, LR, ratio, mid_distance

    #no pattern found
    return False, None, None, None, None, None, None, None

def find_ir_led_pattern_angle(blobs):

    # var1 = var2 = var3 = var4 = var5 = 0
    # d = 0
    # UL = UR = LL = LR = None
    # UL_d = UR_d = LL_d = LR_d = 99999

    for center_blob in blobs:

        cx = center_blob.cx()
        cy = center_blob.cy()

        UL = UR = LL = LR = None
        UL_d = UR_d = LL_d = LR_d = 99999
        # var2 = var3 = var4 = var5 = 0

        for b in blobs:
            if b == center_blob:
                continue

            x = b.cx()
            y = b.cy()
            d = distance(cx, cy, x, y)
            angle = math.atan2(y - cy, x - cx)

            if angle > -math.pi and angle <= -math.pi/2:
                if d <= UL_d:
                    UL_d = d
                    UL = b
                # else:
                #     var2 = 1

            elif angle > -math.pi/2 and angle <= 0:
                if d <= UR_d:
                    UR_d = d
                    UR = b
                # else:
                #     var3 = 1

            elif angle > 0 and angle <= math.pi/2:
                if d <= LR_d:
                    LR_d = d
                    LR = b
                # else:
                #     var5 = 1

            elif angle > math.pi/2 and angle <= math.pi:
                if d <= LL_d:
                    LL_d = d
                    LL = b
            #     else:
            #         var4 = 1
            # print("x -> %d, y-> %d, d -> %d, angle -> %d, cx -> %d, cy -> %d\n" % (x, y, d, angle, cx, cy))

        if not (UL and UR and LL and LR):
            continue

        d1 = distance(cx, cy, UL.cx(), UL.cy())
        d2 = distance(cx, cy, UR.cx(), UR.cy())
        d3 = distance(cx, cy, LR.cx(), LR.cy())
        d4 = distance(cx, cy, LL.cx(), LL.cy())

        if min(d1,d2,d3,d4) == 0:
            continue

        ratio = max(d1,d2,d3,d4) / min(d1,d2,d3,d4)
        # if ratio > 1.8:
        #     continue

        m1 = midpoint(UL.cx(), UL.cy(), LR.cx(), LR.cy())
        m2 = midpoint(UR.cx(), UR.cy(), LL.cx(), LL.cy())

        # if distance(cx, cy, m1[0], m1[1]) > 10:
        #     continue
        # if distance(cx, cy, m2[0], m2[1]) > 10:
        #     continue

        # var1 = var2 = var3 = var4 = var5 = 0
        return True, center_blob, UL, UR, LL, LR

    return False, None, None, None, None, None

def find_ir_led_pattern_angle_dynamic(blobs):

    if len(blobs) < 5:
        return False, None, None, None, None, None

    for center_blob in blobs:
        cx = center_blob.cx()
        cy = center_blob.cy()

        candidates = []
        for b in blobs:
            if b == center_blob:
                continue
            x = b.cx()
            y = b.cy()
            d = distance(cx, cy, x, y)
            angle = math.atan2(y - cy, x - cx)
            candidates.append((d, angle, b))

        if len(candidates) < 4:
            continue

        # take 4 closest blobs as corners
        candidates.sort(key=lambda t: t[0])
        corners = candidates[:4]

        # symmetry check on distance
        dists = [c[0] for c in corners]
        if min(dists) == 0:
            continue
        if max(dists) / min(dists) > 1.8:
            continue

        # sort by angle to check angular spread
        corners_sorted = sorted(corners, key=lambda t: t[1])
        angles = [c[1] for c in corners_sorted]

        gaps = []
        for i in range(4):
            gap = angles[(i+1) % 4] - angles[i]
            if gap < 0:
                gap += 2*math.pi
            gaps.append(gap)

        # each gap should be roughly 90°, allow tolerance
        if min(gaps) < math.radians(40) or max(gaps) > math.radians(140):
            continue

        # assign UL/UR/LR/LL based on sorted angular order
        # order is: most negative angle (top) -> ... -> most positive (going clockwise in image space)
        UL, UR, LR, LL = corners_sorted[0][2], corners_sorted[1][2], corners_sorted[2][2], corners_sorted[3][2]

        d1 = distance(cx, cy, UL.cx(), UL.cy())
        d2 = distance(cx, cy, UR.cx(), UR.cy())
        d3 = distance(cx, cy, LR.cx(), LR.cy())
        d4 = distance(cx, cy, LL.cx(), LL.cy())

        m1 = midpoint(UL.cx(), UL.cy(), LR.cx(), LR.cy())
        m2 = midpoint(UR.cx(), UR.cy(), LL.cx(), LL.cy())

        if distance(cx, cy, m1[0], m1[1]) > 10:
            continue
        if distance(cx, cy, m2[0], m2[1]) > 10:
            continue

        return True, center_blob, UL, UR, LL, LR

    return False, None, None, None, None, None

def find_ir_led_pattern_final(blobs):

    if len(blobs) < 5:
        return False, None, None, None, None, None

    TOLERANCE_DEG = 9   # 10% of 90 degrees

    for center_blob in blobs:
        cx = center_blob.cx()
        cy = center_blob.cy()

        UL = UR = LL = LR = None
        UL_d = UR_d = LL_d = LR_d = 99999

        for b in blobs:
            if b == center_blob:
                continue

            x = b.cx()
            y = b.cy()
            d = distance(cx, cy, x, y)
            angle_rad = math.atan2(y - cy, x - cx)
            angle_deg = math.degrees(angle_rad)

            if angle_deg <= (-90 + TOLERANCE_DEG):
                if d < UL_d:
                    UL_d = d; UL = b
            elif angle_deg <= (0 + TOLERANCE_DEG):
                if d < UR_d:
                    UR_d = d; UR = b
            elif angle_deg <= (90 + TOLERANCE_DEG):
                if d < LR_d:
                    LR_d = d; LR = b
            else:
                if d < LL_d:
                    LL_d = d; LL = b

        if not (UL and UR and LL and LR):
            continue

        d1 = distance(cx, cy, UL.cx(), UL.cy())
        d2 = distance(cx, cy, UR.cx(), UR.cy())
        d3 = distance(cx, cy, LR.cx(), LR.cy())
        d4 = distance(cx, cy, LL.cx(), LL.cy())

        if min(d1,d2,d3,d4) == 0:
            continue

        ratio = max(d1,d2,d3,d4) / min(d1,d2,d3,d4)
        if ratio > 1.8:
            continue

        m1 = midpoint(UL.cx(), UL.cy(), LR.cx(), LR.cy())
        m2 = midpoint(UR.cx(), UR.cy(), LL.cx(), LL.cy())

        if distance(cx, cy, m1[0], m1[1]) > 10:
            continue
        if distance(cx, cy, m2[0], m2[1]) > 10:
            continue

        return True, center_blob, UL, UR, LL, LR

    return False, None, None, None, None, None

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
        "<qfffffbbfff4fBB",
        0,                          # time_usec
        angle_x_rad,                # angle_x in radians
        angle_y_rad,                # angle_y in radians
        FIXED_DISTANCE_M,           # distance in metres
        0.0,                        # size_x
        0.0,                        # size_y
        0,                          # target_num
        MAV_LANDING_TARGET_frame,
        0.0,
        0.0,
        0.0,
        1.0, 0.0, 0.0, 0.0,
        0,
        0,
    )
    temp = struct.pack(
        "<bbbbb60s",
        60,
        packet_sequence & 0xFF,
        MAV_system_id,
        MAV_component_id,
        MAV_LANDING_TARGET_message_id,
        temp,
    )
    temp = struct.pack(
        "<b65sH", 0xFE, temp, checksum(temp, MAV_LANDING_TARGET_extra_crc)
    )
    #print(", packet seq %d" % packet_sequence)
    packet_sequence += 1
    uart.write(temp)

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

Fx, Fy, cx_c, cy_c = recompute_intrinsics()
edgecase = []

while True:

    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(
        TRACKING_THRESHOLDS,
        area_threshold   = SEARCHING_AREA_THRESHOLD,
        pixels_threshold = SEARCHING_PIXEL_THRESHOLD,
        merge            = True,
        x_stride         = 1,    # don't skip pixels — catch small far blobs
        y_stride         = 1,
    )
    # print("len %d\n" % (len(blobs)))
    # for b in blobs:
    #     img.draw_rectangle(b.rect(), color=255)

    Valid_pattern,Centre_Ir,MY_UL,MY_UR,MY_LL,MY_LR= find_ir_led_pattern_angle_dynamic(blobs)

    if Valid_pattern == True:
        if time.ticks_diff(time.ticks_ms(), valid_pattern_last_ms) > MAX_PATTERN_PERSISTANT_TIME:
            angle_x_rad, angle_y_rad = compute_angles(Centre_Ir.cx(), Centre_Ir.cy())
            h_dir, v_dir = angle_to_direction(angle_x_rad, angle_y_rad)

            if time.ticks_diff(time.ticks_ms(), packet_send_last_ms) >= MAVLINK_SEND_INTERVAL_MS:
                if Debug_code:
                    print("Centre_Ir x - %d, y - %d | angle_x=%+.1f deg (%s)  angle_y=%+.1f deg (%s)\n" % (Centre_Ir.cx(),Centre_Ir.cy(),angle_x_rad, h_dir,angle_y_rad, v_dir))
                send_ir_landing_target(angle_x_rad, angle_y_rad)
                green_led.toggle()
                red_led.off()
                packet_send_last_ms = time.ticks_ms()

            if Debug_code:
                img.draw_circle(Centre_Ir.cx(), Centre_Ir.cy(), 8, thickness=2, color=255)
                img.draw_cross(Centre_Ir.cx(), Centre_Ir.cy(), size=10, thickness=2, color=255)

    else:
        # print("ratio %d, mid_distance %d\n" % (my_ratio,my_mid_distance))
        valid_pattern_last_ms = time.ticks_ms()
        if time.ticks_diff(time.ticks_ms(), debug_last_ms) > 500:
            red_led.toggle()
            green_led.off()
            if Debug_code:
                print("No Pattern Found\n")
                # i = 0
                # for b in blobs:
                #     i = i + 1
                #     print("% - dx -> %d, Y -> %d\n" % (i,b.cx(),b.cy()))

            debug_last_ms = time.ticks_ms()

    if Debug_code:
        print("FPS: ", clock.fps())
