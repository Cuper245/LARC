# sudo apt install python3-lgpio
# Run as: sudo python3 crawl_recovery.py
import time
import lgpio as lg

# ================= PCA9685 / I2C setup =================
I2C_BUS, ADDR = 1, 0x40
MODE1, MODE2, PRE_SCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H = 0xFA, 0xFB, 0xFC, 0xFD
SLEEP, AI, RESTART, OUTDRV = 1 << 4, 1 << 5, 1 << 7, 1 << 2

# ================= Channels (adjust to wiring) =================
L_HIP, L_KNEE, L_ANK = 0, 1, 2
L_ARM = 3
R_ARM = 12
R_ANK, R_KNEE, R_HIP = 13, 14, 15  # consistent with walking script
OE_GPIO = 4
FREQ = 50

# ================= Servo limits and config =================
LIMITS_US = {
    "L_HIP": (900, 2100),
    "L_KNEE": (900, 2100),
    "R_HIP": (900, 2100),
    "R_KNEE": (900, 2100),
    "L_ANK": (1100, 1900),
    "R_ANK": (1100, 1900),
    "L_ARM": (900, 2100),
    "R_ARM": (900, 2100),
}
OFFS_DEG = {k: 0 for k in LIMITS_US}
INVERT = {
    "L_HIP": True,  "L_KNEE": True,  "L_ANK": False,
    "R_HIP": False, "R_KNEE": False, "R_ANK": False,
    "L_ARM": False, "R_ARM": False,
}

# ================= Crawling motion tuning =================
ARM_UP_DEG = +55       # arm raised (body up)
ARM_DOWN_DEG = -65     # arm pushes down (body forward)
LEG_STRAIGHT_HIP = 0   # hips neutral when straight
LEG_STRAIGHT_KNEE = 12 # mild bend for support
LEG_RETRACT_HIP = -32  # hips pull back on retract
LEG_RETRACT_KNEE = 60  # knees bend on retract
ANK_NEUTRAL = 0

SMOOTH_HZ = 70
MOVE_TIME = 0.45
DWELL = 0.25
CYCLES = 9999

# ================= Helper functions =================
def w(h, r, v): lg.i2c_write_byte_data(h, r, v & 0xFF)
def rbyte(h, r): return lg.i2c_read_byte_data(h, r)

def pca_all_off(h):
    w(h, ALL_LED_ON_L, 0); w(h, ALL_LED_ON_H, 0)
    w(h, ALL_LED_OFF_L, 0); w(h, ALL_LED_OFF_H, 0x10)

def set_freq(h, f):
    om = rbyte(h, MODE1)
    w(h, MODE1, om | SLEEP)
    ps = max(3, min(255, round(25_000_000/(4096*f)) - 1))
    w(h, PRE_SCALE, ps)
    w(h, MODE1, (om & ~SLEEP) | AI)
    time.sleep(0.001)
    w(h, MODE1, rbyte(h, MODE1) | RESTART)

def angle_to_us_joint(name, deg):
    deg += OFFS_DEG[name]
    deg = max(-90, min(90, deg))
    umin, umax = LIMITS_US[name]
    return int(umin + (deg + 90) * (umax - umin) / 180)

def set_us(h, ch, f, us):
    cnt = max(0, min(4095, int(us * 4096 * f / 1_000_000)))
    base = LED0_ON_L + 4 * ch
    w(h, base + 0, 0); w(h, base + 1, 0)
    w(h, base + 2, cnt & 0xFF); w(h, base + 3, (cnt >> 8) & 0x0F)

def apply_joint(h, ch, name, deg):
    if INVERT[name]:
        deg = -deg
    us = angle_to_us_joint(name, deg)
    set_us(h, ch, FREQ, us)

def write_pose_deg(h,
                   Lhip, Lknee, Lank,
                   Rhip, Rknee, Rank,
                   Larm, Rarm):
    apply_joint(h, L_HIP, "L_HIP", Lhip)
    apply_joint(h, L_KNEE, "L_KNEE", Lknee)
    apply_joint(h, L_ANK, "L_ANK", Lank)
    apply_joint(h, R_HIP, "R_HIP", Rhip)
    apply_joint(h, R_KNEE, "R_KNEE", Rknee)
    apply_joint(h, R_ANK, "R_ANK", Rank)
    apply_joint(h, L_ARM, "L_ARM", Larm)
    apply_joint(h, R_ARM, "R_ARM", Rarm)

def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)

def move_between(h, A, B, duration, Hz=SMOOTH_HZ):
    steps = max(1, int(duration * Hz))
    for i in range(steps):
        t = smoothstep(i / (steps - 1) if steps > 1 else 1.0)
        pose = [a + (b - a) * t for a, b in zip(A, B)]
        write_pose_deg(h, *pose)
        time.sleep(1.0 / Hz)

# ================= OE GPIO Handling =================
gpioh = None
def oe_setup():
    global gpioh
    for n in range(5):
        try:
            gpioh = lg.gpiochip_open(n)
            break
        except lg.error:
            continue
    if gpioh is None:
        raise RuntimeError("No /dev/gpiochipN found")
    lg.gpio_claim_output(gpioh, OE_GPIO, 1)
def oe_enable():  lg.gpio_write(gpioh, OE_GPIO, 0)
def oe_disable(): lg.gpio_write(gpioh, OE_GPIO, 1)

# ================= Key poses =================
def pose_arms_up_legs_straight():
    return [
        LEG_STRAIGHT_HIP, LEG_STRAIGHT_KNEE, ANK_NEUTRAL,
        LEG_STRAIGHT_HIP, LEG_STRAIGHT_KNEE, ANK_NEUTRAL,
        ARM_UP_DEG, ARM_UP_DEG
    ]

def pose_arms_down_legs_retract():
    return [
        LEG_RETRACT_HIP, LEG_RETRACT_KNEE, ANK_NEUTRAL,
        LEG_RETRACT_HIP, LEG_RETRACT_KNEE, ANK_NEUTRAL,
        ARM_DOWN_DEG, ARM_DOWN_DEG
    ]

def pose_neutral():
    return [0, 15, 0, 0, 15, 0, 0, 0]

# ================= Main loop =================
h = lg.i2c_open(I2C_BUS, ADDR)
try:
    oe_setup()
    pca_all_off(h)
    w(h, MODE2, OUTDRV)
    set_freq(h, FREQ)

    # preload neutral pose
    write_pose_deg(h, *pose_neutral())
    oe_enable()
    time.sleep(0.4)

    print("Crawling recovery loop. Ctrl-C to stop.")
    for _ in range(CYCLES):
        move_between(h, pose_neutral(), pose_arms_up_legs_straight(), MOVE_TIME)
        time.sleep(DWELL)

        move_between(h, pose_arms_up_legs_straight(), pose_arms_down_legs_retract(), MOVE_TIME)
        time.sleep(DWELL)

        move_between(h, pose_arms_down_legs_retract(), pose_neutral(), MOVE_TIME)
        time.sleep(DWELL)

finally:
    try:
        write_pose_deg(h, *pose_neutral()); time.sleep(0.3)
        pca_all_off(h)
    except Exception:
        pass
    try: oe_disable()
    except Exception:
        pass
    try: lg.i2c_close(h)
    except Exception:
        pass
    try:
        if gpioh is not None:
            lg.gpiochip_close(gpioh)
    except Exception:
        pass
