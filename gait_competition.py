# sudo apt install python3-lgpio
# Run: sudo python3 gait_competition.py
import time, math
import lgpio as lg

# ---- PCA9685 / I2C ----
I2C_BUS, ADDR = 1, 0x40
MODE1, MODE2, PRE_SCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H = 0xFA, 0xFB, 0xFC, 0xFD
SLEEP, AI, RESTART, OUTDRV = 1<<4, 1<<5, 1<<7, 1<<2

# ---- Your channel map ----
L_HIP, L_KNEE, L_ANK = 0, 1, 2       # ankle = ROLL
R_HIP, R_KNEE, R_ANK = 15, 14, 13    # ankle = ROLL

FREQ = 50
US_MIN, US_MAX, NEUTRAL_US = 1100, 1900, 1500  # conservative limits for comp

# ---- OE control (GPIO4 / pin 7) ----
USE_OE = True
OE_GPIO = 4
gpioh = None

# ---- TUNABLE PARAMETERS (edit these only) ----
HIP_SWING   = 16    # deg forward/back hip during swing
KNEE_SWING  = 32    # deg knee bend during swing
PRETILT     = 5     # deg ankle roll toward stance foot (L:+roll left, R:+roll right)
IDLE_KNEE   = 12    # deg knee bend in neutral (lower CoM)
DWELL_SHIFT = 0.45  # s hold after shifting weight
DWELL_SWING = 0.45  # s hold at peak swing
DWELL_PLACE = 0.40  # s hold after placing foot
SMOOTH_HZ   = 50    # interpolation update rate
CYCLES      = 9999  # how many left<->right cycles to run

# If any joint moves backwards, set invert True for that joint:
INVERT = {
    "L_HIP": False, "L_KNEE": False, "L_ANK": False,
    "R_HIP": False, "R_KNEE": False, "R_ANK": False,
}

# ---- low-level helpers ----
def w(h, r, v): lg.i2c_write_byte_data(h, r, v & 0xFF)
def r(h, r):    return lg.i2c_read_byte_data(h, r)

def pca_all_off(h):
    w(h, ALL_LED_ON_L, 0); w(h, ALL_LED_ON_H, 0)
    w(h, ALL_LED_OFF_L, 0); w(h, ALL_LED_OFF_H, 0x10)

def set_freq(h, f):
    om = r(h, MODE1)
    w(h, MODE1, om | SLEEP)
    ps = max(3, min(255, round(25_000_000/(4096*f)) - 1))
    w(h, PRE_SCALE, ps)
    w(h, MODE1, (om & ~SLEEP) | AI); time.sleep(0.001)
    w(h, MODE1, r(h, MODE1) | RESTART)

def set_us(h, ch, f, us):
    us = max(US_MIN, min(US_MAX, int(us)))
    cnt = int(us * 4096 * f / 1_000_000)
    base = LED0_ON_L + 4*ch
    w(h, base+0, 0); w(h, base+1, 0)
    w(h, base+2, cnt & 0xFF); w(h, base+3, (cnt>>8) & 0x0F)

def angle_to_us(angle_deg):
    # map -90..+90 deg -> US_MIN..US_MAX
    angle = max(-90, min(90, angle_deg))
    return int(US_MIN + (angle + 90) * (US_MAX - US_MIN) / 180)

def apply_joint(h, ch, name, deg):
    if INVERT[name]: deg = -deg
    set_us(h, ch, FREQ, angle_to_us(deg))

def write_pose_deg(h, Lhip, Lknee, Lank, Rhip, Rknee, Rank):
    apply_joint(h, L_HIP, "L_HIP", Lhip)
    apply_joint(h, L_KNEE,"L_KNEE",Lknee)
    apply_joint(h, L_ANK, "L_ANK", Lank)
    apply_joint(h, R_HIP, "R_HIP", Rhip)
    apply_joint(h, R_KNEE,"R_KNEE",Rknee)
    apply_joint(h, R_ANK, "R_ANK", Rank)

def smoothstep(t):
    t = max(0.0, min(1.0, t)); return t*t*(3-2*t)

def move_between(h, A, B, duration, Hz=50):
    steps = max(1, int(duration*Hz))
    for i in range(steps):
        t = smoothstep(i/(steps-1) if steps>1 else 1.0)
        pose = [a + (b-a)*t for a,b in zip(A,B)]
        write_pose_deg(h, *pose)
        time.sleep(1.0/Hz)

def oe_setup():
    global gpioh
    if not USE_OE: return
    for n in (0,1,2,3,4):
        try:
            gpioh = lg.gpiochip_open(n); break
        except lg.error: continue
    if gpioh is None: raise RuntimeError("No /dev/gpiochipN found")
    lg.gpio_claim_output(gpioh, OE_GPIO, 1)  # HIGH = disabled

def oe_enable():  (USE_OE and lg.gpio_write(gpioh, OE_GPIO, 0))
def oe_disable(): (USE_OE and lg.gpio_write(gpioh, OE_GPIO, 1))

# ---- Keyframe builders (avoid hand editing dozens of angles) ----
def neutral_pose():
    # knees slightly bent, hips neutral, ankles level
    return [0, IDLE_KNEE, 0,   0, IDLE_KNEE, 0]

def shift_left():
    # tilt body toward left foot (stance=L) using ankle roll pretlt
    return [ -4, IDLE_KNEE+2, +PRETILT,   +4, IDLE_KNEE, 0 ]

def shift_right():
    # tilt toward right foot (stance=R)
    return [ +4, IDLE_KNEE, 0,   -4, IDLE_KNEE+2, -PRETILT ]

def lift_right():
    # right leg swing: hip forward, knee bend; keep left as stance
    return [ -6, IDLE_KNEE+2, +PRETILT,   +HIP_SWING, KNEE_SWING, 0 ]

def place_right():
    # bring right down and reduce pretlt
    return [ -2, IDLE_KNEE+1, +PRETILT/2,   +4, IDLE_KNEE+8, 0 ]

def lift_left():
    # mirror of lift_right
    return [ +HIP_SWING, KNEE_SWING, 0,   -6, IDLE_KNEE+2, -PRETILT ]

def place_left():
    return [ +4, IDLE_KNEE+8, 0,   -2, IDLE_KNEE+1, -PRETILT/2 ]

# ---- MAIN ----
h = lg.i2c_open(I2C_BUS, ADDR)
try:
    oe_setup()
    pca_all_off(h)
    w(h, MODE2, OUTDRV)
    set_freq(h, FREQ)

    # Preload neutral, then enable
    write_pose_deg(h, *neutral_pose())
    oe_enable()
    time.sleep(0.6)

    print("Starting keyframe gait. Ctrl-C to stop.")
    for _ in range(CYCLES):
        # Double-support neutral
        move_between(h, neutral_pose(), shift_left(),  DWELL_SHIFT, SMOOTH_HZ)
        move_between(h, shift_left(),   lift_right(),  DWELL_SWING, SMOOTH_HZ)
        move_between(h, lift_right(),   place_right(), DWELL_PLACE, SMOOTH_HZ)
        move_between(h, place_right(),  neutral_pose(),0.30,        SMOOTH_HZ)

        move_between(h, neutral_pose(), shift_right(), DWELL_SHIFT, SMOOTH_HZ)
        move_between(h, shift_right(),  lift_left(),   DWELL_SWING, SMOOTH_HZ)
        move_between(h, lift_left(),    place_left(),  DWELL_PLACE, SMOOTH_HZ)
        move_between(h, place_left(),   neutral_pose(),0.30,        SMOOTH_HZ)

finally:
    print("Neutral & release...")
    try:
        write_pose_deg(h, *neutral_pose()); time.sleep(0.6)
        pca_all_off(h); oe_disable()
    except Exception: pass
    try: lg.i2c_close(h)
    except Exception: pass
    try:
        if gpioh is not None: lg.gpiochip_close(gpioh)
    except Exception: pass
