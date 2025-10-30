# sudo apt install python3-lgpio
# Run: sudo python3 gait_competition_tuned.py
import time, math
import lgpio as lg

# ---- PCA9685 / I2C ----
I2C_BUS, ADDR = 1, 0x40
MODE1, MODE2, PRE_SCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H = 0xFA, 0xFB, 0xFC, 0xFD
SLEEP, AI, RESTART, OUTDRV = 1<<4, 1<<5, 1<<7, 1<<2

# ---- Channel map (your wiring)
L_HIP, L_KNEE, L_ANK = 0, 1, 2        # ankle = ROLL
R_HIP, R_KNEE, R_ANK = 15, 14, 13     # ankle = ROLL

FREQ = 50

# ---- SAFETY LIMITS (wider for hips/knees; tighter for ankles)
LIMITS_US = {
    "L_HIP": (900, 2100),
    "L_KNEE": (900, 2100),
    "R_HIP": (900, 2100),
    "R_KNEE": (900, 2100),
    "L_ANK": (1100, 1900),
    "R_ANK": (1100, 1900),
}
# If neutral (1500 µs) is not truly "0°", add small offsets here (deg):
OFFS_DEG = {k: 0 for k in LIMITS_US}

# ---- OE (Output Enable) on GPIO4 (pin 7)
USE_OE = True
OE_GPIO = 4
gpioh = None

# ===================== T U N A B L E S =====================
# Make changes here only (small increments!)
HIP_SWING   = 32     # deg forward/back hip during swing (was 16 -> 32)
KNEE_SWING  = 55     # deg knee bend during swing (was 32 -> 55)
PRETILT     = 10     # deg ankle roll toward stance foot (was 5 -> 10)
IDLE_KNEE   = 18     # deg knee bend at neutral to lower CoM (was 12 -> 18)

# Timings (faster, snappier)
DWELL_SHIFT = 0.28   # s hold after shifting weight
DWELL_SWING = 0.28   # s hold at peak swing
DWELL_PLACE = 0.24   # s settle after placing foot
SMOOTH_HZ   = 70     # interpolation update rate (was 50)
CYCLES      = 9999   # run until Ctrl-C

# If a joint moves the wrong way, flip its sign here:
INVERT = {
    "L_HIP": False, "L_KNEE": False, "L_ANK": False,
    "R_HIP": False, "R_KNEE": False, "R_ANK": False,
}
# ===========================================================

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
    us = max(0, min(4095, int(us)))
    cnt = int(us * 4096 * f / 1_000_000)
    base = LED0_ON_L + 4*ch
    w(h, base+0, 0); w(h, base+1, 0)
    w(h, base+2, cnt & 0xFF); w(h, base+3, (cnt>>8) & 0x0F)

def angle_to_us_joint(name, deg):
    deg += OFFS_DEG[name]
    deg = max(-90, min(90, deg))
    umin, umax = LIMITS_US[name]
    return int(umin + (deg + 90) * (umax - umin) / 180)

def apply_joint(h, ch, name, deg):
    if INVERT[name]: deg = -deg
    set_us(h, ch, FREQ, angle_to_us_joint(name, deg))

def write_pose_deg(h, Lhip, Lknee, Lank, Rhip, Rknee, Rank):
    apply_joint(h, L_HIP, "L_HIP", Lhip)
    apply_joint(h, L_KNEE,"L_KNEE",Lknee)
    apply_joint(h, L_ANK, "L_ANK", Lank)
    apply_joint(h, R_HIP, "R_HIP", Rhip)
    apply_joint(h, R_KNEE,"R_KNEE",Rknee)
    apply_joint(h, R_ANK, "R_ANK", Rank)

def smoothstep(t):
    t = max(0.0, min(1.0, t)); return t*t*(3-2*t)

def move_between(h, A, B, duration, Hz=SMOOTH_HZ):
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

# --- Keyframes (built from tunables) ---
def neutral_pose():
    return [0, IDLE_KNEE, 0,   0, IDLE_KNEE, 0]

def shift_left():
    # stronger pre-tilt & slight torso counter-lean (hips +/-)
    return [ -6, IDLE_KNEE+3, +PRETILT,   +6, IDLE_KNEE, 0 ]

def shift_right():
    return [ +6, IDLE_KNEE, 0,   -6, IDLE_KNEE+3, -PRETILT ]

def lift_right():
    return [ -8, IDLE_KNEE+3, +PRETILT,   +HIP_SWING, KNEE_SWING, 0 ]

def place_right():
    return [ -2, IDLE_KNEE+1, +PRETILT/2,   +6, IDLE_KNEE+10, 0 ]

def lift_left():
    return [ +HIP_SWING, KNEE_SWING, 0,   -8, IDLE_KNEE+3, -PRETILT ]

def place_left():
    return [ +6, IDLE_KNEE+10, 0,   -2, IDLE_KNEE+1, -PRETILT/2 ]

# ---- Main ----
h = lg.i2c_open(I2C_BUS, ADDR)
try:
    # Init
    w(h, MODE2, OUTDRV)
    set_freq(h, FREQ)
    pca_all_off(h)
    oe_setup()

    # Preload neutral, then enable outputs
    write_pose_deg(h, *neutral_pose())
    oe_enable()
    time.sleep(0.5)

    print("Tuned gait running. Ctrl-C to stop.")
    for _ in range(CYCLES):
        move_between(h, neutral_pose(), shift_left(),  DWELL_SHIFT)
        move_between(h, shift_left(),   lift_right(),  DWELL_SWING)
        move_between(h, lift_right(),   place_right(), DWELL_PLACE)
        move_between(h, place_right(),  neutral_pose(),0.22)

        move_between(h, neutral_pose(), shift_right(), DWELL_SHIFT)
        move_between(h, shift_right(),  lift_left(),   DWELL_SWING)
        move_between(h, lift_left(),    place_left(),  DWELL_PLACE)
        move_between(h, place_left(),   neutral_pose(),0.22)

finally:
    try:
        write_pose_deg(h, *neutral_pose()); time.sleep(0.5)
        pca_all_off(h); oe_disable()
    except Exception: pass
    try: lg.i2c_close(h)
    except Exception: pass
    try:
        if gpioh is not None: lg.gpiochip_close(gpioh)
    except Exception: pass
