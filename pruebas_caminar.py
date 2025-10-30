# sudo apt install python3-lgpio
# Run: sudo python3 gait_forward_bias.py
import time, math
import lgpio as lg

# ---- PCA9685 / I2C ----
I2C_BUS, ADDR = 1, 0x40
MODE1, MODE2, PRE_SCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H = 0xFA, 0xFB, 0xFC, 0xFD
SLEEP, AI, RESTART, OUTDRV = 1<<4, 1<<5, 1<<7, 1<<2

# ---- Channels ----
L_HIP, L_KNEE, L_ANK = 0, 1, 2        # ankle = ROLL
R_HIP, R_KNEE, R_ANK = 15, 14, 13     # ankle = ROLL
FREQ = 50

# Wider limits for hips/knees; tighter for ankles (roll)
LIMITS_US = {
    "L_HIP": (900, 2100),
    "L_KNEE": (900, 2100),
    "R_HIP": (900, 2100),
    "R_KNEE": (900, 2100),
    "L_ANK": (1100, 1900),
    "R_ANK": (1100, 1900),
}
OFFS_DEG = {k: 0 for k in LIMITS_US}  # per-joint neutral tweaks if needed

# ==================== T U N E   T H E S E ====================
# Forward motion ingredients
HIP_SWING_FWD = 36   # swing hip flexion (forward)
HIP_SWING_BACK= 18   # stance hip extension (back)  (smaller than forward!)
KNEE_SWING    = 58   # swing knee bend for clearance
GLOBAL_LEAN   = 4    # small global forward lean (+)=both hips flexed a bit

PRETILT       = 10   # ankle roll toward stance foot (lateral balance)
IDLE_KNEE     = 18   # bend both knees at neutral (lower CoM)

# Timing (more single support)
DWELL_SHIFT   = 0.28  # after pre-tilt/weight shift
DWELL_SWING   = 0.34  # keep single support longer to advance COM
DWELL_PLACE   = 0.22
SMOOTH_HZ     = 70
CYCLES        = 9999

# Inversions (you said L_HIP and L_KNEE are inverted)
INVERT = {
    "L_HIP": True,  "L_KNEE": True,  "L_ANK": False,
    "R_HIP": False, "R_KNEE": False, "R_ANK": False,
}
# =============================================================

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
    cnt = max(0, min(4095, int(us * 4096 * f / 1_000_000)))
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
    # add global forward lean to hips
    Lhip += GLOBAL_LEAN
    Rhip += GLOBAL_LEAN
    apply_joint(h, L_HIP, "L_HIP", Lhip)
    apply_joint(h, L_KNEE,"L_KNEE",Lknee)
    apply_joint(h, L_ANK, "L_ANK", Lank)
    apply_joint(h, R_HIP, "R_HIP", Rhip)
    apply_joint(h, R_KNEE,"R_KNEE",Rknee)
    apply_joint(h, R_ANK, "R_ANK", Rank)

def smoothstep(t): t=max(0,min(1,t)); return t*t*(3-2*t)

def move_between(h, A, B, duration, Hz=SMOOTH_HZ):
    steps = max(1, int(duration*Hz))
    for i in range(steps):
        t = smoothstep(i/(steps-1) if steps>1 else 1.0)
        pose = [a + (b-a)*t for a,b in zip(A,B)]
        write_pose_deg(h, *pose)
        time.sleep(1.0/Hz)

# ---- Keyframes (asymmetric sagittal, longer single support) ----
def neutral():
    return [0, IDLE_KNEE, 0,   0, IDLE_KNEE, 0]

def shift_left():
    # stance=L: pre-tilt left ankle, tiny hip counter-lean
    return [-6, IDLE_KNEE+3, +PRETILT,   +4, IDLE_KNEE, 0]

def shift_right():
    return [+4, IDLE_KNEE, 0,   -6, IDLE_KNEE+3, -PRETILT]

def lift_right():
    # R swing: hip forward big, stance hip slight back, swing knee bent
    return [-HIP_SWING_BACK, IDLE_KNEE+4, +PRETILT,
            +HIP_SWING_FWD,  KNEE_SWING,  0]

def place_right():
    # keep pretlt during early touchdown, then reduce
    return [-4, IDLE_KNEE+2, +PRETILT*0.6,
            +10, IDLE_KNEE+10, 0]

def lift_left():
    return [+HIP_SWING_FWD,  KNEE_SWING,  0,
            -HIP_SWING_BACK, IDLE_KNEE+4, -PRETILT]

def place_left():
    return [+10, IDLE_KNEE+10, 0,
            -4,  IDLE_KNEE+2,  -PRETILT*0.6]

# ---- Main ----
h = lg.i2c_open(I2C_BUS, ADDR)
try:
    w(h, MODE2, OUTDRV)
    set_freq(h, FREQ)
    pca_all_off(h)

    # preload neutral, enable (if you wire OE, add that control back)
    write_pose_deg(h, *neutral()); time.sleep(0.5)

    print("Forward-bias gait. Ctrl-C to stop.")
    for _ in range(CYCLES):
        move_between(h, neutral(),    shift_left(),  DWELL_SHIFT)
        move_between(h, shift_left(), lift_right(),  DWELL_SWING)   # single support L
        move_between(h, lift_right(), place_right(), DWELL_PLACE)
        move_between(h, place_right(),neutral(),     0.20)

        move_between(h, neutral(),    shift_right(), DWELL_SHIFT)
        move_between(h, shift_right(),lift_left(),   DWELL_SWING)   # single support R
        move_between(h, lift_left(),  place_left(),  DWELL_PLACE)
        move_between(h, place_left(), neutral(),     0.20)

finally:
    try:
        write_pose_deg(h, *neutral()); time.sleep(0.4)
        pca_all_off(h)
    except Exception:
        pass
    try: lg.i2c_close(h)
    except Exception:
        pass
