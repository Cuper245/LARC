# Requires: sudo apt install python3-lgpio
# Run as: sudo python3 one_step_shift_then_step.py
import time
import lgpio as lg

# I2C / PCA9685 setup
I2C_BUS, ADDR = 1, 0x40
MODE1, MODE2, PRE_SCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H = 0xFA, 0xFB, 0xFC, 0xFD
SLEEP, AI, RESTART, OUTDRV = 1<<4, 1<<5, 1<<7, 1<<2

# Channels
L_HIP, L_KNEE, L_ANK = 0, 1, 2
R_HIP, R_KNEE, R_ANK = 15, 14, 13
L_ARM, R_ARM = 3, 12     # arms
FREQ = 50
OE_GPIO = 4
gpioh = None

# ---------- Offsets (deg) ----------
OFF_DEG = {
    "L_HIP":   -10.0,
    "L_KNEE":  -5.0,
    "L_ANK":    0.0,
    "R_HIP":   10.0,
    "R_KNEE":  40.0,
    "R_ANK":   20.0,
    "L_ARM":    0.0,
    "R_ARM":    0.0,
}
INVERT = {
    "L_HIP":   True,
    "L_KNEE":  True,
    "L_ANK":   False,
    "R_HIP":   False,
    "R_KNEE":  False,
    "R_ANK":   False,
    "L_ARM":   False,
    "R_ARM":   False,
}
LIMITS_US = {
    "L_HIP": (900, 2100),
    "L_KNEE": (900, 2100),
    "L_ANK": (1100, 1900),
    "R_HIP": (900, 2100),
    "R_KNEE": (900, 2100),
    "R_ANK": (1100, 1900),
    "L_ARM": (900, 2100),
    "R_ARM": (900, 2100),
}

# --------- TUNABLES ----------
# Phase A (weight shift)
SHIFT_ANKLE_L_DEG = 14     # bigger stance roll, was ~10
SHIFT_ANKLE_R_DEG = -5     # relax swing ankle a bit more
SHIFT_L_KNEE_DEG  = 20     # bend stance knee more to stick to floor
SHIFT_L_HIP_DEG   = -6     # small counter
SHIFT_HOLD        = 0.75   # longer! hold weight shift BEFORE step

# Phase B (actual step)
STEP_R_HIP_DEG    = 32
STEP_R_KNEE_DEG   = 50
STEP_ARMS         = 25     # left arm fwd, right arm back
STEP_HOLD         = 0.5
PLACE_HOLD        = 0.45
# -----------------------------

def w(h, r, v): lg.i2c_write_byte_data(h, r, v & 0xFF)
def r(h, r):    return lg.i2c_read_byte_data(h, r)

def pca_all_off(h):
    w(h, ALL_LED_ON_L, 0x00)
    w(h, ALL_LED_ON_H, 0x00)
    w(h, ALL_LED_OFF_L, 0x00)
    w(h, ALL_LED_OFF_H, 0x10)

def set_freq(h, f):
    om = r(h, MODE1)
    w(h, MODE1, om | SLEEP)
    prescale = max(3, min(255, round(25_000_000 / (4096*f)) - 1))
    w(h, PRE_SCALE, prescale)
    w(h, MODE1, (om & ~SLEEP) | AI)
    time.sleep(0.001)
    w(h, MODE1, r(h, MODE1) | RESTART)

def deg_to_us_for(name, deg):
    deg = deg + OFF_DEG[name]
    deg = max(-90, min(90, deg))
    umin, umax = LIMITS_US[name]
    return int(umin + (deg + 90) * (umax - umin) / 180)

def set_deg(h, ch, name, deg):
    if INVERT[name]:
        deg = -deg
    us = deg_to_us_for(name, deg)
    cnt = int(us * 4096 * FREQ / 1_000_000)
    cnt = max(0, min(4095, cnt))
    base = LED0_ON_L + 4*ch
    w(h, base+0, 0); w(h, base+1, 0)
    w(h, base+2, cnt & 0xFF); w(h, base+3, (cnt >> 8) & 0x0F)

def oe_setup():
    global gpioh
    for n in (0,1,2,3,4):
        try:
            gpioh = lg.gpiochip_open(n); break
        except lg.error: continue
    if gpioh is None:
        raise RuntimeError("No /dev/gpiochipN found")
    lg.gpio_claim_output(gpioh, OE_GPIO, 1)  # start disabled

def oe_enable():  lg.gpio_write(gpioh, OE_GPIO, 0)
def oe_disable(): lg.gpio_write(gpioh, OE_GPIO, 1)

def send_neutral(h):
    set_deg(h, L_HIP,"L_HIP",0)
    set_deg(h, L_KNEE,"L_KNEE",0)
    set_deg(h, L_ANK,"L_ANK",0)
    set_deg(h, R_HIP,"R_HIP",0)
    set_deg(h, R_KNEE,"R_KNEE",0)
    set_deg(h, R_ANK,"R_ANK",0)
    set_deg(h, L_ARM,"L_ARM",0)
    set_deg(h, R_ARM,"R_ARM",0)

# ---- main ----
h = lg.i2c_open(I2C_BUS, ADDR)
try:
    oe_setup()
    pca_all_off(h)
    w(h, MODE2, OUTDRV)
    set_freq(h, FREQ)

    # preload neutral
    send_neutral(h)

    oe_enable()
    print("Neutral...")
    time.sleep(1.0)

    # ---- PHASE A: WEIGHT SHIFT ONLY ----
    print("Phase A: shift weight to LEFT...")
    # ankles do the big work
    set_deg(h, L_ANK, "L_ANK", SHIFT_ANKLE_L_DEG)
    set_deg(h, R_ANK, "R_ANK", SHIFT_ANKLE_R_DEG)
    # stance leg a bit deeper
    set_deg(h, L_HIP, "L_HIP", SHIFT_L_HIP_DEG)
    set_deg(h, L_KNEE,"L_KNEE",SHIFT_L_KNEE_DEG)
    # right leg stays almost neutral
    set_deg(h, R_HIP, "R_HIP", 0)
    set_deg(h, R_KNEE,"R_KNEE",15)
    time.sleep(SHIFT_HOLD)  # <-- this is what actually unloads the right foot

    # ---- PHASE B: STEP (right leg + arms) ----
    print("Phase B: step with right leg...")
    set_deg(h, R_HIP,  "R_HIP",  STEP_R_HIP_DEG)
    set_deg(h, R_KNEE, "R_KNEE", STEP_R_KNEE_DEG)
    # arms opposite
    set_deg(h, L_ARM,  "L_ARM",  STEP_ARMS)
    set_deg(h, R_ARM,  "R_ARM", -STEP_ARMS)
    time.sleep(STEP_HOLD)

    # place right, KEEP ankle tilt for a moment
    print("Place right leg...")
    set_deg(h, R_HIP,  "R_HIP",  10)
    set_deg(h, R_KNEE, "R_KNEE", 15)
    time.sleep(PLACE_HOLD)

    # deroll & neutral
    print("Back to neutral...")
    send_neutral(h)
    time.sleep(0.7)

finally:
    try: pca_all_off(h)
    except Exception: pass
    try: oe_disable()
    except Exception: pass
    try: lg.i2c_close(h)
    except Exception: pass
    try:
        if gpioh is not None:
            lg.gpiochip_close(gpioh)
    except Exception: pass
