# Requires: sudo apt install python3-lgpio
# Run as: sudo python3 arms_motion_test.py
import time
import lgpio as lg

# ---- PCA9685 constants ----
I2C_BUS, ADDR = 1, 0x40
MODE1, MODE2, PRE_SCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H = 0xFA, 0xFB, 0xFC, 0xFD
SLEEP, AI, RESTART, OUTDRV = 1<<4, 1<<5, 1<<7, 1<<2

# ---- Channel map ----
L_ARM, R_ARM = 3, 12     # your wiring
FREQ = 50
OE_GPIO = 4
gpioh = None

# ---- CONFIG / TUNABLES ----
ARM_SWING_DEG = 40       # how far arms swing each way
CYCLES = 3               # number of forward/back swings
DELAY = 0.5              # seconds per half-swing
OFF_DEG = {              # offset in degrees for each arm
    "L_ARM": 0.0,
    "R_ARM": 0.0,
}
INVERT = {               # flip direction if needed
    "L_ARM": False,
    "R_ARM": False,
}
LIMITS_US = {            # safe µs limits
    "L_ARM": (900, 2100),
    "R_ARM": (900, 2100),
}

# ---- Helpers ----
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
    w(h, MODE1, (om & ~SLEEP) | AI)
    time.sleep(0.001)
    w(h, MODE1, r(h, MODE1) | RESTART)

def deg_to_us_for(name, deg):
    deg += OFF_DEG[name]
    deg = max(-90, min(90, deg))
    umin, umax = LIMITS_US[name]
    return int(umin + (deg + 90) * (umax - umin) / 180)

def set_deg(h, ch, name, deg):
    if INVERT[name]:
        deg = -deg
    us = deg_to_us_for(name, deg)
    cnt = max(0, min(4095, int(us * 4096 * FREQ / 1_000_000)))
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
    lg.gpio_claim_output(gpioh, OE_GPIO, 1)  # start HIGH = disabled
def oe_enable():  lg.gpio_write(gpioh, OE_GPIO, 0)
def oe_disable(): lg.gpio_write(gpioh, OE_GPIO, 1)

# ---- MAIN ----
h = lg.i2c_open(I2C_BUS, ADDR)
try:
    oe_setup()
    pca_all_off(h)
    w(h, MODE2, OUTDRV)
    set_freq(h, FREQ)

    # 1. Neutral preload
    for ch, name in [(L_ARM, "L_ARM"), (R_ARM, "R_ARM")]:
        set_deg(h, ch, name, 0)

    # 2. Enable outputs
    oe_enable()
    print("Arms neutral (with offsets)")
    time.sleep(1.0)

    # 3. Swing test
    print("Swinging arms...")
    for i in range(CYCLES):
        # forward
        set_deg(h, L_ARM, "L_ARM", +ARM_SWING_DEG)
        set_deg(h, R_ARM, "R_ARM", -ARM_SWING_DEG)
        time.sleep(DELAY)

        # backward
        set_deg(h, L_ARM, "L_ARM", -ARM_SWING_DEG)
        set_deg(h, R_ARM, "R_ARM", +ARM_SWING_DEG)
        time.sleep(DELAY)

    # 4. Back to neutral
    print("Back to neutral")
    set_deg(h, L_ARM, "L_ARM", 0)
    set_deg(h, R_ARM, "R_ARM", 0)
    time.sleep(0.8)

finally:
    try:
        pca_all_off(h)
    except Exception: pass
    try:
        oe_disable()
    except Exception: pass
    try:
        lg.i2c_close(h)
    except Exception: pass
    try:
        if gpioh is not None: lg.gpiochip_close(gpioh)
    except Exception: pass
