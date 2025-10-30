# Requires: sudo apt install python3-lgpio
# Run as: sudo python3 neutral_pose_with_offsets.py
import time
import lgpio as lg

# --- PCA9685 constants ---
I2C_BUS, ADDR = 1, 0x40
MODE1, MODE2, PRE_SCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H = 0xFA, 0xFB, 0xFC, 0xFD
SLEEP, AI, RESTART, OUTDRV = 1<<4, 1<<5, 1<<7, 1<<2

# --- Channel mapping ---
L_HIP, L_KNEE, L_ANK = 0, 1, 2
R_HIP, R_KNEE, R_ANK = 15, 14, 13
CHANNELS = [L_HIP, L_KNEE, L_ANK, R_HIP, R_KNEE, R_ANK]
JOINT_NAMES = ["L_HIP","L_KNEE","L_ANK","R_HIP","R_KNEE","R_ANK"]

FREQ = 50
OE_GPIO = 4
gpioh = None

# ----------------- OFFSET SYSTEM -----------------
# Offsets in degrees: positive = same direction as servo positive rotation
OFF_DEG = {
    "L_HIP":   0.0,
    "L_KNEE":  0.0,
    "L_ANK":   0.0,
    "R_HIP":   0.0,
    "R_KNEE":  0.0,
    "R_ANK":   0.0,
}
# If a servo is inverted mechanically, flip here:
INVERT = {
    "L_HIP": False, "L_KNEE": False, "L_ANK": False,
    "R_HIP": False, "R_KNEE": False, "R_ANK": False,
}
# Microsecond limits for safety per joint
LIMITS_US = {
    "L_HIP": (900, 2100),
    "L_KNEE": (900, 2100),
    "L_ANK": (1100,1900),
    "R_HIP": (900, 2100),
    "R_KNEE": (900, 2100),
    "R_ANK": (1100,1900),
}
# -------------------------------------------------

def w(h, r, v): lg.i2c_write_byte_data(h, r, v & 0xFF)
def r(h, r):    return lg.i2c_read_byte_data(h, r)

def pca_all_off(h):
    w(h, ALL_LED_ON_L, 0x00)
    w(h, ALL_LED_ON_H, 0x00)
    w(h, ALL_LED_OFF_L, 0x00)
    w(h, ALL_LED_OFF_H, 0x10)  # bit4=1 => full off

def set_freq(h, f):
    om = r(h, MODE1)
    w(h, MODE1, om | SLEEP)
    prescale = max(3, min(255, round(25_000_000/(4096*f)) - 1))
    w(h, PRE_SCALE, prescale)
    w(h, MODE1, (om & ~SLEEP) | AI)
    time.sleep(0.001)
    w(h, MODE1, r(h, MODE1) | RESTART)

def deg_to_us_for(name, deg):
    # apply offset and map -90..+90 → us range
    deg = deg + OFF_DEG[name]
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

# --- main ---
h = lg.i2c_open(I2C_BUS, ADDR)
try:
    oe_setup()
    pca_all_off(h)
    w(h, MODE2, OUTDRV)
    set_freq(h, FREQ)

    # Preload "neutral with offsets"
    for ch, name in zip(CHANNELS, JOINT_NAMES):
        set_deg(h, ch, name, 0)  # 0° plus offset

    # Enable outputs (go to offset-neutral pose)
    oe_enable()
    print("Neutral pose (with offsets)...")
    time.sleep(1.0)

finally:
    try:
        pca_all_off(h)
    except Exception:
        pass
    try:
        oe_disable()
    except Exception:
        pass
    try:
        lg.i2c_close(h)
    except Exception:
        pass
    try:
        if gpioh is not None:
            lg.gpiochip_close(gpioh)
    except Exception:
        pass
