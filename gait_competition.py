# Requires: sudo apt install python3-lgpio
# Run as: sudo python3 one_step_from_neutral_offsets.py
import time
import lgpio as lg

# I2C / PCA9685 setup
I2C_BUS, ADDR = 1, 0x40
MODE1, MODE2, PRE_SCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H = 0xFA, 0xFB, 0xFC, 0xFD
SLEEP, AI, RESTART, OUTDRV = 1<<4, 1<<5, 1<<7, 1<<2

# Channels: [L_HIP, L_KNEE, L_ANKLE, R_HIP, R_KNEE, R_ANKLE]
L_HIP, L_KNEE, L_ANK, R_HIP, R_KNEE, R_ANK = 0, 1, 2, 15, 14, 13
CHANNELS = [L_HIP, L_KNEE, L_ANK, R_HIP, R_KNEE, R_ANK]

FREQ = 50
OE_GPIO = 4
gpioh = None

# ---------- JOINT NEUTRAL OFFSETS (in degrees) ----------
# put here what you measure on the real robot
OFF_DEG = {
    "L_HIP":   -10.0,
    "L_KNEE":  -5.0,
    "L_ANK":   0.0,
    "R_HIP":   10.0,
    "R_KNEE":  40.0,
    "R_ANK":   20.0,
}
# if a joint moves backwards, flip here (this is separate from offset)
INVERT = {
    "L_HIP":   True,
    "L_KNEE":  True,
    "L_ANK":   False,
    "R_HIP":   False,
    "R_KNEE":  False,
    "R_ANK":   False,
}
# pulse limits per joint (µs)
LIMITS_US = {
    "L_HIP": (900, 2100),
    "L_KNEE": (900, 2100),
    "L_ANK": (1100, 1900),
    "R_HIP": (900, 2100),
    "R_KNEE": (900, 2100),
    "R_ANK": (1100, 1900),
}
# --------------------------------------------------------

# --------- T U N A B L E S (step) ----------
LEFT_ANKLE_ROLL_DEG  = 10    # stance ankle roll (left)
RIGHT_ANKLE_RELAX_DEG= -4    # swing ankle slight opposite roll
RIGHT_HIP_FWD_DEG    = 32    # swing forward
RIGHT_KNEE_BEND_DEG  = 50    # swing knee
L_STANCE_HIP_DEG     = -6    # small counter-lean on stance leg
L_STANCE_KNEE_DEG    = 15    # bend for stability
R_STANCE_KNEE_DEG    = 15

T_SHIFT  = 0.45
T_SWING  = 0.55
T_PLACE  = 0.45
# ------------------------------------------

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
    # add neutral offset
    deg = deg + OFF_DEG[name]
    # clamp
    deg = max(-90, min(90, deg))
    umin, umax = LIMITS_US[name]
    return int(umin + (deg + 90) * (umax - umin) / 180)

def set_deg(h, ch, name, deg):
    # apply invert
    if INVERT[name]:
        deg = -deg
    us = deg_to_us_for(name, deg)
    # write to PCA
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
    # neutral == 0 deg for each joint, but with OFF_DEG applied
    set_deg(h, L_HIP, "L_HIP", 0)
    set_deg(h, L_KNEE,"L_KNEE",0)
    set_deg(h, L_ANK, "L_ANK", 0)
    set_deg(h, R_HIP, "R_HIP", 0)
    set_deg(h, R_KNEE,"R_KNEE",0)
    set_deg(h, R_ANK, "R_ANK", 0)

# ---- main ----
h = lg.i2c_open(I2C_BUS, ADDR)
try:
    oe_setup()
    pca_all_off(h)
    w(h, MODE2, OUTDRV)
    set_freq(h, FREQ)

    # 1) preload neutral to all
    send_neutral(h)

    # 2) enable outputs
    oe_enable()
    print("Neutral (with offsets)...")
    time.sleep(1.0)

    # 3) shift weight to LEFT (ankles)
    print("Shift to left...")
    set_deg(h, L_ANK, "L_ANK", LEFT_ANKLE_ROLL_DEG)
    set_deg(h, R_ANK, "R_ANK", RIGHT_ANKLE_RELAX_DEG)
    # stance leg a bit bent
    set_deg(h, L_HIP, "L_HIP", L_STANCE_HIP_DEG)
    set_deg(h, L_KNEE,"L_KNEE",L_STANCE_KNEE_DEG)
    set_deg(h, R_HIP, "R_HIP", 0)
    set_deg(h, R_KNEE,"R_KNEE",R_STANCE_KNEE_DEG)
    time.sleep(T_SHIFT)

    # 4) swing RIGHT leg forward
    print("Swing right leg...")
    set_deg(h, R_HIP,  "R_HIP",  RIGHT_HIP_FWD_DEG)
    set_deg(h, R_KNEE, "R_KNEE", RIGHT_KNEE_BEND_DEG)
    time.sleep(T_SWING)

    # 5) place right leg (straighten a bit, keep stance ankle rolled)
    print("Place right leg...")
    set_deg(h, R_HIP,  "R_HIP",  10)
    set_deg(h, R_KNEE, "R_KNEE", 15)
    time.sleep(T_PLACE)

    # 6) back to neutral (deroll ankles)
    print("Back to neutral...")
    send_neutral(h)
    time.sleep(0.7)

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
