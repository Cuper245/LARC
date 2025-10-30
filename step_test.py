# sudo apt install python3-lgpio
# Run: sudo python3 one_step_test.py
import time, math
import lgpio as lg

# --- PCA9685 setup ---
I2C_BUS, ADDR = 1, 0x40
MODE1, MODE2, PRE_SCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H = 0xFA, 0xFB, 0xFC, 0xFD
SLEEP, AI, RESTART, OUTDRV = 1<<4, 1<<5, 1<<7, 1<<2

# --- Channels ---
L_HIP, L_KNEE, L_ANK = 0, 1, 2
R_HIP, R_KNEE, R_ANK = 15, 14, 13
FREQ = 50

# Conservative pulse limits
LIMITS_US = {
    "L_HIP": (900, 2100), "L_KNEE": (900, 2100),
    "R_HIP": (900, 2100), "R_KNEE": (900, 2100),
    "L_ANK": (1100,1900), "R_ANK": (1100,1900)
}
OFFS_DEG = {k:0 for k in LIMITS_US}

# Direction corrections if needed
INVERT = {
    "L_HIP": True,  "L_KNEE": True,  "L_ANK": False,
    "R_HIP": False, "R_KNEE": False, "R_ANK": False,
}

# Tunables
HIP_SWING = 32      # forward motion of swing hip
KNEE_SWING = 50     # swing knee bend
PRETILT = 10        # ankle roll (stance foot)
IDLE_KNEE = 18      # neutral knee bend
DURATION_SHIFT = 0.4
DURATION_SWING = 0.5
DURATION_RETURN = 0.4
SMOOTH_HZ = 60

# --- Basic helpers ---
def w(h, r, v): lg.i2c_write_byte_data(h, r, v & 0xFF)
def r(h, r): return lg.i2c_read_byte_data(h, r)

def set_freq(h, f):
    om = r(h, MODE1)
    w(h, MODE1, om | SLEEP)
    ps = max(3, min(255, round(25_000_000/(4096*f))-1))
    w(h, PRE_SCALE, ps)
    w(h, MODE1, (om & ~SLEEP) | AI)
    time.sleep(0.001)
    w(h, MODE1, r(h, MODE1) | RESTART)

def set_us(h, ch, f, us):
    cnt = int(us * 4096 * f / 1_000_000)
    cnt = max(0, min(4095, cnt))
    base = LED0_ON_L + 4*ch
    w(h, base+0,0); w(h,base+1,0)
    w(h, base+2,cnt&0xFF); w(h,base+3,(cnt>>8)&0x0F)

def angle_to_us_joint(name, deg):
    deg += OFFS_DEG[name]
    deg = max(-90, min(90, deg))
    umin, umax = LIMITS_US[name]
    return int(umin + (deg+90)*(umax-umin)/180)

def apply_joint(h, ch, name, deg):
    if INVERT[name]: deg = -deg
    set_us(h, ch, FREQ, angle_to_us_joint(name, deg))

def write_pose(h, Lhip,Lknee,Lank, Rhip,Rknee,Rank):
    apply_joint(h,L_HIP,"L_HIP",Lhip)
    apply_joint(h,L_KNEE,"L_KNEE",Lknee)
    apply_joint(h,L_ANK,"L_ANK",Lank)
    apply_joint(h,R_HIP,"R_HIP",Rhip)
    apply_joint(h,R_KNEE,"R_KNEE",Rknee)
    apply_joint(h,R_ANK,"R_ANK",Rank)

def smooth(t): t=max(0,min(1,t)); return t*t*(3-2*t)
def move(h,A,B,dur,Hz=SMOOTH_HZ):
    steps=int(dur*Hz)
    for i in range(steps):
        t=smooth(i/(steps-1) if steps>1 else 1.0)
        pose=[a+(b-a)*t for a,b in zip(A,B)]
        write_pose(h,*pose)
        time.sleep(1.0/Hz)

# --- Key poses ---
def neutral(): return [0, IDLE_KNEE, 0,  0, IDLE_KNEE, 0]
def shift_left(): # tilt over left foot
    return [-4, IDLE_KNEE+3, +PRETILT,  +2, IDLE_KNEE, -3]
def right_swing(): # right leg forward
    return [-6, IDLE_KNEE+3, +PRETILT,  +HIP_SWING, KNEE_SWING, 0]
def right_place(): # bring it down
    return [-4, IDLE_KNEE+3, +PRETILT*0.5,  +10, IDLE_KNEE+8, 0]

# --- Main run ---
h = lg.i2c_open(I2C_BUS, ADDR)
try:
    w(h, MODE2, OUTDRV)
    set_freq(h, FREQ)
    print("Neutral...")
    write_pose(h,*neutral()); time.sleep(1.0)

    print("Shift weight to left (ankle roll)")
    move(h, neutral(), shift_left(), DURATION_SHIFT)

    print("Swing right leg forward")
    move(h, shift_left(), right_swing(), DURATION_SWING)

    print("Place right leg down")
    move(h, right_swing(), right_place(), DURATION_RETURN)

    print("Back to neutral")
    move(h, right_place(), neutral(), 0.5)
    time.sleep(0.5)
finally:
    # release
    write_pose(h,*neutral()); time.sleep(0.5)
    w(h, ALL_LED_ON_L,0); w(h,ALL_LED_ON_H,0)
    w(h, ALL_LED_OFF_L,0); w(h,ALL_LED_OFF_H,0x10)
    lg.i2c_close(h)
