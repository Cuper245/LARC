# sudo python3 hip_knee_max_test.py
import time, lgpio as lg
I2C_BUS, ADDR = 1, 0x40
MODE1,MODE2,PRE,LED0_ON_L=0x00,0x01,0xFE,0x06
SLEEP,AI,RESTART,OUTDRV=1<<4,1<<5,1<<7,1<<2
def w(h,r,v): lg.i2c_write_byte_data(h,r,v&0xFF)
def r(h,r):   return lg.i2c_read_byte_data(h,r)
def setf(h,f):
    om=r(h,MODE1); w(h,MODE1,om|SLEEP); w(h,PRE,max(3,min(255,round(25_000_000/(4096*f))-1)))
    w(h,MODE1,(om&~SLEEP)|AI); time.sleep(0.001); w(h,MODE1,r(h,MODE1)|RESTART)
def setus(h,ch,f,us):
    cnt=max(0,min(4095,int(us*4096*f/1_000_000))); base=LED0_ON_L+4*ch
    for i,v in enumerate((0,0,cnt&0xFF,(cnt>>8)&0x0F)): w(h,base+i,v)
h=lg.i2c_open(I2C_BUS,ADDR)
try:
    w(h,MODE2,OUTDRV); setf(h,50)
    HIP_L, KNEE_L = 0, 1
    for us in (1500, 1200, 1800, 1100, 1900, 1500):
        setus(h,HIP_L,50,us); setus(h,KNEE_L,50,us); print("HIP/KNEE->",us); time.sleep(0.7)
finally:
    lg.i2c_close(h)
