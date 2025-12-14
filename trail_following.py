import math
import utime
from machine import Pin, PWM

 
# PARAMETERS 
 
r = 0.03          # wheel radius (m)
L = 0.12          # axle length / wheelbase (m)
gear_ratio = 48   # motor revs per wheel rev
CPR = 12          # encoder counts per motor revolution (single channel)

BASE_SPEED = 0.55     # 0..1 (keep moderate for accuracy)
SYNC_K = 0.0025       # straight-line correction gain (tune)
SPIN_SPEED = 0.45     # spin speed (0..1)

 
# PINOUT 
 
PWMA_PIN = 15
AIN1_PIN = 19
AIN2_PIN = 20

PWMB_PIN = 18
BIN1_PIN = 16
BIN2_PIN = 17

STBY_PIN = 14

# Encoder pins )
ENC_L_PIN = 6
ENC_R_PIN = 7


RIGHT_REVERSED = False

MAX_DUTY = 65535

 
# LOW-LEVEL DRIVER
 
def duty_from_percent(pct: float) -> int:
    if pct < 0: pct = 0
    if pct > 1: pct = 1
    return int(pct * MAX_DUTY)

class TB6612:
    def __init__(self):
        self.stby = Pin(STBY_PIN, Pin.OUT)
        self.stby.value(1)

        self.ain1 = Pin(AIN1_PIN, Pin.OUT)
        self.ain2 = Pin(AIN2_PIN, Pin.OUT)
        self.pwma = PWM(Pin(PWMA_PIN))
        self.pwma.freq(20000)

        self.bin1 = Pin(BIN1_PIN, Pin.OUT)
        self.bin2 = Pin(BIN2_PIN, Pin.OUT)
        self.pwmb = PWM(Pin(PWMB_PIN))
        self.pwmb.freq(20000)

        self.stop()

    def enable(self, on=True):
        self.stby.value(1 if on else 0)

    def _set(self, in1: Pin, in2: Pin, pwm: PWM, speed: float):
        # speed: -1..+1
        if speed > 1: speed = 1
        if speed < -1: speed = -1

        mag = abs(speed)
        duty = duty_from_percent(mag)

        if speed > 0:
            in1.value(1); in2.value(0)
        elif speed < 0:
            in1.value(0); in2.value(1)
        else:
            in1.value(0); in2.value(0)
            duty = 0

        pwm.duty_u16(duty)

    def set_left(self, speed: float):
        self._set(self.ain1, self.ain2, self.pwma, speed)

    def set_right(self, speed: float):
        if RIGHT_REVERSED:
            speed = -speed
        self._set(self.bin1, self.bin2, self.pwmb, speed)

    def stop(self):
        self.set_left(0.0)
        self.set_right(0.0)

driver = TB6612()

 
# ENCODER COUNTERS (single-channel rising-edge counting)
 
class EncoderCounter:
    def __init__(self, pin_num: int):
        self.count = 0
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_DOWN)
        self.pin.irq(trigger=Pin.IRQ_RISING, handler=self._cb)

    def _cb(self, pin):
        self.count += 1

    def read(self) -> int:
        return self.count

encL = EncoderCounter(ENC_L_PIN)
encR = EncoderCounter(ENC_R_PIN)

 
# MATH FROM RUBRIC
def counts_for_distance(d_m: float) -> int:
    # C = (d * i * CPR) / (2*pi*r)
    return int((d_m * gear_ratio * CPR) / (2 * math.pi * r))

def counts_for_spin(theta_rad: float) -> int:
    # Cspin = (L * theta * i * CPR) / (4*pi*r)
    return int((L * theta_rad * gear_ratio * CPR) / (4 * math.pi * r))

 
# REQUIRED MOVEMENT FUNCTIONS YOU SAID YOU DON'T HAVE
 
def stop_motors():
    driver.stop()

def drive_straight_distance(d_m: float, base_speed: float = BASE_SPEED):
    """
    Drive straight for distance d_m using encoder counts.
    Uses a simple synchronization controller to keep both wheels even.
    """
    target = counts_for_distance(d_m)

    L0 = encL.read()
    R0 = encR.read()

    driver.enable(True)

    while True:
        dL = encL.read() - L0
        dR = encR.read() - R0

        # Stop condition: both reached target
        if dL >= target and dR >= target:
            break

        # Sync correction: keep dL ~= dR
        error = dL - dR

        # small correction (tune SYNC_K if needed)
        left_cmd = base_speed - SYNC_K * error
        right_cmd = base_speed + SYNC_K * error

        # clamp to safe range
        if left_cmd > 1: left_cmd = 1
        if left_cmd < 0: left_cmd = 0
        if right_cmd > 1: right_cmd = 1
        if right_cmd < 0: right_cmd = 0

        driver.set_left(left_cmd)
        driver.set_right(right_cmd)

        utime.sleep_ms(10)

    stop_motors()

def spin_ccw(theta_rad: float, spin_speed: float = SPIN_SPEED):
    """
    Spin CCW about axle center by theta_rad using encoder counts.
    CCW = right wheel forward, left wheel backward.
    """
    target = counts_for_spin(theta_rad)

    L0 = encL.read()
    R0 = encR.read()

    driver.enable(True)

    while True:
        dL = abs(encL.read() - L0)
        dR = abs(encR.read() - R0)

        if dL >= target and dR >= target:
            break

        driver.set_left(-spin_speed)
        driver.set_right(+spin_speed)

        utime.sleep_ms(10)

    stop_motors()

def spin_cw(theta_rad: float, spin_speed: float = SPIN_SPEED):
    """
    Spin CW about axle center by theta_rad using encoder counts.
    CW = right wheel backward, left wheel forward.
    """
    target = counts_for_spin(theta_rad)

    L0 = encL.read()
    R0 = encR.read()

    driver.enable(True)

    while True:
        dL = abs(encL.read() - L0)
        dR = abs(encR.read() - R0)

        if dL >= target and dR >= target:
            break

        driver.set_left(+spin_speed)
        driver.set_right(-spin_speed)

        utime.sleep_ms(10)

    stop_motors()

 
# TRACK SEQUENCE 
 
D_CP4_CP1 = 0.75
D_CP1_CP2 = 0.50
D_CP2_CP3 = 0.50

# diagonal CP3->CP4
D_CP3_CP4 = math.sqrt((0.75 - 0.50)**2 + (0.50)**2)  # ~0.559
TURN_90 = math.pi / 2
TURN_DIAG = math.atan2(0.50, (0.75 - 0.50))          # atan2(0.5, 0.25) ~ 1.107

def follow_trail():
    # Start at CP4 facing CP1
    drive_straight_distance(D_CP4_CP1)
    utime.sleep(3)

    spin_ccw(TURN_90)
    utime.sleep(1)

    drive_straight_distance(D_CP1_CP2)
    utime.sleep(3)

    spin_cw(TURN_90)
    utime.sleep(1)

    drive_straight_distance(D_CP2_CP3)
    utime.sleep(3)

    spin_ccw(TURN_DIAG)
    utime.sleep(1)

    drive_straight_distance(D_CP3_CP4)
    stop_motors()


    utime.sleep(2)          
    follow_trail()          # runs the full sequence


