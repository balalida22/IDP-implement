# line_follow.py - minimal line follower: 2 line sensors + 2 drive motors.
# Standalone: no other files needed. Run with `mpremote run line_follow.py`,
# or copy to the Pico as main.py to run at power-up. Ctrl-C stops it (motors off).
#
# Sensor layout (top view, robot moving up the page). The two sensors STRADDLE
# the line, each sitting just outside one edge:
#
#            L  |line|  R
#            o  |    |  o        <- sensors ~30-60 mm ahead of the wheel axle
#
#   neither sees the line -> centred, drive straight
#   L sees the line       -> robot drifted right, steer left
#   R sees the line       -> robot drifted left, steer right
#   both see the line     -> crossing/junction, drive straight over it

from machine import Pin, PWM
from utime import sleep_ms

# ---------------- Pins (same as the course examples) ------------------------
# Motor 3 = Motor Driver 2 #1 on GP4/5 (sw/test_motor.py). Motor 4 assumed GP6/7.
LEFT_DIR, LEFT_PWM = 4, 5
RIGHT_DIR, RIGHT_PWM = 7, 6
LEFT_REVERSED = False      # flip if that wheel runs backwards on "forward"
RIGHT_REVERSED = True      # motors face opposite ways, so one is usually reversed

SENSOR_L = 18              # sw/test_input.py uses GP18 with PULL_DOWN
SENSOR_R = 19
LINE_LEVEL = 1             # raw value read when a sensor is OVER THE LINE.
                           # Sensor gives HIGH on white, LOW on black, but your
                           # isolation circuit may invert it - print the values first.

# ---------------- Tuning ----------------------------------------------------
BASE_SPEED = 50            # % duty driving straight
TURN_FAST = 60             # % outer wheel while correcting
TURN_SLOW = 10             # % inner wheel while correcting (negative = pivot harder)
LOOP_MS = 5


class Motor:
    """TB6612FNG channel, same convention as test_motor.py: dir 0 = forward, 1 kHz PWM."""

    def __init__(self, dir_pin, pwm_pin, reversed=False):
        self.dir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.reversed = reversed
        self.off()

    def set(self, speed):
        """speed -100..100, negative = reverse."""
        speed = max(-100, min(100, speed))
        forward = (speed >= 0) != self.reversed
        self.dir.value(0 if forward else 1)
        self.pwm.duty_u16(int(65535 * abs(speed) / 100))

    def off(self):
        self.pwm.duty_u16(0)


left = Motor(LEFT_DIR, LEFT_PWM, LEFT_REVERSED)
right = Motor(RIGHT_DIR, RIGHT_PWM, RIGHT_REVERSED)
sensor_l = Pin(SENSOR_L, Pin.IN, Pin.PULL_DOWN)
sensor_r = Pin(SENSOR_R, Pin.IN, Pin.PULL_DOWN)


def drive(l_speed, r_speed):
    left.set(l_speed)
    right.set(r_speed)


def stop():
    left.off()
    right.off()


def follow():
    while True:
        on_l = sensor_l.value() == LINE_LEVEL
        on_r = sensor_r.value() == LINE_LEVEL

        if on_l and not on_r:          # line under left sensor -> steer left
            drive(TURN_SLOW, TURN_FAST)
        elif on_r and not on_l:        # line under right sensor -> steer right
            drive(TURN_FAST, TURN_SLOW)
        else:                          # centred, or crossing a junction
            drive(BASE_SPEED, BASE_SPEED)

        sleep_ms(LOOP_MS)


def print_sensors():
    """Bench check: move each sensor over line and background, note the values."""
    while True:
        print("L =", sensor_l.value(), " R =", sensor_r.value())
        sleep_ms(200)


def test_motors():
    """Bench check, wheels OFF the table: each wheel should spin forward for 1 s."""
    for name, m in (("left", left), ("right", right)):
        print(name, "forward")
        m.set(40)
        sleep_ms(1000)
        m.off()
        sleep_ms(500)


if __name__ == "__main__":
    try:
        # Uncomment ONE of these:
        # print_sensors()
        # test_motors()
        follow()
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        stop()                 # always leave the motors off
