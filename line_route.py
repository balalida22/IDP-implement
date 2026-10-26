# line_route_online.py - follow a fixed route using 4 light sensors + 2 drive motors.
# Middle sensors sit ON the line. Turn time is learned from the first 90 deg turn.
# Standalone: no other files needed.
#   mpremote run line_route_online.py          # one-off run from the laptop (Ctrl-C stops it)
#   mpremote cp line_route_online.py :main.py  # run at power-up
#
# Sensor bar (top view, robot moving up the page), left to right: OL IL IR OR
#
#     OL            IL|IR            OR
#     o             o |o             o
#                   |line|
#
#   IL / IR both sit ON the line: normal driving reads (0 1 1 0).
#   OL / OR sit further out and only see a line at a junction, the start box or a loading box.
#   Notation: 1 = sensor over a white line, 0 = over the black table.

from machine import Pin, PWM
from utime import sleep_ms, ticks_ms, ticks_diff, ticks_add

# ---------------- Pins (same as the course examples) ------------------------
LEFT_DIR, LEFT_PWM = 4, 5          # Motor 3 (sw/test_motor.py)
RIGHT_DIR, RIGHT_PWM = 7, 6        # Motor 4: direction GP7, PWM GP6
LEFT_REVERSED = False              # flip if that wheel runs backwards on "forward"
RIGHT_REVERSED = True
SENSOR_PINS = (18, 19, 20, 21)     # OL, IL, IR, OR
LINE_LEVEL = 1                     # raw pin value over the WHITE line (check with print_sensors())

# ---------------- Speeds (% duty) - raised for a faster run -------------------
BASE_SPEED = 75                    # straight-line following (was 50)
CORR_FAST = 85                     # outer wheel when one middle sensor slips off the line
CORR_SLOW = 40                     # inner wheel when correcting (lower = sharper)
SEARCH_FAST = 60                   # both middles off the line: swing back towards it
SEARCH_SLOW = -10                  #   (inner wheel slightly backwards = tight pivot)
CREEP_SPEED = 55                   # through junctions and boxes (was 40)
SPIN_SPEED = 55                    # turning on the spot (was 45)

# ---------------- Turns ----------------------------------------------------------
TURN_TOLERANCE = 0.1              # later turns finish when within +/-5 % of the learned time...
LINE_CONFIRM = 2                   # ...AND both middle sensors read the line this many times in a row
TURN_MIN_MS = 900                  # first (learning) turn: spin blind this long to leave the old line
TURN_TIMEOUT_MS = 3000             # first turn: give up if no line by then
TURN_REF_MIN_MS = 900              # a learned 90 deg time outside this range is rejected
TURN_REF_MAX_MS = 1500             #   (and the next turn tries to learn again)
HALF_TURN_FACTOR = 2.0             # half-turn time = this x learned 90 deg time (tune if it over/undershoots)
DEFAULT_HALF_TURN_MS = 1200        # used only if no 90 deg turn has been learned yet

# ---------------- Timings (ms) ------------------------------------------------
LOOP_MS = 5
START_DELAY_MS = 2000              # time to step back after starting
EVENT_CONFIRM = 3                  # consecutive reads before an outer sensor hit counts
JUNCTION_WINDOW_MS = 50            # creep forward this long to see if BOTH outers hit
COOLDOWN_MS = 350                  # ignore new junctions this long after each action
ADVANCE_MS = 1000                  # creep after a junction so the axle is over it before spinning
CLEAR_CONFIRM = 4                  # consecutive "clear" reads to say we've left a line/box
CLEAR_TIMEOUT_MS = 3000
LOST_TIMEOUT_MS = 800              # both middles off the line this long -> stop (safety)
RUN_TIMEOUT_MS = 5 * 60 * 1000     # hard stop after 5 minutes
RACK_TIMEOUT_MS = 2500             # max time following the rack stub before turning round
                                   # (it turns round earlier if the stub line ends first)
PARK_MS = 600                      # after crossing the start-box edge on the way home, keep
                                   # following the line this long at CREEP_SPEED, then stop.
                                   # Tune so the WHOLE robot ends up inside the box.

# ---------------- Route -------------------------------------------------------
# One entry per junction, in the order the robot meets them:
#   (what the outer sensors see, what to do, note[, timeout_ms])
# Events (outer sensors only):
#   "BOTH"    OL and OR both hit a line   - (1 1 1 1) start-box edge / T / loading box
#   "LEFT"    only OL                     - (1 1 1 0) junction to the left
#   "RIGHT"   only OR                     - (0 1 1 1) junction to the right
#   "ANY"     whatever comes next
#   "TIMEOUT" ignore junctions: act when the line ends (0 0 0 0) or timeout_ms has passed
# Optional timeout_ms on any step: act anyway if the event hasn't happened by then.
# Actions: "STRAIGHT", "LEFT", "RIGHT", "BOX_TURN_AROUND", "RACK_TURN_AROUND", "PARK", "STOP"
ROUTE = [
    # --- out: start box -> bottom-right corner -> loading area 3 ---
    ("BOTH",  "STRAIGHT",         "start-box edge (1 1 1 1)"),
    ("LEFT",  "LEFT",             "bottom-right corner (1 1 1 0): turn up the right side  <- learns turn time"),
    ("LEFT",  "LEFT",             "loading area 3 junction (1 1 1 0): turn in"),
    ("BOTH",  "BOX_TURN_AROUND",  "loading area 3 box (1 1 1 1) -> exit (0 0 0 0) -> turn round"),
    ("BOTH",  "LEFT",             "T at the right edge (1 1 1 1): turn left, up the right side"),
    # --- up the right side to rack B ---
    ("LEFT",  "STRAIGHT",         "skip loading area 4 (1 1 1 0)"),
    ("LEFT",  "STRAIGHT",         "skip rack B 1st stub, B1L (1 1 1 0)"),
    ("LEFT",    "LEFT",             "rack B 2nd stub, B2L (1 1 1 0): turn in"),
    ("TIMEOUT", "RACK_TURN_AROUND", "end of the stub at the rack: turn round", RACK_TIMEOUT_MS),
    ("BOTH",    "RIGHT",            "T at the right edge (1 1 1 1): turn right, down the right side"),
    # --- back down the right side: every stub is now on the robot's RIGHT ---
    #("RIGHT", "STRAIGHT",         "skip B1L (0 1 1 1)"),
    ("RIGHT", "STRAIGHT",         "skip loading area 4 (0 1 1 1)"),
    ("RIGHT", "STRAIGHT",         "skip loading area 3 (0 1 1 1)"),
    ("RIGHT", "RIGHT",            "bottom-right corner (0 1 1 1): turn right, towards the start"),
    # --- home ---
    ("BOTH",  "PARK",             "start-box edge (1 1 1 1): drive in and stop"),
]


# ==============================================================================
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
sensors = [Pin(p, Pin.IN, Pin.PULL_DOWN) for p in SENSOR_PINS]

# Learned 90 deg spin times, one per direction (left and right spins aren't the same).
# Each is set once, by the first good turn in that direction.
turn_ref = {-1: None, 1: None}     # -1 = left, +1 = right
DIR_NAME = {-1: "left", 1: "right"}


def read():
    """(OL, IL, IR, OR) with 1 = over the white line."""
    return tuple(1 if p.value() == LINE_LEVEL else 0 for p in sensors)


def fmt(s):
    return "%d %d %d %d" % s


def drive(l_speed, r_speed):
    left.set(l_speed)
    right.set(r_speed)


def spin(direction):
    """direction -1 = turn left on the spot, +1 = turn right."""
    drive(direction * SPIN_SPEED, -direction * SPIN_SPEED)


def stop():
    left.off()
    right.off()


# ---------------- Line following (middle sensors ON the line) -----------------
class Follower:
    def __init__(self):
        self.last_side = 0         # -1 = line last seen under IL only, +1 = under IR only
        self.lost_since = None

    def reset(self):
        self.lost_since = None

    def step(self, s, k=1.0):
        """One control step; k scales all speeds (k < 1 = slower, e.g. when parking).
        Returns False if the line has been lost too long."""
        _, il, ir, _ = s
        if il and ir:                      # (x 1 1 x) centred
            drive(k * BASE_SPEED, k * BASE_SPEED)
            self.last_side = 0
            self.lost_since = None
        elif il:                           # line slipping to the left -> steer left
            drive(k * CORR_SLOW, k * CORR_FAST)
            self.last_side = -1
            self.lost_since = None
        elif ir:                           # line slipping to the right -> steer right
            drive(k * CORR_FAST, k * CORR_SLOW)
            self.last_side = 1
            self.lost_since = None
        else:                              # both off: swing back to where we last saw it
            now = ticks_ms()
            if self.lost_since is None:
                self.lost_since = now
            elif ticks_diff(now, self.lost_since) > LOST_TIMEOUT_MS:
                return False
            if self.last_side < 0:
                drive(k * SEARCH_SLOW, k * SEARCH_FAST)
            elif self.last_side > 0:
                drive(k * SEARCH_FAST, k * SEARCH_SLOW)
            else:
                drive(CREEP_SPEED, CREEP_SPEED)
        return True


# ---------------- Junction handling -------------------------------------------
def identify(first):
    """Creep a little and OR the outer sensors, so an angled approach to a T
    (OL a few ms before OR) still reads as BOTH."""
    seen_l, seen_r = first[0], first[3]
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < JUNCTION_WINDOW_MS:
        drive(CREEP_SPEED, CREEP_SPEED)
        s = read()
        seen_l |= s[0]
        seen_r |= s[3]
        sleep_ms(LOOP_MS)
    if seen_l and seen_r:
        return "BOTH"
    return "LEFT" if seen_l else "RIGHT"


def drive_until(cond, speed=CREEP_SPEED):
    """Drive straight until cond(sensors) holds for CLEAR_CONFIRM reads in a row."""
    count = 0
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < CLEAR_TIMEOUT_MS:
        drive(speed, speed)
        s = read()
        count = count + 1 if cond(s) else 0
        if count >= CLEAR_CONFIRM:
            return True
        sleep_ms(LOOP_MS)
    return False


def outers_clear(s):
    return not s[0] and not s[3]


def all_black(s):
    return not any(s)


def middles_on(s):
    return s[1] and s[2]


def spin_until_middles_on(t0, earliest_ms, latest_ms):
    """Keep spinning; stop when both middle sensors are on the line (LINE_CONFIRM reads)
    at a time between earliest_ms and latest_ms after t0. Returns elapsed ms, or None."""
    while ticks_diff(ticks_ms(), t0) < earliest_ms:
        sleep_ms(LOOP_MS)          # too early: ignore the line we're leaving / lines crossed
    count = 0
    while True:
        elapsed = ticks_diff(ticks_ms(), t0)
        if elapsed >= latest_ms:
            return None
        count = count + 1 if middles_on(read()) else 0
        if count >= LINE_CONFIRM:
            return elapsed
        sleep_ms(LOOP_MS)


def turn(direction):
    """Turn 90 deg at a junction. direction -1 = left, +1 = right.

    Left and right are learned separately. The first turn in each direction (no reference
    yet): spin until both middle sensors are on the new line and remember how long it took.
    Every later turn that way: only accept the line within +/-TURN_TOLERANCE of that time;
    if it isn't seen by then, stop at the upper limit and let line following re-centre."""
    name = DIR_NAME[direction]
    ref = turn_ref[direction]
    drive(CREEP_SPEED, CREEP_SPEED)
    sleep_ms(ADVANCE_MS)               # axle over the junction
    t0 = ticks_ms()
    spin(direction)

    if ref is None:                    # ---- learning turn (once per direction) ----
        elapsed = spin_until_middles_on(t0, TURN_MIN_MS, TURN_TIMEOUT_MS)
        stop()
        if elapsed is None:
            print("    learning %s turn: no line found" % name)
            return False
        if TURN_REF_MIN_MS <= elapsed <= TURN_REF_MAX_MS:
            turn_ref[direction] = elapsed
            print("    learned 90 deg %s turn = %d ms" % (name, elapsed))
        else:
            print("    %s learning turn took %d ms - outside %d..%d, will learn on the next %s turn"
                  % (name, elapsed, TURN_REF_MIN_MS, TURN_REF_MAX_MS, name))
        return True

    lo = int(ref * (1 - TURN_TOLERANCE))   # ---- reference turn ----
    hi = int(ref * (1 + TURN_TOLERANCE))
    elapsed = spin_until_middles_on(t0, lo, hi)
    stop()
    if elapsed is None:
        print("    no line within %d..%d ms - stopped at upper limit (%s ref %d)" % (lo, hi, name, ref))
    else:
        print("    %s turn done in %d ms (ref %d)" % (name, elapsed, ref))
    return True


def half_turn_ms():
    """Half-turn time. Unchanged behaviour: both half-turns spin LEFT, so they use the
    learned LEFT 90 deg time x HALF_TURN_FACTOR."""
    ref = turn_ref[-1]
    return int(ref * HALF_TURN_FACTOR) if ref else DEFAULT_HALF_TURN_MS


def box_turn_around():
    """Cross the loading box (1 1 1 1) until all black (0 0 0 0), half-turn, then drive
    back until the outer sensors are clear of the box again.
    The half-turn is timed (HALF_TURN_FACTOR x learned 90 deg time): inside the white box
    the middle sensors read 'line' everywhere, so the sensor check can't be used here."""
    if not drive_until(all_black):
        return False
    stop()
    sleep_ms(100)
    half = half_turn_ms()
    spin(-1)
    sleep_ms(half)
    stop()
    sleep_ms(100)
    return drive_until(outers_clear)


def rack_turn_around():
    """At the end of the rack stub: stop and turn round on the spot.
    Unlike the box, the table here is black, so the half-turn uses the same rule as the
    90 deg turns: stop when both middles are back on the stub line within +/-TURN_TOLERANCE
    of the expected half-turn time, or at the upper limit if the line isn't seen."""
    stop()
    sleep_ms(100)
    half = half_turn_ms()
    lo = int(half * (1 - TURN_TOLERANCE))
    hi = int(half * (1 + TURN_TOLERANCE))
    t0 = ticks_ms()
    spin(-1)
    elapsed = spin_until_middles_on(t0, lo, hi)
    stop()
    if elapsed is None:
        print("    no line within %d..%d ms - stopped at upper limit" % (lo, hi))
    else:
        print("    half-turn done in %d ms (expected %d)" % (elapsed, half))
    sleep_ms(100)
    return True


def park():
    """Cross the start-box edge, keep following the line slowly for PARK_MS, then stop."""
    if not drive_until(outers_clear):
        return False
    follower = Follower()
    k = CREEP_SPEED / BASE_SPEED
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < PARK_MS:
        follower.step(read(), k)
        sleep_ms(LOOP_MS)
    stop()
    print("    parked")
    return True


def do(action):
    if action == "STRAIGHT":
        return drive_until(outers_clear, BASE_SPEED)
    if action == "LEFT":
        return turn(-1)
    if action == "RIGHT":
        return turn(1)
    if action == "BOX_TURN_AROUND":
        return box_turn_around()
    if action == "RACK_TURN_AROUND":
        return rack_turn_around()
    if action == "PARK":
        return park()
    print("Unknown action", action)
    return False


# ---------------- Main loop ---------------------------------------------------
def run():
    print("Starting in %d ms" % START_DELAY_MS)
    sleep_ms(START_DELAY_MS)

    follower = Follower()
    step = 0
    hits = 0
    armed_at = ticks_ms()
    t0 = ticks_ms()
    step_start = ticks_ms()        # when the current route step became active
    seen_line = False              # TIMEOUT steps: line seen at least once in this step
    line_end = 0                   # TIMEOUT steps: consecutive reads with both middles off

    while step < len(ROUTE):
        if ticks_diff(ticks_ms(), t0) > RUN_TIMEOUT_MS:
            print("Run timeout")
            break

        entry = ROUTE[step]
        want, action, note = entry[0], entry[1], entry[2]
        limit = entry[3] if len(entry) > 3 else None
        waited = ticks_diff(ticks_ms(), step_start)

        s = read()
        event = None
        if want == "TIMEOUT":
            # Ignore junctions; act when the line ends or the time is up.
            if s[1] or s[2]:
                seen_line = True
                line_end = 0
            elif seen_line:
                line_end += 1
            if line_end >= CLEAR_CONFIRM:
                event = "LINE_END"
            elif limit is not None and waited >= limit:
                event = "TIMEOUT"
        else:
            armed = ticks_diff(ticks_ms(), armed_at) >= 0
            hits = hits + 1 if (armed and (s[0] or s[3])) else 0
            if hits >= EVENT_CONFIRM:
                hits = 0
                event = identify(s)
            elif limit is not None and waited >= limit:
                event = "TIMEOUT"

        if event:
            print("[%d] %s  saw %-8s -> %-16s %s" % (step, fmt(s), event, action, note))
            if want not in ("ANY", "TIMEOUT") and event != want:
                print("    warning: route expected %s here" % want)
            step += 1

            if action == "STOP":
                break
            ok = do(action)
            if action == "PARK":
                break
            if not ok:
                print("    %s failed (timed out) - stopping" % action)
                break
            follower.reset()
            armed_at = ticks_add(ticks_ms(), COOLDOWN_MS)
            step_start = ticks_ms()
            seen_line = False
            line_end = 0
            hits = 0
            continue

        if not follower.step(s):
            print("Line lost - stopping")
            break
        sleep_ms(LOOP_MS)

    stop()
    print("Route done")


# ---------------- Bench checks ------------------------------------------------
def print_sensors():
    """Move the bar over line / table / box and check the patterns match your notation."""
    while True:
        print(fmt(read()))
        sleep_ms(200)


def test_motors():
    """Wheels OFF the table: each wheel forward 1 s, then a short left spin."""
    for name, m in (("left", left), ("right", right)):
        print(name, "forward")
        m.set(40)
        sleep_ms(1000)
        m.off()
        sleep_ms(500)
    print("spin left")
    spin(-1)
    sleep_ms(1000)
    stop()


if __name__ == "__main__":
    try:
        # Uncomment ONE:
        # print_sensors()
        # test_motors()
        run()
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        stop()                     # always leave the motors off
