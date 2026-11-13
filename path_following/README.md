# Corner test (line following + corner turns)

Follows the tape clockwise with the straight-line correction, and turns right at every
corner. Corners are found by the 7th tape sensor, **CR**, on the front-right corner of the
robot (wired to W-3).

## Use

1. Flash `CornerTest.hex` (or open `CornerTest.X` in MPLAB X and build).
2. Set the robot on a straight part of the line with FC and BC on the tape, then reset.
3. After 3 s it kicks off, follows the line, swings right 90° at each corner, and keeps going.

## 7th sensor: front-right corner (CR)

Mount it in the angled slot at the **front-right corner** of the base plate, at the same
height above the floor as the other six. That's about **(3.7, 3.8)"** from the axle
(3.7" right, 3.8" forward; check it in Onshape).

**Wiring: signal to W-3** (analog), power and ground like the other tape sensors.

| Sensor wire | Uno32 I/O board |
|---|---|
| signal (out) | **W-3** |
| +5 V / VCC | same supply as the other tape sensors |
| GND | GND |

**Why not Port Y:** the only free Y pin is **Y-11**. Y-03 to Y-05 are LED bank 1, Y-06 to
Y-09 are the motor direction pins, Y-10 and Y-12 are the motor PWM pins. Y-11 is
digital-only, and these sensors put out about 0.5 V on the floor and 1.0 V on tape, which
both read as LOW on a digital pin. Port Y has no analog inputs; Port W does (W-3 to W-8 are free).

**Only if** the sensor is a module with a **digital output** (a DO pin, with a comparator on
the module), wire DO to **Y-11** and uncomment `#define CR_ON_Y11` in `Robot.h`. If DO reads
LOW over the tape, set `CR_Y11_TAPE_LEVEL` to 0.

**Check it:** bank 1 **LED 4** lights when CR sees tape. Slide the robot sideways over a
tape line by hand and watch it.

## How a corner turn works: a swing about the right wheel

The robot drives the perimeter clockwise, so every corner is a right turn. The turn is a
**swing**: the right wheel is braked (locked) and the left wheel drives forward, so the
robot turns right about its right wheel.

Geometry (inches from the axle, x right, y forward): CR is at (3.7, 3.8), the tape is 2"
wide, and the wheels are `HALF_TRACK_IN` (4.5", measure it) either side of the axle center.

- When CR first sees the new line, the axle is 3.8 + 1 = **4.8"** short of the new line's
  centerline.
- A 90° swing about the right wheel moves the axle center **one half-track forward and one
  half-track right**, ending heading along the new line.
- So it drives **4.8 − 4.5 = 0.3"** more, then swings, and the axle lands on the new line.

| State (log) | What happens | Ends when |
|---|---|---|
| `Drive` | straight-line correction (FL FC FR / BL BC BR) | CR sees tape |
| `Corner_CR_on` | drive straight `SWING_DELAY_IN` (about 0.3") | timer |
| `Swing_right` | right wheel braked, left wheel forward at `SWING_SPEED` (60) | FC comes back onto tape after at least half of `SWING_MS` (`Swing_FC_on`), or `SWING_MS` (1.45 s, `Swing_time`) |
| `Swing_extra` | keep swinging (right wheel still locked) `SWING_EXTRA_MS` (0.4 s) more, so it lines up fully | timer |
| `Kick` → `Drive_back_blind` | kick, follow the new line, back row ignored until BC sees tape | `Back_row_on` |

FC swings off the tape halfway through the turn and lands back on the new line at 90°, so
"FC back on" ends the swing at the right angle. If the sensors drop out under motor load,
the timer ends it instead.

## Tuning (`CornerTest.c`, "Corner swing tuning")

- **`HALF_TRACK_IN` (4.5"):** measure center of left tire to center of right tire, divide by 2.
  This sets the straight distance before the swing.
- **Ends the turn right of the new line** (swung too late): lower `SWING_DELAY_IN`, or raise
  `HALF_TRACK_IN`. **Left of it** (swung too early): the opposite.
- **Turns too little:** raise `SWING_MS` (1450). **Too much:** lower it.
- **Not quite lined up after the swing:** raise `SWING_EXTRA_MS` (400). **Over-turns:** lower it.
- **Swing stalls or is weak:** raise `SWING_SPEED` (60).
- `CR_AHEAD_IN` (3.8"): CR's distance ahead of the axle. Measure it in Onshape.

## Serial log (115200 baud)

State lines `S,<ms>,CT,<state>`, plus `T` (tape, 7 digits: FL FC FR BL BC BR CR), `M`
(motors), `P` (snapshots).

## Never shuts off

As in `straight_test`: `AD.c` has the course library's low-battery sleep removed,
`EventCheckers.c` never posts `BATTERY_LOW`, and after a brownout reset it drives again at once.

**Brownout indicator:** a supply dip (motors starting hard on a weak battery) can still reset
the board; software can't prevent that. After such a reset, **all four LEDs of bank 3 light
up and stay lit** (bank 3 is otherwise unused), and the log says `Drive_after_brownout`. The
back-up at each corner starts softly (see the table above) to make that less likely. If bank 3
lights often, charge the battery.

## Files

| File | What it does |
|---|---|
| `CornerTest.c` | straight-line correction + corner swing turn (CR) |
| `TapeService.c` | reads the seven tape sensors (CR on W-3), shows them on LED banks 1 and 2 |
| `Robot.c/.h` | motors (incl. `Robot_SwingRight`: right wheel braked, left forward), pin map, CR on W-3 |
| `AD.c`, `TestHarness.c`, `EventCheckers.c`, `TopHSM.c`, `main.c` | same as `straight_test` |

## Steering (changed from straight_test)

`straight_test` has `LEFT_TURN_PCT 30`, which speeds up the left wheel during a left
correction too. With `LEFT_TRIM 105` that leaves about 61 vs 62: almost no left
correction, so a drift to the right never gets fixed. Here both directions are the same:
outer wheel `+turn`, inner wheel `-turn/2` (never below `INNER_MIN` 45), up to `ST_MAX` 16
(66 vs 45). The correction builds up 1 point per 20 ms and eases off 2 per 20 ms.

Tune in `CornerTest.c`: if it swings back and forth across the line, lower `ST_MAX` or
`ST_KP`/`ST_KH`. If it corrects too slowly, raise them.
