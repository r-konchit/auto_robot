# current_testing

The working copy to iterate from. `new_test.hex` is built from this folder.

## What it does

1. **Wait** 3 s after reset. Set down on the tape (FC and BC on it): it starts following
   right away. Anywhere else: step 2.
2. **Straight**: both wheels forward (kick 80, then 60) until **CR** (front-right corner
   sensor) sees tape.
3. **Find_CR_on**: about 0.3" more (the corner-test geometry).
4. **Find_rotate_right**: rotate right about the right wheel (right wheel braked, left
   wheel forward) until **FC and BC are both on the tape** (`FC_BC_on_tape`), then the same
   in-line check as after a corner. Gives up
   after about 180° (`Find_rotate_time`).
5. **Kick → Drive**: the corner test (6:51 pm build): straight-line balancing on the tape,
   right swing turn at every corner (CR sees the next side). The swing keeps turning
   until **FC and BC are both on the new line** (`Swing_FC_BC_on`), then, while the tape
   is still under FR or BL (turned a little short), keeps turning until **FC and BC are
   in line** (`Aligned_FC_BC_in_line`), then **0.2 s with the right wheel
   spinning backward** so the turn fully completes (`Turn_finish_right_back`,
   `TURN_FINISH_MS`, `TURN_FINISH_REV`), then the balancing (both rows) takes over. This also covers turning onto a cross arm. CR is watched from the start of the
   kick after a turn (not only once Drive starts), so a tight next corner isn't missed. Every sensor off while
   following → `Line_lost_straight` → step 2.

## Build and flash

Open `CornerTest.X` in MPLAB X and build, or from a terminal in `CornerTest.X`:

```
make -f nbproject/Makefile-default.mk CONF=default build
```

The hex comes out as `CornerTest.X/dist/default/production/CornerTest.X.production.hex`;
copy it over `new_test.hex`.

## Files

| File | What it does |
|---|---|
| `CornerTest.c` | the find (straight, CR, rotate right) and the corner test |
| `TapeService.c` | reads the seven tape sensors (CR on W-3), LED banks 1 and 2 |
| `Robot.c/.h` | motors, pin map, timers, tuning constants |
| `AD.c`, `TestHarness.c`, `EventCheckers.c`, `TopHSM.c`, `main.c` | framework glue, serial log (115200 baud: `S,<ms>,CT,<state>`) |

Tuning for the find is at the top of `CornerTest.c` ("Start anywhere"); the corner swing
values are under "Corner swing tuning".
