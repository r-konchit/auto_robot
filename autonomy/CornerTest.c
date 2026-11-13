/*
 * File: CornerTest.c
 *
 * START ANYWHERE: straight until CR sees tape, rotate right until FC and BC are both on
 * the tape, then the corner test below (6:51 pm build) for good.
 *   Straight     both wheels straight (kick KICK_SPEED, then STRAIGHT_SPEED) until CR
 *                sees tape
 *   Find_CR_on   straight SWING_DELAY_IN more (the corner-test geometry: CR on the tape's
 *                near edge = axle 4.8" short of its middle)
 *   Find_rotate  rotate right about the right wheel (right wheel braked, left wheel
 *                forward). That moves the axle HALF_TRACK_IN forward and right while it
 *                turns, so it ends over the tape. Stops when FC and BC are both on the
 *                tape (they must come on during the turn, after FIND_MIN_MS), or after
 *                FIND_MAX_MS.
 *   -> Kick -> Drive: the corner test. Every sensor off while following -> Straight.
 *
 * Line following around the perimeter with corner SWING turns (see CornerTest.h).
 * TopHSM hands every event here. The robot drives CLOCKWISE, so every corner is a
 * right turn.
 *
 * STRAIGHT LINE. The tape is 2" wide and the side sensors are 2" from the center, so
 * with the robot centered only FC and BC see tape:
 *
 *   row error   +2  center off, LEFT sensor on   (row is well right of the line)
 *               +1  center on,  LEFT sensor on   (row is a little right)
 *                0  center only                  (row centered)
 *               -1  center on,  RIGHT sensor on  (row is a little left)
 *               -2  center off, RIGHT sensor on  (row is well left)
 *               no sensor in the row: keep the last value
 *
 *   turn left by  KP * (front + back) / 2  +  KH * (front - back), ramped.
 *   Steering is the same strength both ways (outer wheel +turn, inner -turn/2).
 *
 * CORNER: a 90 degree swing turn about the right wheel.
 *   Sensor positions (CAD, inches from the axle, x right, y forward):
 *     FC (0, 4.4)  FL (-2, 2.4)  FR (2, 2.4)  BL (-2, -2.4)  BR (2, -2.4)  BC (0, -4.4)
 *     CR (3.7, 3.8)  front-right corner sensor, W-3 (Robot.h)
 *   The wheels are HALF_TRACK_IN to each side of the axle center.
 *
 *   When CR first sees the new line (its near edge), the axle is CR_AHEAD_IN + 1 = 4.8"
 *   short of the new line's centerline. A 90 degree swing about the right wheel moves
 *   the axle center HALF_TRACK_IN forward and HALF_TRACK_IN to the right, ending
 *   heading along the new line. So the swing starts after 4.8 - HALF_TRACK_IN more of
 *   travel (about 0.3"), and the axle lands on the new line.
 *
 *   Pre_swing  CR saw tape: drive straight SWING_DELAY_IN (about 0.3")
 *   Swing      right wheel braked, left wheel forward at SWING_SPEED. Ends when FC AND BC
 *              are both on the new line (FC swings off the tape mid-turn and back on; not
 *              before half of SWING_MS) -> Kick and balance with both rows. If BC isn't on
 *              by SWING_MS it keeps turning up to SWING_EXTRA_MS more (Swing_extra), still
 *              stopping as soon as FC and BC are on; then balances anyway.
 *   Turn_align FC and BC land on the tape's edge a little before the robot is lined up:
 *              while the tape is still under FR or BL (turned short) it keeps turning
 *              right until they're off = FC and BC in line, both rows centered. Then Kick
 *              and balance with both rows. Same after the first turn onto the tape.
 *   Turn_finish every right turn ends with TURN_FINISH_MS (0.2 s) of the right wheel
 *              spinning BACKWARD (left wheel forward), so it fully completes the turn
 *              and doesn't run off the tape. Then Kick and balance.
 *   Kick       then Drive on the new line. The back row is ignored ("back blind") until
 *              BC sees tape (or BLIND_MAX_MS goes by).
 *
 *   Wait   3 s after reset to set the robot down (once; skipped after a brownout)
 *   Ready  waits until FC and BC are both on tape, then kicks
 *
 * The serial log (115200 baud) prints S,<ms>,CT,<state> on every state change.
 */

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "LED.h"
#include "Robot.h"
#include "TapeService.h"
#include "TestHarness.h"
#include "CornerTest.h"
#include <stdio.h>

/* ---- Straight-line tuning ---- */
#define ST_SPEED    BASE_SPEED  // 50
#define ST_KP       8           // points per unit of sideways offset
#define ST_KH       8           // points per unit of angle
#define ST_MAX      16          // biggest correction, in points: outer wheel +16, inner wheel -8
#define ST_RAMP     1           // the correction moves toward its target by this many points...
#define RAMP_MS     20          // ...every this many ms (16 points takes about 320 ms)
#define ST_FADE     2           // ...and fades back toward 0 this many per RAMP_MS
#define INNER_MIN   45          // the inner wheel never drops below this (below ~45 it stalls)
#define KICK_SPEED  80
#define KICK_MS     150

/* ---- Corner swing tuning [TUNE] ---- */
#define CR_AHEAD_IN     3.8     // CR in front of the axle (CAD; measure in Onshape)
// Axle center to the middle of a wheel (half the wheel spacing). MEASURE IT: center of
// the left tire to center of the right tire, divided by 2.
#define HALF_TRACK_IN   4.5
// Straight travel after CR sees tape, before the swing starts (4.8" - HALF_TRACK_IN).
// Robot ends the turn RIGHT of the new line (swung too late): lower it.
// LEFT of the new line / turns before reaching it: raise it.
#define SWING_DELAY_IN  (CR_AHEAD_IN + TAPE_W_IN / 2 - HALF_TRACK_IN)
#define SWING_SPEED     60      // left wheel power during the swing (the right wheel is braked)
// How long the swing takes for 90 degrees. The left wheel travels a quarter circle of
// radius 2 x HALF_TRACK_IN (about 14"); at SWING_SPEED that's about 1.45 s by the speed
// estimate. Turns too little: raise it. Too much: lower it.
#define SWING_MS        1450
// After the swing would end (FC back on tape, or SWING_MS), keep swinging this much
// longer (right wheel still locked) so the robot lines up fully with the new line.
#define SWING_EXTRA_MS  400
// After a right turn FC and BC first land on the EDGE of the tape, still turned a bit
// short. Then the tape is also under FR (front row left of the line) and/or BL (back
// row right of it). Keep turning right until FR and BL are off: FC and BC in line, both
// rows centered. At most this long.
#define ALIGN_MAX_MS    500
// The last part of every right turn: the right wheel spins BACKWARD (left wheel still
// forward) for TURN_FINISH_MS so the turn fully completes and the robot doesn't run
// off the tape. Still short: raise TURN_FINISH_MS or TURN_FINISH_REV. Too far: lower.
#define TURN_FINISH_MS  200
#define TURN_FINISH_REV 45
#define BLIND_MAX_MS    1500    // back row ignored at most this long after a corner

/* ---- Start anywhere [TUNE] ---- */
#define STRAIGHT_SPEED  60      // straight search speed
#define FIND_MIN_MS     (SWING_MS / 3)  // FC+BC before this in the rotation don't count
#define FIND_MAX_MS     (2 * SWING_MS)  // about 180 degrees: give up, follow anyway
#define LOST_MS         150     // following, every sensor off this long: straight again

#define KICK_TIMER   TOP_TIMER
#define RAMP_TIMER   FOLLOW_TIMER        // free in this test
#define CORNER_TIMER FIND_PERIM_TIMER    // free in this test

typedef enum {
    CT_WAIT,        // 3 s start delay
    CT_STRAIGHT,    // straight until CR sees tape
    CT_FIND_PRE,    // CR on: SWING_DELAY_IN more
    CT_FIND_ROTATE, // rotate right until FC and BC are both on the tape
    CT_KICK,
    CT_DRIVE,       // straight-line correction, watching CR
    CT_PRE_SWING,   // CR saw tape: a little further straight
    CT_SWING,       // right wheel locked, left wheel driving: 90 degrees right
    CT_SWING_EXTRA, // the same, SWING_EXTRA_MS longer
    CT_TURN_ALIGN,  // FC+BC on: keep turning right until FC and BC are in line
    CT_TURN_FINISH  // last TURN_FINISH_MS of the turn: right wheel spinning backward
} CornerState;

static CornerState state = CT_WAIT;
static int8_t lastFrontErr = 0;
static int8_t lastBackErr = 0;
static int16_t targetTurn = 0;      // what the sensors ask for (+ = turn left)
static int16_t turn = 0;            // what the motors get now; ramps toward targetTurn

static uint8_t backBlind = FALSE;   // just turned: ignore the back row until BC sees tape
static uint32_t blindStart = 0;
static uint32_t swingStart = 0;
static uint8_t fcSeenOff = FALSE;   // during the swing: FC has been off tape at least once

static void SetState(CornerState s, const char *name)
{
    state = s;
    Harness_LogState("CT", name);
}

/* ============================ straight-line logic ============================ */

// Where one row is relative to the line (table at the top)
static int8_t RowError(uint8_t left, uint8_t center, uint8_t right, int8_t last)
{
    if (left && !right) {
        return center ? 1 : 2;
    }
    if (right && !left) {
        return center ? -1 : -2;
    }
    if (center) {
        return 0;               // center only, or a crossing line under the row
    }
    return last;
}

// New sensor reading: work out the target correction (the ramp applies it)
static void UpdateTarget(uint8_t m)
{
    int8_t f = RowError(ON(m, FL_BIT), ON(m, FC_BIT), ON(m, FR_BIT), lastFrontErr);
    int8_t b;
    int16_t t;

    if (backBlind) {
        b = 0;                  // just after a corner: the back row may see the old line
    } else {
        b = RowError(ON(m, BL_BIT), ON(m, BC_BIT), ON(m, BR_BIT), lastBackErr);
    }
    lastFrontErr = f;
    lastBackErr = b;
    t = (ST_KP * (f + b)) / 2 + ST_KH * (f - b);
    if (t > ST_MAX) {
        t = ST_MAX;
    } else if (t < -ST_MAX) {
        t = -ST_MAX;
    }
    targetTurn = t;
}

// Steer the same way for both directions: the outer wheel speeds up by the full
// correction and the inner wheel slows by half of it, never below INNER_MIN.
// turn +16 (left): left 45 (50-8, held at 45), right 66.  turn -16: left 66, right 45.
static void ApplyTurn(void)
{
    int16_t mag = (turn >= 0) ? turn : -turn;
    int16_t outer = ST_SPEED + mag;
    int16_t inner = ST_SPEED - mag / 2;

    if (inner < INNER_MIN) {
        inner = INNER_MIN;
    }
    if (turn >= 0) {
        Robot_SetMotors((int8_t) inner, (int8_t) outer);    // turn left: right wheel faster
    } else {
        Robot_SetMotors((int8_t) outer, (int8_t) inner);    // turn right: left wheel faster
    }
}

// Every RAMP_MS: move the correction one step toward the target. Building up is
// gentle (ST_RAMP); easing off toward straight is quicker (ST_FADE).
static void RampStep(void)
{
    int16_t step;

    if (turn < targetTurn) {
        step = (turn < 0) ? ST_FADE : ST_RAMP;
        turn += step;
        if (turn > targetTurn) {
            turn = targetTurn;
        }
    } else if (turn > targetTurn) {
        step = (turn > 0) ? ST_FADE : ST_RAMP;
        turn -= step;
        if (turn < targetTurn) {
            turn = targetTurn;
        }
    }
    ApplyTurn();
}

static void Kick(void)
{
    lastFrontErr = 0;
    lastBackErr = 0;
    targetTurn = 0;
    turn = 0;
    Robot_Forward(KICK_SPEED);
    RobotTimer_Start(KICK_TIMER, KICK_MS);
    SetState(CT_KICK, "Kick");
}

static void BackBlindCheck(uint8_t m)
{
    if (backBlind && (ON(m, BC_BIT) || (ES_Timer_GetTime() - blindStart) >= BLIND_MAX_MS)) {
        backBlind = FALSE;
        Harness_LogState("CT", "Back_row_on");
    }
}

/* ================================ corner swing ================================ */

static void BeginSwing(void)
{
    swingStart = ES_Timer_GetTime();
    fcSeenOff = !ON(Tape_GetMask(), FC_BIT);
    Robot_SwingRight(SWING_SPEED);      // right wheel locked, left wheel forward
    RobotTimer_Start(CORNER_TIMER, SWING_MS);
    SetState(CT_SWING, "Swing_right");
}

// CR saw the new line: drive straight SWING_DELAY_IN more, then swing
static void BeginPreSwing(void)
{
    uint32_t ms = (uint32_t) (SWING_DELAY_IN * MS_PER_INCH_BASE);

    targetTurn = 0;
    turn = 0;
    if (ms == 0) {
        BeginSwing();
        return;
    }
    Robot_Forward(ST_SPEED);
    RobotTimer_Start(CORNER_TIMER, ms);
    SetState(CT_PRE_SWING, "Corner_CR_on");
}

static void EndSwing(const char *why);

// The swing reached its end (FC back on, or SWING_MS): keep it going SWING_EXTRA_MS more
static void SwingExtra(const char *why)
{
    Harness_LogState("CT", why);
    if (SWING_EXTRA_MS == 0) {
        EndSwing("Swing_done");
        return;
    }
    RobotTimer_Start(CORNER_TIMER, SWING_EXTRA_MS);
    SetState(CT_SWING_EXTRA, "Swing_extra");
}

static void TurnFinish(uint8_t blind);

static void EndSwing(const char *why)
{
    Harness_LogState("CT", why);
    TurnFinish(TRUE);           // then back blind: the back row may see the old line
}

// Turned right a little short of the tape: the tape still under FR (front row left of
// the line) or BL (back row right of it)
static uint8_t TurnedShort(uint8_t m)
{
    return (ON(m, FR_BIT) && !ON(m, FL_BIT)) || (ON(m, BL_BIT) && !ON(m, BR_BIT));
}

static void TurnDone(const char *why);

// FC and BC both on the new line after a right turn. If it's still turned short, keep
// turning right (right wheel locked) until FC and BC are in line; else it's done.
static void SwingAligned(const char *why)
{
    uint8_t m = Tape_GetMask();

    Harness_LogState("CT", why);
    if (TurnedShort(m)) {
        Robot_SwingRight(SWING_SPEED);  // (already swinging; make sure)
        RobotTimer_Start(CORNER_TIMER, ALIGN_MAX_MS);
        SetState(CT_TURN_ALIGN, "Turn_align_FC_BC");
        return;
    }
    TurnDone("Aligned_FC_BC_in_line");
}

// FC and BC in line: stop and let the straight-line balancing (front AND back rows, no
// back blind) take over
static uint8_t finishBlind = FALSE;  // back blind after the finish (gave-up turns only)

// The turn is about done: finish it with the right wheel spinning backward for
// TURN_FINISH_MS (left wheel forward), then Kick and balance
static void TurnFinish(uint8_t blind)
{
    finishBlind = blind;
    Robot_SetMotors(SWING_SPEED, -TURN_FINISH_REV);
    RobotTimer_Start(CORNER_TIMER, TURN_FINISH_MS);
    SetState(CT_TURN_FINISH, "Turn_finish_right_back");
}

static void TurnDone(const char *why)
{
    RobotTimer_Stop(CORNER_TIMER);
    Harness_LogState("CT", why);
    TurnFinish(FALSE);
}

// During the swing: FC and BC both on the new line (FC came back on, past half the turn)
static uint8_t SwingLinedUp(uint8_t m)
{
    return ON(m, FC_BIT) && ON(m, BC_BIT) && fcSeenOff
            && (ES_Timer_GetTime() - swingStart) >= SWING_MS / 2;
}

/* ================================ start anywhere ================================ */

static void BeginStraight(const char *why)
{
    targetTurn = 0;
    turn = 0;
    backBlind = FALSE;
    Robot_Forward(KICK_SPEED);
    RobotTimer_Start(KICK_TIMER, KICK_MS);
    RobotTimer_Stop(CORNER_TIMER);
    SetState(CT_STRAIGHT, why);
}

static void LostStart(void)
{
    lastFrontErr = 0;
    lastBackErr = 0;
    targetTurn = 0;
    turn = 0;
    ApplyTurn();
    RobotTimer_Start(CORNER_TIMER, LOST_MS);
}

// FC and BC on the tape (or gave up): stop the rotation, follow with the corner test
static void FindDone(const char *why)
{
    Robot_Stop();
    RobotTimer_Stop(CORNER_TIMER);
    Harness_LogState("CT", why);
    backBlind = FALSE;
    Kick();
}

/* ================================= top level ================================= */

ES_Event RunCornerTest(ES_Event ThisEvent)
{
    uint8_t m;

    if ((ThisEvent.EventType == ES_INIT) && Robot_WasBrownout) {
        // The supply dipped and reset the board mid-run: drive again right away,
        // no wait, no FC+BC check, no kick (its current spike may be what dipped it).
        // All four LEDs of bank 3 light up and stay lit: "the board reset from a brownout".
        LED_SetBank(LED_BANK3, 0x0F);
        RobotTimer_Start(RAMP_TIMER, RAMP_MS);
        if (Tape_GetMask() == 0) {
            BeginStraight("Straight_after_brownout");
            ThisEvent.EventType = ES_NO_EVENT;
            return ThisEvent;
        }
        targetTurn = 0;
        turn = 0;
        SetState(CT_DRIVE, "Drive_after_brownout");
        UpdateTarget(Tape_GetMask());
        ApplyTurn();
        RobotTimer_Start(RAMP_TIMER, RAMP_MS);
        ThisEvent.EventType = ES_NO_EVENT;
        return ThisEvent;
    }
    if (ThisEvent.EventType == ES_INIT) {
        Robot_Stop();
        SetState(CT_WAIT, "Wait");
        RobotTimer_Start(KICK_TIMER, START_DELAY_MS);
        ThisEvent.EventType = ES_NO_EVENT;
        return ThisEvent;
    }

    // The ramp ticks the whole time; it only drives the motors while following the line
    if (RobotTimer_Expired(ThisEvent, RAMP_TIMER)) {
        if (state == CT_DRIVE) {
            RampStep();
        }
        RobotTimer_Start(RAMP_TIMER, RAMP_MS);
        ThisEvent.EventType = ES_NO_EVENT;
        return ThisEvent;
    }

    switch (state) {
    case CT_WAIT:
        if (RobotTimer_Expired(ThisEvent, KICK_TIMER)) {
            m = Tape_GetMask();
            if (ON(m, FC_BIT) && ON(m, BC_BIT)) {
                Kick();                     // set down on the tape: follow it
            } else {
                BeginStraight("Straight");
            }
        }
        break;

    /* ---- Straight until CR sees tape ---- */
    case CT_STRAIGHT:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            if (ON((uint8_t) ThisEvent.EventParam, CR_BIT)) {
                RobotTimer_Stop(KICK_TIMER);
                Robot_Forward(ST_SPEED);
                RobotTimer_Start(CORNER_TIMER, (uint32_t) (SWING_DELAY_IN * MS_PER_INCH_BASE) + 1);
                SetState(CT_FIND_PRE, "Find_CR_on");
            }
        } else if (RobotTimer_Expired(ThisEvent, KICK_TIMER)) {
            Robot_Forward(STRAIGHT_SPEED);
        }
        break;

    case CT_FIND_PRE:
        if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            swingStart = ES_Timer_GetTime();
            Robot_SwingRight(SWING_SPEED);  // rotate right about the right wheel
            RobotTimer_Start(CORNER_TIMER, FIND_MAX_MS);
            SetState(CT_FIND_ROTATE, "Find_rotate_right");
        }
        break;

    /* ---- Rotate right until FC and BC are both on the tape ---- */
    case CT_FIND_ROTATE:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            if (ON(m, FC_BIT) && ON(m, BC_BIT)
                    && (ES_Timer_GetTime() - swingStart) >= FIND_MIN_MS) {
                SwingAligned("FC_BC_on_tape");  // then in line, like after a corner
            }
        } else if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            FindDone("Find_rotate_time");   // gave up: the line following takes over
        }
        break;

    case CT_KICK:
        // CR is watched during the kick too (it used to start only when Drive did, about
        // 0.15-0.2 s later): on a short side the next corner can come right after a turn.
        if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            BackBlindCheck(m);
            if (ON(m, CR_BIT) && !backBlind) {
                RobotTimer_Stop(KICK_TIMER);
                BeginPreSwing();            // the next corner, already
            }
        } else if (RobotTimer_Expired(ThisEvent, KICK_TIMER)) {
            SetState(CT_DRIVE, backBlind ? "Drive_back_blind" : "Drive");
            m = Tape_GetMask();
            if (ON(m, CR_BIT) && !backBlind) {
                BeginPreSwing();            // CR already on (no change event would come)
                break;
            }
            UpdateTarget(m);
            RampStep();
            RobotTimer_Start(RAMP_TIMER, RAMP_MS);
            if (m == 0) {
                LostStart();
            }
        }
        break;

    case CT_DRIVE:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            if (m == 0) {
                LostStart();
                break;
            }
            RobotTimer_Stop(CORNER_TIMER);
            BackBlindCheck(m);
            if (ON(m, CR_BIT) && !backBlind) {
                BeginPreSwing();            // the corner
            } else {
                UpdateTarget(m);            // straight-line correction
            }
        } else if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            if (Tape_GetMask() == 0) {
                BeginStraight("Line_lost_straight");
            }
        }
        break;

    case CT_PRE_SWING:
        if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            BeginSwing();
        }
        break;

    case CT_SWING:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            if (!ON(m, FC_BIT)) {
                fcSeenOff = TRUE;
            } else if (SwingLinedUp(m)) {
                SwingAligned("Swing_FC_BC_on");  // FC and BC on the new line: done
            }
        } else if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            SwingExtra("Swing_time");           // SWING_MS and BC not on yet: keep turning
        }
        break;

    case CT_SWING_EXTRA:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            if (!ON(m, FC_BIT)) {
                fcSeenOff = TRUE;
            } else if (SwingLinedUp(m)) {
                SwingAligned("Swing_FC_BC_on"); // FC and BC on the new line: done
            }
        } else if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            EndSwing("Swing_done");             // gave up waiting: balance from here
        }
        break;

    /* ---- After a right turn: keep turning until FC and BC are in line ---- */
    case CT_TURN_ALIGN:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            if (!TurnedShort(m)) {
                TurnDone("Aligned_FC_BC_in_line");  // FR and BL off: both rows centered
            }
        } else if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            TurnDone("Turn_align_time");    // close enough: the balancing does the rest
        }
        break;

    /* ---- Last part of the turn: right wheel backward, then balance ---- */
    case CT_TURN_FINISH:
        if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            Robot_Stop();
            backBlind = finishBlind;
            blindStart = ES_Timer_GetTime();
            Kick();                         // the turn is complete: balance on the tape
        }
        break;

    default:
        break;
    }

    ThisEvent.EventType = ES_NO_EVENT;
    return ThisEvent;
}
