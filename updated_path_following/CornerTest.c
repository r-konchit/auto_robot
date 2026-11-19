/*
 * File: CornerTest.c
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
 *   Swing      right wheel braked, left wheel forward at SWING_SPEED. Ends when FC comes
 *              back onto tape (it swings off the tape mid-turn and back onto the new line
 *              at 90 degrees), but not before half of SWING_MS; or after SWING_MS.
 *              Then SWING_EXTRA_MS more (Swing_extra) so it lines up fully.
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
#define ST_SPEED    75          // straight-driving speed (not in a corner turn)
#define ST_KP       8           // points per unit of sideways offset
#define ST_KH       8           // points per unit of angle
#define ST_MAX      16          // biggest correction, in points: outer wheel +16, inner wheel -8
#define ST_MAX_LEFT 14          // biggest LEFT correction (right wheel +14): 16 overshot, 12 too weak
#define FC_OFF_MIN  8           // the moment FC is off tape: at least this much RIGHT correction (right wheel backspins), no ramp
#define ST_RAMP     1          // the correction moves toward its target by this many points...
#define RAMP_MS     20          // ...every this many ms (16 points takes about 320 ms)
#define ST_FADE     2           // ...and fades back toward 0 this many per RAMP_MS
#define INNER_MIN   45          // the inner wheel never drops below this (below ~45 it stalls)
#define LEFT_CORR_TRIM_X10  (LEFT_TRIM * 10 + 25)  // left trim while steering left: +2.5% (107.5)
#define LEFT_RCORR_TRIM_X10 (LEFT_TRIM * 10 + 15)  // left trim while steering right: +1.5% (106.5)
#define RIGHT_CORR_TRIM_X10 (RIGHT_TRIM * 10 + 5)  // right trim while steering left: +0.5% (100.5)
#define RIGHT_RCORR_TRIM_X10 (RIGHT_TRIM * 10 - 5) // right trim while steering right: -0.5% (99.5)
#define CORR_BACKSPIN     10   // steering right: right wheel spins BACKWARD at this power...
#define CORR_BACKSPIN_MS  250  // ...for this long at the start of each right correction
#define KICK_SPEED  80          // startup kick
#define CORNER_KICK_SPEED 75    // kick after a corner: a bit lighter than KICK_SPEED
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
#define SWING_REV_MS    300     // right wheel runs BACKWARD this long at the start of the swing
#define BLIND_MAX_MS    1500    // back row ignored at most this long after a corner

#define KICK_TIMER   TOP_TIMER
#define RAMP_TIMER   FOLLOW_TIMER        // free in this test
#define CORNER_TIMER FIND_PERIM_TIMER    // free in this test

typedef enum {
    CT_WAIT,        // 3 s start delay
    CT_READY,       // needs FC and BC on tape
    CT_KICK,
    CT_DRIVE,       // straight-line correction, watching CR
    CT_PRE_SWING,   // CR saw tape: a little further straight
    CT_SWING,       // right wheel locked, left wheel driving: 90 degrees right
    CT_SWING_EXTRA  // the same, SWING_EXTRA_MS longer
} CornerState;

static CornerState state = CT_WAIT;
static int8_t lastFrontErr = 0;
static int8_t lastBackErr = 0;
static int16_t targetTurn = 0;      // what the sensors ask for (+ = turn left)
static int16_t turn = 0;            // what the motors get now; ramps toward targetTurn
static uint8_t backspinArmed = TRUE;   // next right correction starts with a backspin
static uint32_t backspinStart = 0;
static int16_t baseSpeed = ST_SPEED; // speed the correction steers around (KICK_SPEED during a kick)

static uint8_t backBlind = FALSE;   // just turned: ignore the back row until BC sees tape
static uint32_t blindStart = 0;
static uint32_t swingStart = 0;
static uint8_t fcRecover = FALSE;   // FC went off the tape: turning right until FC and BC are back on
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

    b = RowError(ON(m, BL_BIT), ON(m, BC_BIT), ON(m, BR_BIT), lastBackErr);  // BC always used
    lastFrontErr = f;
    lastBackErr = b;
    t = (ST_KP * (f + b)) / 2 + ST_KH * (f - b);
    if (t > ST_MAX_LEFT) {
        t = ST_MAX_LEFT;             // left corrections capped lower: they overshoot
    } else if (t < -ST_MAX) {
        t = -ST_MAX;
    }
    if (!ON(m, FC_BIT) && (f <= 0)) {
        fcRecover = TRUE;                // FC left the tape: turn right until FC and BC are both back on
    }
    if (fcRecover && ON(m, FC_BIT) && ON(m, BC_BIT)) {
        fcRecover = FALSE;               // FC and BC are back on the tape: done
    }
    if (fcRecover) {
        if (t > -FC_OFF_MIN) {
            t = -FC_OFF_MIN;             // hold a right correction (right wheel backspins)
        }
        if (turn > t) {
            turn = t;                    // skip the ramp: right now
        }
    }
    targetTurn = t;
}

// Steer the same way for both directions: the outer wheel speeds up by the full
// correction and the inner wheel slows by half of it, never below INNER_MIN.
// turn +16 (left): left 45 (50-8, held at 45), right 66.  turn -16: left 66, right 45.
static void ApplyTurn(void)
{
    int16_t mag = (turn >= 0) ? turn : -turn;
    int16_t outer = baseSpeed + mag;
    int16_t inner = baseSpeed - mag / 2;

    if (inner < INNER_MIN) {
        inner = INNER_MIN;
    }
    if (turn >= 0) {
        backspinArmed = TRUE;               // not steering right: arm the next backspin
    }
    if (turn > 0) {
        Robot_LeftTrimX10 = LEFT_CORR_TRIM_X10;             // left wheel +2.5% while steering left
        Robot_RightTrimX10 = RIGHT_CORR_TRIM_X10;           // right wheel +0.5% while steering left
        Robot_SetMotors((int8_t) inner, (int8_t) outer);    // turn left: right wheel faster
        Robot_LeftTrimX10 = LEFT_TRIM * 10;                 // back to normal for everything else
        Robot_RightTrimX10 = RIGHT_TRIM * 10;
    } else if (turn == 0) {
        Robot_SetMotors((int8_t) inner, (int8_t) outer);
    } else {
        Robot_LeftTrimX10 = LEFT_RCORR_TRIM_X10;            // left wheel +1.5% while steering right
        Robot_RightTrimX10 = RIGHT_RCORR_TRIM_X10;          // right wheel -0.5% while steering right
        if (backspinArmed) {
            backspinArmed = FALSE;          // a right correction just started: backspin now
            backspinStart = ES_Timer_GetTime();
        }
        if (fcRecover || ((ES_Timer_GetTime() - backspinStart) < CORR_BACKSPIN_MS)) {
            Robot_SetMotorsRightRaw((int8_t) outer, -CORR_BACKSPIN);  // right wheel backward, 10
        } else {
            Robot_SetMotors((int8_t) outer, (int8_t) inner);    // turn right: left wheel faster
        }
        Robot_LeftTrimX10 = LEFT_TRIM * 10;
        Robot_RightTrimX10 = RIGHT_TRIM * 10;
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

static void Kick(int16_t speed)
{
    lastFrontErr = 0;
    lastBackErr = 0;
    fcRecover = FALSE;
    targetTurn = 0;
    turn = 0;
    baseSpeed = speed;
    UpdateTarget(Tape_GetMask());       // keep correcting FC/BC during the kick too
    ApplyTurn();
    RobotTimer_Start(KICK_TIMER, KICK_MS);
    RobotTimer_Start(RAMP_TIMER, RAMP_MS);
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
    Robot_SetMotors(SWING_SPEED, -SWING_SPEED);  // first SWING_REV_MS: right wheel backward
    RobotTimer_Start(KICK_TIMER, SWING_REV_MS);  // then right wheel locked (Robot_SwingRight)
    RobotTimer_Start(CORNER_TIMER, SWING_MS);
    SetState(CT_SWING, "Swing_right");
}

// CR saw the new line: drive straight SWING_DELAY_IN more, then swing
static void BeginPreSwing(void)
{
    uint32_t ms = (uint32_t) (SWING_DELAY_IN * MS_PER_INCH_BASE);

    if (ms == 0) {
        BeginSwing();
        return;
    }
    baseSpeed = BASE_SPEED;             // corner approach stays at the old speed (50), not ST_SPEED
    ApplyTurn();                        // keep correcting FC/BC until the swing starts
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

static void EndSwing(const char *why)
{
    Robot_Stop();
    Harness_LogState("CT", why);
    backBlind = TRUE;           // the back row may see the old line for a moment
    blindStart = ES_Timer_GetTime();
    Kick(CORNER_KICK_SPEED);        // lighter kick after a corner
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
        if (state == CT_DRIVE || state == CT_KICK || state == CT_PRE_SWING) {
            RampStep();
        }
        RobotTimer_Start(RAMP_TIMER, RAMP_MS);
        ThisEvent.EventType = ES_NO_EVENT;
        return ThisEvent;
    }

    switch (state) {
    case CT_WAIT:
        if (RobotTimer_Expired(ThisEvent, KICK_TIMER)) {
            SetState(CT_READY, "Stop_need_FC_and_BC");
            m = Tape_GetMask();
            if (ON(m, FC_BIT) && ON(m, BC_BIT)) {
                Kick(KICK_SPEED);
            }
        }
        break;

    case CT_READY:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            if (ON(m, FC_BIT) && ON(m, BC_BIT)) {
                Kick(KICK_SPEED);
            }
        }
        break;

    case CT_KICK:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            BackBlindCheck(m);
            UpdateTarget(m);                    // correcting during the kick
        } else if (RobotTimer_Expired(ThisEvent, KICK_TIMER)) {
            baseSpeed = ST_SPEED;
            SetState(CT_DRIVE, backBlind ? "Drive_back_blind" : "Drive");
            UpdateTarget(Tape_GetMask());
            RampStep();
            RobotTimer_Start(RAMP_TIMER, RAMP_MS);
        }
        break;

    case CT_DRIVE:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            BackBlindCheck(m);
            if (ON(m, CR_BIT) && !backBlind) {
                BeginPreSwing();            // the corner
            } else {
                UpdateTarget(m);            // straight-line correction
            }
        }
        break;

    case CT_PRE_SWING:
        if (ThisEvent.EventType == TAPE_CHANGED) {
            UpdateTarget((uint8_t) ThisEvent.EventParam);   // correcting until the swing
        } else if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            BeginSwing();
        }
        break;

    case CT_SWING:
        if (RobotTimer_Expired(ThisEvent, KICK_TIMER)) {
            Robot_SwingRight(SWING_SPEED);      // reverse done: right wheel locked, left forward
        } else if (ThisEvent.EventType == TAPE_CHANGED) {
            m = (uint8_t) ThisEvent.EventParam;
            if (!ON(m, FC_BIT)) {
                fcSeenOff = TRUE;
            } else if (fcSeenOff && (ES_Timer_GetTime() - swingStart) >= SWING_MS / 2) {
                RobotTimer_Stop(CORNER_TIMER);
                SwingExtra("Swing_FC_on");      // FC back on: on the new line
            }
        } else if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            SwingExtra("Swing_time");           // SWING_MS: about 90 degrees
        }
        break;

    case CT_SWING_EXTRA:
        if (RobotTimer_Expired(ThisEvent, KICK_TIMER)) {
            Robot_SwingRight(SWING_SPEED);      // reverse done: right wheel locked, left forward
        } else if (RobotTimer_Expired(ThisEvent, CORNER_TIMER)) {
            EndSwing("Swing_done");             // lined up: kick and follow the line
        }
        break;

    default:
        break;
    }

    ThisEvent.EventType = ES_NO_EVENT;
    return ThisEvent;
}
