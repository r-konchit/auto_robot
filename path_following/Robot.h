/*
 * File: Robot.h
 *
 * Hardware layer for the perimeter test build (design 3, 6, 7, v17 CAD sensor layout):
 *   - pin map (design 3.7)
 *   - tunable constants (design 11)
 *   - L298N drive motors and the six tape sensor signals
 *   - edge following and tape-pattern helpers
 *   - RobotTimer_* wrappers that reject stale ES_TIMEOUT events
 *
 * Sensor layout (Onshape base plate, sensor at each slot center), inches from the
 * robot center (= the wheel axle, the pivot point), x to the right, y forward:
 *
 *              FC (0, 4.4)
 *   FL (-2, 2.4)          FR (2, 2.4)
 *              [ wheels ]
 *   BL (-2,-2.4)          BR (2,-2.4)
 *              BC (0,-4.4)
 *
 * The tape is 2" wide, the same as the side spacing. So the side sensors cannot
 * center the robot on the tape. Instead the robot follows the INNER edge of the
 * tape (the field side):
 *   - FC on tape, outer-side sensor on wood  -> the center is over the tape: steer toward the field
 *   - FC on wood, outer-side sensor on tape  -> the center is on the wood:   steer away from the field
 *   - both on                                -> right on the edge: straight
 *   The back row (BC + outer back sensor) does the same at the back, which gives the angle.
 * On a straight edge the field-side sensor (FL when counterclockwise) can never reach
 * the tape without the robot already being out. It only sees tape where a new edge
 * crosses in front of the robot: that is how a corner is detected.
 *
 * [TUNE] marks numbers to measure on the real robot or field.
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "AD.h"
#include "pwm.h"
#include "IO_Ports.h"

/*******************************************************************************
 * PIN MAP: every motor wire on Port Y, every tape sensor on Port V            *
 *                                                                             *
 *   Uno32 pin   HiLetgo L298N pin (use the board's printed labels)            *
 *   Y-10        ENA              left motor speed (PWM3)                      *
 *   Y-06        IN1              left motor direction                         *
 *   Y-07        IN2              left motor direction                         *
 *   Y-08        IN3              right motor direction                        *
 *   Y-09        IN4              right motor direction                        *
 *   Y-12        ENB              right motor speed (PWM2)                     *
 *                                                                             *
 *   Left motor on OUT1/OUT2, right motor on OUT3/OUT4.                        *
 *   Pull the ENA and ENB jumpers. Keep the 5V regulator jumper on and leave   *
 *   the +5V terminal empty.                                                   *
 *   Port Y is the only port with two PWM pins clear of the LED banks.         *
 *                                                                             *
 *   LED banks (not used for I/O): Bank 1 Y-03 Y-04 Y-05 X-12,                 *
 *   Bank 2 X-07..X-10, Bank 3 X-03..X-06.                                     *
 *                                                                             *
 *   Uno32 pin   Sensor signal                                                 *
 *   V-3 V-4 V-5 FL FC FR (front row)                                          *
 *   V-6 V-7 V-8 BL BC BR (back row)                                           *
 *   W-3         CR, front-right corner sensor (7th, analog)                   *
 ******************************************************************************/

// L298N
#define LEFT_EN_PWM         PWM_PORTY10
#define LEFT_IN1_LAT        PORTY06_LAT
#define LEFT_IN2_LAT        PORTY07_LAT
#define RIGHT_EN_PWM        PWM_PORTY12
#define RIGHT_IN3_LAT       PORTY08_LAT
#define RIGHT_IN4_LAT       PORTY09_LAT
#define PORTY_OUTPUT_PINS   (PIN6 | PIN7 | PIN8 | PIN9)

// Tape sensors
#define TAPE_FL_PIN         AD_PORTV3
#define TAPE_FC_PIN         AD_PORTV4
#define TAPE_FR_PIN         AD_PORTV5
#define TAPE_BL_PIN         AD_PORTV6
#define TAPE_BC_PIN         AD_PORTV7
#define TAPE_BR_PIN         AD_PORTV8

// 7th sensor: front-right CORNER sensor ("CR"), on the angled slot at the front-right
// corner of the base plate, about (3.7, 3.8)" from the axle [TUNE: measure in Onshape].
// Wire its signal to W-3 (analog, read exactly like the other six), +5V and GND as usual.
// Port Y has no analog inputs: the only free Y pin (Y-11) is digital, and a 0.5 V / 1.0 V
// sensor signal reads LOW there on tape and floor alike. Only if the sensor is a module
// with a DIGITAL output (DO pin, comparator on the board) can it go on Y-11: then
// uncomment CR_ON_Y11 (and set CR_Y11_TAPE_LEVEL to what DO reads over the tape).
#define TAPE_CR_PIN         AD_PORTW3
// #define CR_ON_Y11
#define CR_Y11_TAPE_LEVEL   1       // most TCRT5000 modules: DO high over black tape
#define CR_Y11_BIT          PORTY11_BIT
#define CR_Y11_TRIS         PORTY11_TRIS

#ifdef CR_ON_Y11
#define TAPE_ALL_PINS       (TAPE_FL_PIN | TAPE_FC_PIN | TAPE_FR_PIN | \
                             TAPE_BL_PIN | TAPE_BC_PIN | TAPE_BR_PIN)
#else
#define TAPE_ALL_PINS       (TAPE_FL_PIN | TAPE_FC_PIN | TAPE_FR_PIN | \
                             TAPE_BL_PIN | TAPE_BC_PIN | TAPE_BR_PIN | TAPE_CR_PIN)
#endif

// Tape thresholds and the tape LEDs live in TapeService.c.

/*******************************************************************************
 * TAPE MASK BITS (design 4.1; bit 3 was MC)                                   *
 ******************************************************************************/

#define FL_BIT  0
#define FC_BIT  1
#define FR_BIT  2
#define BL_BIT  4
#define BC_BIT  5
#define BR_BIT  6
#define CR_BIT  3       // front-right corner sensor (the free bit, was MC)
#define NUM_TAPE_SENSORS 7

#define ON(m, b)  (((m) >> (b)) & 1)

/*******************************************************************************
 * TUNABLE CONSTANTS (design 11)                                               *
 ******************************************************************************/

// Speeds, percent of full duty
#define BASE_SPEED          50      // following and legs
#define CREEP_SPEED         40      // creeps (v19: was 25, which ran at 28 and stalled on the robot)
#define PIVOT_SPEED         45      // all pivots (v19: was 30; a pivot needs the most torque)
#define MIN_DUTY            40      // below this the wheels stall [TUNE with MOTOR_TEST]
// Motor trim, percent: every command to that motor is multiplied by this (after
// MIN_DUTY). Raise the SLOWER motor above 100 so both wheels match and the robot
// drives straight. Robot drifts right -> left wheel is faster -> raise RIGHT_TRIM.
// Robot drifts left -> raise LEFT_TRIM. Go in steps of 2-3. [TUNE with LineTest]
#define LEFT_TRIM           105     // left motor is the slower one
#define RIGHT_TRIM          100
// Motor direction: -1 reverses that motor (fixes swapped motor wires in software).
// If a wheel spins backward on a forward command, flip its sign.
#define LEFT_DIR            (-1)
#define RIGHT_DIR           (-1)
// v19: the L298N drops 2-3 V, so at 28-30 % of a 9.9 V pack a motor gets about 1 V and
// a robot this heavy doesn't move. That is why the robot sat still in creeps and pivots.

// Distance and angle from time (design 6.4) [TUNE: measure, design 12.1]
// The state machines only use timing for short legs (at most 2.1"), so a 25% error
// in these costs under an inch. Measure them anyway (design 12.1).
#define MS_PER_INCH_BASE    122     // at BASE_SPEED (estimated, not measured)
#define MS_PER_INCH_CREEP   203     // at CREEP_SPEED (estimated: measure with MOTOR_TEST)
#define MS_PER_DEG          13      // at PIVOT_SPEED (estimated: measure with MOTOR_TEST)

#define IN_TO_MS(in)        ((uint32_t) ((in) * MS_PER_INCH_BASE))
#define IN_TO_MS_CREEP(in)  ((uint32_t) ((in) * MS_PER_INCH_CREEP))
#define DEG_TO_MS(deg)      ((uint32_t) ((deg) * MS_PER_DEG))

// Edge following (see the top of this file), percent of speed
#define EDGE_KP             20      // per unit of "where the edge is" (front + back average)
#define EDGE_KH             20      // per unit of angle (front minus back)
#define EDGE_MAX            40      // clamp: the slow wheel never drops below 60% of speed

// Geometry from the CAD, inches
#define FC_AHEAD_IN         4.4     // FC (and BC) from the wheel axle
#define SIDE_AHEAD_IN       2.4     // FL/FR (and BL/BR) from the wheel axle
#define TAPE_W_IN           2.0
// Creeps end with the wheels a little short of the field-side edge of the line, not on
// its middle: the robot follows that edge anyway, and a robot up to 25% faster than the
// estimate (plus coasting after the brake) then still stops well short of the outer edge.
#define WHEELS_SHORT_IN     0.3
// FC just crossed the far (outer) edge of a line -> creep this much more
#define CENTER_EXTRA_IN     (FC_AHEAD_IN - TAPE_W_IN - WHEELS_SHORT_IN)     // 2.1
// The field-side sensor just crossed the far edge of the new edge -> creep this much more
#define CORNER_EXTRA_IN     (SIDE_AHEAD_IN - TAPE_W_IN - WHEELS_SHORT_IN)   // 0.1
// Pivoting from there, FC lands on the outer edge of the line about 58 degrees
// into the turn (cos = 2.3 / 4.4), then turns CENTER_EXTRA_DEG more to be parallel.
#define CENTER_EXTRA_DEG    23      // cos^-1(2.3/4.4) = 58, so 32 on paper; 23 tested best: the robot keeps turning a little after the brake
#define LINE_GAP_IN         3       // front row sees no tape this long: the line ended / is lost

// Find perimeter (design 9.1)
#define FIND_SEARCH_IN      165     // field diagonal + 20%
#define FP_BACKUP_IN        4
#define FP_TURN_DEG         90
#define SQUARE_UP_DEG       45      // a side sensor hit the line first: turn this much toward it
#define CENTER_SEARCH_IN    10      // CenterOverTape gives up waiting for FC to cross after this
#define ARM_MAX_IN          26      // a line followed longer than this is the perimeter (arms are 17")
#define ALIGN_MAX_DEG       200     // AlignToLine safety (v5)

// Follow perimeter (design 9.2)
#define CORNER_CREEP_MAX_IN 4       // CornerCreep safety: the field-side sensor should clear the 2" tape well before
#define CORNER_MAX_DEG      150     // TurnCorner safety (FC lands after about 58 degrees)
#define LOST_MAX_IN         6
#define REALIGN_MAX_DEG     90

// Top level
#define START_DELAY_MS      3000    // time to set the robot down after reset

// Corner test: the battery voltage never stops the robot (EventCheckers.c, AD.c), and
// after a brownout reset it drives again right away (CornerTest.c).
#define NEVER_CUT_POWER
extern uint8_t Robot_WasBrownout;   // set in main.c: this start was a brownout reset

// Battery: 9.9 V pack (3S LiFe is about empty at 9.0 V)
#define BATTERY_LOW_MV      9000    // [TUNE for your pack]
#define BATTERY_ABSENT_MV   5000    // below this the board is on USB power only: never "low"

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

// Hardware setup: call from main after BOARD_Init(), before ES_Initialize()
void Robot_Init(void);

// Motion (design 6.1). Commands are -100..100; MIN_DUTY is applied inside.
void Robot_SetMotors(int8_t left, int8_t right);
void Robot_Stop(void);                          // dynamic brake (EN high, IN1 = IN2)
void Robot_Coast(void);                         // EN low, motors free
void Robot_Forward(int8_t speed);
void Robot_Reverse(int8_t speed);
void Robot_PivotLeft(int8_t speed);             // absolute directions
void Robot_PivotRight(int8_t speed);
void Robot_SwingRight(int8_t speed);            // right wheel braked, left wheel forward
void Robot_PivotTowardField(int8_t speed);
void Robot_PivotAwayFromField(int8_t speed);

// Edge following (top of this file). Call on every TAPE_CHANGED while following.
void Robot_EdgeStep(uint8_t mask, int8_t speed);
void Robot_ResetEdge(void);                     // forget the last edge readings (after a turn)
int8_t Robot_LastFrontEdge(void);               // +1 last seen over the tape, -1 on the wood, 0 on the edge

// Field side (design 6.2): TRUE = clockwise, field on our right
void Robot_SetFieldOnRight(uint8_t onRight);
uint8_t Robot_FieldOnRight(void);

// Raw inputs
uint16_t Robot_BatteryMv(void);

// Top-level state number on LED bank 3 (TapeService uses banks 1 and 2)
void Robot_ShowState(uint8_t state);

// Tape-pattern helpers (design 2.3, 7.1, v17)
uint8_t FrontCount(uint8_t m);
uint8_t AnyTape(uint8_t m);
uint8_t FieldSideOn(uint8_t m);                 // the front field-side sensor sees tape: a new edge ahead (corner)
uint8_t FrontSeesLine(uint8_t m);               // FC or the front outer-side sensor sees tape: still on our line

// Timers. Use these instead of ES_Timer_* in the state machines: a timer's
// ES_TIMEOUT can already be queued when its state exits, and the next state
// that reuses the timer must not take it as its own (design 5.4).
void RobotTimer_Start(uint8_t timer, uint32_t ms);
void RobotTimer_Stop(uint8_t timer);
uint8_t RobotTimer_Expired(ES_Event ThisEvent, uint8_t timer);
uint8_t RobotTimer_Running(uint8_t timer);

#endif /* ROBOT_H */
