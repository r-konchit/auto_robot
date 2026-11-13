/*
 * File: Robot.c
 *
 * Hardware layer for the perimeter test build. See Robot.h for the overview.
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include <xc.h>
#include "BOARD.h"
#include "AD.h"
#include "pwm.h"
#include "IO_Ports.h"
#include "LED.h"
#include "Robot.h"
#include "TestHarness.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

#define LEFT  0
#define RIGHT 1

#define NUM_ES_TIMERS 16

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/

static void SetMotor(uint8_t motor, int16_t cmd);
static int16_t ApplyMinDuty(int16_t cmd);
static int8_t EdgePosition(uint8_t center, uint8_t outer, int8_t last);

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/

uint8_t Robot_WasBrownout = FALSE;      // set in main.c (Robot.h)

static uint8_t fieldOnRight = FALSE;    // v5: the robot follows counterclockwise

// Last edge reading under each row: +1 over the tape, 0 on the edge, -1 on the wood
static int8_t lastFront = 0;
static int8_t lastBack = 0;

// RobotTimer bookkeeping
static uint32_t timerStart[NUM_ES_TIMERS];
static uint32_t timerLength[NUM_ES_TIMERS];

/*******************************************************************************
 * PUBLIC FUNCTIONS: setup                                                     *
 ******************************************************************************/

void Robot_Init(void)
{
    // BOARD_Init() has already started the AD system (battery monitor)
    AD_AddPins(TAPE_ALL_PINS);

    PWM_Init();
    PWM_AddPins(LEFT_EN_PWM | RIGHT_EN_PWM);

    IO_PortsSetPortOutputs(PORTY, PORTY_OUTPUT_PINS);   // IN1-4
#ifdef CR_ON_Y11
    CR_Y11_TRIS = 1;                                    // corner sensor DO on Y-11 (input)
#endif

    // LED library: TapeService adds banks 1 and 2, the state display uses bank 3
    LED_Init();
    LED_AddBanks(LED_BANK3);
    LED_SetBank(LED_BANK3, 0);

    Robot_Coast();
}

/*******************************************************************************
 * PUBLIC FUNCTIONS: motion (design 6)                                         *
 ******************************************************************************/

void Robot_SetMotors(int8_t left, int8_t right)
{
    int16_t l = ApplyMinDuty(left);
    int16_t r = ApplyMinDuty(right);

    SetMotor(LEFT, l * LEFT_DIR);       // LEFT_DIR/RIGHT_DIR fix the wiring (Robot.h)
    SetMotor(RIGHT, r * RIGHT_DIR);     // trim is applied inside, on the PWM duty
    Harness_LogMotors((int8_t) l, (int8_t) r, FALSE);
}

void Robot_Stop(void)
{
    // Dynamic brake: EN high, IN1 = IN2 (L298N truth table)
    LEFT_IN1_LAT = 0;
    LEFT_IN2_LAT = 0;
    RIGHT_IN3_LAT = 0;
    RIGHT_IN4_LAT = 0;
    PWM_SetDutyCycle(LEFT_EN_PWM, MAX_PWM);
    PWM_SetDutyCycle(RIGHT_EN_PWM, MAX_PWM);
    Harness_LogMotors(0, 0, TRUE);
}

void Robot_Coast(void)
{
    PWM_SetDutyCycle(LEFT_EN_PWM, 0);
    PWM_SetDutyCycle(RIGHT_EN_PWM, 0);
    Harness_LogMotors(0, 0, FALSE);
}

void Robot_Forward(int8_t speed)
{
    Robot_SetMotors(speed, speed);
}

void Robot_Reverse(int8_t speed)
{
    Robot_SetMotors(-speed, -speed);
}

void Robot_PivotLeft(int8_t speed)
{
    Robot_SetMotors(-speed, speed);
}

void Robot_PivotRight(int8_t speed)
{
    Robot_SetMotors(speed, -speed);
}

// Swing turn to the right: the right wheel is braked (locked), the left wheel drives
// forward, so the robot turns right about the right wheel.
void Robot_SwingRight(int8_t speed)
{
    int16_t l = ApplyMinDuty(speed);

    SetMotor(LEFT, l * LEFT_DIR);
    RIGHT_IN3_LAT = 0;                  // right motor: dynamic brake (EN high, IN3 = IN4)
    RIGHT_IN4_LAT = 0;
    PWM_SetDutyCycle(RIGHT_EN_PWM, MAX_PWM);
    Harness_LogMotors((int8_t) l, 0, FALSE);
}

void Robot_PivotTowardField(int8_t speed)
{
    if (fieldOnRight) {
        Robot_PivotRight(speed);
    } else {
        Robot_PivotLeft(speed);
    }
}

void Robot_PivotAwayFromField(int8_t speed)
{
    if (fieldOnRight) {
        Robot_PivotLeft(speed);
    } else {
        Robot_PivotRight(speed);
    }
}

// Follow the inner edge of the tape (Robot.h, top). Each row says where the
// edge is under it; the average says how far off the edge the robot is, the
// difference says how much it is angled.
void Robot_EdgeStep(uint8_t mask, int8_t speed)
{
    uint8_t frontOuter = fieldOnRight ? FL_BIT : FR_BIT;
    uint8_t backOuter = fieldOnRight ? BL_BIT : BR_BIT;
    int8_t front = EdgePosition(ON(mask, FC_BIT), ON(mask, frontOuter), lastFront);
    int8_t back = EdgePosition(ON(mask, BC_BIT), ON(mask, backOuter), lastBack);
    int16_t steer;      // percent, + = turn toward the field
    int16_t slow, fast;

    lastFront = front;
    lastBack = back;
    steer = (EDGE_KP * (front + back)) / 2 + EDGE_KH * (front - back);
    if (steer > EDGE_MAX) {
        steer = EDGE_MAX;
    } else if (steer < -EDGE_MAX) {
        steer = -EDGE_MAX;
    }
    slow = (speed * (100 - steer)) / 100;   // the wheel on the field side
    fast = (speed * (100 + steer)) / 100;
    if (fieldOnRight) {
        Robot_SetMotors((int8_t) fast, (int8_t) slow);
    } else {
        Robot_SetMotors((int8_t) slow, (int8_t) fast);
    }
}

void Robot_ResetEdge(void)
{
    lastFront = 0;
    lastBack = 0;
}

int8_t Robot_LastFrontEdge(void)
{
    return lastFront;
}

void Robot_SetFieldOnRight(uint8_t onRight)
{
    fieldOnRight = onRight ? TRUE : FALSE;
}

uint8_t Robot_FieldOnRight(void)
{
    return fieldOnRight;
}

/*******************************************************************************
 * PUBLIC FUNCTIONS: raw inputs, LEDs                                          *
 ******************************************************************************/

uint16_t Robot_BatteryMv(void)
{
    // 10:1 divider on AN1, 3.3 V full scale (design 3.6)
    uint32_t reading = AD_ReadADPin(BAT_VOLTAGE);
    if (reading > 1023) {
        return 0;                       // no reading yet
    }
    return (uint16_t) ((reading * 33000) / 1023);
}

void Robot_ShowState(uint8_t state)
{
    LED_SetBank(LED_BANK3, state & 0x0F);
}

/*******************************************************************************
 * PUBLIC FUNCTIONS: tape patterns (design 2.3, v16)                           *
 ******************************************************************************/

uint8_t FrontCount(uint8_t m)
{
    return ON(m, FL_BIT) + ON(m, FC_BIT) + ON(m, FR_BIT);
}

uint8_t AnyTape(uint8_t m)
{
    return (FrontCount(m) > 0) || ON(m, BL_BIT) || ON(m, BC_BIT) || ON(m, BR_BIT);
}

uint8_t FieldSideOn(uint8_t m)
{
    return ON(m, fieldOnRight ? FR_BIT : FL_BIT);
}

uint8_t FrontSeesLine(uint8_t m)
{
    return ON(m, FC_BIT) || ON(m, fieldOnRight ? FL_BIT : FR_BIT);
}

/*******************************************************************************
 * PUBLIC FUNCTIONS: timers                                                    *
 ******************************************************************************/

void RobotTimer_Start(uint8_t timer, uint32_t ms)
{
    if (ms == 0) {
        ms = 1;                         // the framework refuses a 0 ms timer
    }
    timerStart[timer] = ES_Timer_GetTime();
    timerLength[timer] = ms;
    ES_Timer_InitTimer(timer, ms);
}

void RobotTimer_Stop(uint8_t timer)
{
    ES_Timer_StopTimer(timer);
    timerLength[timer] = 0;
}

uint8_t RobotTimer_Running(uint8_t timer)
{
    return timerLength[timer] != 0;
}

uint8_t RobotTimer_Expired(ES_Event ThisEvent, uint8_t timer)
{
    if ((ThisEvent.EventType != ES_TIMEOUT) || (ThisEvent.EventParam != timer)) {
        return FALSE;
    }
    if (timerLength[timer] == 0) {
        return FALSE;                   // stopped since: stale
    }
    // A timeout left over from the previous user of this timer arrives too early
    if ((ES_Timer_GetTime() - timerStart[timer]) + 1 < timerLength[timer]) {
        return FALSE;
    }
    timerLength[timer] = 0;
    return TRUE;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

// One motor: PWM on EN, direction on IN1/IN2 (never equal while driving)
static void SetMotor(uint8_t motor, int16_t cmd)
{
    // Trim (Robot.h) on the 0..1000 duty, so 1% steps are exact at any speed
    uint32_t duty = (uint32_t) (cmd >= 0 ? cmd : -cmd) * (MAX_PWM / 100)
            * (uint32_t) (motor == LEFT ? LEFT_TRIM : RIGHT_TRIM) / 100;

    if (duty > MAX_PWM) {
        duty = MAX_PWM;
    }
    if (motor == LEFT) {
        LEFT_IN1_LAT = (cmd >= 0) ? 1 : 0;
        LEFT_IN2_LAT = (cmd >= 0) ? 0 : 1;
        PWM_SetDutyCycle(LEFT_EN_PWM, duty);
    } else {
        RIGHT_IN3_LAT = (cmd >= 0) ? 1 : 0;
        RIGHT_IN4_LAT = (cmd >= 0) ? 0 : 1;
        PWM_SetDutyCycle(RIGHT_EN_PWM, duty);
    }
}

// Clamp to -100..100 and lift any non-zero command to at least MIN_DUTY
static int16_t ApplyMinDuty(int16_t cmd)
{
    if (cmd > 100) {
        cmd = 100;
    } else if (cmd < -100) {
        cmd = -100;
    }
    if ((cmd > 0) && (cmd < MIN_DUTY)) {
        cmd = MIN_DUTY;
    } else if ((cmd < 0) && (cmd > -MIN_DUTY)) {
        cmd = -MIN_DUTY;
    }
    return cmd;
}

// Where the inner edge of the tape is under one row (Robot.h, top):
//   center sensor on tape, outer-side sensor on wood -> +1 (robot center over the tape)
//   both on                                          ->  0 (on the edge)
//   center sensor on wood, outer-side sensor on tape -> -1 (robot center on the wood)
//   neither                                          -> the last reading
static int8_t EdgePosition(uint8_t center, uint8_t outer, int8_t last)
{
    if (center && !outer) {
        return 1;
    }
    if (center && outer) {
        return 0;
    }
    if (outer) {
        return -1;
    }
    return last;
}
