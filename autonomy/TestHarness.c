/*
 * File: TestHarness.c
 *
 * Logs states, tape sensors and motor commands on the serial port, and (with
 * MOTOR_TEST) drives a fixed motor sequence for measuring the speed constants.
 * See TestHarness.h for the log format.
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "Robot.h"
#include "TapeService.h"
#include "TestHarness.h"
#include <stdio.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

#ifdef MOTOR_TEST
// One step of the motor test: run these motor commands for this long.
// brake = TRUE: brake instead (left/right ignored).
typedef struct {
    uint16_t ms;
    int8_t left;
    int8_t right;
    uint8_t brake;
    const char *name;
} MotorStep;

// Mark the floor at the robot center before each forward step, measure after it:
//   MS_PER_INCH_BASE  = 2000 / inches driven in "Forward_BASE_2s"
//   MS_PER_INCH_CREEP = 4000 / inches driven in "Forward_CREEP_4s"
//   MS_PER_DEG        = 3000 / degrees turned in "PivotLeft_3s" (and "PivotRight_3s")
static const MotorStep steps[] = {
    { 3000, 0, 0, TRUE, "SetDown_3s" },
    { 2000, BASE_SPEED, BASE_SPEED, FALSE, "Forward_BASE_2s" },
    { 2000, 0, 0, TRUE, "Measure_2s" },
    { 4000, CREEP_SPEED, CREEP_SPEED, FALSE, "Forward_CREEP_4s" },
    { 2000, 0, 0, TRUE, "Measure_2s" },
    { 3000, -PIVOT_SPEED, PIVOT_SPEED, FALSE, "PivotLeft_3s" },
    { 2000, 0, 0, TRUE, "Measure_2s" },
    { 3000, PIVOT_SPEED, -PIVOT_SPEED, FALSE, "PivotRight_3s" },
    { 0, 0, 0, TRUE, "Done" },          // ms = 0: stay here
};
#define NUM_STEPS (sizeof(steps) / sizeof(steps[0]))
#endif

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/

static void PrintSnapshot(void);
static void SensorBits(uint8_t mask, char *out);
#ifdef MOTOR_TEST
static void StartStep(uint8_t i);
#endif

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/

static uint8_t MyPriority;

// Current state of each machine, for the snapshot line
static const char *topState = "-";
static const char *fpState = "-";
static const char *fwState = "-";

// Last motor command logged
static int8_t lastLeft = 0;
static int8_t lastRight = 0;
static uint8_t lastBrake = TRUE;

#ifdef MOTOR_TEST
static uint8_t step = 0;
static uint32_t stepStart = 0;
#endif

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitTestHarness(uint8_t Priority)
 * @brief Prints the log header and starts the snapshot timer. */
uint8_t InitTestHarness(uint8_t Priority)
{
    ES_Event ThisEvent;

    MyPriority = Priority;
    printf("\r\n# perimeter_test harness. Lines: S=state T=tape M=motors P=snapshot");
    printf("\r\n# P,ms,top,fp,fw,FL FC FR BL BC BR,rawFL,rawFC,rawFR,rawBL,rawBC,rawBR,left,right,mV");
#ifdef MOTOR_TEST
    printf("\r\n# MOTOR_TEST: the state machines won't drive. Mark the floor before each step.");
    StartStep(0);
#endif
    ES_Timer_InitTimer(HARNESS_TIMER, LOG_SNAPSHOT_MS);

    ThisEvent.EventType = ES_INIT;
    return ES_PostToService(MyPriority, ThisEvent);
}

uint8_t PostTestHarness(ES_Event ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunTestHarness(ES_Event ThisEvent)
 * @brief Every LOG_SNAPSHOT_MS: advance the motor test (MOTOR_TEST) and print a snapshot. */
ES_Event RunTestHarness(ES_Event ThisEvent)
{
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT;

    if ((ThisEvent.EventType != ES_TIMEOUT) || (ThisEvent.EventParam != HARNESS_TIMER)) {
        return ReturnEvent;
    }
#ifdef MOTOR_TEST
    if ((steps[step].ms != 0) && ((ES_Timer_GetTime() - stepStart) >= steps[step].ms)) {
        StartStep(step + 1);
    }
#endif
#ifndef HARNESS_QUIET
    PrintSnapshot();
#endif
    ES_Timer_InitTimer(HARNESS_TIMER, LOG_SNAPSHOT_MS);
    return ReturnEvent;
}

void Harness_LogState(const char *machine, const char *state)
{
    // A new top-level state restarts its sub-machine, which logs its own first state
    if (machine[0] == 'T') {
        topState = state;
        fpState = "-";
        fwState = "-";
    } else if (machine[1] == 'P') {
        fpState = state;
    } else if (machine[1] == 'W') {
        fwState = state;
    }
    printf("\r\nS,%lu,%s,%s", (unsigned long) ES_Timer_GetTime(), machine, state);
}

void Harness_LogTape(uint8_t mask)
{
#ifndef HARNESS_QUIET
    char bits[16];

    SensorBits(mask, bits);
    printf("\r\nT,%lu,%02X,%s", (unsigned long) ES_Timer_GetTime(), mask, bits);
#endif
}

void Harness_LogMotors(int8_t left, int8_t right, uint8_t brake)
{
    if (brake) {
        left = 0;
        right = 0;
    }
    if ((left == lastLeft) && (right == lastRight) && (brake == lastBrake)) {
        return;                         // only log changes
    }
    lastLeft = left;
    lastRight = right;
    lastBrake = brake;
#ifndef HARNESS_QUIET
    if (brake) {
        printf("\r\nM,%lu,B,B", (unsigned long) ES_Timer_GetTime());
    } else {
        printf("\r\nM,%lu,%d,%d", (unsigned long) ES_Timer_GetTime(), left, right);
    }
#endif
}

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

static void PrintSnapshot(void)
{
    char bits[16];

    SensorBits(Tape_GetMask(), bits);
    printf("\r\nP,%lu,%s,%s,%s,%s,%u,%u,%u,%u,%u,%u,%u,%d,%d,%u",
            (unsigned long) ES_Timer_GetTime(), topState, fpState, fwState, bits,
            Tape_GetReading(0), Tape_GetReading(1), Tape_GetReading(2),
            Tape_GetReading(3), Tape_GetReading(4), Tape_GetReading(5), Tape_GetReading(6),
            lastLeft, lastRight, Robot_BatteryMv());
}

// "1 0 0 0 1 0 0": one digit per sensor, in the order FL FC FR BL BC BR CR
static void SensorBits(uint8_t mask, char *out)
{
    const uint8_t bit[7] = { FL_BIT, FC_BIT, FR_BIT, BL_BIT, BC_BIT, BR_BIT, CR_BIT };
    uint8_t i;

    for (i = 0; i < 7; i++) {
        out[2 * i] = ON(mask, bit[i]) ? '1' : '0';
        out[2 * i + 1] = ' ';
    }
    out[13] = '\0';
}

#ifdef MOTOR_TEST
static void StartStep(uint8_t i)
{
    if (i >= NUM_STEPS) {
        i = NUM_STEPS - 1;
    }
    step = i;
    stepStart = ES_Timer_GetTime();
    if (steps[i].brake) {
        Robot_Stop();
    } else {
        Robot_SetMotors(steps[i].left, steps[i].right);
    }
    Harness_LogState("MT", steps[i].name);
}
#endif
