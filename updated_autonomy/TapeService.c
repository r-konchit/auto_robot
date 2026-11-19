/*
 * File: TapeService.c
 *
 * Reads the 6 tape sensors, decides for each one whether it sees tape, posts
 * TAPE_CHANGED when any decision changes, and shows every decision on an LED.
 *
 * Sensor signal (measured):
 *   Tape seen    (no reflection)  about 1.0 V = 310 counts
 *   No tape      (reflection)     about 0.5 V = 155 counts
 * The Uno32 ADC is 10-bit with a 3.3 V reference: counts = volts * 1023 / 3.3.
 *
 * Hysteresis, per sensor:
 *   reading above TAPE_ON_THRESH   -> tape seen
 *   reading below TAPE_OFF_THRESH  -> tape not seen
 *   reading in between             -> keep the previous decision
 * and "not seen" only counts after OFF_CONFIRM low readings in a row (short dropouts ignored).
 *
 * LEDs (the blue LED CONTROL jumpers must be installed). They show the same
 * decision the state machine gets, after hysteresis, not the raw reading.
 *   Bank 1, front sensors:  LED 1 = front left, LED 2 = front center, LED 3 = front right
 *   Bank 2, back sensors:   LED 1 = back left,  LED 2 = back center,  LED 3 = back right
 *   Bank 1 LED 4 = front-right corner sensor (CR). Bank 2 LED 4 stays off.
 * If the banks or left/right look swapped on the robot, change the LED #defines below.
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "AD.h"
#include "LED.h"
#include "Robot.h"
#include "TapeService.h"
#include "TopHSM.h"
#include "TestHarness.h"
#include "IO_Ports.h"
#include <xc.h>
#include <stdio.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

// Thresholds in ADC counts (shared by all sensors; see onThresh/offThresh to tune one sensor)
// v19: centered between tape (310) and wood (155) so a sensor that reads 15-20 % low
// still sees the tape. 280 left only 30 counts of margin: a weak sensor never saw tape.
#define TAPE_ON_THRESH   250    // about 0.81 V: above this, tape seen
#define TAPE_OFF_THRESH  215    // about 0.69 V: below this, tape not seen

// Which LED bank shows which row, and which bit is which LED
#define FRONT_LED_BANK   LED_BANK1
#define BACK_LED_BANK    LED_BANK2
#define LED_LEFT         0x01   // LED 1
#define LED_CENTER       0x02   // LED 2
#define LED_RIGHT        0x04   // LED 3
#define LED_CORNER       0x08   // LED 4 (bank 1 only): front-right corner sensor

#define TAPE_SAMPLE_MS   8      // time between readings
// A sensor must read below TAPE_OFF_THRESH this many readings in a row before it
// counts as off the tape (4 x 8 ms = about 30 ms). One bad reading, e.g. from a
// supply dip when the motors pull hard, is ignored. Going ON the tape is immediate.
#define OFF_CONFIRM      4
#define PRINT_EVERY      31     // readings between wiring-check prints (~250 ms)

// Sensor index, used for every array below
#define FRONT_LEFT       0
#define FRONT_CENTER     1
#define FRONT_RIGHT      2
#define BACK_LEFT        3
#define BACK_CENTER      4
#define BACK_RIGHT       5
#define CORNER_RIGHT     6      // front-right corner sensor (Robot.h, CR)

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/

static void ShowTapeLeds(void);

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/

static const unsigned int tapePin[NUM_TAPE_SENSORS] = {
    TAPE_FL_PIN, TAPE_FC_PIN, TAPE_FR_PIN, TAPE_BL_PIN, TAPE_BC_PIN, TAPE_BR_PIN, TAPE_CR_PIN
};

// Bit of each sensor in the TAPE_CHANGED mask (Robot.h)
static const uint8_t tapeBit[NUM_TAPE_SENSORS] = {
    FL_BIT, FC_BIT, FR_BIT, BL_BIT, BC_BIT, BR_BIT, CR_BIT
};

// Per-sensor thresholds. To retune one sensor, replace its entry with a number.
static const uint16_t onThresh[NUM_TAPE_SENSORS] = {
    TAPE_ON_THRESH, TAPE_ON_THRESH, TAPE_ON_THRESH, TAPE_ON_THRESH, TAPE_ON_THRESH, TAPE_ON_THRESH,
    TAPE_ON_THRESH
};
static const uint16_t offThresh[NUM_TAPE_SENSORS] = {
    TAPE_OFF_THRESH, TAPE_OFF_THRESH, TAPE_OFF_THRESH, TAPE_OFF_THRESH, TAPE_OFF_THRESH, TAPE_OFF_THRESH,
    TAPE_OFF_THRESH
};

static uint8_t tapeSeen[NUM_TAPE_SENSORS];  // saved decision per sensor
static uint8_t offCount[NUM_TAPE_SENSORS];  // readings in a row below the off threshold
static unsigned int reading[NUM_TAPE_SENSORS];  // last raw reading per sensor (ADC counts)
static uint8_t currentMask = 0;             // last mask posted
static uint8_t ledsFrozen = FALSE;          // Tape_FreezeLeds(): someone else owns the LEDs now

static uint8_t MyPriority;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function InitTapeService(uint8_t Priority)
 * @param Priority - internal variable to track which event queue to use
 * @return TRUE or FALSE
 * @brief Pins were set up by Robot_Init(), which also starts the LED library.
 *        Adds the two tape LED banks and starts the sample timer. */
uint8_t InitTapeService(uint8_t Priority)
{
    ES_Event ThisEvent;

    MyPriority = Priority;
    LED_AddBanks(FRONT_LED_BANK | BACK_LED_BANK);
    ShowTapeLeds();                 // all off until the first reading
    ES_Timer_InitTimer(TAPE_TIMER, TAPE_SAMPLE_MS);

    // post the initial transition event
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent) == TRUE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @Function PostTapeService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be posted to queue
 * @return TRUE or FALSE */
uint8_t PostTapeService(ES_Event ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunTapeService(ES_Event ThisEvent)
 * @param ThisEvent - the event (type and param) to be responded.
 * @return ES_NO_EVENT
 * @brief Reads every sensor on each TAPE_TIMER timeout. */
ES_Event RunTapeService(ES_Event ThisEvent)
{
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

    uint8_t i;
    uint8_t newMask = 0;
    ES_Event TapeEvent;

    if ((ThisEvent.EventType != ES_TIMEOUT) || (ThisEvent.EventParam != TAPE_TIMER)) {
        return ReturnEvent;     // ES_INIT, ES_TIMERACTIVE, ES_TIMERSTOPPED: nothing to do
    }

    // Threshold step, with hysteresis
    for (i = 0; i < NUM_TAPE_SENSORS; i++) {
#ifdef CR_ON_Y11
        if (i == CORNER_RIGHT) {
            // digital module on Y-11: turn its level into a full-scale "reading"
            reading[i] = (CR_Y11_BIT == CR_Y11_TAPE_LEVEL) ? 1000 : 0;
        } else {
            reading[i] = AD_ReadADPin(tapePin[i]);
        }
#else
        reading[i] = AD_ReadADPin(tapePin[i]);
#endif
        if (reading[i] > 1023) {
            // no reading yet: keep the previous decision
        } else if (reading[i] > onThresh[i]) {
            tapeSeen[i] = TRUE;
            offCount[i] = 0;
        } else if (reading[i] < offThresh[i]) {
            if (offCount[i] < OFF_CONFIRM) {
                offCount[i]++;
            }
            if (offCount[i] >= OFF_CONFIRM) {
                tapeSeen[i] = FALSE;    // off for OFF_CONFIRM readings in a row
            }
        } else {
            offCount[i] = 0;            // in between: keep the decision, restart the count
        }
        if (tapeSeen[i]) {
            newMask |= (1 << tapeBit[i]);
        }
    }

    if (!ledsFrozen) {
        ShowTapeLeds();
    }

    // Post only when at least one sensor's decision changed
    if (newMask != currentMask) {
        currentMask = newMask;
        TapeEvent.EventType = TAPE_CHANGED;
        TapeEvent.EventParam = currentMask;
        PostTopHSM(TapeEvent);
        Harness_LogTape(currentMask);
    }

#ifdef TAPE_CHECK_ONLY
    static uint8_t printCount = 0;
    if (++printCount >= PRINT_EVERY) {
        printCount = 0;
        printf("\r\nFL %s %4u  FC %s %4u  FR %s %4u  |  BL %s %4u  BC %s %4u  BR %s %4u  |  CR %s %4u  |  %u mV",
                tapeSeen[FRONT_LEFT] ? "TAPE " : "floor", reading[FRONT_LEFT],
                tapeSeen[FRONT_CENTER] ? "TAPE " : "floor", reading[FRONT_CENTER],
                tapeSeen[FRONT_RIGHT] ? "TAPE " : "floor", reading[FRONT_RIGHT],
                tapeSeen[BACK_LEFT] ? "TAPE " : "floor", reading[BACK_LEFT],
                tapeSeen[BACK_CENTER] ? "TAPE " : "floor", reading[BACK_CENTER],
                tapeSeen[BACK_RIGHT] ? "TAPE " : "floor", reading[BACK_RIGHT],
                tapeSeen[CORNER_RIGHT] ? "TAPE " : "floor", reading[CORNER_RIGHT],
                Robot_BatteryMv());
    }
#endif

    ES_Timer_InitTimer(TAPE_TIMER, TAPE_SAMPLE_MS);

    return ReturnEvent;
}

void Tape_FreezeLeds(void)
{
    ledsFrozen = TRUE;
}

uint8_t Tape_GetMask(void)
{
    return currentMask;
}

unsigned int Tape_GetReading(uint8_t sensor)
{
    return (sensor < NUM_TAPE_SENSORS) ? reading[sensor] : 0;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

// One LED per sensor: front row on one bank, back row on the other
static void ShowTapeLeds(void)
{
    uint8_t frontLeds = 0;
    uint8_t backLeds = 0;

    if (tapeSeen[FRONT_LEFT]) {
        frontLeds |= LED_LEFT;
    }
    if (tapeSeen[FRONT_CENTER]) {
        frontLeds |= LED_CENTER;
    }
    if (tapeSeen[FRONT_RIGHT]) {
        frontLeds |= LED_RIGHT;
    }
    if (tapeSeen[CORNER_RIGHT]) {
        frontLeds |= LED_CORNER;    // bank 1 LED 4: the front-right corner sensor
    }
    if (tapeSeen[BACK_LEFT]) {
        backLeds |= LED_LEFT;
    }
    if (tapeSeen[BACK_CENTER]) {
        backLeds |= LED_CENTER;
    }
    if (tapeSeen[BACK_RIGHT]) {
        backLeds |= LED_RIGHT;
    }

    LED_SetBank(FRONT_LED_BANK, frontLeds);
    LED_SetBank(BACK_LED_BANK, backLeds);
}
