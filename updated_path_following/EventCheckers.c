/*
 * File:   EventCheckers.c
 *
 * Event checker for the perimeter test build (from TemplateEventChecker.c).
 */

/*******************************************************************************
 * MODULE #INCLUDE                                                             *
 ******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "EventCheckers.h"
#include "ES_Events.h"
#include "Robot.h"
#include "TopHSM.h"

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

#define BATTERY_LOW_HOLD_MS 1000

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/

static uint8_t batteryLowPosted = FALSE;
static uint8_t batteryWasLow = FALSE;
static uint32_t batteryLowSince = 0;

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function CheckBattery(void)
 * @return TRUE or FALSE
 * @brief Posts BATTERY_LOW (param = mV) once, after the battery has read below
 *        BATTERY_LOW_MV for 1 s. Readings below BATTERY_ABSENT_MV mean the board
 *        is running from USB with no battery, which is not "low". */
uint8_t CheckBattery(void)
{
    ES_Event thisEvent;
    uint16_t mv;
    uint8_t low;

#ifdef NEVER_CUT_POWER
    return FALSE;   // never react to the battery voltage
#endif
    if (batteryLowPosted) {
        return FALSE;
    }
    mv = Robot_BatteryMv();
    low = (mv > BATTERY_ABSENT_MV) && (mv < BATTERY_LOW_MV);
    if (low && !batteryWasLow) {
        batteryLowSince = ES_Timer_GetTime();
    }
    batteryWasLow = low;
    if (low && ((ES_Timer_GetTime() - batteryLowSince) >= BATTERY_LOW_HOLD_MS)) {
        batteryLowPosted = TRUE;
        thisEvent.EventType = BATTERY_LOW;
        thisEvent.EventParam = mv;
        PostTopHSM(thisEvent);
        return TRUE;
    }
    return FALSE;
}
