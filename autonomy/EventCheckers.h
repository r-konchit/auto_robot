/*
 * File: EventCheckers.h
 *
 * Event checker for the perimeter test build (design 4.4):
 *   CheckBattery - BATTERY_LOW once, after the battery reads low for 1 s
 *
 * Remember that EventCheckers should only return TRUE when an event has occured,
 * and that the event is a TRANSITION between two detectable differences.
 */

#ifndef EVENTCHECKERS_H
#define	EVENTCHECKERS_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT
#include "BOARD.h"

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

uint8_t CheckBattery(void);

#endif	/* EVENTCHECKERS_H */
