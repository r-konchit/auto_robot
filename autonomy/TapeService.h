/*
 * File: TapeService.h
 *
 * Simple service that reads the 6 tape sensors every 8 ms, decides for each
 * one whether it sees tape (two thresholds, with hysteresis), posts
 * TAPE_CHANGED (param = tape mask) to the top-level HSM when any decision
 * changes, and shows every decision on LED banks 1 (front) and 2 (back).
 *
 * Mask bits: bit0 FL, bit1 FC, bit2 FR, bit4 BL, bit5 BC, bit6 BR.
 *
 * With TAPE_CHECK_ONLY defined it prints what every sensor sees about every
 * 250 ms, to check the wiring.
 */

#ifndef TAPESERVICE_H
#define TAPESERVICE_H

/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT
#include "ES_Framework.h"

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

uint8_t InitTapeService(uint8_t Priority);
uint8_t PostTapeService(ES_Event ThisEvent);
ES_Event RunTapeService(ES_Event ThisEvent);

// Latest confirmed mask (the same value the last TAPE_CHANGED carried)
uint8_t Tape_GetMask(void);

// Last raw reading of one sensor, ADC counts (0 FL, 1 FC, 2 FR, 3 BL, 4 BC, 5 BR)
unsigned int Tape_GetReading(uint8_t sensor);

// Stop showing the sensors on LED banks 1 and 2 (the corner test flashes them instead)
void Tape_FreezeLeds(void);

#endif /* TAPESERVICE_H */
