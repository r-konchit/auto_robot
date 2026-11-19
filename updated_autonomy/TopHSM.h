/*
 * File: TopHSM.h
 *
 * Top-level service for the straight-line test build: runs StraightTest.c.
 */

#ifndef TOPHSM_H
#define TOPHSM_H


/*******************************************************************************
 * PUBLIC #INCLUDES                                                            *
 ******************************************************************************/

#include "ES_Configure.h"   // defines ES_EventTyp_t
#include "ES_Framework.h"   // defines ES_Event, INIT_EVENT, ENTRY_EVENT, and EXIT_EVENT

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

uint8_t InitTopHSM(uint8_t Priority);
uint8_t PostTopHSM(ES_Event ThisEvent);
ES_Event RunTopHSM(ES_Event ThisEvent);

#endif /* TOPHSM_H */
