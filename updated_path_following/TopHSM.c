/*
 * File: TopHSM.c
 *
 * Top-level service for the corner test build. It only hands every event
 * (ES_INIT, TAPE_CHANGED, timer timeouts) to the corner test in
 * CornerTest.c, which does all the work.
 */

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "TopHSM.h"
#include "CornerTest.h"

static uint8_t MyPriority;

/**
 * @Function InitTopHSM(uint8_t Priority)
 * @brief Called by the framework at start-up; posts ES_INIT to itself. */
uint8_t InitTopHSM(uint8_t Priority)
{
    MyPriority = Priority;
    return ES_PostToService(MyPriority, INIT_EVENT);
}

/**
 * @Function PostTopHSM(ES_Event ThisEvent)
 * @brief TapeService posts TAPE_CHANGED here; timers 2-4 post their timeouts here. */
uint8_t PostTopHSM(ES_Event ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

/**
 * @Function RunTopHSM(ES_Event ThisEvent)
 * @brief Runs the corner test (CornerTest.c). */
ES_Event RunTopHSM(ES_Event ThisEvent)
{
    return RunCornerTest(ThisEvent);
}
