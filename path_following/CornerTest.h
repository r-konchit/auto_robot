/*
 * File: CornerTest.h
 *
 * Line following with corners. Set the robot on a straight part of the line
 * (FC and BC on the tape), reset, and it follows the line with the same
 * straight-line correction as StraightTest, notices a 90 degree corner in
 * either direction, turns onto the next line and keeps going.
 *
 * TopHSM.c hands it every event.
 */

#ifndef CORNERTEST_H
#define CORNERTEST_H

#include "ES_Configure.h"
#include "ES_Framework.h"

ES_Event RunCornerTest(ES_Event ThisEvent);

#endif /* CORNERTEST_H */
