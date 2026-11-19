/*
 * File: TestHarness.h
 *
 * Test harness for the perimeter test build. It runs as its own service and
 * prints a log on the serial port (115200 baud) while the robot drives:
 *
 *   S,<ms>,<machine>,<state>      a state machine changed state (TOP, FP, FW or MT)
 *   T,<ms>,<mask>,<FL FC FR BL BC BR CR>
 *                                 a tape sensor changed (mask in hex, then 1 = tape per sensor)
 *   M,<ms>,<left>,<right>         the motor command changed (-100..100 after MIN_DUTY; B = brake)
 *   P,<ms>,<top>,<fp>,<fw>,<FL FC FR BL BC BR CR>,<raw FL>,<raw FC>,<raw FR>,<raw BL>,<raw BC>,<raw BR>,<raw CR>,<left>,<right>,<mV>
 *                                 a snapshot every LOG_SNAPSHOT_MS
 *
 * Every line is comma separated, so the log can be pasted into a spreadsheet.
 *
 * Modes (set in the project's preprocessor macros, define at most one):
 *   (none)          normal run: the state machines drive the motors, everything is logged
 *   MOTOR_TEST      the state machines don't drive; the harness runs a fixed motor
 *                   sequence (forward at BASE and CREEP speed, pivot left and right) so
 *                   MS_PER_INCH_BASE, MS_PER_INCH_CREEP and MS_PER_DEG can be measured
 *   TAPE_CHECK_ONLY the robot never drives (TapeService prints its wiring check)
 *   HARNESS_QUIET   normal run with only the state lines (no T, M, P lines)
 */

#ifndef TESTHARNESS_H
#define TESTHARNESS_H

#include "ES_Configure.h"
#include "ES_Framework.h"

#define LOG_SNAPSHOT_MS 100     // P line period; 100 ms is about 800 bytes/s at 115200 baud

uint8_t InitTestHarness(uint8_t Priority);
uint8_t PostTestHarness(ES_Event ThisEvent);
ES_Event RunTestHarness(ES_Event ThisEvent);

// Called by the other modules
void Harness_LogState(const char *machine, const char *state);  // every state change
void Harness_LogTape(uint8_t mask);                             // every TAPE_CHANGED posted
void Harness_LogMotors(int8_t left, int8_t right, uint8_t brake); // every motor command

#endif /* TESTHARNESS_H */
