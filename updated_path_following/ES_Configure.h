/****************************************************************************
 Module
     ES_Configure.h
 Description
     This file contains macro definitions that are edited by the user to
     adapt the Events and Services framework to the perimeter test build.
 Notes
     Local copy: it must be found before C:\ECE118\include (include path
     ".." first in the MPLAB X project) and be the first #include in every
     module.

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 10:03 jec      started coding
 11/08/26       team     perimeter test: find + follow only
 11/10/26       team     v17: CROSS_SEEN removed (Find perimeter tells lines apart by length)
 11/10/26       team     v18: TestHarness service (logging, MOTOR_TEST)
 *****************************************************************************/

#ifndef CONFIGURE_H
#define CONFIGURE_H

//defines for keyboard input
//#define USE_KEYBOARD_INPUT
//What State machine are we testing
//#define POSTFUNCTION_FOR_KEYBOARD_INPUT PostTopHSM

//define for TattleTale (off for floor runs: TopHSM and the subs print each state instead)
//#define USE_TATTLETALE

//uncomment to supress the entry and exit events
//#define SUPPRESS_EXIT_ENTRY_IN_TATTLE

// Test harnesses (define at most one, in the project's preprocessor macros):
//   SIMPLESERVICE_TEST  - only the bumper and tape services run; they print their own events

/****************************************************************************/
// Name/define the events of interest
// Universal events occupy the lowest entries, followed by user-defined events

/****************************************************************************/
typedef enum {
    ES_NO_EVENT, ES_ERROR, /* used to indicate an error from the service */
    ES_INIT, /* used to transition from initial pseudo-state */
    ES_ENTRY, /* used to enter a state*/
    ES_EXIT, /* used to exit a state*/
    ES_KEYINPUT, /* used to signify a key has been pressed*/
    ES_LISTEVENTS, /* used to list events in keyboard input, does not get posted to fsm*/
    ES_TIMEOUT, /* signals that the timer has expired */
    ES_TIMERACTIVE, /* signals that a timer has become active */
    ES_TIMERSTOPPED, /* signals that a timer has stopped*/
    /* User-defined events start here */
    TAPE_CHANGED, /* 10  param = tape mask (bit 3 unused: no MC sensor) */
    BATTERY_LOW, /* 11  param = mV */
    PERIMETER_FOUND, /* 12  returned by Find perimeter */
    LINE_LOST, /* 13  returned by Follow perimeter */
    /* User-defined events end here */
    NUMBEROFEVENTS,
} ES_EventTyp_t;

static const char *EventNames[] = {
	"ES_NO_EVENT",
	"ES_ERROR",
	"ES_INIT",
	"ES_ENTRY",
	"ES_EXIT",
	"ES_KEYINPUT",
	"ES_LISTEVENTS",
	"ES_TIMEOUT",
	"ES_TIMERACTIVE",
	"ES_TIMERSTOPPED",
	"TAPE_CHANGED",
	"BATTERY_LOW",
	"PERIMETER_FOUND",
	"LINE_LOST",
	"NUMBEROFEVENTS",
};




/****************************************************************************/
// This are the name of the Event checking function header file.
#define EVENT_CHECK_HEADER "EventCheckers.h"

/****************************************************************************/
// This is the list of event checking functions (list order is priority)
#define EVENT_CHECK_LIST CheckBattery

/****************************************************************************/
// These are the definitions for the post functions to be executed when the
// corresponding timer expires. All 16 must be defined. If you are not using
// a timers, then you can use TIMER_UNUSED
// Every timer posts only to its owner.
#define TIMER_UNUSED ((pPostFunc)0)
#define TIMER0_RESP_FUNC TIMER_UNUSED
#define TIMER1_RESP_FUNC PostTapeService
#define TIMER2_RESP_FUNC PostTopHSM      /* top level: start delay */
#define TIMER3_RESP_FUNC PostTopHSM      /* Find perimeter */
#define TIMER4_RESP_FUNC PostTopHSM      /* Follow perimeter */
#define TIMER5_RESP_FUNC PostTestHarness /* harness snapshots, motor test */
#define TIMER6_RESP_FUNC TIMER_UNUSED
#define TIMER7_RESP_FUNC TIMER_UNUSED
#define TIMER8_RESP_FUNC TIMER_UNUSED
#define TIMER9_RESP_FUNC TIMER_UNUSED
#define TIMER10_RESP_FUNC TIMER_UNUSED
#define TIMER11_RESP_FUNC TIMER_UNUSED
#define TIMER12_RESP_FUNC TIMER_UNUSED
#define TIMER13_RESP_FUNC TIMER_UNUSED
#define TIMER14_RESP_FUNC TIMER_UNUSED
#define TIMER15_RESP_FUNC TIMER_UNUSED


/****************************************************************************/
// Give the timer numbers symbolc names to make it easier to move them
// to different timers if the need arises. Keep these definitons close to the
// definitions for the response functions to make it easire to check that
// the timer number matches where the timer event will be routed

#define TAPE_TIMER        1
#define TOP_TIMER         2
#define FIND_PERIM_TIMER  3
#define FOLLOW_TIMER      4
#define HARNESS_TIMER     5


/****************************************************************************/
// The maximum number of services sets an upper bound on the number of
// services that the framework will handle. Reasonable values are 8 and 16
// HOWEVER: at this time only a value of 8 is supported.
#define MAX_NUM_SERVICES 8

/****************************************************************************/
// This macro determines that nuber of services that are *actually* used in
// a particular application. It will vary in value from 1 to MAX_NUM_SERVICES
// Service 0 (keyboard input) is always present, so 3 user services make 4.
#define NUM_SERVICES 4

/****************************************************************************/
// These are the definitions for Service 0, the lowest priority service
// every Events and Services application must have a Service 0. Further
// services are added in numeric sequence (1,2,3,...) with increasing
// priorities
// the header file with the public function prototypes
#define SERV_0_HEADER "ES_KeyboardInput.h"
// the name of the Init function
#define SERV_0_INIT InitKeyboardInput
// the name of the run function
#define SERV_0_RUN RunKeyboardInput
// How big should this service's Queue be?
#define SERV_0_QUEUE_SIZE 9

/****************************************************************************/
// These are the definitions for Service 1 (lowest user priority: only logs)
#if NUM_SERVICES > 1
#define SERV_1_HEADER "TestHarness.h"
#define SERV_1_INIT InitTestHarness
#define SERV_1_RUN RunTestHarness
#define SERV_1_QUEUE_SIZE 4
#endif

// These are the definitions for Service 2
#if NUM_SERVICES > 2
#define SERV_2_HEADER "TapeService.h"
#define SERV_2_INIT InitTapeService
#define SERV_2_RUN RunTapeService
#define SERV_2_QUEUE_SIZE 4
#endif

// These are the definitions for Service 3 (highest priority: the state machines)
#if NUM_SERVICES > 3
#define SERV_3_HEADER "TopHSM.h"
#define SERV_3_INIT InitTopHSM
#define SERV_3_RUN RunTopHSM
// Tape changes come in bursts and every timer start/stop also posts an event here.
#define SERV_3_QUEUE_SIZE 16
#endif

/****************************************************************************/
// These are the definitions for Service 4
#if NUM_SERVICES > 4
// the header file with the public function prototypes
#define SERV_4_HEADER "TestService.h"
// the name of the Init function
#define SERV_4_INIT TestServiceInit
// the name of the run function
#define SERV_4_RUN TestServiceRun
// How big should this services Queue be?
#define SERV_4_QUEUE_SIZE 3
#endif

/****************************************************************************/
// These are the definitions for Service 5
#if NUM_SERVICES > 5
// the header file with the public function prototypes
#define SERV_5_HEADER "TestService.h"
// the name of the Init function
#define SERV_5_INIT TestServiceInit
// the name of the run function
#define SERV_5_RUN TestServiceRun
// How big should this services Queue be?
#define SERV_5_QUEUE_SIZE 3
#endif

/****************************************************************************/
// These are the definitions for Service 6
#if NUM_SERVICES > 6
// the header file with the public function prototypes
#define SERV_6_HEADER "TestService.h"
// the name of the Init function
#define SERV_6_INIT TestServiceInit
// the name of the run function
#define SERV_6_RUN TestServiceRun
// How big should this services Queue be?
#define SERV_6_QUEUE_SIZE 3
#endif

/****************************************************************************/
// These are the definitions for Service 7
#if NUM_SERVICES > 7
// the header file with the public function prototypes
#define SERV_7_HEADER "TestService.h"
// the name of the Init function
#define SERV_7_INIT TestServiceInit
// the name of the run function
#define SERV_7_RUN TestServiceRun
// How big should this services Queue be?
#define SERV_7_QUEUE_SIZE 3
#endif

/****************************************************************************/
// the name of the posting function that you want executed when a new
// keystroke is detected.
// The default initialization distributes keystrokes to all state machines
#define POST_KEY_FUNC ES_PostAll



/****************************************************************************/
// These are the definitions for the Distribution lists. Each definition
// should be a comma seperated list of post functions to indicate which
// services are on that distribution list.
#define NUM_DIST_LISTS 0
#if NUM_DIST_LISTS > 0
#define DIST_LIST0 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 1
#define DIST_LIST1 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 2
#define DIST_LIST2 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 3
#define DIST_LIST3 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 4
#define DIST_LIST4 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 5
#define DIST_LIST5 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 6
#define DIST_LIST6 PostTemplateFSM
#endif
#if NUM_DIST_LISTS > 7
#define DIST_LIST7 PostTemplateFSM
#endif



#endif /* CONFIGURE_H */
