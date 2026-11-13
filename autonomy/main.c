/*
 * File: main.c (local copy of TemplateES_Main.c)
 *
 * Perimeter test build: hardware setup, then the Events and Services framework.
 */

#include <BOARD.h>
#include <xc.h>
#include <stdio.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "Robot.h"

void main(void)
{
    ES_Return_t ErrorType;

    BOARD_Init();

    printf("Starting ECE 118 perimeter test\r\n");
    printf("using the 2nd Generation Events & Services Framework\r\n");

    // Why did the board (re)start? A reset in the middle of a run stops the
    // motors and starts the 3 s wait again. BOR = the supply dipped (motors pulling
    // the battery down), EXTR = reset button, POR = power switched on, WDTO = watchdog.
    printf("# reset cause: RCON=0x%04X%s%s%s%s%s\r\n", (unsigned) RCON,
            (RCON & _RCON_POR_MASK) ? " POR" : "",
            (RCON & _RCON_BOR_MASK) ? " BOR(brownout)" : "",
            (RCON & _RCON_EXTR_MASK) ? " EXTR(button)" : "",
            (RCON & _RCON_WDTO_MASK) ? " WDTO(watchdog)" : "",
            (RCON & _RCON_SWR_MASK) ? " SWR(software)" : "");
    // Brownout only (power-on sets BOR too, so exclude POR): drive again at once
    Robot_WasBrownout = ((RCON & _RCON_BOR_MASK) && !(RCON & _RCON_POR_MASK)) ? TRUE : FALSE;
    RCON = 0;

    // Your hardware initialization function calls go here
    Robot_Init();

    // now initialize the Events and Services Framework and start it running
    ErrorType = ES_Initialize();
    if (ErrorType == Success) {
        ErrorType = ES_Run();

    }
    //if we got to here, there was an error
    Robot_Coast();
    switch (ErrorType) {
    case FailedPointer:
        printf("Failed on NULL pointer");
        break;
    case FailedInit:
        printf("Failed Initialization");
        break;
    default:
        printf("Other Failure: %d", ErrorType);
        break;
    }
    for (;;)
        ;

};

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
