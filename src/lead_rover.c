/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    lead_rover.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "lead_rover.h"
#include "lead_rover_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

LEAD_ROVER_DATA lead_roverData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/
void writeCommandQFromISR(unsigned char towrite)
{
    xQueueSendToBackFromISR(lead_roverData.receive_q, &towrite, NULL);
}

void writeCommandQ(unsigned char towrite)
{
    xQueueSendToBack(lead_roverData.receive_q, &towrite, portMAX_DELAY);
}

// *****************************************************************************
// *****************************************************************************
// Section: Rover movement functions
// *****************************************************************************
// *****************************************************************************
void forward(void)
{
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_D,  0x03 );
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_G,  0x00 );
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_C,  0x0000 );
}

void reverse(void)
{
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_D,  0x03 );
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_G,  0x02 );
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_C,  0x4000 );
}

void right(void)
{
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_D,  0x03 );
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_G,  0x00 );
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_C,  0x4000 );
}

void left(void)
{
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_D,  0x03 );
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_G,  0x02 );
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_C,  0x0000 );    
}

void stop(void)
{
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_D,  0x00 );
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void LEAD_ROVER_Initialize ( void )

  Remarks:
    See prototype in lead_rover.h.
 */

void LEAD_ROVER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    lead_roverData.state = LEAD_ROVER_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    lead_roverData.transmit_q = xQueueCreate(10, sizeof(unsigned char));
    if(lead_roverData.transmit_q == 0)
    {
        stopEverything();
    }
    
    lead_roverData.receive_q = xQueueCreate(25, sizeof(unsigned char));
    if(lead_roverData.receive_q == 0)
    {
        stopEverything();
    }
    
    lead_roverMessage.command = 0x00;
    
    stop();
    /* Initialization is done, allow the state machine to continue */
    //lead_roverData.state = LEAD_ROVER_STATE_SEND_REPLY;
    lead_roverData.state = LEAD_ROVER_STATE_RUN;

}

/******************************************************************************
  Function:
    void LEAD_ROVER_Tasks ( void )

  Remarks:
    See prototype in lead_rover.h.
 */
void LEAD_ROVER_Tasks ( void )
{
    
    /* Check the application's current state. */
    switch ( lead_roverData.state )
    {
        /* Application's initial state. */
        case LEAD_ROVER_STATE_INIT:
        {
            break;
        }

        /* TODO: implement your application state machine.*/
        case LEAD_ROVER_STATE_RUN:
        {
            unsigned char command;
            BaseType_t received = xQueueReceive(lead_roverData.receive_q,
                    &command, portMAX_DELAY);
            //sendByteToWIFLY(command);  
            //If not received, stop and turn on LED.
            if(received == pdFALSE)
            {
                stopEverything();
            }
            debugBuffer("Received Command: ", 18);
            switch(command)
            {
                case 'F':
                {
                    forward();
                    debugChar('F');
                    lead_roverData.state = LEAD_ROVER_STATE_SEND_REPLY;
                    break;
                }
                case 'S':
                {
                    stop();
                    debugChar('S');
                    lead_roverData.state = LEAD_ROVER_STATE_SEND_REPLY;
                    break;
                }
                case 'L':
                {
                    left();
                    debugChar('L');
                    lead_roverData.state = LEAD_ROVER_STATE_SEND_REPLY;
                    break;
                }
                case 'B':
                {
                    reverse();
                    debugChar('B');
                    lead_roverData.state = LEAD_ROVER_STATE_SEND_REPLY;
                    break;
                }
                case 'R':
                {
                    right();
                    debugChar('R');
                    lead_roverData.state = LEAD_ROVER_STATE_SEND_REPLY;
                    break;
                }
                default:
                {
                    break;
                }
            }
            //lead_roverData.state = LEAD_ROVER_STATE_CHECK_MESSAGE;
            break;
        }
        case LEAD_ROVER_STATE_SEND_REPLY:
        {
            vTaskDelay(400);
            unsigned char message[10] = {0x81, 'F', 0x01, 0, 0, 0, 0, 0, 0, 0x88};
            sendMsgToWIFLY(message, 10);
            debugChar(0xCC);
            lead_roverData.state = LEAD_ROVER_STATE_RUN;
        }
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 


/*******************************************************************************
 End of File
 */
