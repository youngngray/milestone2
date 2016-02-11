/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    messaging_task.c

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

#include "messaging_task.h"
#include "messaging_task_public.h"
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

MESSAGING_TASK_DATA msg_taskData;

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
void sendMsgToWIFLY(unsigned char message[], unsigned int num)
{
    int i;
    for(i = 0; i < num; i++)//i < sizeof(message)/sizeof(unsigned char); i++)
    {
        sendByteToWIFLY(message[i]);
    }
}

void sendByteToWIFLY(unsigned char byte)
{
    xQueueSendToBack(msg_taskData.sendMsg_q, &byte, portMAX_DELAY);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
}

void ReceiveUSARTMsgFromMsgQ(unsigned char usartMsg)
{
    xQueueSendToBackFromISR(msg_taskData.receiveMsg_q, &usartMsg, NULL);
    //stopEverything();
}
/* TODO:  Add any necessary local functions.
*/
char isQueueEmpty()
{
    if(pdFALSE == xQueueIsQueueEmptyFromISR(msg_taskData.sendMsg_q))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

unsigned char messageQ()
{
    unsigned char data = 0x0;
    BaseType_t errors;
    errors = xQueueReceiveFromISR(msg_taskData.sendMsg_q, &data, NULL);
    if(errors != pdTRUE)
    {
        stopEverything();
    }
    return data;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MESSAGING_TASK_Initialize ( void )

  Remarks:
    See prototype in messaging_task.h.
 */

void MESSAGING_TASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    msg_taskData.state = MESSAGING_TASK_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    msg_taskData.sendMsg_q = xQueueCreate(50, sizeof(unsigned char));
    if(msg_taskData.sendMsg_q == 0)
    {
        stopEverything();
    }
    msg_taskData.receiveMsg_q = xQueueCreate(50, sizeof(unsigned char));
    if(msg_taskData.sendMsg_q == 0)
    {
        stopEverything();
    }
    //stopEverything();
    /* Initialization is done, allow the state machine to continue */
    msg_taskData.state = MESSAGING_TASK_STATE_RUN;
    
#ifdef MACRO_DEBUG
      debugChar(0x09);      
#endif
      //stopEverything();
}


/******************************************************************************
  Function:
    void MESSAGING_TASK_Tasks ( void )

  Remarks:
    See prototype in messaging_task.h.
 */

void MESSAGING_TASK_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( msg_taskData.state )
    {
        /* Application's initial state. */
        case MESSAGING_TASK_STATE_INIT:
        {
            break;
        }

        /* TODO: implement your application state machine.*/
        case MESSAGING_TASK_STATE_RUN:
        {
            //stopEverything();
#ifdef MACRO_DEBUG
      debugChar(0x07);      
#endif
            unsigned char temp;
            xQueueReceive(msg_taskData.receiveMsg_q, &temp, portMAX_DELAY);
#ifdef MACRO_DEBUG
      debugBuffer(0x08,5);      
#endif
      writeCommandQ(temp);
            //sendByteToWIFLY(temp);  
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
