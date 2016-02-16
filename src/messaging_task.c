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
MESSAGE_FORMAT msg_Format;
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
    debugChar(0xC0);
    int i;
    for(i = 0; i < num; i++)//i < sizeof(message)/sizeof(unsigned char); i++)
    {
        sendByteToWIFLY(message[i]);
    }
}

void sendByteToWIFLY(unsigned char byte)
{
    debugChar(0xC1);
    xQueueSendToBack(msg_taskData.sendMsg_q, &byte, portMAX_DELAY);
    debugChar(0xC2);
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
    msg_Format.count = 0;
    msg_Format.validHeader = 0;
    msg_Format.validFooter = 0;
    msg_Format.numInvalid = 0;
    
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
      if((temp == 0x81) && (msg_Format.count == 0))
            {
//                sendByteToWIFLY(temp);
                debugChar(0xD1);
                //sendByteToWIFLY(msg_Format.header);
                msg_Format.validHeader = 5;
                msg_Format.header = temp;
                msg_Format.count++;
            }
            else if ((temp != 0x81) && (msg_Format.count == 0))
            {
                //Invalid Message
//                sendByteToWIFLY(temp);
                debugChar(0xF3);
                msg_Format.numInvalid++;
                //sendMsgToWIFLY("NO\t");
                sendByteToWIFLY(msg_Format.numInvalid);
                //sendByteToWIFLY('\n');            
                msg_Format.validHeader = 0;
                msg_Format.validFooter = 0;
                msg_Format.count = 0;
            }
            if((msg_Format.validHeader == 5) && (msg_Format.count != 0))
            {
                debugChar(0xEE);
//                sendByteToWIFLY(temp);
                if(msg_Format.count == 2){
                    debugChar(0xD2);
//                    sendByteToWIFLY(temp);
                    msg_Format.dst = temp;
                }
                else if(msg_Format.count == 3) {
                    debugChar(0xD3);
//                    sendByteToWIFLY(temp);
                    msg_Format.type = temp;
                }
                else if(msg_Format.count == 4) {
                    debugChar(0xD4);
//                    sendByteToWIFLY(temp);
                    msg_Format.msgNum1 = temp;
                }
                else if(msg_Format.count == 5) {
                    debugChar(0xD5);
//                    sendByteToWIFLY(temp);
                    msg_Format.msgNum2 = temp;
                }
                else if(msg_Format.count == 6) {
                    debugChar(0xD6);
//                    sendByteToWIFLY(temp);
                    msg_Format.data1 = temp;
                }
                else if(msg_Format.count == 7) {
                    debugChar(0xD7);
//                    sendByteToWIFLY(temp);
                    msg_Format.data2 = temp;
                }
                else if(msg_Format.count == 8) {
                    debugChar(0xD8);
//                    sendByteToWIFLY(temp);
                    msg_Format.data3 = temp;
                }
                else if(msg_Format.count == 9) {
                    debugChar(0xD9);
//                    sendByteToWIFLY(temp);
                    msg_Format.data4 = temp;
                    
                }
                msg_Format.count++;
            }
            if((msg_Format.validHeader == 5)&&(temp == 0x88)&&(msg_Format.count == 11))
            {
                debugChar(0xE0);
//                sendByteToWIFLY(temp);
                msg_Format.footer = temp;
                msg_Format.validFooter = 5;
                msg_Format.count = 0;
            }
            else if((msg_Format.count == 11) && (msg_Format.validHeader == 5) && (temp != 0x88))
            {
                debugChar(0xF1);
                //sendByteToWIFLY(0xF1);
                msg_Format.numInvalid++;              
                //sendMsgToWIFLY("NO\t");
                sendByteToWIFLY(msg_Format.numInvalid);
                //sendByteToWIFLY('\n');
                msg_Format.validHeader = 0;
                msg_Format.valid = 0;
                msg_Format.validFooter = 0;
                msg_Format.count = 0;
            }
            if((msg_Format.validHeader == 5) && (msg_Format.validFooter == 5))
            {
                //sendByteToWIFLY(0xE1);
                msg_Format.valid = 1;
                msg_Format.count = 0;
            }
      
          if(msg_Format.valid == 1)
          {
                //sendByteToWIFLY(0xE1);
              //debugChar(msg_Format.dst);
              msg_Format.count = 0;
              msg_taskData.state = MESSAGING_TASK_STATE_READ;
          }
            //debugChar(temp);
            //sendByteToWIFLY(temp);  
            break;
        }
        case MESSAGING_TASK_STATE_READ:
        {
            
            debugChar(msg_Format.header);
            debugChar(msg_Format.dst);
            debugChar(msg_Format.type);
            debugChar(msg_Format.msgNum1);
            debugChar(msg_Format.msgNum2);
            debugChar(msg_Format.data1);
            debugChar(msg_Format.data2);
            debugChar(msg_Format.data3);
            debugChar(msg_Format.data4);
            debugChar(msg_Format.footer);
            /*
            sendByteToWIFLY(msg_Format.header);
            sendByteToWIFLY(msg_Format.dst);
            sendByteToWIFLY(msg_Format.type);
            sendByteToWIFLY(msg_Format.msgNum1);
            sendByteToWIFLY(msg_Format.msgNum2);
            sendByteToWIFLY(msg_Format.data1);
            sendByteToWIFLY(msg_Format.data2);
            sendByteToWIFLY(msg_Format.data3);
            sendByteToWIFLY(msg_Format.data4);
            sendByteToWIFLY(msg_Format.footer);*/
            
            if(msg_Format.type == 0x07)
            {
                writeCommandQ(msg_Format.data1);
            }
            
            msg_Format.count = 0;
            msg_Format.validHeader = 0;
            msg_Format.validFooter = 0;
            msg_Format.valid = 0;
            msg_taskData.state = MESSAGING_TASK_STATE_RUN;
            break;
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
