/* ************************************************************************** */
/** WiFly Implementation

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */

#include "wifly.h"

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

// *****************************************************************************

/** 
  @Function
    int ExampleInterfaceFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Remarks
    Refer to the example_file.h interface header for function usage details.
 */
void sendByte(unsigned char toSend, QueueHandle_t putQ)
{
    xQueueSendToBack( putQ, &toSend, portMAX_DELAY );
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
}

void sendMessage(unsigned char toSend[], unsigned int num, QueueHandle_t putQ)
{
    unsigned int i = 0;
    for(i = 0; i < num; i++)
    {
        sendByte(toSend[i], putQ);
    }
}


/* *****************************************************************************
 End of File
 */
