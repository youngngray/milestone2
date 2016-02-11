/* ************************************************************************** */
/** Debug Interface

  @File Name
    debug.c

  @Summary
 Functions for debugging purposes.

  @Description
 Create an interface for debugging.
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

#include "debug.h"

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
    void outputValue(unsigned char value) 

  @Summary
 Write 'value' to output debug ports.

  @Remarks
 Output pins used are BOARD pins 30-37
 */
void outputValue(unsigned char value)
{
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_E,  value);
}

void outputBuffer(unsigned char buffer[], unsigned int num_chars)
{
    unsigned int i = 0;
    for(i = 0; i < num_chars; i++)
    {
        outputValue(buffer[i]);
    }
}

/** 
  @Function
    void stopEverything( void ) 

  @Summary
 Stops everything and outputs an ascii "!"
 Also turns on a board LED LD4

  @Remarks
 Output pins used are BOARD pins 30-37
 */
void stopEverything( void )
{
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_E,  0x21);
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_A,  0x08);
    vTaskSuspend(NULL);
    for(;;){}
}


/* *****************************************************************************
 End of File
 */
