/* ************************************************************************** */
/** Debug Header

  @File Name
    debug.h

  @Summary
 The debug header.

  @Description
 This declares functions to be used for debugging. More will be added as used.
 */
/* ************************************************************************** */

#ifndef _DEBUG_H    /* Guard against multiple inclusion */
#define _DEBUG_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    /**
      @Function
        void outputValue(unsigned char value)

      @Summary
     Sends a value to the debug output

      @Description
     Sets the debug output lines high and low to represent the value that
     was given to the function. 

      @Precondition
     None.

      @Parameters
        @param value The value for the output.

      @Returns
     None.

      @Remarks
     None.

      @Example
        @code
        outputValue(34);
     */
    void outputValue(unsigned char value);
    
    void outputBuffer(unsigned char buffer[], unsigned int num_chars);
    
    /**
      @Function
        void stopEverything( void )

      @Summary
     Sends an "!" to debug and turns on an LED.

      @Description
     Sends an "!" to debug and turns on an LED.

      @Precondition
     None.

      @Parameters
     None

      @Returns
     None.

      @Remarks
     None.

      @Example
        @code
        stopEverything();
     */
    void stopEverything( void );

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DEBUG_H */

/* *****************************************************************************
 End of File
 */
