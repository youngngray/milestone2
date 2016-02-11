/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _LEAD_ROVER_PUBLIC_H    /* Guard against multiple inclusion */
#define _LEAD_ROVER_PUBLIC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
    
    //*******************************************************************
    // PUBLIC DATA
    //*******************************************************************
    
    typedef struct{
    
        unsigned char command;

    } LEAD_ROVER_MESSAGE;
    
    LEAD_ROVER_MESSAGE lead_roverMessage;
    
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
        int app1SendTimerValToMsgQ(unsigned int millisecondsElapsed) 

      @Summary
     Puts a value on the queue.

      @Description
     This function is used by outside methods to put a value on the queue of 
     the app. When it is called, the integer is put onto the back of the 
     queue and exits with 0 if it was successful, or 1 if the queue is full.

      @Precondition
     None.

      @Parameters
        @param millisecondsElapsed The integer to be put on the queue

      @Returns
     0 for success, 1 for failure

      @Remarks
     None.

      @Example
        @code
        app1SendTimerValToMsgQ(100);
     */
    int app1SendTimerValToMsgQ(unsigned int millisecondsElapsed);
    /*
    unsigned char readTransmitQ();
    
    unsigned char transmitQNotEmpty();
    
    void writeCommandQ(unsigned char towrite);
    
    void writeCommandQFromISR(unsigned char towrite);
    
    void writeMessageBufferFromISR(unsigned char write);*/
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _LEAD_ROVER_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
