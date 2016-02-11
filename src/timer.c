/* ************************************************************************** */
/** The local timer callback

  @Company
 Team 5

  @File Name
    timer.c

  @Summary
 Contains the callback for the timer.

  @Description
 This is the callback for the timer. It causes the timer to send the correct
 value onto the queue.
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

#include "system_config.h"
#include "system_definitions.h"
#include "timers.h"
#include "lead_rover.h"


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

// *****************************************************************************

//Number of times the timer has completed
unsigned int ticks = 0;

/** 
  @Function
    void vTimerCallback( TimerHandle_t pxTimer )

  @Summary
 Sends the elapsed ms to the queue.

  @Remarks
 None.
 */
void vTimerCallback( TimerHandle_t pxTimer ) {
    ticks++;
    //Send the number of ms elapsed. One tick is 50ms.
    app1SendTimerValToMsgQ(ticks * 50);
}


/* *****************************************************************************
 End of File
 */
