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

#ifndef _DEBUGGING_TASK_PUBLIC_H    /* Guard against multiple inclusion */
#define _DEBUGGING_TASK_PUBLIC_H


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

    void debugChar(unsigned char toSend);
    void debugCharFromISR(unsigned char toSend);
    void debugBuffer(unsigned char buffer[], unsigned int num);
    void stopEverything( void );
    
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DEBUGGING_TASK_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
