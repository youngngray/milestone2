/* ************************************************************************** */
/** WiFly Send/Recieve functions

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

#ifndef _WIFLY_H    /* Guard against multiple inclusion */
#define _WIFLY_H


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

#include "queue.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    void sendByte(unsigned char toSend, QueueHandle_t putQ);

    void sendMessage(unsigned char toSend[], unsigned int num, QueueHandle_t putQ);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
