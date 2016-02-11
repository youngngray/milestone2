/* 
 * File:   messaging_task_public.h
 * Author: Andrew Wang
 *
 * Created on February 10, 2016, 5:20 PM
 */

#ifndef MESSAGING_TASK_PUBLIC_H
#define	MESSAGING_TASK_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif
    void sendMsgToWIFLY(unsigned char message[], unsigned int num);
    void sendByteToWIFLY(unsigned char byte);
    void ReceiveUSARTMsgFromMsgQ(unsigned char usartMsg);

#ifdef	__cplusplus
}
#endif

#endif	/* MESSAGING_TASK_PUBLIC_H */

