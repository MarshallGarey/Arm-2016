/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#ifndef __ISR_HANDLER_H__ 
#define __ISR_HANDLER_H__ 

#include <project.h>

int compRxEventHandler();
void heartbeatEventHandler();
void reportPositionEvent();

// event variables
extern volatile uint32_t events;

#define SERVO_NEUTRAL 1500
#define SERVO_MAX 2000
#define SERVO_MIN 1000

// events
#define COMP_RX_EVENT 0x0001
#define HEARTBEAT_EVENT 0x0002

// positional feedback events
#define TURRET_POS_EVENT 0x0400
#define SHOULDER_POS_EVENT 0x0800
#define ELBOW_POS_EVENT 0x1000
#define FOREARM_POS_EVENT 0x2000
#define WRIST_TILT_POS_EVENT 0x4000
#define WRIST_SPIN_POS_EVENT 0x8000

// positional feedback event group - just an or of all the position events
#define POS_EVENT_GROUP (TURRET_POS_EVENT | SHOULDER_POS_EVENT | \
                        ELBOW_POS_EVENT | FOREARM_POS_EVENT) /* | \
                        WRIST_TILT_POS_EVENT | WRIST_SPIN_POS_EVENT)*/

// general macros
#define SUCCESS 0
#define UART_READ_ERROR 1
#define MESSAGE_ERROR 2

#endif
/* [] END OF FILE */
