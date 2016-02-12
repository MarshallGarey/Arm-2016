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

// events
#define COMP_RX_EVENT 0x0001
#define HEARTBEAT_EVENT 0x0002

// positional feedback event group
//#define POS_EVENT_GROUP 0xfc00
// events in the group
#define TURRET_POS_EVENT 0x0400
#define SHOULDER_POS_EVENT 0x0800
#define ELBOW_POS_EVENT 0x1000
#define FOREARM_POS_EVENT 0x2000
#define WRIST_SPIN_POS_EVENT 0x4000
#define WRIST_TILT_POS_EVENT 0x8000

#define POS_EVENT_GROUP 0x1800

// general macros
#define SUCCESS 0
#define UART_READ_ERROR 1
#define MESSAGE_ERROR 2

#endif
/* [] END OF FILE */
