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
#include "isr.h"
#include "isrHandler.h"

volatile uint16_t turretPos;
volatile uint16_t shoulderPos;
volatile uint16_t elbowPos;
volatile uint16_t forearmPos;
volatile uint16_t wristTiltPos;
volatile uint16_t wristSpinPos;

//CY_ISR_PROTO(CompRxISR);
CY_ISR(CompRxISR) {
    // clear interrupt on uart module
    UART_Computer_GetRxInterruptSource();

    // Clear the pending interrupt
    //NOTE: the interrupt for the UART will stay high so long as 
    //the FIFO buffer has data in it. 
    Comp_RX_ISR_ClearPending();
    
    // queue up computer rx event
    events |= COMP_RX_EVENT;
}

//CY_ISR_PROTO(TurretRxISR);
CY_ISR(TurretRxISR) {
    static enum states_e {low, high} state = low;
    
    UART_Turret_GetRxInterruptSource();
    TurretRxIsr_ClearPending();
    
    switch(state) {
        case low:
            turretPos = UART_Turret_GetByte() & 0xff;
            state = high;
        break;
        case high:
            turretPos |= (UART_Turret_GetByte() << 8) & 0xff00;
            state = low;
            events |= TURRET_POS_EVENT;
        break;
        default:
            state = low;
        break;
    }
}

//CY_ISR_PROTO(ShoulderRxISR);
CY_ISR(ShoulderRxISR) {
    static enum states_e {low, high} state = low;
    
    UART_Shoulder_GetRxInterruptSource();
    ShoulderRxIsr_ClearPending();
    
    switch(state) {
        case low:
            shoulderPos = UART_Shoulder_GetByte() & 0xff;
            state = high;
        break;
        case high:
            shoulderPos |= (UART_Shoulder_GetByte() << 8) & 0xff00;
            state = low;
            events |= SHOULDER_POS_EVENT;
        break;
        default:
            state = low;
        break;
    }
}

//CY_ISR_PROTO(ElbowRxISR);
CY_ISR(ElbowRxISR) {
	static enum states_e {low, high} state = low;
    
    UART_Elbow_GetRxInterruptSource();
    ElbowRxIsr_ClearPending();
    
    switch(state) {
        case low:
            elbowPos = UART_Elbow_GetByte() & 0xff;
            state = high;
        break;
        case high:
            elbowPos |= (UART_Elbow_GetByte() << 8) & 0xff00;
            state = low;
            events |= ELBOW_POS_EVENT;
        break;
        default:
            state = low;
        break;
    }
}

//CY_ISR_PROTO(ForearmRxISR);
CY_ISR(ForearmRxISR) {
	static enum states_e {low, high} state = low;
    
    UART_Forearm_GetRxInterruptSource();
    ForearmRxIsr_ClearPending();
    
    switch(state) {
        case low:
            forearmPos = UART_Forearm_GetByte() & 0xff;
            state = high;
        break;
        case high:
            forearmPos |= (UART_Forearm_GetByte() << 8) & 0xff00;
            state = low;
            events |= FOREARM_POS_EVENT;
        break;
        default:
            state = low;
        break;
    }
}

//CY_ISR_PROTO(WristRxISR);
/* CY_ISR(WristRxISR) {
    
} */

//CY_ISR_PROTO(HeartbeatISR);
CY_ISR(HeartbeatISR) {
    static unsigned count = 0;
    
    // clears interrupt on counter module
    PWM_Hand_ReadStatusRegister();
    heartbeatIsr_ClearPending();
    
    // Use count as a divider - only queue up event every 5 interrupts
    count++;
    if (count >= 5) {
        count = 0;
        // queue up heartbeat event
        events |= HEARTBEAT_EVENT;
    }
}

/* [] END OF FILE */
