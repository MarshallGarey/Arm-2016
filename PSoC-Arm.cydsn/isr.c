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

extern volatile struct ArmPosition {
    uint16_t turretPos;
    uint16_t shoulderPos;
    uint16_t elbowPos;
    uint16_t forearmPos;
    uint16_t wristSpinPos;
    uint16_t wristTiltPos;
} ArmPosition;

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
            ArmPosition.shoulderPos = UART_Shoulder_GetByte();
            state = high;
        break;
        case high:
            ArmPosition.shoulderPos |= UART_Shoulder_GetByte() << 8;
            state = low;
            events |= SHOULDER_POS_EVENT;
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
            ArmPosition.shoulderPos = UART_Shoulder_GetByte();
            state = high;
        break;
        case high:
            ArmPosition.shoulderPos |= UART_Shoulder_GetByte() << 8;
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
            ArmPosition.elbowPos = UART_Elbow_GetByte();
            state = high;
        break;
        case high:
            ArmPosition.elbowPos |= UART_Elbow_GetByte() << 8;
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
            ArmPosition.forearmPos = UART_Forearm_GetByte();
            state = high;
        break;
        case high:
            ArmPosition.forearmPos |= UART_Forearm_GetByte() << 8;
            state = low;
            events |= FOREARM_POS_EVENT;
        break;
        default:
            state = low;
        break;
    }
}

//CY_ISR_PROTO(WristTiltRxISR);

//CY_ISR_PROTO_WristSpinRxISR);

//CY_ISR_PROTO(HeartbeatISR);
CY_ISR(HeartbeatISR) {
    // clears interrupt on counter module
    HeartbeatCounter_GetInterruptSource();
    Heartbeat_ISR_ClearPending();
    
    // queue up heartbeat event
    events |= HEARTBEAT_EVENT;
}

/* [] END OF FILE */
