/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
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
volatile int16_t temperature;
volatile int16_t humidity;

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

//CY_ISR_PROTO(ScienceRxISR);
CY_ISR(ScienceRxISR) {
    UART_ScienceMCU_ReadRxStatus(); // clear interrupt
    ScienceRxIsr_ClearPending();
    
    // Get feedback from science sensors: temperature and humidity
    enum states_e { templo, temphi, humlo, humhi };
    static enum states_e state = templo;
    
    // Get next byte from UART
    int8_t byte;
    byte = 0xff & UART_ScienceMCU_GetByte();
    
    static volatile int16_t temp; // temporary data storage
    switch (state) {
    // Temperature low byte
    case templo:
        temp = byte;
        state = temphi;
        break;
    // Temperature high byte
    case temphi:
        temp |= (byte << 8);
        temperature = temp; // now assign temperature to this value
        state = humlo;
        break;
    // Humidity low byte
    case humlo:
        temp = byte;
        state = humhi;
        break;
    // Humidity high byte
    case humhi:
        temp |= (byte << 8);
        humidity = temp;
        state = templo;
        break;
    // Shouldn't ever get here
    default:
        break;
    }
}

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
