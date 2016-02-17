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
#include "isrHandler.h"
#include "pololuControl.h"

volatile uint32_t events = 0;

// Arm payload struct
// These are the last positions we wrote to the motors
static struct ArmPayload {
    uint16_t turretDest;
    uint16_t shoulderDest;
    uint16_t elbowDest;
    uint16_t forearmDest;
    uint16_t wristTiltDest;
    uint16_t wristSpinDest;
    uint16_t handDest; // really just open/close
} ArmPayload;

// Arm positions
// These are the most recent positions received as feedback from the motors
// We really don't need feedback from the hand since it's just open/close.
extern volatile uint16_t turretPos;
extern volatile uint16_t shoulderPos;
extern volatile uint16_t elbowPos;
extern volatile uint16_t forearmPos;
extern volatile uint16_t wristTiltPos;
extern volatile uint16_t wristSpinPos;

#define POSITION_PAYLOAD_SIZE (12) // 2 bytes per joint, 6 joints
// The positions are stored in little endian format - low byte first, then
// high byte, in order from joints closest to the rover outward
// [turretlo, turrethi, shoulderlo, shoulderhi, elbowlo, elbowhi,
// forearmlo, forearmhi, wristspinlo, wristspinhi, wristtiltlo, wristtilthi]
static uint8_t positionArray[POSITION_PAYLOAD_SIZE];

// State machine states to receive commands from computer
// The state machine is defined in the function compRxEventHandler
#define PREAMBLE0 0xEA
#define PREAMBLE1 0xE3
static enum compRxStates_e { pre0, pre1, turretlo, turrethi, shoulderlo, 
    shoulderhi, elbowlo, elbowhi, forearmlo, forearmhi,
    wristtiltlo, wristtilthi, wristspinlo, wristspinhi, hand } compRxState;

// Receive a message from the computer
int compRxEventHandler() {
    // get next element in uart rx buffer
    static uint16_t data;
    static uint8_t byte;
    
    // Keep reading the rx buffer until empty.
    // GetRxBufferSize gets the number of bytes in the software buffer,
    // but not the hardware FIFO (which is 4 bytes in size), so we also want
    // to call ReadRxStatus and check if the RX_STS_FIFO_NOTEMPTY bit was set.
    while (UART_Computer_GetRxBufferSize() || 
          (UART_Computer_ReadRxStatus() & UART_Computer_RX_STS_FIFO_NOTEMPTY))
    {
        // MSB contains status, LSB contains data; if status is nonzero, an 
        // error has occurred
        data = UART_Computer_GetByte();
        
        // check status
        // TODO: modify to actually return the status byte
        // use this in the main function
        if (data & 0xff00) {
            //LED0_Write(!LED0_Read());
            return UART_READ_ERROR;
        }
        
        // mask the data to a single byte*
        byte = data & 0xff;
        
        // state machine
        switch(compRxState) {
        case pre0:
            if (byte == PREAMBLE0) {
                compRxState = pre1; // change state
            }
            break;
        case pre1:
            if (byte == PREAMBLE1) {
                compRxState = turretlo; // change state
            }
            else {
                compRxState = pre0; // change state
                return MESSAGE_ERROR;
            }
            break;
        case turretlo:
            ArmPayload.turretDest = byte;
            compRxState = turrethi; // change state
            break;
        case turrethi:
            ArmPayload.turretDest |= byte << 8;
            pololuControl_driveMotor(ArmPayload.turretDest,
                POLOLUCONTROL_TURRET);
            compRxState = shoulderlo; // change state
            break;
        case shoulderlo:
            ArmPayload.shoulderDest = byte;
            compRxState = shoulderhi; // change state
            break;
        case shoulderhi:
            ArmPayload.shoulderDest |= byte << 8;
            pololuControl_driveMotor(ArmPayload.shoulderDest, 
                POLOLUCONTROL_SHOULDER);
            compRxState = elbowlo; // change state
            break;
        case elbowlo:
            ArmPayload.elbowDest = byte;
            compRxState = elbowhi; // change state
            break;
        case elbowhi:
            ArmPayload.elbowDest |= byte << 8;
            pololuControl_driveMotor(ArmPayload.elbowDest,
                POLOLUCONTROL_ELBOW);
            compRxState = forearmlo; // change state
            break;
        case forearmlo:
            ArmPayload.forearmDest = byte;
            compRxState = forearmhi;
            break;
        case forearmhi:
            ArmPayload.forearmDest |= byte << 8;
            pololuControl_driveMotor(ArmPayload.forearmDest,
                POLOLUCONTROL_FOREARM);
            compRxState = wristtiltlo; // change state
            break;
            // TODO: call dynamixel commands
        case wristtiltlo:
            ArmPayload.wristTiltDest = byte;
            compRxState = wristtilthi; // change state
            break;
        case wristtilthi:
            ArmPayload.wristTiltDest |= byte << 8;
            // TODO: call dynamixel command
            compRxState = wristspinlo; // change state
            break;
        case wristspinlo:
            ArmPayload.wristSpinDest = byte;
            compRxState = wristspinhi; // change state
            break;
        case wristspinhi:
            ArmPayload.wristSpinDest |= byte << 8;
            // TODO: call dynamixel command
            compRxState = pre0;
            break;
            /*
        case hand:
            TODO: get the byte - open or close - and translate to PWM compare.
            break;
            */
        default:
            // shouldn't get here!!!
            break;
        }
    }
    
    // Check if any data came in that we didn't get. If so, then queue up
    // this event again in the events variable.
    if (UART_Computer_GetRxBufferSize() || 
        UART_Computer_ReadRxStatus() & UART_Computer_RX_STS_FIFO_NOTEMPTY) 
    {
        events |= COMP_RX_EVENT;
    }
    return SUCCESS; // success
}

// Ask the motor controller boards for feedback.
void heartbeatEventHandler() {
    // Turret
    //pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
	//	POLOLUCONTROL_TURRET);
    
    // Shoulder
    //pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
	//	POLOLUCONTROL_SHOULDER);
    
    // Elbow
    //pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
	//	POLOLUCONTROL_ELBOW);
    
    // Forearm
    //pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
	//	POLOLUCONTROL_FOREARM);
	
    // Wrist tilt
    // TODO: call dynamixelReadPosition() for wrist tilt
    
    // Wrist spin
    // TODO: call dynamixelReadPosition() for wrist spin
}

// Report received positional feedback to the computer.
void reportPositionEvent() {
    static int i = 0;
    i++;
    turretPos += i;
    shoulderPos += 2*i;
    elbowPos += 3*i;
    forearmPos += 4*i;
    // Send positions to computer
    positionArray[0] =  (turretPos & 0xff);
    positionArray[1] = ((turretPos >> 8) & 0xff);
    positionArray[2] =  (shoulderPos & 0xff);
    positionArray[3] = ((shoulderPos >> 8) & 0xff);
    positionArray[4] =  (elbowPos & 0xff);
    positionArray[5] = ((elbowPos  >> 8) & 0xff);
    positionArray[6] =  (forearmPos & 0xff);
    positionArray[7] = ((forearmPos >> 8) & 0xff);
	positionArray[8] = ((wristTiltPos & 0xff));
	positionArray[9] = ((wristTiltPos >> 8) & 0xff);
	positionArray[10] =((wristSpinPos & 0xff));
	positionArray[11] =((wristSpinPos >> 8) & 0xff);
	UART_Computer_PutArray(positionArray, POSITION_PAYLOAD_SIZE);
}

/* [] END OF FILE */
