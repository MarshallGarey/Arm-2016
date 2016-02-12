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
// Dest is the last position we wrote to the motor
// Pos is the most recent positional feedback from the motor
static struct ArmPayload {
    uint16_t turretDest;
    uint16_t shoulderDest;
    uint16_t elbowDest;
    uint16_t forearmDest;
    uint16_t wristTiltDest;
    uint16_t wristSpinDest;
} ArmPayload;

volatile struct ArmPosition {
    uint16_t turretPos;
    uint16_t shoulderPos;
    uint16_t elbowPos;
    uint16_t forearmPos;
    uint16_t wristTiltPos;
    uint16_t wristSpinPos;
} ArmPosition;

#define POSITION_PAYLOAD_SIZE (12) // 2 bytes per joint, 6 joints
// turretlo, turrethi, shoulderlo, shoulderhi, elbowlo, elbowhi,
// forearmlo, forearmhi, wristspinlo, wristspinhi, wristtiltlo, wristtilthi
static uint8_t positionArray[POSITION_PAYLOAD_SIZE];

// uart0 - comm with computer state machine:
#define PREAMBLE0 0xEA
#define PREAMBLE1 0xE3
static enum compRxStates_e { pre0, pre1, turretlo, turrethi, shoulderlo, 
    shoulderhi, elbowlo, elbowhi, forearmlo, forearmhi,
    wristtiltlo, wristtilthi, wristspinlo, wristspinhi } compRxState;

// Receive a message from the computer
int compRxEventHandler() {
    // get next element in uart rx buffer
    static uint16_t data;
    static uint8_t byte;
    
    // Keep reading rx buffer until empty
    while (UART_Computer_GetRxBufferSize()) {
        // MSB contains status, LSB contains data; if status is nonzero, an 
        // error has occurred
        data = (uint16_t) UART_Computer_GetByte();
        
        // check status
        // TODO: modify to actually return the status byte
        // use this in the main function
        if (data & 0xff00) {
            LED0_Write(!LED0_Read());
            return UART_READ_ERROR;
        }
        
        // mask the data to a single byte
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
            compRxState = pre0; // change state
            break;
        case elbowlo:
            
            break;
        case elbowhi:
            
            break;
        case forearmlo:
            
            break;
        case forearmhi:
            
            break;
        case wristtiltlo:
            
            break;
        case wristtilthi:
            // TODO: ask dynamixel for feedback after sending position
            break;
        case wristspinlo:
            
            break;
        case wristspinhi:
            // TODO: ask dynamixel for feedback after sending position
            break;
        default:
            // shouldn't get here!!!
            break;
        }
    }
    return SUCCESS; // success
}

// Get feedback from motors and send the data to the computer
// TODO: Get rid of this event and ask the pololu for feedback immediately
// after I send it a position in the computer Rx event handler.
void heartbeatEventHandler() {
    // Report positional feedback to computer
    //pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
	//	POLOLUCONTROL_TURRET);
    
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_SHOULDER);
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_ELBOW);
    
    //pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
	//	POLOLUCONTROL_FOREARM);
	
    // TODO: call dynamixelReadPosition() for wrist rotate and tilt
	
    //UART_Computer_WriteTxData('H');
    //UART_Computer_WriteTxData('I');
    //UART_Computer_WriteTxData('\n');
}

void reportPositionEvent() {
    // Send positions to computer
    positionArray[0] = (ArmPosition.turretPos & 0xff);
    positionArray[1] = ((ArmPosition.turretPos >> 8) & 0xff);
    positionArray[2] = (ArmPosition.shoulderPos & 0xff);
    positionArray[3] = ((ArmPosition.shoulderPos >> 8) & 0xff);
    positionArray[4] = (ArmPosition.elbowPos & 0xff);
    positionArray[5] = ((ArmPosition.elbowPos  >> 8) & 0xff);
    positionArray[6] = (ArmPosition.forearmPos & 0xff);
    positionArray[7] = ((ArmPosition.forearmPos >> 8) & 0xff);
	positionArray[8] = ((ArmPosition.wristTiltPos & 0xff));
	positionArray[9] = ((ArmPosition.wristTiltPos >> 8) & 0xff);
	positionArray[10] = ((ArmPosition.wristSpinPos & 0xff));
	positionArray[11] = ((ArmPosition.wristSpinPos >> 8) & 0xff);
	UART_Computer_PutArray(positionArray, POSITION_PAYLOAD_SIZE);
    
}

/* [] END OF FILE */
