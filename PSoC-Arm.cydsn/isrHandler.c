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
static struct Payload {
    uint16_t leftWheels;
    uint16_t rightWheels;
    uint16_t cameraPan;
    uint16_t cameraTilt;
    uint8_t cameraNum;
    uint8_t chutes;
    uint16_t turretDest;
    uint16_t shoulderDest;
    uint16_t elbowDest;
    uint16_t forearmDest;
    uint16_t wristTiltDest;
    uint16_t wristSpinDest;
    uint16_t handDest;
    uint16_t shovel;
} Payload;

// Arm positions
// These are the most recent positions received as feedback from the motors
// We really don't need feedback from the hand since it's just open/close.
extern volatile uint16_t turretPos;
extern volatile uint16_t shoulderPos;
extern volatile uint16_t elbowPos;
extern volatile uint16_t forearmPos;
extern volatile uint16_t wristTiltPos;
extern volatile uint16_t wristSpinPos;

#define POSITION_PAYLOAD_SIZE (13) // 1 start byte, 2 bytes per joint, 6 joints
// The positions are stored in little endian format - low byte first, then
// high byte, in order from joints closest to the rover outward
// [turretlo, turrethi, shoulderlo, shoulderhi, elbowlo, elbowhi,
// forearmlo, forearmhi, wristspinlo, wristspinhi, wristtiltlo, wristtilthi]
static uint8_t positionArray[POSITION_PAYLOAD_SIZE];

// State machine states to receive commands from computer
// The state machine is defined in the function compRxEventHandler
#define PREAMBLE0 0xEA
static enum compRxStates_e { pre0, leftlo, lefthi, rightlo, righthi, campanlo,
    campanhi, camtiltlo, camtilthi, cam1, cam2, turretlo, turrethi, shoulderlo, 
    shoulderhi, elbowlo, elbowhi, forearmlo, forearmhi,
    wristtiltlo, wristtilthi, wristspinlo, wristspinhi, 
    handlo, handhi, chutes, shovello, shovelhi, boxLid } compRxState;

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
                compRxState = leftlo; // change state
            }
            break;
        case leftlo:
            Payload.leftWheels = byte;
            compRxState = lefthi;
            break;
        case lefthi:
            Payload.leftWheels |= byte << 8;
            PWM_Drive_WriteCompare1(Payload.leftWheels);
            compRxState = rightlo;
            break;
        case rightlo:
            Payload.rightWheels = byte;
            compRxState = righthi;
            break;
        case righthi:
            Payload.rightWheels |= byte << 8;
            PWM_Drive_WriteCompare2(Payload.rightWheels);
            compRxState = campanlo;
            break;
        case campanlo:
            Payload.cameraPan = byte;
            compRxState = campanhi;
            break;
        case campanhi:
            Payload.cameraPan |= byte << 8;
            compRxState = camtiltlo;
            PWM_Gimbal_WriteCompare1(Payload.cameraPan);
            compRxState = camtiltlo;
            break;
        case camtiltlo:
            Payload.cameraTilt = byte;
            compRxState = camtilthi;
            break;
        case camtilthi:
            Payload.cameraTilt |= byte << 8;
            PWM_Gimbal_WriteCompare2(Payload.cameraTilt);
            compRxState = cam1;
            break;
        case cam1:
            switch(byte) {
            case 1:
                PWM_VideoMux2_WriteCompare(VIDEO1);
                break;
            case 2:
                PWM_VideoMux2_WriteCompare(VIDEO2);
                break;
            case 3:
                PWM_VideoMux2_WriteCompare(VIDEO3);
                break;
            default:
                PWM_VideoMux2_WriteCompare(VIDEO1);
                break;
            }
            break;
            compRxState = turretlo;
            /*
            compRxState = cam2;
            break;
        case cam2:
            switch(byte) {
            case 1:
                PWM_VideoMux_WriteCompare(VIDEO1);
                break;
            case 2:
                PWM_VideoMux_WriteCompare(VIDEO2);
                break;
            case 3:
                PWM_VideoMux_WriteCompare(VIDEO3);
                break;
            default:
                PWM_VideoMux_WriteCompare(VIDEO1);
                break;
            }
            compRxState = turretlo;
            */
            break;
        case turretlo:
            Payload.turretDest = byte;
            compRxState = turrethi; // change state
            break;
        case turrethi:
            Payload.turretDest |= byte << 8;
            pololuControl_driveMotor(Payload.turretDest,
                POLOLUCONTROL_TURRET);
            compRxState = shoulderlo; // change state
            break;
        case shoulderlo:
            Payload.shoulderDest = byte;
            compRxState = shoulderhi; // change state
            break;
        case shoulderhi:
            Payload.shoulderDest |= byte << 8;
            pololuControl_driveMotor(Payload.shoulderDest, 
                POLOLUCONTROL_SHOULDER);
            compRxState = elbowlo; // change state
            break;
        case elbowlo:
            Payload.elbowDest = byte;
            compRxState = elbowhi; // change state
            break;
        case elbowhi:
            Payload.elbowDest |= byte << 8;
            pololuControl_driveMotor(Payload.elbowDest,
                POLOLUCONTROL_ELBOW);
            compRxState = forearmlo; // change state
            break;
        case forearmlo:
            Payload.forearmDest = byte;
            compRxState = forearmhi;
            break;
        case forearmhi:
            Payload.forearmDest |= byte << 8;
            pololuControl_driveMotor(Payload.forearmDest,
                POLOLUCONTROL_FOREARM);
            compRxState = wristtiltlo; // change state
            break;
            // TODO: call dynamixel commands
        case wristtiltlo:
            Payload.wristTiltDest = byte;
            compRxState = wristtilthi; // change state
            break;
        case wristtilthi:
            Payload.wristTiltDest |= byte << 8;
            // TODO: call dynamixel command
            compRxState = wristspinlo; // change state
            break;
        case wristspinlo:
            Payload.wristSpinDest = byte;
            compRxState = wristspinhi; // change state
            break;
        case wristspinhi:
            Payload.wristSpinDest |= byte << 8;
            // TODO: call dynamixel command
            compRxState = handlo;
            break;
        case handlo:
            Payload.handDest = byte;
            compRxState = handhi;
            break;
        case handhi:
            Payload.handDest |= byte << 8;
            driveHand(Payload.handDest);
            compRxState = chutes;
            break;
        case chutes:
            // the chute values are packed into the first 6 bits of the byte
            chute1_Write(byte & 0x01);
            chute2_Write((byte >> 1) & 0x01);
            chute3_Write((byte >> 2) & 0x01);
            chute4_Write((byte >> 3) & 0x01);
            chute5_Write((byte >> 4) & 0x01);
            chute6_Write((byte >> 5) & 0x01);
            if ((byte >> 6) & 0x01) {
                PWM_BoxLid_WriteCompare(SERVO_MIN); // open box
            }
            else {
                PWM_BoxLid_WriteCompare(SERVO_MAX); // close box
            }
            compRxState = shovello;
            break;
        case shovello:
            Payload.shovel = byte;
            compRxState = shovelhi;
            break;
        case shovelhi:
            Payload.shovel |= (byte << 8);
            PWM_Hand_WriteCompare2(Payload.shovel);
            compRxState = pre0;
            break;
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

// Drives hand to correct position: open, close, or middle
void driveHand(uint16_t pos) {
    if (pos < SERVO_MIN) {
        pos = SERVO_MIN;
    }
    else if (pos > SERVO_MAX) {
        pos = SERVO_MAX;
    }
    PWM_Hand_WriteCompare1(pos);
}

// Ask the motor controller boards for feedback.
void heartbeatEventHandler() {
    // Turret
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_TURRET);
    
    // Shoulder
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_SHOULDER);
    
    // Elbow
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_ELBOW);
    
    // Forearm
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_FOREARM);
	
    // Wrist tilt
    // TODO: call dynamixelReadPosition() for wrist tilt
    
    // Wrist spin
    // TODO: call dynamixelReadPosition() for wrist spin
}

// Report received positional feedback to the computer.
void reportPositionEvent() {
    //static int i = 0;
    //i++;
    //turretPos += i;
    //shoulderPos += 2*i;
    //elbowPos += 3*i;
    //forearmPos += 4*i;
    // Send positions to computer
    positionArray[0] = 0xE3; // start byte;
    positionArray[1] =  (turretPos & 0xff);
    positionArray[2] = ((turretPos >> 8) & 0xff);
    positionArray[3] =  (shoulderPos & 0xff);
    positionArray[4] = ((shoulderPos >> 8) & 0xff);
    positionArray[5] =  (elbowPos & 0xff);
    positionArray[6] = ((elbowPos  >> 8) & 0xff);
    positionArray[7] =  (forearmPos & 0xff);
    positionArray[8] = ((forearmPos >> 8) & 0xff);
	positionArray[9] = ((wristTiltPos & 0xff));
	positionArray[10] = ((wristTiltPos >> 8) & 0xff);
	positionArray[11] =((wristSpinPos & 0xff));
	positionArray[12] =((wristSpinPos >> 8) & 0xff);
	UART_Computer_PutArray(positionArray, POSITION_PAYLOAD_SIZE);
}

/* [] END OF FILE */
