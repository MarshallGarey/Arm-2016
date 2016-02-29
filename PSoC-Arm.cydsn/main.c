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

/* ===========================================================================
* Arm controller architecture
* Event driven
* Required functionality:
*   Receive messages from the computer
*   Transmit messages to the computer
*   Command each arm motor
*   Receive feedback from each arm motor
* 
* Computer messages:
*   Receive from computer:
*     2-byte preface
*     Positions for each of the 7 joints (2 bytes per position) - turret,
*       shoulder, elbow, forearm, wristtilt, wristspin, hand (close/open)
*     
*   Transmit to computer:
*     2-byte preface
*     Current positions of each of the 6 arm motors (not the hand)
*
* Events:
*   Receive message from the computer. This comes less often than the heartbeat
*   event.
*     Actuates the motors as we get the values
*   Heartbeat - 5 - 10 times per second.
*     Queue getting current feedback positions from motors
*     Lock the computer receive message event.
*   Position
*     Queued when all feedback positions have been received
*     Sends feedback positions to onboard computer
*     Unlock the computer receive message event.
*
* ========================================================================= */
#include <project.h>
#include "isr.h"
#include "isrHandler.h"
#include "pololuControl.h"

#define TOGGLE_LED0 LED0_Write(!LED0_Read())

// the main event loop
void eventLoop();

// an automated test to control multiple pololu joints with just the PSoC
void multiJointTest();

// an automated test to open and close the hand.
void handTest();

// an automated test to actuate the solenoids to open/close the chute doors
void chuteTest();

// CompRxEvent will not happen if this is locked.
enum { LOCKED = 0, UNLOCKED = 1 } compRxEvent;

int main() {
    
    // 5 second delay before we start everything up
    CyDelay(5000);
    
    // Initialize variables
    events = 0; // no pending events initially
    compRxEvent = UNLOCKED; // allow computer to talk to us
    chute1_Write(0); // all chutes are closed
    chute2_Write(0);
    chute3_Write(0);
    chute4_Write(0);
    chute5_Write(0);
    chute6_Write(0);
    LED0_Write(0); // LED is initially off
    
    // Enable global interrupts
    CyGlobalIntEnable;
    
    // Initialize and start hardware components:
    
    // computer uart
    UART_Computer_Start();
    Comp_RX_ISR_StartEx(CompRxISR);
    
    // drive
    Clock_PWM_Start();
    PWM_Drive_Start();
    PWM_Drive_WriteCompare1(SERVO_NEUTRAL);
    PWM_Drive_WriteCompare2(SERVO_NEUTRAL);
    
    // gimbal (main camera pan/tilt)
    PWM_Gimbal_Start();
    PWM_Gimbal_WriteCompare1(SERVO_NEUTRAL);
    PWM_Gimbal_WriteCompare2(SERVO_NEUTRAL);
    
    // turret
    UART_Turret_Start();
    TurretRxIsr_StartEx(TurretRxISR);
    pololuControl_turnMotorOff(POLOLUCONTROL_TURRET);
    
    // shoulder uart
    UART_Shoulder_Start();
    ShoulderRxIsr_StartEx(ShoulderRxISR);
    pololuControl_turnMotorOff(POLOLUCONTROL_SHOULDER);
    
    // elbow uart
    UART_Elbow_Start();
    ElbowRxIsr_StartEx(ElbowRxISR);
    pololuControl_turnMotorOff(POLOLUCONTROL_ELBOW);
    
    // forearm uart
    UART_Forearm_Start();
    ForearmRxIsr_StartEx(ForearmRxISR);
    pololuControl_turnMotorOff(POLOLUCONTROL_FOREARM);
    
    // wrist uart
    // TODO: initialize this here
    
    // hand pwm (also the heartbeat timer).
    PWM_Hand_Start();
    PWM_Hand_WriteCompare1(SERVO_NEUTRAL);
    PWM_Hand_WriteCompare2(SERVO_NEUTRAL);
    heartbeatIsr_StartEx(HeartbeatISR);
    
    // loop - the while(1) here is just to make the compiler happy
    while(1) {
        //multiJointTest();
        //handTest();
        chuteTest();
        //eventLoop();
    }
}

void eventLoop() {
    // Main loop
    while(1) {
        if (events) {
            // Receive message from computer
            if ((events & COMP_RX_EVENT) && (compRxEvent == UNLOCKED)) {
                events &= ~COMP_RX_EVENT;
                TOGGLE_LED0;
                compRxEventHandler();
            }
            
            // Heartbeat event
            else if (events & HEARTBEAT_EVENT) {
                events &= ~HEARTBEAT_EVENT;
                compRxEvent = LOCKED;
                heartbeatEventHandler(); // TODO: this doesn't do anything right now.
                events |= POS_EVENT_GROUP; // TODO: remove this.
            }
            
            // Position event group - wait for all bits to be set
            else if ((events & POS_EVENT_GROUP) == POS_EVENT_GROUP) {
                events &= ~POS_EVENT_GROUP; // clear event group
                reportPositionEvent();
                TOGGLE_LED0;
                compRxEvent = UNLOCKED;
            }
            
            // Invalid event
            else {
                // TODO: manage invalid event
            }
        }
    }
}

void chuteTest() {
    while(1) {
        TOGGLE_LED0;
        chute1_Write(0);
        chute2_Write(0);
        chute3_Write(0);
        chute4_Write(0);
        chute5_Write(0);
        chute6_Write(0);
        CyDelay(2000);
        TOGGLE_LED0;
        chute1_Write(1);
        chute2_Write(1);
        chute3_Write(1);
        chute4_Write(1);
        chute5_Write(1);
        chute6_Write(1);
        CyDelay(2000);
    }
}

// automated test that moves 4 arm joints (all controlled with 
// pololu PID boards)
void multiJointTest() {
    int i;
    uint16_t shoulder = 2048;
    uint16_t turret = 2048;
    uint16_t elbow = 2048;
    uint16_t forearm = 2048;
    uint16_t target = 2048;

    while (1) {
        // back to neutral
        TOGGLE_LED0;
        target = 2048;
        turret = shoulder = elbow = forearm = target;
        pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
        pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
        pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
        pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
        CyDelay(5000);
        
        // slowly move forward
        for (i = 0; i < 5; i++) {
            TOGGLE_LED0;
            target += 200;
            turret = shoulder = elbow = forearm = target;
            pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
            pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
            pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
            pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
            CyDelay(5000);
        }
        
        // back to neutral
        TOGGLE_LED0;
        target = 2048;
        turret = shoulder = elbow = forearm = target;
        pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
        pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
        pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
        pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
        CyDelay(5000);    
        
        // slowly move backward
        for (i = 0; i < 5; i++) {
            TOGGLE_LED0;
            target -= 200;
            turret = shoulder = elbow = forearm = target;
            pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
            pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
            pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
            pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
            CyDelay(5000);           
        }
    } 
}

// automated test that opens and closes the hand
void handTest() {
    
    while(1) {
        PWM_Hand_WriteCompare1(SERVO_MAX);
        TOGGLE_LED0;
        CyDelay(4000);
        PWM_Hand_WriteCompare1(SERVO_NEUTRAL);
        TOGGLE_LED0;
        CyDelay(4000);
        PWM_Hand_WriteCompare1(SERVO_MIN);
        TOGGLE_LED0;
        CyDelay(4000);
        PWM_Hand_WriteCompare1(SERVO_NEUTRAL);
        TOGGLE_LED0;
        CyDelay(4000);
    }
}

/* [] END OF FILE */
