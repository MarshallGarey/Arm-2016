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

// CompRxEvent will not happen if this is locked.
enum { LOCKED = 0, UNLOCKED = 1 } compRxEvent;

int main() {
    
    // Initialize variables
    events = 0; // no pending events initially
    compRxEvent = UNLOCKED; // allow computer to talk to us
    
    // Enable global interrupts
    CyGlobalIntEnable;
    
    // Initialize and start hardware components
    LED0_Write(0); // LED is initially off
    
    // heartbeat timer
    Clock_2_Start();
    HeartbeatCounter_Start();
    Heartbeat_ISR_StartEx(HeartbeatISR);
    
    // computer uart
    UART_Computer_Start();
    Comp_RX_ISR_StartEx(CompRxISR);
    
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
    
    // 5 second delay before we start everything up
    CyDelay(5000);
    
    // loop - the while(1) here is just to make the compiler happy
    while(1) {
        //multiJointTest();
        eventLoop();
        /*
        TOGGLE_LED0;
        pololuControl_driveMotor(200, POLOLUCONTROL_TURRET);
        CyDelay(3000);
        TOGGLE_LED0;
        pololuControl_driveMotor(4000, POLOLUCONTROL_TURRET);
        CyDelay(3000);
        TOGGLE_LED0;
        pololuControl_driveMotor(2048, POLOLUCONTROL_TURRET);
        CyDelay(3000);
        */
    }
}

void eventLoop() {
    // Main loop
    while(1) {
        if (events) {
            // Receive message from computer
            if ((events & COMP_RX_EVENT) && (compRxEvent == UNLOCKED)) {
                events &= ~COMP_RX_EVENT;
                compRxEventHandler();
            }
            // Heartbeat event
            else if (events & HEARTBEAT_EVENT) {
                events &= ~HEARTBEAT_EVENT;
                compRxEvent = LOCKED;
                heartbeatEventHandler();
            }
            
            //else if (events & SHOULDER_POS_EVENT) {
            //    events &= ~SHOULDER_POS_EVENT;
            //    reportPositionEvent();
            //}
            
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
        //pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
        //pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
        CyDelay(5000);
        
        // slowly move forward
        for (i = 0; i < 5; i++) {
            TOGGLE_LED0;
            target += 200;
            turret = shoulder = elbow = forearm = target;
            pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
            pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
            //pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
            //pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
            CyDelay(5000);
        }
        
        // back to neutral
        TOGGLE_LED0;
        target = 2048;
        turret = shoulder = elbow = forearm = target;
        pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
        pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
        //pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
        //pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
        CyDelay(5000);    
        
        // slowly move backward
        for (i = 0; i < 5; i++) {
            TOGGLE_LED0;
            target -= 200;
            turret = shoulder = elbow = forearm = target;
            pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
            pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
            //pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
            //pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
            CyDelay(5000);           
        }
    } 
}

/* [] END OF FILE */
