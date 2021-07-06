/*
 * ODrive.h
 *
 *  Created on: Mar 17, 2021
 *      Author: A. Ambrose
 */

#ifndef ODRIVE_H_
#define ODRIVE_H_

/* Header file for Odrive Robotics v3.6 motor controller. See ODrive.c for external functions
 *
 *
 *
 */
#endif /* ODRIVE_H_ */

#define AXIS0       "axis0" // MACRO for the string for axis 0
#define AXIS1       "axis1" // MACRO for the string for axis 1
#define MOTOR0      " 0" // string version of "0"
#define MOTOR1      " 1" // string version of "1"

#define CONTROL_MODE_VOLTAGE_CONTROL        "0" // voltage control mode(rarely used)
#define CONTROL_MODE_TORQUE_CONTROL         "1" // torque control mode
#define CONTROL_MODE_VELOCITY_CONTROL       "2" // position control mode
#define CONTROL_MODE_POSITION_CONTROL       "3" // velocity control mode

#define AXIS_STATE_UNDEFINED                        "0" //<! will fall through to idle
#define AXIS_STATE_IDLE                             "1" //<! disable PWM and do nothing
#define AXIS_STATE_STARTUP_SEQUENCE                 "2" //<! the actual sequence is defined by the config.startup_... flags
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE        "3" //<! run all calibration procedures, then idle
#define AXIS_STATE_MOTOR_CALIBRATION                "4" //<! run motor calibration
#define AXIS_STATE_SENSORLESS_CONTROL               "5" //<! run sensorless control
#define AXIS_STATE_ENCODER_INDEX_SEARCH             "6" //<! run encoder index search
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION       "7" //<! run encoder offset calibration
#define AXIS_STATE_CLOSED_LOOP_CONTROL              "8"  //<! run closed loop control

#define FEEDBACK        0
#define VOLTAGE         1
#define CURRENT         2

/*
 * Sends the command to receive the vbus-voltage of the ODrive controller.
 *  This is a good function to use to debug your UARt functionality of your MCU.
*/
//extern void vbus_voltage(void);
//extern void zeroPosition(char axis[]);
//extern void controlMode(char axis[], char controlSetting[]);
extern void startCalibration(char axis[], char calibration[]);
//extern void axisControl(char axis[], char control[]);
//extern void torqueGains(char axis[], uint16_t gain);
extern void inputTorque(char motor[], float command);
extern char readFeedback(char motor[]);
//extern char readVoltage(char axis[]);
//extern char readCurrent(char axis[]);

