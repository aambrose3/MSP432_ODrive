/*
 * ODrive.c
 * This file contains global functions that follow the communicaiton protocol for the ODrive motor controller. Please see
 * ODrive documentation for more information.
 *  Created on: Mar 17, 2021
 *      Author: A. Ambrose
 */

#include "driverlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "ODrive.h"

#define FEEDBACK        0
#define VOLTAGE         1
#define CURRENT         2


void startCalibration(char axis[], char calibration[]){
    int i = 0;
    char str[] = "w ";
    char str2[] = ".requested_state ";
    char message[64] = {0};
    strcpy(message, str);
    strcat(message, axis);
    strcat(message, str2);
    strcat(message, calibration);
    for (i=0; i< sizeof message; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, message[i]); // write parameter
    }
    while((UCA0IFG & 0x02) == 0){}
    UART_transmitData(EUSCI_A2_BASE, 0x0A);
}

void inputTorque(char motor[], float command){ // Command a torque to the motor (may need to specify direction)
    int i = 0;
    char str[] = "c";
    char torque[5] = {0};
    int len = sprintf(torque, "%.3f", command);
    char message[9] = {0};
    strcpy(message, str);
    strcat(message, motor);
    strcat(message, " ");
    strcat(message, torque);
    for (i=0; i< sizeof message; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, message[i]); // write parameter
    }
    while((UCA0IFG & 0x02) == 0){}
    UART_transmitData(EUSCI_A2_BASE, 0x0A);
}

char readFeedback(char motor[]){ // Reads the current position estimate of the motor
    int i = 0;
    char str[] = "f";
    char message[3] = {0};
    strcpy(message, str);
    strcat(message, motor);
    int c = sizeof message;
    for (i=0; i< sizeof message; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, message[i]); // write parameter
    }
    while((UCA0IFG & 0x02) == 0){}
    UART_transmitData(EUSCI_A2_BASE, 0x0A);
    return FEEDBACK; // identifier code for feedback
}

/*

These functions are not used by the main file and have been removed.

void vbus_voltage(void){
    int i = 0;
    char str[] = "r vbus_voltage";
    for (i=0; i< sizeof str; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, str[i]);
    }
    while((UCA0IFG & 0x02) == 0){}
    UART_transmitData(EUSCI_A2_BASE, 0x0A);
}

void zeroPosition(char axis[]){ // Zeros the encoder positions (DOESNT WORK!)
    int i = 0;
    char str[] = "w ";
    char str2[] = ".encoder.set_linear_count 0";
    char message[64] = {0};
    strcpy(message, str);
    strcat(message, axis);
    strcat(message, str2);
    for (i=0; i< sizeof message; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, message[i]); // write parameter
    }
    while((UCA0IFG & 0x02) == 0){}
    UART_transmitData(EUSCI_A2_BASE, 0x0A);
}

void controlMode(char axis[], char controlSetting){
    int i = 0;
    char str[] = "w axis";
    char motor[1] = (char) (axis+48); // convert int to ascii
    char str2[] = ".controller.config.control_mode ";
    char mode[1] = (char) (controlSetting+48);
    char message[64] = {0};
    strcpy(message, str);
    strcat(message, motor);
    strcat(message, str2);
    strcat(message, mode);
    message[40] = 0;
    for (i=0; i< sizeof message; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, message[i]); // write parameter
    }
    while((UCA0IFG & 0x02) == 0){}
    UART_transmitData(EUSCI_A2_BASE, 0x0A);
}




void axisControl(char axis[], char control[]){ // starts closed loop control
    int i = 0;
    char str[] = "w axis";
    char motor[1] = (char) (axis+48); // convert int to ascii
    char str2[] = ".requested_state ";
    char message[64] = {0};
    strcpy(message, str);
    strcat(message, motor);
    strcat(message, str2);
    strcat(message, control);
    for (i=0; i< sizeof message; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, message[i]); // write parameter
    }
    while((UCA0IFG & 0x02) == 0){}
    UART_transmitData(EUSCI_A2_BASE, 0x0A);
}

void torqueGains(char axis[], uint16_t gain){ // Set the torque Gain "motor.config.current_control_bandwidth"
    int i = 0;
    char str[] = "w axis";
    char motor[1] = (char) (axis+48); // convert int to ascii
    char str2[] = ".motor.config.current_control_bandwidth ";
    char dest[4] = {0};
    int len = sprintf(dest, "%i", gain);
    char message[52] = {0};
    strcpy(message, str);
    strcat(message, motor);
    strcat(message, str2);
    strcat(message, dest);
    for (i=0; i< sizeof message; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, message[i]); // write parameter
    }
    while((UCA0IFG & 0x02) == 0){}
    UART_transmitData(EUSCI_A2_BASE, 0x0A);
}


char readVoltage(char axis[]){ // Reads the current voltage through the motor
    int i = 0;
    char str[] = "r axis";
    char motor[1] = (char) (axis+48); // convert int to ascii
    char str2[] = ".motor.current_control.v_current_control_integral_q\n";
    char message[40] = {0};
    strcpy(message, str);
    strcat(message, motor);
    strcat(message, str2);
    for (i=0; i< sizeof message; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, message[i]); // write parameter
    }
    return VOLTAGE; // identifier code for feedback

}

char readCurrent(char axis[]){ // Reads the current through the motor
    int i = 0;
    char str[] = "r axis";
    char motor[1] = (char) (axis+48); // convert int to ascii
    char str2[] = ".motor.current_control.v_current_control_integral_d\n";
    char message[40] = {0};
    strcpy(message, str);
    strcat(message, motor);
    strcat(message, str2);
    for (i=0; i< sizeof message; i++){
        while((UCA0IFG & 0x02) == 0){}
        UART_transmitData(EUSCI_A2_BASE, message[i]); // write parameter
    }
    return CURRENT; // identifier code for feedback
}

*/
