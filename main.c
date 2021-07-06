// 1 DOF compliant mechanism to simulate torsional stiffness and damping

#include "msp.h"
#include "driverlib.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "ODrive.h"


volatile int i = 0;
volatile char rBuffer[64];
volatile int j = 0;
volatile char ident = 0;
volatile int len;
volatile float pos0 = 0;
volatile float vel0 = 0;
volatile float pos1 = 0;
volatile float vel1 = 0;
volatile float posHist0[3] = {0};
volatile float posHist1[3] = {0};
volatile bool flag = false;
float K0 = 15;
float C0 = .25;
float K1 = 15;
float C1 = .25;
volatile float command0 = 0;
volatile float command1 = 0;
float t = 0.01;
volatile char temp[8] = {0};
int tic = 0;
int toc = 0;
volatile int myCount = 0;
volatile char motor = 'Y';
volatile char calibCase = '0';

// Define the Timer A1 and A2 initiation structure
const Timer_A_UpModeConfig upConfig_1 =
{   TIMER_A_CLOCKSOURCE_SMCLK,// Tie Timer A to SMCLK
    TIMER_A_CLOCKSOURCE_DIVIDER_64,// Increment counter every 8 clock cycles
    469,// Period of Timer A 187500 = 1 Hz; 625 = 300 Hz
    TIMER_A_TAIE_INTERRUPT_DISABLE,// Disable Timer A rollover interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,// Enable Capture Compare interrupt
    TIMER_A_DO_CLEAR
};
const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
    // Configuration parameters for 115200 baud rate:-
    1, // clockPrescalar
    10, // firstModReg
    0, // secondModReg
    EUSCI_A_UART_NO_PARITY, // No Parity
    EUSCI_A_UART_LSB_FIRST, // LSB First
    EUSCI_A_UART_ONE_STOP_BIT, // One stop bit
    EUSCI_A_UART_MODE, // UART mode
    1 // Oversampling ON
};


// Define a function that sets up Timer A and ADC14 Module
void setup(void){
    Interrupt_disableMaster();
    // Set DCO Clock Source Frequency to 1 MHz
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN3 | GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    PCM_setCoreVoltageLevel(PCM_VCORE1);
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_setExternalClockSourceFrequency(32E+3, 48E+6);
    CS_startHFXT(false);
    // Initiate SMCLK Clock Signal
    CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
    // Initiate Timer A with configuration structure
    Timer_A_configureUpMode(TIMER_A1_BASE,&upConfig_1);
    Timer_A_enableInterrupt(TIMER_A1_BASE);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

    // Setup UART
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A2_BASE, &uartConfig);
    UART_enableModule(EUSCI_A2_BASE);
    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    UART_disableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

    // Setup buttons
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1|GPIO_PIN4);
    // Set GPIO transition that sparks interrupt
    GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN1|GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    // Clear the GPIO flags and enable GPIO interrupt
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1|GPIO_PIN4);
    // Setup interrupts for GPIO
    GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN1|GPIO_PIN4);

    // Setup GPIO for Debug
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);

    Interrupt_setPriority(INT_EUSCIA2, 0xA0);
    Interrupt_setPriority(INT_TA1_0, 0xC0);
    Interrupt_setPriority(INT_PORT1, 0xE0);


    // Enable Port 1 interrupt
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableInterrupt(INT_EUSCIA2);

    // Enabling NVIC Interrupts
    Interrupt_enableMaster();
}

void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    setup();
    while(1){
        if (flag == true){
            switch(motor){ // determine which motor to control
                case 'Y' : // yaw motor control
                    for (i=0; i<8; i++){
                        temp[i] = rBuffer[i]; // make array of first number in rBuffer
                    }
                    pos0 = atof(temp); // convert string number to float
                    for (i=0;i<2;i++){
                        posHist0[i] = posHist0[i+1]; // update position history
                    }
                    posHist0[2] = pos0;
                    vel0 = (3*posHist0[2] - 4*posHist0[1] + posHist0[0])/2/t; // calculate velocity of motor (approx)
                    command0 = -K0*pos0 - C0*vel0; // calcualte the motor torque command based on position and velocity (impedance control)
                    motor = 'P'; // Next do pitch control
                    break;
                case 'P' : // pitch motor control
                    for (i=0; i<8; i++){
                        temp[i] = rBuffer[i]; // make array of first number in rBuffer
                    }
                    pos1 = atof(temp); // convert string number to float
                    for (i=0;i<2;i++){
                        posHist1[i] = posHist1[i+1]; // update position history
                    }
                    posHist1[2] = pos1;
                    vel1 = (3*posHist1[2] - 4*posHist1[1] + posHist1[0])/2/t; // calculate velocity of motor (approx)
                    if (myCount == 1 || myCount == 2){
                        command1 = -K1*(pos1+0.075) - C1*vel1; // calcualte the motor torque command based on position and velocity (impedance control). Added offset to account for droop due to low stiffness
                    }
                    if (myCount == 0 || myCount == 3) {
                        command1 = -K1*(pos1+0.025) - C1*vel1; // calcualte the motor torque command based on position and velocity (impedance control)
                    }
                    motor = 'Y'; // Next do yaw control
                    break;
            }
            flag = false;
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN5);
            //int hold = 0;
        }
    }
}
void EUSCIA2_IRQHandler(void){
    if (j==0){
        //GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN5);
    }
    rBuffer[j] = UART_receiveData(EUSCI_A2_BASE); // Receives and clears interrupt flag
    j+=1;
    if (rBuffer[j-1] == 10){ // if an enter key is read
        flag = true; // flag for updating motor commands
        //len = j-3;
        j=0; // reset j for next read
        //UART_disableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
        //GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN5);
    }
}

// Define ISR for Timer A For Control
void TA1_0_IRQHandler(void){
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                                         TIMER_A_CAPTURECOMPARE_REGISTER_0);
    //UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    //GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN5);
    switch(motor){
        case 'Y' : // Yaw Motor
            inputTorque(MOTOR0, command0); // send command to motor
            ident = readFeedback(MOTOR0); // request feedback from the motor
            break;
        case 'P' : // Pitch Motor
            inputTorque(MOTOR1, command1); // send command to motor
            ident = readFeedback(MOTOR1); // request feedback from the motor
            break;
    }
    //GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN5);
}



// IRQ Handler for GPIO Encoder Interrupts
void PORT1_IRQHandler(void){
    Interrupt_disableMaster();
    GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN1|GPIO_PIN4);
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0){
        //startCalibration(AXIS0, AXIS_STATE_MOTOR_CALIBRATION); // calibrate motors
        switch(calibCase){
        case '0':
            startCalibration(AXIS0, AXIS_STATE_MOTOR_CALIBRATION);// calibrate motors
            startCalibration(AXIS0, AXIS_STATE_ENCODER_OFFSET_CALIBRATION);// calibrate encoders
            calibCase = '1';
            break;
        case '1':
            startCalibration(AXIS1, AXIS_STATE_MOTOR_CALIBRATION);// calibrate motors
            startCalibration(AXIS1, AXIS_STATE_ENCODER_OFFSET_CALIBRATION);// calibrate encoders
            calibCase = '2';
            break;
        case '2':
            startCalibration(AXIS0, AXIS_STATE_IDLE);
            startCalibration(AXIS1, AXIS_STATE_IDLE);
            break;
        }
        int ii;
        for(ii=0;ii<100000;ii++){}
    }
    if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0){
        // set stiffness and run current control mode on motors
        switch(myCount){
        case 0:
            K0 = 10; C0 = 0.1;
            K1 = 0.75; C1 = 0.015;
            printf("\nLow Stiffnes / Low Damping\n");
            myCount = 1;
            break;
        case 1:
            K0 = 10; C0 = .5;
            K1 = 0.75; C1 = 0.03;
            printf("\nLow Stiffnes / High Damping\n");
            myCount = 2;
            break;
        case 2:
            K0 = 30; C0 = 0.1;
            K1 = 2.25; C1 = 0;
            printf("\nHigh Stiffnes / Low Damping\n");
            myCount = 3;
            break;
        case 3:
            K0 = 30; C0 = .5;
            K1 = 2.25; C1 = .075;
            printf("\nHigh Stiffnes / High Damping\n");
            myCount = 0;
            break;
        }
        printf("\nmyCount = %i\n", myCount);
        int ii;
        for(ii=0;ii<1000000;ii++){}

        Interrupt_enableInterrupt(INT_TA1_0);

        //zeroPosition(AXIS1); //zero encoder after calibration (doesn't work)
        startCalibration(AXIS0, AXIS_STATE_CLOSED_LOOP_CONTROL);
        startCalibration(AXIS1, AXIS_STATE_CLOSED_LOOP_CONTROL);

    }
    Interrupt_enableMaster();
}
