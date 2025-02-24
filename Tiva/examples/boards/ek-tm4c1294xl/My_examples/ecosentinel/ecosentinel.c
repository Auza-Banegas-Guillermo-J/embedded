/* 
EcoSentinel Project
Guillermo Javier Auza Banegas
Embedded systems II
IMT UCB 2024 S2
*/

// Libraries
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <stdlib.h>
#include <math.h> 

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"

#include "utils/uartstdio.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"

// Macros
#define CLOCK 120000000
#define PINS (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)
#define MAX_LEDS 4

#define PPR 11
#define REDUCTION_RATIO 26
#define RAD_TO_RPM 9.5492

// Global Variables
volatile uint32_t timeMotor_A = 0;
volatile uint32_t timeMotor_B = 0;

volatile uint32_t encoderCount_A = 0;
volatile uint32_t encoderCount_B = 0;

volatile float rpmMotor_A = 0.0;
volatile float rpmMotor_B = 0.0;

const int filterLength = 3;

float rpmHist_A[3] = {0};
uint32_t filterIndex_A = 0; 

float rpmHist_B[3] = {0};
uint32_t filterIndex_B = 0; 

bool allowMovement = 0;

bool ledTimer_A = 0;
bool ledTimer_B = 0;

volatile int32_t reg_val;

uint32_t minPulseWidth = 120000;
uint32_t maxPulseWidth = 240000;

uint32_t value[2];
int32_t width[7] = {1,1,1,1,1,1,1};

uint32_t pinMatrix[4] = {GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_4, GPIO_PIN_0};
uint32_t portMatrix[4] = {GPIO_PORTN_BASE, GPIO_PORTN_BASE, GPIO_PORTF_BASE, GPIO_PORTF_BASE};

// Function Prototypes
void delay(uint32_t);
void peripheralStartup(void);

void startTimer(uint32_t, volatile uint32_t*, bool*, uint32_t, uint32_t, void (*)(void));
void stopTimer(uint32_t, bool*, uint32_t);

void timerInterruptHandler_A(void);
void timerInterruptHandler_B(void);

void motorEncoder_A(void);
void motorEncoder_B(void);

void gpioOn(uint32_t, uint32_t);
void gpioOff(uint32_t, uint32_t);

void uart2digit(void);

void ConfigureServo(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
void angle2pwm(uint32_t, uint32_t, float);

float mediaMovil(float, float*, uint32_t*);

//Class Definitions
typedef struct {
    uint32_t kp, ki;
    float kd;
    float setpoint, error, prevError;
    float integral, derivative, dt;
    float output, prevMeasurement;
} PIDController;

float calculatePID(PIDController*, float);

PIDController pidMotor_A = {12000, 0.001, 10, 0.0, 0, 0, 0, 0, 1000, 1, 0}; //motor izquierdo
PIDController pidMotor_B = {12000*1.225, 0.001, 10, 0.0, 0, 0, 0, 0, 1000, 1, 0}; // motor derecho 

int percentageCoord_x=50, percentageCoord_y=50;
short receiveIndex = 0; 
char receivedData[4];

// Main Function
int main(void) {

    float porcentage_x,porcentage_y; 

    uint32_t g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                                    SYSCTL_OSC_MAIN |
                                                     SYSCTL_USE_PLL |
                                                SYSCTL_CFG_VCO_240), CLOCK);

    peripheralStartup();

    FPUEnable();
    FPUStackingEnable();

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,GPIO_PIN_1|GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    /*
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_0,400);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 400);

    PWMGenEnable(PWM0_BASE,PWM_GEN_0);
    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);

    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    */

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);

    ADCSequenceEnable(ADC0_BASE,3);
    ADCIntClear(ADC0_BASE, 3);

    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock / 1000) - 1);

    TimerIntRegister(TIMER0_BASE, TIMER_A, timerInterruptHandler_A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    IntPrioritySet(INT_TIMER0A, 0);
    TimerEnable(TIMER0_BASE, TIMER_A);

    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, (g_ui32SysClock / 1000) - 1);

    TimerIntRegister(TIMER1_BASE, TIMER_A, timerInterruptHandler_B);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER1A);
    IntPrioritySet(INT_TIMER1A, 0);
    TimerEnable(TIMER1_BASE, TIMER_A);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);

    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_4);

    GPIOIntRegister(GPIO_PORTB_BASE, motorEncoder_B);
    GPIOIntRegister(GPIO_PORTM_BASE, motorEncoder_A);

    IntEnable(INT_GPIOB);
    IntEnable(INT_GPIOM);

    IntMasterEnable();

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, g_ui32SysClock);

    ConfigureServo(PWM_GEN_0, PWM_OUT_1_BIT, GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PF1_M0PWM1, 50);
    ConfigureServo(PWM_GEN_1, PWM_OUT_2_BIT, GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PF2_M0PWM2, 50);

    //necesario hacerlo con los motores DC pq PF2 y PF3 comparten el PWM_GEN :) :) :)
    // 5 hrs alv :,)

    ConfigureServo(PWM_GEN_1, PWM_OUT_3_BIT, GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PF3_M0PWM3, 50);
    ConfigureServo(PWM_GEN_2, PWM_OUT_4_BIT, GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PG0_M0PWM4, 50);
    ConfigureServo(PWM_GEN_2, PWM_OUT_5_BIT, GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PG1_M0PWM5, 50);
    ConfigureServo(PWM_GEN_3, PWM_OUT_6_BIT, GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PK4_M0PWM6, 50);

    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

    GPIOPinWrite(GPIO_PORTF_BASE, PINS, 0x00);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1);

    GPIOPinWrite(GPIO_PORTL_BASE, PINS, 0x00);
    
    //gpioOn(GPIO_PORTL_BASE,GPIO_PIN_1);
    //gpioOn(GPIO_PORTL_BASE,GPIO_PIN_2);
    /*
    UARTprintf("PID_A_INIT: error=%.2f, integral=%.2f, derivative=%.2f, dt=%.2f, output=%.2f\n",
               pidMotor_A.error, pidMotor_A.integral, pidMotor_A.derivative, pidMotor_A.dt, pidMotor_A.output);
    UARTprintf("PID_B_INIT: error=%f, integral=%f, derivative=%f, dt=%f, output=%f\n",
               pidMotor_B.error, pidMotor_B.integral, pidMotor_B.derivative, pidMotor_B.dt, pidMotor_B.output);*/
    UARTprintf("\n\nBooting UP!!!\n\n--------------------------------------------------------------\n\n");
    delay(1000);
    uint32_t testValue = 10123;
    int testValue_2 = -10;
    UARTprintf("Test: %u\n", testValue);
    UARTprintf("Test: %d\n", abs(testValue_2));

    //gpioOn(GPIO_PORTL_BASE,GPIO_PIN_0);
    //gpioOn(GPIO_PORTL_BASE,GPIO_PIN_3);

    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ((SysCtlClockGet()/50))/5);
    //PWMOutputState(PWM0_BASE,PWM_OUT_2_BIT,true);

    /*
    uint32_t adcClock = 120000000 / 64;
    uint32_t sampleRate = 10;
    uint32_t sampleDivider = adcClock / sampleRate;*/

    //ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_HALF, sampleDivider);

    //pidMotor_A.setpoint=100;
    //pidMotor_B.setpoint=100;
    /*
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,(uint32_t)(SysCtlClockGet()/500)*1.25);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,(uint32_t)(SysCtlClockGet()/500)*0.625);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,(uint32_t)(SysCtlClockGet()/500)*0.3125);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,(uint32_t)(SysCtlClockGet()/500)*0.15625);
    PWMOutputState(PWM0_BASE,PWM_OUT_3_BIT,true);
    PWMOutputState(PWM0_BASE,PWM_OUT_4_BIT,true);
    PWMOutputState(PWM0_BASE,PWM_OUT_5_BIT,true);
    PWMOutputState(PWM0_BASE,PWM_OUT_6_BIT,true);*/

    /*
    angle2pwm(PWM_OUT_3_BIT,PWM_OUT_3,-90);
    angle2pwm(PWM_OUT_4_BIT,PWM_OUT_4,-45);
    angle2pwm(PWM_OUT_5_BIT,PWM_OUT_5,45);
    angle2pwm(PWM_OUT_6_BIT,PWM_OUT_6,90);
    */

    /*
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_0);
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_3);
    pidMotor_A.setpoint = 1.35;
    pidMotor_B.setpoint = 1.35;*/

    //UARTprintf("Arm porperly Written\n\n");
    delay(1000);
    UARTprintf("Starting main Algorithm...\n\n");
    delay(1000);
    porcentage_x = 50.0;
    porcentage_y = 50.0;
    while (1){
        //gpioOn(GPIO_PORTN_BASE,GPIO_PIN_1);
        //delay(1000);
        //GPIOPinWrite(GPIO_PORTN_BASE,PINS, 0x00);
        //UARTprintf(".\n");

        if(!GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_4)==0){
            
            uart2digit();

            UARTprintf("Object Detected !\n\n");
            UARTprintf("I Heard:\n Number 1 : %d\nNumber 2 _ %d",percentageCoord_x,percentageCoord_y);

            // ADCS completamente inecesarios :,)

            ADCProcessorTrigger(ADC0_BASE, 3);
            while (!ADCIntStatus(ADC0_BASE, 3, false)) {;}
            ADCIntClear(ADC0_BASE, 3);
            ADCSequenceDataGet(ADC0_BASE, 3, &value[0]);
            porcentage_x = (int)(((float)value[0] / 4095.0) * 100);
            //UARTprintf("ADC 1 porcentual : %d\n",(int)porcentage_x);

            ADCProcessorTrigger(ADC0_BASE, 2);
            while (!ADCIntStatus(ADC0_BASE, 2, false)) {;}
            ADCIntClear(ADC0_BASE, 2);
            ADCSequenceDataGet(ADC0_BASE, 2, &value[1]);
            porcentage_y = (int)(((float)value[1] / 4095.0) * 100);
            //UARTprintf("ADC 2 porcentual : %d\n",(int)porcentage_y);
            
            float error_x = percentageCoord_x - 50.0;
            float error_y = percentageCoord_y - 50.0;

            float numericalError_x = abs(error_x);
            float numericalError_y = abs(error_y);

            //UARTprintf("Errores de centro :\n1 : %d\n2 : %d\n", (int)numericalError_x, (int)numericalError_y);

            while(numericalError_x>4){
                //UARTprintf("PWM motor A : %u\n",(uint32_t)calculatePID(&pidMotor_A, mediaMovil(rpmMotor_A,rpmHist_A,&filterIndex_A)));
                /*
                ADCProcessorTrigger(ADC0_BASE, 3);
                while (!ADCIntStatus(ADC0_BASE, 3, false)) {;}
                ADCIntClear(ADC0_BASE, 3);
                ADCSequenceDataGet(ADC0_BASE, 3, &value[0]);*/

                uart2digit();

                //porcentage_x = (int)(((float)value[0] / 4095.0) * 100);
                error_x = percentageCoord_x - 50.0;
                numericalError_x = abs(error_x);
                if(error_x>0)
                {
                    UARTprintf("Girando izquierda\n");
                    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_0);
                    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_2);
                }
                else{
                    UARTprintf("Girando Derecha\n");
                    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_1);
                    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_3);
                }
                pidMotor_A.setpoint = 1.35;
                pidMotor_B.setpoint = 1.35;
                delay(100);
                pidMotor_A.setpoint = 0;
                pidMotor_B.setpoint = 0;
                GPIOPinWrite(GPIO_PORTL_BASE, PINS, 0x0);
                delay(50);
            }
            while(numericalError_y>5.0){
                //UARTprintf("PWM motor B : %u\n",(uint32_t)calculatePID(&pidMotor_B, mediaMovil(rpmMotor_B,rpmHist_B,&filterIndex_B)));
                /*
                ADCProcessorTrigger(ADC0_BASE, 2);
                while (!ADCIntStatus(ADC0_BASE, 2, false)) {;}
                ADCIntClear(ADC0_BASE, 2);
                ADCSequenceDataGet(ADC0_BASE, 2, &value[1]);*/
                error_x = percentageCoord_x - 50.0;
                numericalError_x = abs(error_x);
                while(numericalError_x>2){
                    uart2digit();
                    error_x = percentageCoord_x - 50.0;
                    numericalError_x = abs(error_x);
                    if(error_x>0)
                    {
                        UARTprintf("Girando izquierda\n");
                        gpioOn(GPIO_PORTL_BASE,GPIO_PIN_0);
                        gpioOn(GPIO_PORTL_BASE,GPIO_PIN_2);
                    }
                    else{
                        UARTprintf("Girando Derecha\n");
                        gpioOn(GPIO_PORTL_BASE,GPIO_PIN_1);
                        gpioOn(GPIO_PORTL_BASE,GPIO_PIN_3);
                    }
                    pidMotor_A.setpoint = 1.35;
                    pidMotor_B.setpoint = 1.35;
                    delay(30);
                    pidMotor_A.setpoint = 0;
                    pidMotor_B.setpoint = 0;
                    GPIOPinWrite(GPIO_PORTL_BASE, PINS, 0x0);
                    delay(250);
                }
                uart2digit();

                error_y = percentageCoord_y - 50.0;
                numericalError_y = abs(error_y);

                //porcentage_y = (int)(((float)value[1] / 4095.0) * 100);
                //error_y = porcentage_y - 50;
                //numericalError_y = abs(error_y);
                if(error_y>0)
                {
                    UARTprintf("Retrocediendo\n");
                    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_1);
                    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_2);
                }
                else{
                    UARTprintf("Avanzando\n");
                    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_0);
                    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_3);
                }
                pidMotor_A.setpoint = 1.5;
                pidMotor_B.setpoint = 1.5;
                delay(250);
                pidMotor_A.setpoint = 0;
                pidMotor_B.setpoint = 0;
                GPIOPinWrite(GPIO_PORTL_BASE, PINS, 0x0);
                delay(10);
            }
            uart2digit();
            error_x = percentageCoord_x - 50.0;
            numericalError_x = abs(error_x);
            error_y = percentageCoord_y - 50.0;
            numericalError_y = abs(error_y);
            if((numericalError_x<=2)&&(numericalError_y<=5)){
                angle2pwm(PWM_OUT_3_BIT,PWM_OUT_3,-90);
                angle2pwm(PWM_OUT_4_BIT,PWM_OUT_4,-45);
                angle2pwm(PWM_OUT_5_BIT,PWM_OUT_5,45);
                angle2pwm(PWM_OUT_6_BIT,PWM_OUT_6,90);
            }
            pidMotor_A.setpoint = 0;
            pidMotor_B.setpoint = 0;
            GPIOPinWrite(GPIO_PORTL_BASE, PINS, 0x0);
            delay(1000);
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
            PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false);
        }
        /*
        if(numericalError_x>5.0){
            if(error_x>0)
            {
                UARTprintf("Error Positivo X\n");
            }
            else{
                UARTprintf("Error Negativo X\n");
            }
        }

        if(numericalError_y>5.0){
            if(error_y>0)
            {
                UARTprintf("Error Positivo Y\n");
            }
            else{
                UARTprintf("Error Negativo Y\n");
            }
        }*/
        /*
        UARTprintf("PID_A : %u \nPID_B : %u \n",calculatePID(&pidMotor_A, rpmMotor_A),calculatePID(&pidMotor_B, rpmMotor_B));
        UARTprintf("Encoder A: %u\n", encoderCount_A);
        UARTprintf("Encoder B: %u\n", encoderCount_B);
        UARTprintf("RPM A: %u\n", rpmMotor_A);
        UARTprintf("RPM B: %u\n", rpmMotor_B);*/
    }
}

void peripheralStartup(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){;}
}

void startTimer(uint32_t clock, volatile uint32_t *cronometer, bool *isRunning,
                 uint32_t timerBase, uint32_t interruptTimer, void (*interruptHandler)(void)){
    *cronometer = 0;
    *isRunning = true;

    IntEnable(interruptTimer);
    TimerIntEnable(timerBase, TIMER_TIMA_TIMEOUT);
    TimerEnable(timerBase, TIMER_A);
}

void stopTimer(uint32_t timerBase, bool *isRunning, uint32_t interruptTimer) {
    *isRunning = false;

    IntDisable(interruptTimer);
    TimerIntDisable(timerBase, TIMER_TIMA_TIMEOUT);
    TimerDisable(timerBase, TIMER_A);
}

void timerInterruptHandler_A(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    if(1){
        //timeMotor_A++;

        rpmMotor_A = (float)(encoderCount_A * 60 * 1000) / (PPR * REDUCTION_RATIO);
        encoderCount_A = 0;

        float pwm_output_A = calculatePID(&pidMotor_A, mediaMovil(rpmMotor_A,rpmHist_A,&filterIndex_A));
        //UARTprintf("Effective PWM A : %u\n",(uint32_t)pwm_output_A);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (uint32_t)pwm_output_A);
    }
    if(!ledTimer_A){
        gpioOn(GPIO_PORTF_BASE,GPIO_PIN_4);
        ledTimer_A = 1;
    }
    //UARTprintf("Check_Timer_1");
}

void timerInterruptHandler_B(void) {
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    if(1){
        //timeMotor_B++;

        rpmMotor_B = (float)(encoderCount_B * 60 * 1000) / (PPR * REDUCTION_RATIO);
        encoderCount_B = 0;

        float pwm_output_B = calculatePID(&pidMotor_B, mediaMovil(rpmMotor_B,rpmHist_B,&filterIndex_B));
        //UARTprintf("Effective PWM B : %u\n",(uint32_t)pwm_output_B);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (uint32_t)pwm_output_B);
    }
    if(!ledTimer_B){
        gpioOn(GPIO_PORTF_BASE,GPIO_PIN_0);
        ledTimer_B = 1;
    }
    //UARTprintf("Check_Timer_2");
}

void motorEncoder_A(void){
    GPIOIntClear(GPIO_PORTM_BASE, GPIO_PIN_4);
    encoderCount_A++;
    //UARTprintf("Encoder A check\n");
    gpioOn(GPIO_PORTN_BASE,GPIO_PIN_0);
} 

void motorEncoder_B(void){
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2);
    encoderCount_B++;
    //UARTprintf("Encoder B check\n");
    gpioOn(GPIO_PORTN_BASE,GPIO_PIN_1);
} 

void gpioOn(uint32_t port, uint32_t pin){
    reg_val = GPIOPinRead(port,PINS);
    GPIOPinWrite(port, PINS, reg_val|pin);
}

void gpioOff(uint32_t port, uint32_t pin){
    reg_val = GPIOPinRead(port,PINS);
    GPIOPinWrite(port, PINS, reg_val&(~pin));
}

void delay(uint32_t time){
    SysCtlDelay((CLOCK/3000) * time);
}

float calculatePID(PIDController *pid, float current_value){
    pid->error = pid->setpoint - current_value;
    pid->integral += pid->error * pid->dt;
    pid->derivative = (pid->error - pid->prevError) / pid->dt;

    pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * pid->derivative);
    pid->prevError = pid->error;
    /*
    if (pid->integral > SysCtlClockGet()/2){
        pid->integral = SysCtlClockGet()/2;
    } else if (pid->integral < -SysCtlClockGet()/2){
        pid->integral = -SysCtlClockGet()/2;
    }*/

    if (pid->output > SysCtlClockGet() / 50) {
        pid->output = SysCtlClockGet() / 50;
    } else if (pid->output < 1) {
        pid->output = 1;
    }
    return pid->output;
}

void uart2digit(void) {
    receiveIndex = 0;
    char c;
    bool uartStart = 0;
    while(!uartStart){
        c = UARTCharGet(UART0_BASE);
        if(c=='s'){
            uartStart = 1;
        }
    }

    while(receiveIndex<4){
        c = UARTCharGet(UART0_BASE);
        receivedData[receiveIndex++] = c;
    }
    percentageCoord_x = (receivedData[0] - '0') * 10 + (receivedData[1] - '0');
    percentageCoord_y = (receivedData[2] - '0') * 10 + (receivedData[3] - '0');
    while(UARTCharsAvail(UART0_BASE)){
        UARTCharGetNonBlocking(UART0_BASE);
    }
}

void ConfigureServo(uint32_t pwmGen, uint32_t pwmOutBit, uint32_t gpioBase, uint32_t gpioPin, uint32_t gpioPinConfig, uint32_t targetPeriod){
    
    uint32_t pwmPeriod = SysCtlClockGet() / targetPeriod;

    GPIOPinConfigure(gpioPinConfig);
    GPIOPinTypePWM(gpioBase, gpioPin);

    PWMGenConfigure(PWM0_BASE, pwmGen, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, pwmGen, pwmPeriod);

    PWMGenEnable(PWM0_BASE, pwmGen);
    PWMOutputState(PWM0_BASE,pwmOutBit,false);
    //PWMOutputState(PWM0_BASE,pwmOutBit,true);
}

void angle2pwm(uint32_t pwmOutBit, uint32_t pwmOut, float angle){
    uint32_t pulseWidth = minPulseWidth + ((angle + 90) * (maxPulseWidth - minPulseWidth)) / 180;
    PWMOutputState(PWM0_BASE, pwmOutBit, true);
    PWMPulseWidthSet(PWM0_BASE, pwmOut, pulseWidth);
}

float mediaMovil(float rpm, float *historyPointer, uint32_t *indexPointer){
    historyPointer[*indexPointer] = rpm;
    (*indexPointer)++;

    if (*indexPointer == filterLength) {
        *indexPointer = 0;
    }

    float sum = 0.0;
    for (int i = 0; i < filterLength; i++) {
        sum += historyPointer[i];
    }
    return sum / filterLength;
}