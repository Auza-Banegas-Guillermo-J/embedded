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

uint32_t minPulseWidth = 120000;
uint32_t maxPulseWidth = 240000;

// Function Prototypes
void delay(uint32_t);
void peripheralStartup(void);
void ConfigurePWM(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
void angle2pwm(uint32_t, uint32_t, float);

// Main Function
int main(void) {
    bool flag = 1;
    uint32_t g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                                    SYSCTL_OSC_MAIN |
                                                     SYSCTL_USE_PLL |
                                                SYSCTL_CFG_VCO_240), CLOCK);

    peripheralStartup();
    
    //ConfigurePWM(PWM_GEN_0, PWM_OUT_1_BIT, GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PF1_M0PWM1, 50);
    //ConfigurePWM(PWM_GEN_1, PWM_OUT_2_BIT, GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PF2_M0PWM2, 50);
    ConfigurePWM(PWM_GEN_1, PWM_OUT_3_BIT, GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PF3_M0PWM3, 50);
    //ConfigurePWM(PWM_GEN_2, PWM_OUT_4_BIT, GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PG0_M0PWM4, 50);
    //ConfigurePWM(PWM_GEN_2, PWM_OUT_5_BIT, GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_PG1_M0PWM5, 50);
    //ConfigurePWM(PWM_GEN_3, PWM_OUT_6_BIT, GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PK4_M0PWM6, 50);
    //ConfigurePWM(PWM_GEN_3, PWM_OUT_7_BIT, GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PK5_M0PWM7, 50);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, g_ui32SysClock);
    UARTprintf("Booting UP !!! \n\n");
    delay(5000);
    while (1){
        if(flag){
            flag = 0;
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
            for(uint32_t i=1; i<2400000; i+=100){
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, i);
                delay(10);
                UARTprintf("Current PWM : %u\n",i);
                //UARTprintf("Angle : %d\n PWM : %u%%\n",(int)i, (uint32_t)(100*(minPulseWidth + ((i + 90) * (maxPulseWidth - minPulseWidth)) / 180)/2400000));
            }
            /*
            for(float i=-71; i>-73; i-=0.01){
                angle2pwm(PWM_OUT_3_BIT,PWM_OUT_3,i);
                delay(500);
                flag = 0;
                UARTprintf("Angle : %d\n PWM : %u%%\n",(int)i, (uint32_t)(100*(minPulseWidth + ((i + 90) * (maxPulseWidth - minPulseWidth)) / 180)/2400000));
            }*/
        }
        delay(1000);
        UARTprintf("Check\n");
        /*
        delay(1000);
        for(int i=0; i>-90; i-=5){
            angle2pwm(PWM_OUT_3_BIT,PWM_OUT_3,i);
            delay(100);
        }
        delay(1000);*/
    }
}

void peripheralStartup(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)){;}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)){;}
}

void delay(uint32_t time){
    SysCtlDelay((CLOCK/3000) * time);
}

void ConfigureServo(uint32_t pwmGen, uint32_t pwmOutBit, uint32_t gpioBase, uint32_t gpioPin, uint32_t gpioPinConfig, uint32_t targetPeriod){
    
    uint32_t pwmPeriod = SysCtlClockGet() / targetPeriod;

    GPIOPinConfigure(gpioPinConfig);
    GPIOPinTypePWM(gpioBase, gpioPin);

    PWMGenConfigure(PWM0_BASE, pwmGen, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, pwmGen, pwmPeriod);

    PWMGenEnable(PWM0_BASE, pwmGen);
    PWMOutputState(PWM0_BASE,pwmOutBit,false);
}

void angle2pwm(uint32_t pwmOutBit, uint32_t pwmOut, float angle){
    uint32_t pulseWidth = minPulseWidth + ((angle + 90) * (maxPulseWidth - minPulseWidth)) / 180;
    PWMOutputState(PWM0_BASE, pwmOutBit, true);
    PWMPulseWidthSet(PWM0_BASE, pwmOut, pulseWidth);
}