/* 
Guillermo Javier Auza Banegas
Embedded systems II
IMT UCB 2024 S2
*/

// Libraries
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"

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

//Macros
#define CLOCK 120000000
#define PINS (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)

volatile int32_t reg_val;
uint32_t value;

int32_t width = 135;

void interruptGuille(void);
void ConfigureUART(void);

void gpioOn(uint32_t, uint32_t);
void gpioOff(uint32_t, uint32_t);
void gpioReset(uint32_t);

void Delay(uint32_t);
void peripheralStartup(void);

uint32_t g_ui32SysClock;
uint32_t g_ui32Flags;

int main(void)
{
    uint32_t miliseconds = 100;
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);

    peripheralStartup();
    ConfigureUART();
    
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,GPIO_PIN_1|GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0|GPIO_PIN_1,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);    

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
    
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_0,400);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,400);

    PWMGenEnable(PWM0_BASE,PWM_GEN_0);
    PWMGenEnable(PWM0_BASE,PWM_GEN_1);

    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);
    PWMOutputState(PWM0_BASE,PWM_OUT_2_BIT,true);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
    ADCSequenceEnable(ADC0_BASE,3);
    ADCIntClear(ADC0_BASE, 3);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/1000)*miliseconds - 1);

    TimerIntRegister(TIMER0_BASE, TIMER_A, interruptGuille);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntPrioritySet(INT_TIMER0A, 0);

    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A);

    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_0);
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_1);
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_2);
    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_3);

    while(1){
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false)){;}
        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, &value);
        width = (int)((((float)(value)/4094) * 400)-1);
        if(width<50){width=50;}
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, width);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width);
        Delay(10);
    }
}

void Delay(uint32_t time){
    SysCtlDelay((CLOCK/3000) * time);
}

void peripheralStartup(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){;}
}


void gpioOn(uint32_t port, uint32_t pin){
    reg_val = GPIOPinRead(port,PINS);
    GPIOPinWrite(port, PINS, reg_val|pin);
}

void gpioOff(uint32_t port, uint32_t pin){
    reg_val = GPIOPinRead(port,PINS);
    GPIOPinWrite(port, PINS, reg_val&(~pin));
}

void gpioReset(uint32_t port){
    reg_val = GPIOPinRead(port,PINS);
    GPIOPinWrite(port, PINS, 0x0);
}

void interruptGuille(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)==0){
        IntMasterDisable();
        UARTprintf("Motor 1\n\r");
        IntMasterEnable();
    }
    else if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1)==0){
        IntMasterDisable();
        UARTprintf("Motor 2\n\r");
        IntMasterEnable();    
    }
}

void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

/*
ADELANTE:
    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_0);
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_1);
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_2);
    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_3);
ATRAS:
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_0);
    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_1);
    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_2);
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_3);
DERECHA:
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_0);
    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_1);
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_2);
    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_3);
IZQUIERDA:
    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_0);
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_1);
    gpioOff(GPIO_PORTL_BASE,GPIO_PIN_2);
    gpioOn(GPIO_PORTL_BASE,GPIO_PIN_3);
ALTO:
    gpioReset(GPIO_PORTL_BASE);
*/