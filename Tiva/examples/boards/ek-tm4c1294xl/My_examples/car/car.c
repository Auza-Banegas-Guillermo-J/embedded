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

volatile int32_t reg_val;
volatile uint32_t value;

int32_t width = 300;

void Delay(uint32_t);
void peripheralStartup(void);

uint32_t g_ui32SysClock;
uint32_t g_ui32Flags;

int main(void)
{
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);

    peripheralStartup();


    GPIOPinConfigure(GPIO_PF1_M0PWM1);    

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
    
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_0,400);
    PWMGenEnable(PWM0_BASE,PWM_GEN_0);
    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
    ADCSequenceEnable(ADC0_BASE,3);
    ADCIntClear(ADC0_BASE, 3);

    while(1){
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false)){;}
        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, &value);
        width = (int)((((float)(value)/4094) * 400)-1);
        if(width<50){width=50;}
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, width);
        Delay(10);
    }
}

void Delay(uint32_t time){
    SysCtlDelay((CLOCK/3000) * time);
}

void peripheralStartup(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){;}
}