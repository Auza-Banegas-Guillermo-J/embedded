/* 
Guillermo Javier Auza Banegas
Embedded systems II
IMT UCB 2024 S2
*/

// Libraries
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

//Macros
#define CLOCK 120000000
#define PINS (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)

volatile int32_t reg_val;

void Delay(uint32_t);
void peripheralStartup(void);

void interruptGuille(void);

void gpioOn(uint32_t, uint32_t);
void gpioOff(uint32_t, uint32_t);
void gpioReset(uint32_t);

uint32_t g_ui32SysClock;
uint32_t g_ui32Flags;

uint32_t count = 0;
int32_t pinMatrix[4] = {GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_4, GPIO_PIN_0};
int32_t portMatrix[4] = {GPIO_PORTN_BASE, GPIO_PORTN_BASE, GPIO_PORTF_BASE, GPIO_PORTF_BASE};

int main(void)
{
    uint32_t miliseconds = 500;
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);

    peripheralStartup();

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,GPIO_PIN_1|GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0|GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0|GPIO_PIN_1,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/1000)*miliseconds - 1);

    TimerIntRegister(TIMER0_BASE, TIMER_A, interruptGuille);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntPrioritySet(INT_TIMER0A, 0);

    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    while(1){
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)==0){
            IntMasterDisable();
            miliseconds = 500;
            TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/1000)*miliseconds - 1);
            IntMasterEnable();
        }
        else if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1)==0){
            IntMasterDisable();
            miliseconds = 2000;
            TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/1000)*miliseconds - 1);
            IntMasterEnable();;
        }
    }
}

void Delay(uint32_t time){
	SysCtlDelay((CLOCK/3000) * time);
}

void peripheralStartup(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){;}
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
    for (int bit = 0; bit < 4; bit++) {
        if (count & (1 << bit)) {
            gpioOn(portMatrix[3-bit], pinMatrix[3-bit]);
        }
        else {
            gpioOff(portMatrix[3-bit], pinMatrix[3-bit]);
        }
    }
    IntMasterDisable();
    count = (count<15) ? (count + 1) : 0;
    IntMasterEnable();
}