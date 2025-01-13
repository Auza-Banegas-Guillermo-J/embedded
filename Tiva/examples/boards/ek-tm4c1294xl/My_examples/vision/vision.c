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

//Macros
#define CLOCK 120000000
#define PINS (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)

volatile int32_t reg_val;
uint32_t value;

int32_t width = 135;
char data[100] = {'0'};

void manualGuilleInterrupt();

void interruptGuille(void);
void ConfigureUART(void);

void gpioOn(uint32_t, uint32_t);
void gpioOff(uint32_t, uint32_t);
void gpioReset(uint32_t);

void delay(uint32_t);
void peripheralStartup(void);

uint32_t g_ui32SysClock;
uint32_t g_ui32Flags;

short toggle;
short flag=0;

int main(void)
{
    uint32_t miliseconds = 1000;
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);

    peripheralStartup();
    ConfigureUART();
    
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,GPIO_PIN_1|GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    GPIOPinTypeGPIOInput(GPIO_PORTL_BASE,GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_4,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/1000)*miliseconds - 1);

    TimerIntRegister(TIMER0_BASE, TIMER_A, interruptGuille);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntPrioritySet(INT_TIMER0A, 0);
    TimerEnable(TIMER0_BASE, TIMER_A);

    IntMasterEnable();

    while(1){
        /*UARTgets(data, 100);
        UARTprintf(data);
        remove_chars(data);
        switch (data[0]){
            case 'q':
                flag=0;
                break;
            case 'w':
                flag=1;
                break;
            default:
                break;
        }*/
        if(!(GPIOPinRead(GPIO_PORTL_BASE,GPIO_PIN_4)==0))
        {
            flag=1;
            gpioOn(GPIO_PORTN_BASE,GPIO_PIN_1);
        }
        else{
            flag=0;
            gpioOff(GPIO_PORTN_BASE,GPIO_PIN_1);
        }
        delay(10);
    }
}

void delay(uint32_t time){
    SysCtlDelay((CLOCK/3000) * time);
}

void peripheralStartup(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){;}
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
    IntMasterDisable();
    if(flag){
        if((toggle%4)==0){
            gpioOn(GPIO_PORTF_BASE,GPIO_PIN_1);
            toggle=0;
        }
        else{
            gpioReset(GPIO_PORTF_BASE);
        }
    }
    toggle+=1;
    IntMasterEnable();
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

void remove_chars(char* data){
    int i, j = 0;
    int len = strlen(data);
    
    for (i = 0; i < len; i++) {
        if ((data[i]!='\r') && (data[i]!='\n')){
            data[j++] = data[i];
        }
    }
    data[j] = '\0';
}