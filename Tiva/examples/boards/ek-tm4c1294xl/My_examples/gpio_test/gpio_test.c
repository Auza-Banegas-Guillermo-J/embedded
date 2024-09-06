/* 
Guillermo Javier Auza Banegas
Embedded systems II
IMT UCB 2024 S2
*/

// Libraries
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

//Macros
#define CLOCK 120000000
#define NLEDS (GPIO_PIN_0 | GPIO_PIN_1) 
#define FLEDS (GPIO_PIN_0 | GPIO_PIN_4)
#define PINS (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)

//GPIOPinWrite(uint32_t ui32Port, uint8_t ui8Pins, uint8_t ui8Val)
//#define GPIO_PIN_0              0x00000001 

/*
#if defined(codered) || defined(gcc) || defined(sourcerygxx)
void __attribute__((naked))
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysCtlDelay\n"
          "    bx      lr");
}
*/

/*SysCtlPeripheralEnable(uint32_t ui32Peripheral)
{
    //
    // Check the arguments.
    //
    ASSERT(_SysCtlPeripheralValid(ui32Peripheral));

    //
    // Enable this peripheral.
    //
    HWREGBITW(SYSCTL_RCGCBASE + ((ui32Peripheral & 0xff00) >> 8),
              ui32Peripheral & 0xff) = 1;
}*/

//#define SYSCTL_PERIPH_GPION     0xf000080c  // GPIO N

//#define GPIO_PORTN_BASE         0x40064000  // GPIO Port N
//{ GPIO_PORTN_BASE, INT_GPION_TM4C123 },

// 0b binary constants arent available in gcc compiler O_O

void Delay(uint32_t);
void peripheralStartup(void);
void gpioOn(uint32_t, uint32_t);
void gpioOff(uint32_t, uint32_t);

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

int32_t reg_val;

int main(void)
{
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), CLOCK);
    peripheralStartup();
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
    while(1)
    {
        gpioOn(GPIO_PORTN_BASE,GPIO_PIN_1);
        Delay(500);
        gpioOn(GPIO_PORTN_BASE,GPIO_PIN_0);
        Delay(500);
        gpioOn(GPIO_PORTF_BASE,GPIO_PIN_4);
        Delay(500);
        gpioOn(GPIO_PORTF_BASE,GPIO_PIN_0);
        Delay(500);
        gpioOff(GPIO_PORTF_BASE,GPIO_PIN_0);
        Delay(500);
        gpioOff(GPIO_PORTF_BASE,GPIO_PIN_4);
        Delay(500);
        gpioOff(GPIO_PORTN_BASE,GPIO_PIN_0);
        Delay(500);
        gpioOff(GPIO_PORTN_BASE,GPIO_PIN_1);
        Delay(500);
    }
}

void Delay(uint32_t time){
	SysCtlDelay((CLOCK/3000) * time);
}

void peripheralStartup(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)){;}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){;}
}

void gpioOn(uint32_t port, uint32_t pin){
    reg_val = GPIOPinRead(port,PINS);
    GPIOPinWrite(port, PINS, reg_val|pin);
}

void gpioOff(uint32_t port, uint32_t pin){
    reg_val = GPIOPinRead(port,PINS);
    GPIOPinWrite(port, PINS, reg_val&(~pin));
}

bool gpioRead(uint32_t port, uint32_t pin){
    reg_val = GPIOPinRead(port,PINS);
    
}