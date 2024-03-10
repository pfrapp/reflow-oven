#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"



//*****************************************************************************
//
// Internal functions.
//
//*****************************************************************************
static void turnOnRedLed() {
    // Red: PF1
    uint8_t pin = GPIO_PIN_1;
    GPIOPinWrite(GPIO_PORTF_BASE, pin, pin);
}

static void turnOffRedLed() {
    uint8_t pin = GPIO_PIN_1;
    GPIOPinWrite(GPIO_PORTF_BASE, pin, 0);
}

static void turnOnGreenLed() {
    // Green: PF3
    uint8_t pin = GPIO_PIN_3;
    GPIOPinWrite(GPIO_PORTF_BASE, pin, pin);
}

static void turnOffGreenLed() {
    uint8_t pin = GPIO_PIN_3;
    GPIOPinWrite(GPIO_PORTF_BASE, pin, 0);
}

static void turnOnBlueLed() {
    // Blue: PF2
    uint8_t pin = GPIO_PIN_2;
    GPIOPinWrite(GPIO_PORTF_BASE, pin, pin);
}

static void turnOffBlueLed() {
    uint8_t pin = GPIO_PIN_2;
    GPIOPinWrite(GPIO_PORTF_BASE, pin, 0);
}

/**
 * main.c
 */
int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    // Green: PF3
    // Blue: PF2
    // Red: PF1
    // For usage with the relay: PF4
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1);
    turnOffRedLed();
    turnOffGreenLed();
    turnOffBlueLed();

    while(true) {
        // Put voltage on pin PF4.
        // Tested via measurement: works.
        SysCtlDelay(SysCtlClockGet() / 2);
        turnOnBlueLed();
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
        SysCtlDelay(SysCtlClockGet() / 2);
        turnOffBlueLed();
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_0);
    }


	return 0;
}
