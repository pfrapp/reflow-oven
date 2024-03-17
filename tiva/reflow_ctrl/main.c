#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#define TEMPERATURE_SENSOR_PERIPHERAL_I2C_ADDRESS 0x76
#define TEMPERATURE_SENSOR_REGISTER_WHO_AM_I 0xD0


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

int readIMURegister(const uint8_t ui8_register, uint8_t *ui8_value) {

    uint32_t ui32_received = 0;

    // Set to false as we want to write to the peripheral.
    I2CMasterSlaveAddrSet(I2C1_BASE, TEMPERATURE_SENSOR_PERIPHERAL_I2C_ADDRESS, false);
    I2CMasterDataPut(I2C1_BASE, ui8_register);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    // Note: You need to either use "I2CMasterBusy" (without the word "Bus")
    // _or_ use "I2CMasterBusBusy" in conjunction with waiting via "SysCtlDelay".
    // Using "I2CMasterBusBusy" does not work --> you would end up performing
    // a reading operation only (without telling the peripheral which register
    // to read in the first place).
    while(I2CMasterBusy(I2C1_BASE)) {}

    // Now read the value.
    I2CMasterSlaveAddrSet(I2C1_BASE, TEMPERATURE_SENSOR_PERIPHERAL_I2C_ADDRESS, true);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Note that we need to wait here for the uC to actually receive
    // the data via the I2C bus before calling "I2CMasterDataGet".
    while(I2CMasterBusy(I2C1_BASE)) {}

    ui32_received = I2CMasterDataGet(I2C1_BASE);

    // Note that we read from the bus in any case.
    // We just do not return the value if it is not wanted.
    if (!ui8_value) {
        return -1;
    }

    *ui8_value = (uint8_t) (0x000000FF & ui32_received);

    return 0;
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

    //
    // Enable I2C (that is, I2C1) on the following pins
    // - SCL: PA6 (clock, yellow)
    // - SDA: PA7 (data, green)
    //
    // First, enable and configure the GPIO pins, then enable and configure the I2C interface.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1)) {}
    I2CMasterEnable(I2C1_BASE);
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);

    //
    // Read out the Who-Am-I register
    //
    uint8_t who_am_i_register_value = 0;
    readIMURegister(TEMPERATURE_SENSOR_REGISTER_WHO_AM_I, &who_am_i_register_value);
    SysCtlDelay(SysCtlClockGet() / 3 / 4000);

    if (who_am_i_register_value != 0x58) {
        turnOnRedLed();
        turnOffGreenLed();
        turnOffBlueLed();
        // If we get here, there is a problem with the IMU or the
        // IMU communication.
        // Signal this fatal error with a red LED and halt the program.
        while(1) { }
    } else {
        // All good, halt nevertheless.
        turnOffRedLed();
        turnOnGreenLed();
        turnOffBlueLed();
        while(1) { }
    }


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
