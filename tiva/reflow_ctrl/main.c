#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/adc.h"
#include "driverlib/ssi.h"

// USB from driverlib
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
// Own USB files
#include "usb_serial_structs.h"
#include "../../host/usb_serial_data.h"

// For USB, also link the usblib and put the USB0DeviceIntHandler in the
// interrupt vector table (you do not need to define USB0DeviceIntHandler,
// it is likely defined in the usblib).
// If you fail to add USB0DeviceIntHandler to the ISR, you end up in the
// default interrupt.

//
// Definitions for the Bosch BMP280.
//
#define TEMPERATURE_SENSOR_PERIPHERAL_I2C_ADDRESS 0x76
#define TEMPERATURE_SENSOR_REGISTER_WHO_AM_I 0xD0
#define TEMPERATURE_SENSOR_TEMPERATURE_MSB 0xFA
#define TEMPERATURE_SENSOR_PRESSURE_MSB 0xF7
#define TEMPERATURE_SENSOR_REGISTER_CONTROL_REGISTER 0xF4
#define TEMPERATURE_SENSOR_REGISTER_CONFIG_REGISTER 0xF5


//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;

// Global flag indicating that a USB configuration has been set.
static volatile bool g_bUSBConfigured = false;
// The USB data packet that we are receiving.
static volatile usb_serial_data_pc_to_tiva usb_packet_recv;
// The USB data packet that we are sending.
static volatile usb_serial_data_tiva_to_pc usb_packet_sent;

// 16-bit PWM value (0x0000 to 0xFFFF) for the controller
static volatile uint16_t ui16_PWM_Controller = 0;


//**********
// ADC
//**********
#define ADC_SEQUENCE_NUM    3 // was 0
#define ADC_SAMPLE_BUF_SIZE 1 // We're just using one sample


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

int writeIMURegister(const uint8_t ui8_register, const uint8_t ui8_value) {

    //
    // See https://forum.digikey.com/t/i2c-communication-with-the-ti-tiva-tm4c123gxl/13451
    //

    // Set the third argument to false, as we want to write to the peripheral.
    // The third argument is the read/not write flag.
    // read/not write = false <==> not read/write = true
    I2CMasterSlaveAddrSet(I2C1_BASE, TEMPERATURE_SENSOR_PERIPHERAL_I2C_ADDRESS, false);

    I2CMasterDataPut(I2C1_BASE, ui8_register);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C1_BASE)) { } // note: not BusBusy, but Busy

    I2CMasterDataPut(I2C1_BASE, ui8_value);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C1_BASE)) { } // note: not BusBusy, but Busy

    return 0;
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



//! Read N consecutive registers.
int readConsecutiveIMURegisters(const uint8_t ui8_register, const unsigned int N, uint8_t *ui8_values) {
    
    uint32_t ui32_received = 0;
    unsigned int idx = 0;
    unsigned int loop_counter = 0;

    // Opposed to readIMURegister, not not actually issue any I2C bus commands
    // if the given values pointer is NULL.
    // This means that this very function readConsecutiveIMURegisters can _not_
    // be used to just look at the bus traffic on the scope when
    // setting ui8_values to NULL.
    if (!ui8_values) {
        return -1;
    }

    //
    // See https://forum.digikey.com/t/i2c-communication-with-the-ti-tiva-tm4c123gxl/13451
    //
    
    // If we are only requested to read N = 1 registers, dispatch
    // to the corresponding function.
    if (N == 1) {
        return readIMURegister(ui8_register, ui8_values);
    }


    // Set the third argument to false, as we want to write to the peripheral.
    // The third argument is the read/not write flag.
    // read/not write = false <==> not read/write = true
    I2CMasterSlaveAddrSet(I2C1_BASE, TEMPERATURE_SENSOR_PERIPHERAL_I2C_ADDRESS, false);
    I2CMasterDataPut(I2C1_BASE, ui8_register);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    while(I2CMasterBusy(I2C1_BASE)) {}

    //
    // Now read the values.
    //
    idx = 0;

    I2CMasterSlaveAddrSet(I2C1_BASE, TEMPERATURE_SENSOR_PERIPHERAL_I2C_ADDRESS, true);

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C1_BASE)) {}
    ui32_received = I2CMasterDataGet(I2C1_BASE);
    ui8_values[idx] = (uint8_t) (0x000000FF & ui32_received);
    idx += 1;

    for (loop_counter = 1; loop_counter < (N-1); loop_counter++) {
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C1_BASE)) {}
        ui32_received = I2CMasterDataGet(I2C1_BASE);
        ui8_values[idx] = (uint8_t) (0x000000FF & ui32_received);
        idx += 1;
    }

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C1_BASE)) {}
    ui32_received = I2CMasterDataGet(I2C1_BASE);
    ui8_values[idx] = (uint8_t) (0x000000FF & ui32_received);
    idx += 1;

    return 0;
}



//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
// CDC: Communications device class.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host. These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
ControlHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    uint32_t ui32IntsOff;

    //
    // Which event are we being asked to process?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
            g_bUSBConfigured = true;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            //
            // Tell the main loop to update the display.
            //
            ui32IntsOff = IntMasterDisable();
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            if(!ui32IntsOff)
            {
                IntMasterEnable();
            }

            // Turn on (green) LED.
            turnOnGreenLed();
            break;

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
            g_bUSBConfigured = false;
            ui32IntsOff = IntMasterDisable();
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            if(!ui32IntsOff)
            {
                IntMasterEnable();
            }

            // Turn off (green) LED.
            turnOffGreenLed();
            break;

        //
        // Return the current serial communication parameters.
        // --> Removed.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
            break;

        //
        // Set the current serial communication parameters.
        // --> Removed.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
            break;

        //
        // Set the current serial communication parameters.
        // --> Removed.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
            break;

        //
        // Send a break condition on the serial line.
        // --> Removed.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
            break;

        //
        // Clear the break condition on the serial line.
        // --> Removed.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
            break;

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }

    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{

    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // A new packet has been received.
        //
        // Todo: Only set a value and read the buffer outside of the interrupt.
        case USB_EVENT_RX_AVAILABLE:
        {
            // See https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/497724/usb_dev_serial-example---please-explain-how-it-is-working
            // The received packet is in the Rx buffer.
            // Read buffer with USBBufferRead(...)
            uint32_t ui32Read;

            ui32Read = USBBufferRead((tUSBBuffer *) &g_sRxBuffer, &usb_packet_recv, sizeof(usb_packet_recv));

            //
            // Here you can actually do something with the
            // data that has been received from the host.
            // ...
            //
            ui16_PWM_Controller = usb_packet_recv.pwm_controller;

            // Send the current IMU data back.
            USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, &usb_packet_sent, sizeof(usb_packet_sent));

            break;
        }

        //
        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the UART is currently idle or 1 if it is
        // in the process of transmitting something. The actual number of
        // bytes in the UART FIFO is not important here, merely whether or
        // not everything previously sent to us has been transmitted.
        //
        case USB_EVENT_DATA_REMAINING:
        {
            //
            // Get the number of bytes in the buffer and add 1 if some data
            // still has to clear the transmitter.
            //
            // Todo: This should actually only return 0 if there are no data
            //       left to process. Have a look again in the USB Lib PDF.
            //
            return 0;
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }

    return(0);
}


/**
 * main.c
 */
int main(void)
{
    uint32_t ui32TxCount;
    uint32_t ui32RxCount;
    uint8_t ui8TemperatureValues[3];
    uint8_t ui8PressureValues[3];
    uint8_t ui8ControlRegisterValue;
    uint8_t ui8ConfigRegisterValue;
    uint8_t ui8TrimmingValues[24];

    // Do we want to use the BMP280?
    const int bUseTemperatureSensor = 1;

    // We use 16 bit out of those 32 bit.
    uint32_t ui32DigitalThermocoupleData;

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Wait for the GPIO module to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

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

    // Unlock PF4 for GPIO use
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_4;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // Enable the PF4 pin to drive the LED.
    // Configure PF4 as a GPIO output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);

    // Set the drive strength to 8 mA and configure the pin as standard push-pull
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);



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
    // Read out the Who-Am-I register of the BMP280 temperature sensor
    //
    if (bUseTemperatureSensor) {
        uint8_t who_am_i_register_value = 0;
        readIMURegister(TEMPERATURE_SENSOR_REGISTER_WHO_AM_I, &who_am_i_register_value);
        SysCtlDelay(SysCtlClockGet() / 3 / 4000);

        if (who_am_i_register_value != 0x58) {
            turnOnRedLed();
            turnOffGreenLed();
            turnOffBlueLed();
            // If we get here, there is a problem with the BMP280 or the
            // BMP280 communication.
            // Signal this fatal error with a red LED and halt the program.
            while(1) { }
        } else {
            // All good.
            turnOffRedLed();
            turnOnGreenLed();
            turnOffBlueLed();
            // while(1) { }
        }
    }

    // The temperature sensor is in sleep mode after power up,
    // see BMP280 data sheet, page 15 (of 49).
    // We need to set it in normal mode.
    // Read the register first in order to only change the power mode
    // of the control register.
    // readIMURegister(TEMPERATURE_SENSOR_REGISTER_CONTROL_REGISTER, &ui8ControlRegisterValue);
    // ui8ControlRegisterValue |= 0x03;
    // writeIMURegister(TEMPERATURE_SENSOR_REGISTER_CONTROL_REGISTER, ui8ControlRegisterValue);

    // We also need to set the measurement and filter options (see BMP280 data sheet,
    // page 16 of 49).
    // Standard resolution (page 19 of 49):
    // - osrs_p: x4 --> 011 = 0x03
    // - osrs_t: x1 --> 001 = 0x01
    // - IIR filter coeff.: 16 = 0x10
    //
    // If we only enable the normal power mode but not the other configuration,
    // the sensor sends 0x80000
    ui8ControlRegisterValue = (0x01 << 5) | (0x03 << 2) | 0x03;
    ui8ConfigRegisterValue = 0x10 << 2;
    if (bUseTemperatureSensor) {
        writeIMURegister(TEMPERATURE_SENSOR_REGISTER_CONTROL_REGISTER, ui8ControlRegisterValue);
        writeIMURegister(TEMPERATURE_SENSOR_REGISTER_CONFIG_REGISTER, ui8ConfigRegisterValue);
    }

    // Read the trimming values (calibration values).
    // The LSB comes first.
    if (bUseTemperatureSensor) {
        readConsecutiveIMURegisters(0x88, 24, ui8TrimmingValues);
        usb_packet_sent.dig_T1 = (uint16_t) ((ui8TrimmingValues[0] << 0) + (ui8TrimmingValues[1] << 8));
        usb_packet_sent.dig_T2 = (int16_t) ((ui8TrimmingValues[2] << 0) + (ui8TrimmingValues[3] << 8));
        usb_packet_sent.dig_T3 = (int16_t) ((ui8TrimmingValues[4] << 0) + (ui8TrimmingValues[5] << 8));

        usb_packet_sent.dig_P1 = (uint16_t) ((ui8TrimmingValues[6] << 0) + (ui8TrimmingValues[7] << 8));
        usb_packet_sent.dig_P2 = (int16_t) ((ui8TrimmingValues[8] << 0) + (ui8TrimmingValues[9] << 8));
        usb_packet_sent.dig_P3 = (int16_t) ((ui8TrimmingValues[10] << 0) + (ui8TrimmingValues[11] << 8));
        usb_packet_sent.dig_P4 = (int16_t) ((ui8TrimmingValues[12] << 0) + (ui8TrimmingValues[13] << 8));
        usb_packet_sent.dig_P5 = (int16_t) ((ui8TrimmingValues[14] << 0) + (ui8TrimmingValues[15] << 8));
        usb_packet_sent.dig_P6 = (int16_t) ((ui8TrimmingValues[16] << 0) + (ui8TrimmingValues[17] << 8));
        usb_packet_sent.dig_P7 = (int16_t) ((ui8TrimmingValues[18] << 0) + (ui8TrimmingValues[19] << 8));
        usb_packet_sent.dig_P8 = (int16_t) ((ui8TrimmingValues[20] << 0) + (ui8TrimmingValues[21] << 8));
        usb_packet_sent.dig_P9 = (int16_t) ((ui8TrimmingValues[22] << 0) + (ui8TrimmingValues[23] << 8));
    }

    //
    // Enable SPI in order to read out the digital thermocouple MAX6675.
    //

    // Enable the peripherals for GPIOB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    // Configure the pin muxing for SSI2 functions on port B pins
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    //GPIOPinConfigure(GPIO_PB5_SSI2FSS);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);

    // Configure PB5 as a GPIO output for manual CS control
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

    // Enable push-pull on PB5
    // (same as for PF4)
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    // Configure the pins for SSI functionality
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);

    // Enable the SSI2 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2)) {}

    // Configure the SSI2 settings
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                    SSI_MODE_MASTER, 1000000, 16); // 1 MHz, 16-bit data

    // Enable the SSI2 module
    SSIEnable(SSI2_BASE);



    //
    // Enable USB.
    // The data pins are on PD4 and PD5.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // USB is not configured initially.
    g_bUSBConfigured = false;

    // Initialize the transmit and receive buffers.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    //
    // Set the USB stack mode to Device mode with VBUS monitoring.
    //
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
    USBDCDCInit(0, &g_sCDCDevice);

    //
    // Clear our local byte counters.
    //
    ui32RxCount = 0;
    ui32TxCount = 0;

    //
    // Read in voltage from an ADC pin (0 to 3.3 V).
    // Note: Ensure that the input voltage actually
    // is between 0 and 3.3V -- neither lower nor higher.
    //
    // Enable ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)); // Wait for ADC0 to be ready

    // Enable GPIO port E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); // Wait for GPIOE to be ready

    // Configure GPIO pin PE3 as analog input
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // Configure ADC0 sequencer 0 to capture from channel 0 (corresponding to PE3)
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE_NUM, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE_NUM, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE_NUM);

    uint32_t adcSampleBuf[ADC_SAMPLE_BUF_SIZE]; // Buffer to store ADC samples



    while(true) {
        float pwm_percentage = 0.0f;
        uint32_t clock_ticks_on = 0u;
        uint32_t clock_ticks_off = 0u;

        // Trigger ADC conversion
        ADCProcessorTrigger(ADC0_BASE, ADC_SEQUENCE_NUM);

        // Wait for ADC conversion to complete
        while (!ADCIntStatus(ADC0_BASE, ADC_SEQUENCE_NUM, false));

        // Read ADC values
        ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE_NUM, adcSampleBuf);

        // Your ADC value is now in adcSampleBuf[0]

        // Example: Convert ADC value to voltage assuming Vref is 3.3V and ADC resolution is 12 bits
        // float voltage = ((float)adcSampleBuf[0] / 4096) * 3.3;


        // Put voltage on pin PF4.
        // Tested via measurement: works.
        pwm_percentage = 100.0f * ui16_PWM_Controller / 0xFFFF;
        clock_ticks_on = pwm_percentage * 0.01f * SysCtlClockGet() / 5;
        clock_ticks_off = (100.0f - pwm_percentage) * 0.01f * SysCtlClockGet() / 5;
        
        // On
        turnOnBlueLed();
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
        if (clock_ticks_on > 0) {
            // Do not pass 0, otherwise it waits forever.
            SysCtlDelay(clock_ticks_on);
        }
        // Off
        turnOffBlueLed();
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_0);
        if (clock_ticks_off > 0) {
            SysCtlDelay(clock_ticks_off);
        }

        //
        // -------------------------------------------------------------------------------------------
        // Read the BMP280
        //
        if (bUseTemperatureSensor) {

        // Read the power mode
        readIMURegister(TEMPERATURE_SENSOR_REGISTER_CONTROL_REGISTER, &ui8ControlRegisterValue);
        usb_packet_sent.power_mode = ui8ControlRegisterValue & 0x03;


        // Read the temperature measurement from the sensor
        readConsecutiveIMURegisters(TEMPERATURE_SENSOR_TEMPERATURE_MSB, 3, ui8TemperatureValues);
        // Read the pressure measurement from the sensor
        readConsecutiveIMURegisters(TEMPERATURE_SENSOR_PRESSURE_MSB, 3, ui8PressureValues);
                    
        }
        //
        // -------------------------------------------------------------------------------------------
        //


        // Toggle down and up before actually trying to read the MAX6675
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
        SysCtlDelay(SysCtlClockGet() / (100000 * 3));
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);  // <-- This should now really be 3.3V, not 1.0V
        SysCtlDelay(SysCtlClockGet() / (100000 * 3));

        // Read the digital thermocouple MAX6675 (blocking).
        //SSIDataGet(SSI2_BASE, &ui32DigitalThermocoupleData);
        ui32DigitalThermocoupleData = 0;
        // Assert CS (active low)
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);

        // Wait for a short delay (e.g., 23 us) for the MAX6675 to prepare data
        SysCtlDelay(SysCtlClockGet() / (100000 * 3));

        // Sending dummy data to generate clock signals
        SSIDataPut(SSI2_BASE, 0x0000);

        // Waiting for transmission to complete
        while (SSIBusy(SSI2_BASE)) {}

        SSIDataGet(SSI2_BASE, &ui32DigitalThermocoupleData);

//        while (SSIBusy(SSI2_BASE)) {}

        // Wait for a short delay (e.g., 23 us)
        SysCtlDelay(SysCtlClockGet() / (100000 * 3));

        // De-assert CS (inactive high)
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);

        // Toggle down and up again
        SysCtlDelay(SysCtlClockGet() / (100000 * 3));
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
        SysCtlDelay(SysCtlClockGet() / (100000 * 3));
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);

        // Hinweis: Das Datenblatt des MAX6675 sagt:
        // "In order to allow the operation of the
        // open thermocouple detector, T- must be grounded."
        // Das heisst T- muss auf Masse gelegt werden, damit der Chip
        // ueberhaupt in der Lage ist zu erkennen ob die connection offen ist.
        // Vgl. auch
        // https://www.mikrocontroller.net/topic/221529
        // Das erklaert warum bei mir am Anfang Bit D2 noch auf 0 war, d.h.
        // die Offenheit (bzw. Brucherkennung) nicht erkannt hat.
        // Auf meiner Platine ist T- ja nicht auf GND.
        // Es tat dann aber, als ich T- mit GND verbunden habe.

        // Process the received data
        // For example, you can store it in a buffer or print it

        // Fill out the USB-struct
        usb_packet_sent.temp_msb = ui8TemperatureValues[0];
        usb_packet_sent.temp_lsb = ui8TemperatureValues[1];
        usb_packet_sent.temp_xlsb = ui8TemperatureValues[2];
        usb_packet_sent.press_msb = ui8PressureValues[0];
        usb_packet_sent.press_lsb = ui8PressureValues[1];
        usb_packet_sent.press_xlsb = ui8PressureValues[2];

        // Add the ADC0 on PE3 voltage of the amplified thermo couple
        // to the USB struct
        usb_packet_sent.amp_thermocouple_voltage = adcSampleBuf[0];

        usb_packet_sent.digital_amp_thermocouple = ui32DigitalThermocoupleData;

        //
        // Have we been asked to update the status display?
        //
        if(g_ui32Flags & COMMAND_STATUS_UPDATE)
        {
            //
            // Clear the command flag
            //
            IntMasterDisable();
            g_ui32Flags &= ~COMMAND_STATUS_UPDATE;
            IntMasterEnable();
        }
    }


	return 0;
}
