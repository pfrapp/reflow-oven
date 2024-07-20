#ifndef USB_SERIAL_DATA_H
#define USB_SERIAL_DATA_H

#include <stdint.h>

// Message from Tiva to PC.
typedef struct usb_serial_data_tiva_to_pc_ {

    // Trimming values for the temperature
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    // Trimming values for the pressure
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    // 12 x 2 byte = 6 x 4 byte --> aligned on 32-bit boundary.

    // Temperature measurement values
    // 0xFA, 0xFB, and part of 0xFC
    uint8_t temp_msb;    // 0xFA
    uint8_t temp_lsb;    // 0xFB
    uint8_t temp_xlsb;    // 0xFC

    // Pressure measurement values
    // 0xF7, 0xF8, and part of 0xF9
    uint8_t press_msb;    // 0xF7
    uint8_t press_lsb;    // 0xF8
    uint8_t press_xlsb;    // 0xF9

    // Power mode
    uint8_t power_mode;

    // 7 bytes, so adding 1 padding byte to align on 32-bit boundary.
    uint8_t padding1;

    // Amplified thermocouple voltage.
    uint32_t amp_thermocouple_voltage;

    // Value read from the digital thermocouple (only lower 16 bits are valid).
    uint32_t digital_amp_thermocouple;


} usb_serial_data_tiva_to_pc;


// Message from PC to Tiva.
typedef struct usb_serial_data_pc_to_tiva_ {

    // 16-bit PWM value (0x0000 to 0xFFFF) for the controller.
    uint16_t pwm_controller;

    // Any other business (status).
    uint16_t status;

} usb_serial_data_pc_to_tiva;

#endif // #ifndef USB_SERIAL_DATA_H

