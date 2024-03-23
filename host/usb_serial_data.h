#ifndef USB_SERIAL_DATA_H
#define USB_SERIAL_DATA_H

#include <stdint.h>

// Message from Tiva to PC.
typedef struct usb_serial_data_tiva_to_pc_ {

    // Quadrature position.
    uint32_t quadrature_position;

    // Quadrature velocity.
    // The MSB is used for the direction that the QEI gives.
    uint32_t quadrature_velocity;

    // Accelerometer.
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;

    // Gyroscope.
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;

    // Any other business (status).
    uint16_t status;

} usb_serial_data_tiva_to_pc;


// Message from PC to Tiva.
typedef struct usb_serial_data_pc_to_tiva_ {

    // Motor direction (MSB) and speed.
    uint16_t motor;

    // Any other business (status).
    uint16_t status;

} usb_serial_data_pc_to_tiva;

#endif // #ifndef USB_SERIAL_DATA_H

