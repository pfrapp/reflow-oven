//
// P. Rapp, Mar-23, 2024
// Copied over from the self-balancing robot host main file.
//

// C library headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // for memset

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <math.h>

#include "usb_serial_data.h"
#include "logging.h"
#include "timing.h"
#include "usb_serial.h"
#include "bmp.h"



int main(void)
{

    // You can find this name by issuing
    // $ system_profiler SPUSBDataType
    // on the terminal.
    // Then you find the serial number of the respective USB device,
    // for which you can grep in /dev
    // $ cd /dev
    // $ ll | grep -ie 'the_serial_number'
    //
    // Note that the In-Circuit Debug Interface has
    // Serial Number: 0E230906
    // cu.usbmodem0E2309061
    //
    // For communicating, you want to use the actual USB device.
    // (Change switch from Debug to Device).
    const char virtualCOMPortName[] = "/dev/cu.usbmodem123456781";

    // The USB data packet that we are sending.
    usb_serial_data_pc_to_tiva usb_packet_to_tiva;
    int speed_percent;
    int direction;

    // The USB data packet that we are receiving from the Tiva.
    usb_serial_data_tiva_to_pc usb_packet_from_tiva;

    int ii;
    int err;

    // Milliseconds since epoch at program start.
    // This means after the time that the connection to the Tiva has been established.
    long long milliseconds_since_epoch_at_start;
    long long current_milliseconds_since_epoch;
    long long diff_ms;
    int sample_index;
    int sample_time_ms;
    int max_runtime_seconds;

    // File for logging measurement signals to.
    FILE *log_fid;
    int first_log_call;

    // BMP structures
    struct bmp2_dev bmp_device;
    struct bmp2_uncomp_data raw_bmp_data;
    struct bmp2_data physical_bmp_data;


    // Init logging
    log_fid = fopen("signals.log", "w");
    first_log_call = 1;


    printf("*\n");
    printf("* Connecting to the Tiva device via the virtual COM port.\n");
    printf("* COM port name: %s\n", virtualCOMPortName);
    printf("*\n");

    int serial_port = open(virtualCOMPortName, O_RDWR);
    if (serial_port < 0)
    {
        fprintf(stderr, "### Error %i from open()\n", errno);
        fprintf(stderr, "### Make sure the device is plugged in and the switch\n");
        fprintf(stderr, "### is set to DEVICE (not DEBUG)\n");
        fprintf(stderr, "### Exiting!\n");
        return -1;
    }
    printf("Successfully opened the serial port.\n");

    //--------------------------------------------------

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;
    // Configure the tty, return on error.
    err = configure_tty(serial_port, &tty);
    if (err != 0) {
        return 1;
    }

    milliseconds_since_epoch_at_start = getMilliSecondsSinceEpoch();
    // Sample time of 500 ms, corresponding to 2 Hz.
    sample_time_ms = 500;
    sample_index = 0;
    max_runtime_seconds = 10;


    while (1)
    {
        float time_sec;
        float thermocouple_voltage;
        uint16_t digital_thermocouple;


        // Idle (wait) until the sampling interval is over.
        // Perhaps a callback function is more appropriate in the long term.
        do {
            current_milliseconds_since_epoch = getMilliSecondsSinceEpoch();
            diff_ms = current_milliseconds_since_epoch - milliseconds_since_epoch_at_start;
        } while (diff_ms < sample_index*sample_time_ms);

        time_sec = 0.001f * diff_ms;
        printf("--------------------------------\nTime:                 % 8.3f s (of %i s)\n", time_sec, max_runtime_seconds);

        // printf("Power mode: 0x%02X\n", usb_packet_from_tiva.power_mode);
        // printf("Temperature: 0x%02X,0x%02X,0x%02X\n",
        //     usb_packet_from_tiva.temp_msb,
        //     usb_packet_from_tiva.temp_lsb,
        //     usb_packet_from_tiva.temp_xlsb);

        bmp_device.calib_param.dig_t1 = usb_packet_from_tiva.dig_T1;
        bmp_device.calib_param.dig_t2 = usb_packet_from_tiva.dig_T2;
        bmp_device.calib_param.dig_t3 = usb_packet_from_tiva.dig_T3;

        bmp_device.calib_param.dig_p1 = usb_packet_from_tiva.dig_P1;
        bmp_device.calib_param.dig_p2 = usb_packet_from_tiva.dig_P2;
        bmp_device.calib_param.dig_p3 = usb_packet_from_tiva.dig_P3;
        bmp_device.calib_param.dig_p4 = usb_packet_from_tiva.dig_P4;
        bmp_device.calib_param.dig_p5 = usb_packet_from_tiva.dig_P5;
        bmp_device.calib_param.dig_p6 = usb_packet_from_tiva.dig_P6;
        bmp_device.calib_param.dig_p7 = usb_packet_from_tiva.dig_P7;
        bmp_device.calib_param.dig_p8 = usb_packet_from_tiva.dig_P8;
        bmp_device.calib_param.dig_p9 = usb_packet_from_tiva.dig_P9;
        bmp_device.calib_param.dig_p10 = 0; // seems to not be used anyway

        raw_bmp_data.temperature = ((usb_packet_from_tiva.temp_xlsb & 0xF0) >> 4)
                                   | (usb_packet_from_tiva.temp_lsb << 4)
                                   | (usb_packet_from_tiva.temp_msb << (8+4));
        raw_bmp_data.pressure = ((usb_packet_from_tiva.press_xlsb & 0xF0) >> 4)
                                   | (usb_packet_from_tiva.press_lsb << 4)
                                   | (usb_packet_from_tiva.press_msb << (8+4));

        // printf("Trimming values:\n");
        // printf("dig_T1 = %u\n", bmp_device.calib_param.dig_t1);
        // printf("dig_T2 = %i\n", bmp_device.calib_param.dig_t2);
        // printf("dig_T3 = %i\n", bmp_device.calib_param.dig_t3);
        // printf("dig_P1 = %u\n", bmp_device.calib_param.dig_p1);
        // printf("dig_P2 = %i\n", bmp_device.calib_param.dig_p2);
        // printf("dig_P3 = %i\n", bmp_device.calib_param.dig_p3);
        // printf("----------\n");
        // printf("Temperature: 0x%08X (%u)\n", raw_bmp_data.temperature, raw_bmp_data.temperature);
        // printf("Pressure: 0x%08X (%u)\n", raw_bmp_data.pressure, raw_bmp_data.pressure);

        bmp2_compensate_data(&raw_bmp_data, &physical_bmp_data, &bmp_device);

        printf("Temperature:          % 6.2f C\n", physical_bmp_data.temperature);
        printf("Pressure:             % 8.2f Pa\n", physical_bmp_data.pressure);

        // Convert thermocouple voltage
        thermocouple_voltage = ((float)usb_packet_from_tiva.amp_thermocouple_voltage / 4096) * 3.3;
        printf("Thermocouple voltage: % 7.4f V\n", thermocouple_voltage);

        // Digital thermocouple value
        digital_thermocouple = usb_packet_from_tiva.digital_amp_thermocouple & 0x0000FFFF;
        if (digital_thermocouple & 0x0004) {
            printf("Digital thermocouple is open\n");
        } else {
            printf("Digital thermocouple is closed\n");
        }
        printf("Digital thermocouple:  %f deg C\n", (digital_thermocouple >> 3) * 0.25f);


        current_milliseconds_since_epoch = getMilliSecondsSinceEpoch();
        diff_ms = current_milliseconds_since_epoch - milliseconds_since_epoch_at_start;

        // Write to serial port
        // unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o' };
        // write(serial_port, msg, sizeof(msg));
        usb_packet_to_tiva.pwm_controller = 0.5f * 0xFFFF;
        usb_packet_to_tiva.status = 0;
        write(serial_port, &usb_packet_to_tiva, sizeof(usb_packet_to_tiva));

        // Allocate memory for read buffer, set size according to your needs
        uint8_t read_buf[256];

        // Normally you wouldn't do this memset() call, but since we will just receive
        // ASCII data for this example, we'll set everything to 0 so we can
        // call printf() easily.
        memset(&read_buf, '\0', sizeof(read_buf));

        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
        if (num_bytes < 0)
        {
            fprintf(stderr, "Error reading (error code %i)", errno);
            return 1;
        }

        // Check if we read a Tiva-To-PC structure.
        // At the moment, num_bytes is expected to be 14.
        if (num_bytes == sizeof(usb_serial_data_tiva_to_pc))
        {
            memcpy(&usb_packet_from_tiva, read_buf, num_bytes);
        }
        else
        {
            printf("Did NOT read a dedicated Tiva-to-PC packet\n");
        }

        // Log to file.
        logSignalSample(log_fid, sample_index, diff_ms, physical_bmp_data.temperature,
        physical_bmp_data.pressure, thermocouple_voltage, first_log_call);
        first_log_call = 0;

        // Ready for next sample.
        sample_index += 1;

        if (diff_ms > 1000 * max_runtime_seconds) {
            break;
        }
    }

    //--------------------------------------------------
    fclose(log_fid);
    close(serial_port);
    printf("* Closed the serial port and log file.\n");
    printf("* Finished cleanly -- bye.\n");
    return 0;
}
