//
// P. Rapp, Mar-23, 2024
// Copied over from the self-balancing robot host main file.
//
// This is the main host program that runs the controller for
// the reflow oven.
//
// We try and start with a PI controller.
//
// This program is also used for the system identification
// by disabling the controller and giving a step function
// onto the system (that is, the oven).
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
#include <sys/utsname.h> // For determining on which OS we are running

#include "usb_serial_data.h"
#include "logging.h"
#include "timing.h"
#include "usb_serial.h"
#include "bmp.h"
#include "signals.h"

// Currently supported platforms
enum {
    platform_mac = 0,    // macOS
    platform_rpi = 1,     // Raspberry Pi running Raspian
    platform_invalid = 2
};

int get_platform() {
    struct utsname unameData;
    uname(&unameData);
    if (strcmp(unameData.sysname, "Darwin") == 0) {
        return platform_mac;
    } else if (strcmp(unameData.sysname, "Linux") == 0) {
        return platform_rpi;
    }

    printf("uname sysname: %s\n", unameData.sysname);
    return platform_invalid;
}


enum {
    // Open-loop control, use the control signal from the file.
    open_loop = 0,
    // Closed-loop control, use the reference signal from the file.
    closed_loop
};

// The reference and control signals are saved in a file.
//
// Note: In closed-loop operation, the reference signal
// is used and the control signal is computed.
//
// In open-loop operation, the control signal from the file
// is used and the reference signal is discarded.

// Structure and functions to hold information about the controller.
// Put to separate file if needed.
typedef struct controller_ {
    // The control signal (in percent, from 0.0 to 100.0).
    double pwm_controller_percent;

    // The reference signal in degree Celcius.
    double reference_deg_C;

    // Measured temperature in degree Celcius.
    double temperature_deg_C;

    // Control error in degree Celcius.
    double control_error_deg_C;

    // Integrated control error (in degree Celcius times seoncs)
    double integrated_control_error_deg_C_sec;

    // Open or closed loop control.
    int open_or_closed_loop;

} controller;


int main(void)
{

    // Determine the OS (macOS or Raspian).
    int platform = get_platform();
    if (platform == platform_invalid) {
        printf("Could not determine your OS, exiting\n");
        return -1;
    }

    //
    // On macOS, you can find the name of the virtual COM port by issuing
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
    // On the Raspberry Pi, the name of the virtual COM
    // port is /dev/ttyACM0
    //
    // Regarding the Tiva:
    // For communicating (instead of flashing and debugging), you want to use the actual USB device.
    // (Change the switch on the board from Debug to Device).
    //
    const char virtualCOMPortNameRaspberryPi[] = "/dev/ttyACM0";
    const char virtualCOMPortNameMac[] = "/dev/cu.usbmodem123456781";
    const char *virtualCOMPortName = NULL;
    switch(platform) {
        case platform_mac:
            virtualCOMPortName = virtualCOMPortNameMac;
            printf("* OS identified as Mac\n");
            break;
        case platform_rpi:
            virtualCOMPortName = virtualCOMPortNameRaspberryPi;
            printf("* OS identified as RPi\n");
            break;
    }

    // The USB data packet that we are sending.
    usb_serial_data_pc_to_tiva usb_packet_to_tiva;

    // The USB data packet that we are receiving from the Tiva.
    usb_serial_data_tiva_to_pc usb_packet_from_tiva;

    int err;

    int dummy1, dummy2;
    int num_values_read;

    // Time keeping
    time_keeping timing;

    // Control (and measurement) parameters.
    control_parameters control_and_measurement_parameters;

    // File for logging measurement signals to and reading the reference
    // and control signals from.
    FILE *reference_control_fid;
    measurement_logging logging;

    // BMP structures
    struct bmp2_dev bmp_device;
    struct bmp2_uncomp_data raw_bmp_data;
    struct bmp2_data physical_bmp_data;

    // Keep track of the signals associated with the reflow oven.
    reflow_oven_signals current_reflow_oven_signals;

    // Controller data
    controller ctrl;
    ctrl.open_or_closed_loop = open_loop;
    ctrl.pwm_controller_percent = 50.0;


    // Init logging
    logging.log_fid = fopen("signals.log", "w");
    logging.first_log_call = 1;

    // Read the reference and control signals
    reference_control_fid = fopen("reference_control_signals.log", "r");
    // Skip the first line, see
    // https://stackoverflow.com/questions/2799612/how-to-skip-a-line-when-fscanning-a-text-file
    fscanf(reference_control_fid, "%*[^\n]\n");


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

         // Gave error code 13 on the Raspberry Pi
         // except when being run as root via sudo ./tiva_usb_connect
         // In order to not resort to running the program as root, add the current user
         // to the group dialout.
         // Check if the device is attached to the group dialout either via ll in the
         // /dev folder, or via stat /dev/ttyACM0 and look for Gid.
         fprintf(stderr, "### On Linux, if the error code is 13, add your user to the dialout group.\n");
         fprintf(stderr, "sudo usermod -a -G dialout $USER\n");
         fprintf(stderr, "Reboot after you did this.\n");

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

    timing.milliseconds_since_epoch_at_start = getMilliSecondsSinceEpoch();
    // Sample time of 500 ms, corresponding to 2 Hz.
    control_and_measurement_parameters.sample_time_ms = 500;
    // Start with -1 in order to allow sample index 0 to already have captured all
    // the measurement data.
    // This is necessary because we only receive the first measurement after we
    // sent the first control signal.
    // It will still start at time 0 as we are using the sample index
    // for active waiting.
    control_and_measurement_parameters.sample_index = -1;
    control_and_measurement_parameters.max_runtime_seconds = 720;

    int ever_exceeded_100_degC = 0;

    while (1)
    {
        float time_sec;
        float thermocouple_voltage;
        uint16_t digital_thermocouple;


        // Idle (wait) until the sampling interval is over.
        // Perhaps a callback function is more appropriate in the long term.
        do {
            timing.current_milliseconds_since_epoch = getMilliSecondsSinceEpoch();
            timing.diff_ms = timing.current_milliseconds_since_epoch - timing.milliseconds_since_epoch_at_start;
        } while (timing.diff_ms < control_and_measurement_parameters.sample_index * control_and_measurement_parameters.sample_time_ms);



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

        // Convert thermocouple voltage
        thermocouple_voltage = ((float)usb_packet_from_tiva.amp_thermocouple_voltage / 4096) * 3.3;

        // Digital thermocouple value
        digital_thermocouple = usb_packet_from_tiva.digital_amp_thermocouple & 0x0000FFFF;
        current_reflow_oven_signals.thermocouple_is_open = digital_thermocouple & 0x0004;
        if (current_reflow_oven_signals.thermocouple_is_open) {
            printf("Digital thermocouple is open\n");
            // printf("You will not receive any measurements -- Exiting!\n");
            // return -1;
        } else {
            // printf("Digital thermocouple is closed\n");
        }
        ctrl.temperature_deg_C = (digital_thermocouple >> 3) * 0.25f;
        current_reflow_oven_signals.oven_temperature_deg_C = (digital_thermocouple >> 3) * 0.25f;

        // Check if we ever exceeded 100 deg C.
        if (current_reflow_oven_signals.oven_temperature_deg_C > 100.0) {
            ever_exceeded_100_degC = 1;
        }

        // Fill sample index and ambient temperature values into the current measurement sample.
        current_reflow_oven_signals.index = control_and_measurement_parameters.sample_index;
        current_reflow_oven_signals.ambient_temperature_deg_C = physical_bmp_data.temperature;


        // Read the reference and control signals from file.
        num_values_read = fscanf(reference_control_fid, "%d,%d,%lf,%lf",
                &dummy1, &dummy2,
                &(ctrl.reference_deg_C),
                &(ctrl.pwm_controller_percent));
        if (num_values_read == 4) {
            // printf("ref %f, ctrl %f\n", ctrl.reference_deg_C, ctrl.pwm_controller_percent);
        } else {
            printf("Failed to read reference and control signals (num_values_read = %i)!\n", num_values_read);
            printf("dummy1 = %i, dummy2 = %i\n", dummy1, dummy2);
        }

        timing.current_milliseconds_since_epoch = getMilliSecondsSinceEpoch();
        timing.diff_ms = timing.current_milliseconds_since_epoch - timing.milliseconds_since_epoch_at_start;

        // Disable controller if the temperature was above 100 deg C.
        if (ever_exceeded_100_degC) {
            current_reflow_oven_signals.pwm_controller_percent = 0.0;
        }


        // Write to serial port
        usb_packet_to_tiva.pwm_controller = 0.01f * current_reflow_oven_signals.pwm_controller_percent * 0xFFFF;
        usb_packet_to_tiva.status = 0;
        write(serial_port, &usb_packet_to_tiva, sizeof(usb_packet_to_tiva));

        // Allocate memory for read buffer, set size according to your needs
        uint8_t read_buf[256];

        // Set buffer to zero.
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

        // Check if we have read a Tiva-To-PC structure.
        if (num_bytes == sizeof(usb_serial_data_tiva_to_pc))
        {
            memcpy(&usb_packet_from_tiva, read_buf, num_bytes);
        }
        else
        {
            printf("Did NOT read a dedicated Tiva-to-PC packet\n");
        }

        // Print to terminal
        time_sec = 0.001f * timing.diff_ms;
        printf("---------------------------------------\nTime:                 % 8.3f s (of %i s)\n", time_sec, control_and_measurement_parameters.max_runtime_seconds);
        printf("Temperature:           % 6.2f C\n", physical_bmp_data.temperature);
        // printf("Pressure:             % 8.2f Pa\n", physical_bmp_data.pressure);
        // printf("Thermocouple voltage: % 7.4f V\n", thermocouple_voltage);
        printf("Measured temperature:  %6.2f deg C\n", current_reflow_oven_signals.oven_temperature_deg_C);
        printf("Reference temperature: %6.2f deg C\n", ctrl.reference_deg_C);
        printf("PWM controller signal: %6.2f %%\n", current_reflow_oven_signals.pwm_controller_percent);

        // Log to file.
        if (control_and_measurement_parameters.sample_index >= 0) {
            logSignalSample(&logging,
                            &control_and_measurement_parameters,
                            &timing,
                            &current_reflow_oven_signals);
        }

        // Ready for next sample.
        control_and_measurement_parameters.sample_index += 1;

        if (timing.diff_ms >= 1000 * control_and_measurement_parameters.max_runtime_seconds) {
            break;
        }
    }

    //--------------------------------------------------
    fclose(logging.log_fid);
    close(serial_port);
    printf("* Closed the serial port and log file.\n");
    printf("* Finished cleanly -- bye.\n");
    return 0;
}
