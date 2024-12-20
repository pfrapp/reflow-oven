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
#include "solder_profile.h"

// Currently supported platforms
enum {
    platform_mac = 0,    // macOS
    platform_rpi = 1,     // Raspberry Pi running Raspian
    platform_invalid = 2
};

// Current mode
enum
{
    operation_mode_turn_off = 0,             // Just turn the PWM off.

    operation_mode_system_identification,    // Excert a step function for system identification.

    operation_mode_control,                  // Run the controller to follow
                                             // the reflow temperature profile.

    operation_mode_test,                     // Short solder run: follow a 5 second profile
                                             // which should not actually
                                             // heat up the ofen that much.

    operation_mode_invalid                   // Invalid operation mode.
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

int main(int argc, char *argv[])
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

    // Measurement logging.
    measurement_logging logging;

    // BMP structures
    struct bmp2_dev bmp_device;
    struct bmp2_uncomp_data raw_bmp_data;
    struct bmp2_data physical_bmp_data;

    // Keep track of the signals associated with the reflow oven.
    reflow_oven_signals current_reflow_oven_signals;

    // Controller data
    controller ctrl;
    ctrl.pwm_controller_percent                     = 0.0;
    ctrl.reference_deg_C                            = 0.0;
    ctrl.temperature_deg_C                          = 0.0;
    ctrl.control_error_deg_C                        = 0.0;
    ctrl.integrated_control_error_deg_C_sec         = 0.0;
    ctrl.previous_control_error_deg_C               = 0.0;
    ctrl.differentiated_control_error_deg_C_per_sec = 0.0;
    ctrl.max_runtime_seconds                        = 720;
    ctrl.kP                                         = 3.0;
    ctrl.kI                                         = 0.0200;
    ctrl.kD                                         = 20.0;

    // System identification data structure.
    system_identification sys_ident;

    // Operation mode (defaults to invalid)
    int operation_mode = operation_mode_invalid;

    //
    // Set up some parameters.
    //
    sys_ident.start_time_seconds = 10.0; // 30
    sys_ident.heating_until_deg_C = 25.0; // 100
    sys_ident.max_runtime_seconds = 720; // 720
    sys_ident.maximum_heating_temperature_ever_exceeded = 0;

    // Sample time of 500 ms, corresponding to 2 Hz.
    control_and_measurement_parameters.sample_time_ms = 500;
    // Start with -1 in order to allow sample index 0 to already have captured all
    // the measurement data.
    // This is necessary because we only receive the first measurement after we
    // sent the first control signal.
    // It will still start at time 0 as we are using the sample index
    // for active waiting.
    control_and_measurement_parameters.sample_index = -1;

    control_and_measurement_parameters.request_to_turn_off = 0;

    // Retrieve the desired operation mode.
    if (argc > 1) {
        if (strcmp(argv[1], "off") == 0) {
            control_and_measurement_parameters.request_to_turn_off = 1;
            operation_mode                                         = operation_mode_turn_off;
        } else if (strcmp(argv[1], "sysident") == 0) {
            operation_mode = operation_mode_system_identification;
        } else if (strcmp(argv[1], "solder") == 0) {
            operation_mode = operation_mode_control;
        } else if (strcmp(argv[1], "test") == 0) {
            operation_mode = operation_mode_test;
        }
    }
    if (operation_mode == operation_mode_invalid) {
        printf("## No or invalid operation mode provided as first argument.\n"
               "## Valid operation modes are: off, sysident, solder, and test.\n"
               "## Exiting (with return code -1).\n");
        return -1;
    }

    // Set the runtime of the program.
    switch (operation_mode) {
        case operation_mode_turn_off:
            control_and_measurement_parameters.max_runtime_seconds = 1;
            break;

        case operation_mode_system_identification:
            control_and_measurement_parameters.max_runtime_seconds = sys_ident.max_runtime_seconds;
            break;

        case operation_mode_control:
            control_and_measurement_parameters.max_runtime_seconds = ctrl.max_runtime_seconds;
            break;

        case operation_mode_test:
            control_and_measurement_parameters.max_runtime_seconds = 5;
            break;
    }

    // Init logging and log some general parameters
    if (!control_and_measurement_parameters.request_to_turn_off) {
        logging.log_fid = fopen("signals.log", "w");
        logging.first_log_call = 1;

        // Log some parameters before the actual tabular data begins.
        // Needs to be skipped when reading the csv file.
        fprintf(logging.log_fid, "Sample time (ms): %i\n", control_and_measurement_parameters.sample_time_ms);
        fprintf(logging.log_fid, "Maximum runtime (s): %i\n", control_and_measurement_parameters.max_runtime_seconds);
    }

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

    // Capture the time of the start of the actual program logic.
    timing.milliseconds_since_epoch_at_start = getMilliSecondsSinceEpoch();

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

        // Check if we ever exceeded the defined temperature.
        if (current_reflow_oven_signals.oven_temperature_deg_C > sys_ident.heating_until_deg_C) {
            sys_ident.maximum_heating_temperature_ever_exceeded = 1;
        }

        // Fill sample index and ambient temperature values into the current measurement sample.
        current_reflow_oven_signals.index = control_and_measurement_parameters.sample_index;
        current_reflow_oven_signals.ambient_temperature_deg_C = physical_bmp_data.temperature;

        timing.current_milliseconds_since_epoch = getMilliSecondsSinceEpoch();
        timing.diff_ms = timing.current_milliseconds_since_epoch - timing.milliseconds_since_epoch_at_start;

        //
        // Compute the control signal
        //

        switch(operation_mode) {
            case operation_mode_turn_off:
                if (control_and_measurement_parameters.request_to_turn_off) {
                    current_reflow_oven_signals.pwm_controller_percent = 0.0;
                }
                break;

            case operation_mode_system_identification:
                if (timing.diff_ms / 1000.0 < sys_ident.start_time_seconds) {
                    current_reflow_oven_signals.pwm_controller_percent = 0.0;
                } else {
                    current_reflow_oven_signals.pwm_controller_percent = 100.0;
                }
                // Disable controller if the temperature was above the defined limit.
                if (sys_ident.maximum_heating_temperature_ever_exceeded) {
                    current_reflow_oven_signals.pwm_controller_percent = 0.0;
                }
                break;

            case operation_mode_control:
            case operation_mode_test:
                // controller update function
                // controller output function
                ctrl.reference_deg_C = 0.0;
                if (current_reflow_oven_signals.index >= 0) {
                    // Take over the value from the precomputed arrays.
                    if (operation_mode == operation_mode_control) {
                        ctrl.reference_deg_C = g_solder_profile[current_reflow_oven_signals.index];
                    } else if (operation_mode == operation_mode_test) {
                        ctrl.reference_deg_C =
                            g_solder_test_profile[current_reflow_oven_signals.index];
                    }
                }
                ctrl.control_error_deg_C = ctrl.reference_deg_C - current_reflow_oven_signals.oven_temperature_deg_C;
                ctrl.integrated_control_error_deg_C_sec +=
                        control_and_measurement_parameters.sample_time_ms * 1.0e-3 * ctrl.control_error_deg_C;
                ctrl.differentiated_control_error_deg_C_per_sec =
                    (ctrl.control_error_deg_C - ctrl.previous_control_error_deg_C) /
                        (control_and_measurement_parameters.sample_time_ms * 1.0e-3);
                ctrl.pwm_controller_percent = 
                        ctrl.kP * ctrl.control_error_deg_C
                        + ctrl.kI * ctrl.integrated_control_error_deg_C_sec
                        + ctrl.kD * ctrl.differentiated_control_error_deg_C_per_sec;
                ctrl.previous_control_error_deg_C = ctrl.control_error_deg_C;

                // Limit values
                if (ctrl.pwm_controller_percent < 0.0) {
                    ctrl.pwm_controller_percent = 0.0;
                }
                if (ctrl.pwm_controller_percent > 100.0) {
                    ctrl.pwm_controller_percent = 100.0;
                }

                // Take over computed controller PWM value
                current_reflow_oven_signals.pwm_controller_percent = ctrl.pwm_controller_percent;
                break;
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
        printf("------------------------------------------------------------------\n"
               "Time:                 % 8.3f s (of %i s)\n", time_sec, control_and_measurement_parameters.max_runtime_seconds);
        printf("Ambient temperature:   % 6.2f C\n", physical_bmp_data.temperature);
        // printf("Pressure:             % 8.2f Pa\n", physical_bmp_data.pressure);
        // printf("Thermocouple voltage: % 7.4f V\n", thermocouple_voltage);
        printf("Oven temperature:      %6.2f deg C", current_reflow_oven_signals.oven_temperature_deg_C);
        printf(" (thermocouple is %s)\n", current_reflow_oven_signals.thermocouple_is_open ? "open" : "closed");
        printf("Reference temperature: %6.2f deg C\n", ctrl.reference_deg_C);
        printf("PWM controller signal: %6.2f %%\n", current_reflow_oven_signals.pwm_controller_percent);
        if (time_sec > 550.0) {
            printf("Please open the door now!\n");
        }

        // Log to file.
        if (control_and_measurement_parameters.sample_index >= 0 && !control_and_measurement_parameters.request_to_turn_off) {
            logSignalSample(&logging,
                            &control_and_measurement_parameters,
                            &timing,
                            &current_reflow_oven_signals,
                            &ctrl);
        }

        // Ready for next sample.
        control_and_measurement_parameters.sample_index += 1;

        if (timing.diff_ms >= 1000 * control_and_measurement_parameters.max_runtime_seconds) {
            break;
        }
    }

    //--------------------------------------------------
    if (!control_and_measurement_parameters.request_to_turn_off) {
        fclose(logging.log_fid);
    }
    close(serial_port);
    printf("* Closed the serial port and log file.\n");
    printf("* Finished cleanly -- bye.\n");
    return 0;
}
