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
#include <sys/time.h>    // for timing functions and structures
#include <math.h>

#include "usb_serial_data.h"

long long getMilliSecondsSinceEpoch() {
    struct timeval tv;
    long long ms;
    gettimeofday(&tv, NULL);
    ms = (long long) (tv.tv_sec) * 1000 + (long long) (tv.tv_usec) / 1000;
    return ms;
}

int logSignalSample(FILE *log_fid, int index, int time_ms,
                    usb_serial_data_pc_to_tiva *usb_packet_to_tiva,
                    usb_serial_data_tiva_to_pc *usb_packet_from_tiva,
                    int bWriteHeader) {

    if (!log_fid || !usb_packet_to_tiva || !usb_packet_from_tiva) {
        return -1;
    }

    if (bWriteHeader) {
        fprintf(log_fid, "Index,Time (ms),Motor,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,QuadraturePosition,QuadratureVelocity\n");
    }

    //
    fprintf(log_fid, "%05i,%09i,0x%04x,0x%04X,0x%04X,0x%04X,0x%04X,0x%04X,0x%04X,0x%08X,0x%08X\n",
            index, time_ms,
            usb_packet_to_tiva->motor,
            usb_packet_from_tiva->accel_x,
            usb_packet_from_tiva->accel_y,
            usb_packet_from_tiva->accel_z,
            usb_packet_from_tiva->gyro_x,
            usb_packet_from_tiva->gyro_y,
            usb_packet_from_tiva->gyro_z,
            usb_packet_from_tiva->quadrature_position,
            usb_packet_from_tiva->quadrature_velocity);

    return 0;
}


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


    // Initialize the motor speed percentage and direction.
    speed_percent = 0;
    direction = 0;

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

    // Read in existing settings, and handle any error
    if (tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr\n", errno);
        return 1;
    }

    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr\n", errno);
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
            int16_t acc_x;
            int16_t acc_y;
            int16_t gyro_z;

            // Idle (wait) until the sampling interval is over.
            // Perhaps a callback function is more appropriate in the long term.
            do {
                current_milliseconds_since_epoch = getMilliSecondsSinceEpoch();
                diff_ms = current_milliseconds_since_epoch - milliseconds_since_epoch_at_start;
            } while (diff_ms < sample_index*sample_time_ms);

            time_sec = 0.001f * diff_ms;
            acc_x = (int16_t) usb_packet_from_tiva.accel_x;
            acc_y = (int16_t) usb_packet_from_tiva.accel_y;
            gyro_z = (int16_t) usb_packet_from_tiva.gyro_z;
            speed_percent = 0;

            if (speed_percent < 0) {
                speed_percent = -speed_percent;
                direction = 1;
            } else {
                direction = 0;
            }
            // Limit the motor speed to 98 percent.
            if (speed_percent > 98) {
                speed_percent = 98;
            }


        current_milliseconds_since_epoch = getMilliSecondsSinceEpoch();
        diff_ms = current_milliseconds_since_epoch - milliseconds_since_epoch_at_start;

        // Write to serial port
        // unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o' };
        // write(serial_port, msg, sizeof(msg));
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
        logSignalSample(log_fid, sample_index, diff_ms, &usb_packet_to_tiva, &usb_packet_from_tiva, first_log_call);
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
