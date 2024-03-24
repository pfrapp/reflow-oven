#ifndef USB_SERIAL_H
#define USB_SERIAL_H

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

// Configure the tty.
// Returns 0 on success, nonzero otherwise.
int configure_tty(int serial_port, struct termios *tty);

#endif // #ifndef USB_SERIAL_H
