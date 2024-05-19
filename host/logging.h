#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h> // For FILE
#include "usb_serial_data.h"


int logSignalSample(FILE *log_fid, int index, int time_ms,
                    double temperature,
                    double pressure,
                    double amp_thermocouple_voltage,
                    int bWriteHeader);

#endif // #ifndef LOGGING_H
