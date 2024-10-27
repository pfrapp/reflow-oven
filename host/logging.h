#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h> // For FILE
#include "usb_serial_data.h"

typedef struct measurement_logging_ {
    // File identifier
    FILE *log_fid;

    // Is this the first log call?
    // If so, we need to write the header columns.
    int first_log_call;

} measurement_logging;


int logSignalSample(measurement_logging *logging,
                    int index, int time_ms,
                    double temperature,
                    double pwm_controller);

#endif // #ifndef LOGGING_H
