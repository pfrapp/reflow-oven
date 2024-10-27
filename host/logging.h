#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h> // For FILE
#include "usb_serial_data.h"
#include "signals.h"

typedef struct measurement_logging_ {
    // File identifier
    FILE *log_fid;

    // Is this the first log call?
    // If so, we need to write the header columns.
    int first_log_call;

} measurement_logging;


int logSignalSample(measurement_logging *logging,
                    control_parameters *control_params,
                    time_keeping *time_params,
                    reflow_oven_signals *oven_signals);

#endif // #ifndef LOGGING_H
