#include "logging.h"

int
logSignalSample(measurement_logging* logging,
                control_parameters* control_params,
                time_keeping* time_params,
                reflow_oven_signals* oven_signals,
                controller* ctrl)
{

    if (!logging) {
        return -1;
    }

    if (!logging->log_fid) {
        return -1;
    }

    if (!control_params || !time_params || !oven_signals) {
        return -1;
    }

    if (logging->first_log_call) {
        fprintf(logging->log_fid,
                "Index,"                           // First column
                "Time (ms),"                       // Second column
                "Oven temperature (C),"            // Third column
                "Ambient temperature (C),"         // 4th column
                "PWM controller (percent),"        // 5th column
                "Thermocouple open,"               // 6th column
                "Reference temperature (C)\n");    // 7th column
        logging->first_log_call = 0;
    }

    //
    fprintf(logging->log_fid,
            "%05i,%09i,%06.2f,%05.2f,%06.2f,%1i,%06.2f\n",
            oven_signals->index,
            time_params->diff_ms,
            oven_signals->oven_temperature_deg_C,
            oven_signals->ambient_temperature_deg_C,
            oven_signals->pwm_controller_percent,
            oven_signals->thermocouple_is_open,
            ctrl->reference_deg_C);

    return 0;
}
