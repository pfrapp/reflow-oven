#include "logging.h"

int logSignalSample(measurement_logging *logging,
                    int index, int time_ms,
                    double temperature,
                    double pwm_controller) {

    if (!logging) {
        return -1;
    }

    if (!logging->log_fid) {
        return -1;
    }

    if (logging->first_log_call) {
        fprintf(logging->log_fid, "Index,Time (ms),Temperature (C),pwm_controller (percent)\n");
        logging->first_log_call = 0;
    }

    //
    fprintf(logging->log_fid, "%05i,%09i,%06.2f,%06.2f\n",
            index, time_ms,
            temperature,
            pwm_controller);

    return 0;
}
