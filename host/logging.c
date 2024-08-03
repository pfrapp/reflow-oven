#include "logging.h"

int logSignalSample(FILE *log_fid, int index, int time_ms,
                    double temperature,
                    double pwm_controller,
                    int bWriteHeader) {

    if (!log_fid) {
        return -1;
    }

    if (bWriteHeader) {
        fprintf(log_fid, "Index,Time (ms),Temperature (C),pwm_controller (percent)\n");
    }

    //
    fprintf(log_fid, "%05i,%09i,%06.2f,%06.2f\n",
            index, time_ms,
            temperature,
            pwm_controller);

    return 0;
}
