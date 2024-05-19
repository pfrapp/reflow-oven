#include "logging.h"

int logSignalSample(FILE *log_fid, int index, int time_ms,
                    double temperature,
                    double pressure,
                    double amp_thermocouple_voltage,
                    int bWriteHeader) {

    if (!log_fid) {
        return -1;
    }

    if (bWriteHeader) {
        fprintf(log_fid, "Index,Time (ms),Temperature (C),Pressure (Pa),Amplified Thermocouple voltage (V)\n");
    }

    //
    fprintf(log_fid, "%05i,%09i,%06.2f,%08.2f,%07.4f\n",
            index, time_ms,
            temperature,
            pressure,
            amp_thermocouple_voltage);

    return 0;
}
