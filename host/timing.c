#include <sys/time.h>    // for timing functions and structures
#include "timing.h"

long long getMilliSecondsSinceEpoch() {
    struct timeval tv;
    long long ms;
    gettimeofday(&tv, NULL);
    ms = (long long) (tv.tv_sec) * 1000 + (long long) (tv.tv_usec) / 1000;
    return ms;
}
