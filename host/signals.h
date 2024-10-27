#ifndef SIGNALS_H
#define SIGNALS_H

//
// Input and output signals of the reflow
// oven subsystem.
//
// Contains the control input, the measured ambient temperature (disturbance),
// the measured oven temperature, and the state of the thermocouple connection.
//
typedef struct reflow_oven_signals_ {
    // Index (used when this strucuture is used in an array of structures).
    int index;

    // Timestamp as relative time since start of the program in ms.
    int relative_timestamp_ms;

    // The control signal u_pwm (in percent, from 0.0 to 100.0).
    double pwm_controller_percent;

    // The measured oven temperature theta_o in degree Celcius.
    double oven_temperature_deg_C;

    // The measured ambient temperature theta_o in degree Celcius.
    double ambient_temperature_deg_C;

    // Indicator whether the thermocouple is open (1) or closed (0).
    int thermocouple_is_open;

} reflow_oven_signals;

//
// Overall control parameters.
//
typedef struct control_parameters_ {
    // Current sample index.
    int sample_index;

    // Sample time in ms.
    int sample_time_ms;

    // Maximum runtime in seconds.
    int max_runtime_seconds;

    // Request to turn the PWM controller (and hence the oven) off.
    int request_to_turn_off;

} control_parameters;

//
// Structure used for time keeping.
//
typedef struct time_keeping_ {
    // Milliseconds since epoch at program start.
    // This means after the time that the connection to the Tiva has been established.
    long long milliseconds_since_epoch_at_start;

    // The current time in milliseconds since epoch.
    long long current_milliseconds_since_epoch;

    // Relative time between current time and program start time.
    long long diff_ms;

} time_keeping;

//
// Structure used for system identification.
//
typedef struct system_identification_ {
    // Start of the procedure in seconds.
    double start_time_seconds;

    // Temperature after which we stop heating in degree C.
    double heating_until_deg_C;

    // Overall time in seconds.
    int max_runtime_seconds;

    // Internal state denoting whether we already reached / ever exceeded the maximum heating temperature.
    int maximum_heating_temperature_ever_exceeded;

} system_identification;


#endif // #ifndef SIGNALS_H

