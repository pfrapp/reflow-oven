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
    float pwm_controller_percent;

    // The measured oven temperature theta_o in degree Celcius.
    float oven_temperature_deg_C;

    // The measured ambient temperature theta_o in degree Celcius.
    float ambient_temperature_deg_C;

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

} control_parameters;


#endif // #ifndef SIGNALS_H

