#include "config.h"
#include "control.h"
#include "barometer.h"
#include "altitude.h"
#include "util.h"
#include "drv_time.h"
#include <math.h>

#define VELOCITY_THROTTLE_GAIN  2.0f   // How does 1 m/s desired velocity translate to throttle increase?
#define HOVER_THROTTLE_DEFAULT  0.1f   // the estimated hover throttle
#define HOVER_THROTTLE_MIN      0.1f   // minimum possible hover throttle
#define HOVER_THROTTLE_MAX      1.0f   // maximum possible hover throttle

// Define maximum velocity for full throttle in m/s
#define FULL_THROTTLE_ALT_SET  0.3f

#define ALTITUDE_THROTTLE_GAIN 1.0f
#define ALT_P 0.3f
#define ALT_I 0.15f
#define ALT_D 3.0f

extern float looptime;
extern double press_fl, temp_fl;

extern float rx[];
extern float gyro[];
extern float accel[];

double altitude = 0;
double alt_set = 0;
double alt_cal= 0;

double last_alt_e, alt_i, alt_corr = 0;

float ah_throttle = HOVER_THROTTLE_DEFAULT;

void altitude_read(void)
{
    read_pressure();
    altitude = (1.0f - pow(press_fl/101325.0, 0.190295)) * 44330.0;// - alt_cal; // pressure to altitude @sea-level
}
float zaccel;
float vel;

void altitude_cal(void)
{
    // prime pressure reading lpf
    for (int y = 0; y < 1000; y++)
        {
//             read_pressure();
            altitude_read();
            delay(300);
        }
//     alt_cal = (1.0 - powf(press_fl/101325.0, 0.190295)) * 44330.0;
//     alt_set = altitude;
}


float altitude_hold(void)
{
    float new_ah_throttle = HOVER_THROTTLE_DEFAULT;

    double new_alt_e, alt_e, new_alt_d, alt_d = 0;
    double new_alt_corr;

    float newrx = rx[3] - 0.5f;           // zero center throttle

    if (fabs(newrx) > 0.05f)
    {
        if (newrx > 0) newrx -= 0.05f;
        else newrx += 0.05f;
        newrx *= 2.222222f; // nexrx => (-1.0f, 1.0f)
        alt_set = altitude + newrx * FULL_THROTTLE_ALT_SET;     // Add +/- FULL_THROTTLE_ALT_SET meter to altitude for full throttle travel
    }

    // ALT PID
    alt_e = (alt_set - altitude);                  // m
    dconstrain(&alt_e, -1.0 * FULL_THROTTLE_ALT_SET, FULL_THROTTLE_ALT_SET);  // Apply FULL_THROTTLE_ALT_SET leash

    alt_i += alt_e * looptime;
    alt_d = (alt_e - last_alt_e) / looptime;
//     dlpf(&alt_d, new_alt_d, lpfcalc(looptime, 0.015f));

    alt_corr = ALT_P * alt_e + ALT_I * alt_i + ALT_D * alt_d;
//     dlpf(&alt_corr, new_alt_corr, lpfcalc(looptime, 0.1f));

    new_ah_throttle = ah_throttle + alt_corr * ALTITUDE_THROTTLE_GAIN;
    constrain(&new_ah_throttle, HOVER_THROTTLE_MIN, HOVER_THROTTLE_MAX);

    lpf(&ah_throttle, new_ah_throttle, lpfcalc(looptime, 0.01f));

    last_alt_e = alt_e;

    return ah_throttle;
}

