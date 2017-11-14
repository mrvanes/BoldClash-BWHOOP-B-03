/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "config.h"
#include "control.h"
#include "barometer.h"
#include "altitude.h"
#include "util.h"
#include "drv_time.h"
#include <math.h>

#define AH_REFRESH_FREQ        75.0f   // AH logic refresh rate

#define HOVER_THROTTLE_MIN      0.3f   // minimum possible hover throttle
#define HOVER_THROTTLE_MAX      0.8f   // maximum possible hover throttle

// Define maximum velocity for full throttle in m/s
#define FULL_THROTTLE_ALT_SET  0.2f

#define ALT_P 0.1f
#define ALT_I 0.1f
#define ALT_D 0.15f

extern float looptime;
extern double press_fl;

extern float rx[];
extern float gyro[];
extern float accel[];

double new_accel_alt, accel_alt = 0;
double new_accel_vel, accel_vel = 0;

double altitude = 0;
double alt_set = 0;

double last_alt_e, last_accel_alt, last_accel_vel, alt_i = 0;     // PID loop memory

float ah_throttle = HOVER_THROTTLE_MIN;

void altitude_read(void)
{
    double accel_z;

    read_pressure();
    altitude = (1.0 - pow(press_fl/101325.0, 0.190295)) * 44330.0; // pressure to altitude @sea-level

     // Use gyro in altitude estimation, compensate for lpf baro lag
    accel_z = accel[2] - 1;

    new_accel_vel += accel_z * looptime;
    hpfd(&accel_vel, new_accel_vel - last_accel_vel, lpfcalc(looptime, 4.0f));    // Remove DC component from accel_vel, we have baro for that
//     accel_vel = new_accel_vel;
    last_accel_vel = new_accel_vel;

    new_accel_alt += accel_vel * looptime;
    hpfd(&accel_alt, new_accel_alt - last_accel_alt, lpfcalc(looptime, 4.0f));    // Remove DC component from accel_alt, we have baro for that
//     accel_alt = new_accel_alt;
    last_accel_alt = new_accel_alt;

    altitude += accel_alt;
}

void altitude_cal(void)
{
    // prime pressure reading ldf
    for (int y = 0; y < 1000; y++)
        {
            altitude_read();
            delay(300);
        }
     alt_set = altitude; // Start with target at resting position
}

float last_ah_time;

float altitude_hold(void)
{
    float new_ah_throttle = HOVER_THROTTLE_MIN;

    double new_alt_e, alt_e, new_alt_d, alt_d, alt_corr = 0;
    double new_alt_corr, new_alt_set;

    float dt;

    float ah_time = gettime();
    dt = (ah_time - last_ah_time) * 1e-6;                   // dt in seconds
    if (dt < 1.0/AH_REFRESH_FREQ) {                         // AH refresh rate
        lpf(&ah_throttle, ah_throttle, lpfcalc_hz(dt, AH_REFRESH_FREQ));    // Interpolate values between refresh
        return ah_throttle;
    }
    last_ah_time = ah_time;

//     dt = looptime;
    float newrx = rx[3] - 0.5f;           // Zero center throttle

    if (fabs(newrx) > 0.05f)
    {
        if (newrx > 0) newrx -= 0.05f;
        else newrx += 0.05f;
        newrx *= 2.222222f; // newrx [-1.0f, 1.0f]
        new_alt_set = altitude + newrx * FULL_THROTTLE_ALT_SET;     // Add +/- FULL_THROTTLE_ALT_SET meter to altitude for full throttle travel
        lpfd(&alt_set, new_alt_set, lpfcalc(dt, 0.2f));             // Easy climbing and descending
    }

    // ALT PID
    alt_e = alt_set - altitude;                  // m
    constrain(&alt_e, -1.0 * FULL_THROTTLE_ALT_SET, FULL_THROTTLE_ALT_SET);  // Apply FULL_THROTTLE_ALT_SET leash

    alt_i += alt_e * dt;
//     constrain(&alt_i, -100.0, 100.0);

    new_alt_d = (alt_e - last_alt_e) / dt;
//     lpfd(&alt_d, new_alt_d, lpfcalc(dt, 0.02f));
    alt_d = new_alt_d;

    alt_corr = ALT_P * alt_e + ALT_I * alt_i + ALT_D * alt_d;
//     lpfd(&alt_corr, new_alt_corr, lpfcalc(dt, 0.1f));

    new_ah_throttle = ah_throttle + alt_corr;
    constrainf(&new_ah_throttle, HOVER_THROTTLE_MIN, HOVER_THROTTLE_MAX);

//     lpf(&ah_throttle, new_ah_throttle, lpfcalc_hz(dt, AH_REFRESH_FREQ));
    ah_throttle = new_ah_throttle;

    last_alt_e = alt_e;

    return ah_throttle;
}

