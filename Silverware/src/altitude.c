#include "config.h"
#include "control.h"
#include "barometer.h"
#include "altitude.h"
#include "util.h"
#include "drv_time.h"
#include <math.h>


extern float looptime;
extern double press_fl, temp_fl;

extern float rx[];
extern float gyro[];
extern float accel[];

float ah_time, last_ah_time;

double altitude = 0;
double last_altitude = 0;
double alt_set = 0;

double alt_cal= 0;

float last_vel_e = 0;

#define HOVER_THROTTLE_DEFAULT  0.1f   // the estimated hover throttle
#define HOVER_THROTTLE_MIN      0.1f   // minimum possible hover throttle
#define HOVER_THROTTLE_MAX      1.0f   // maximum possible hover throttle

// Define maximum velocity for full throttle in m/s
#define FULL_THROTTLE_VELOCITY 1.0f
#define VELOCITY_THROTTLE_GAIN 2.0f

#define ALT_P 2.0f

#define VEL_P 2.0f
#define VEL_I 1.0f
#define VEL_D 0.0f


float ah_throttle = HOVER_THROTTLE_DEFAULT;
float hover_throttle = HOVER_THROTTLE_DEFAULT;

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
    for (int y = 0; y < 100; y++)
        {
//             read_pressure();
            altitude_read();
            delay(3000);
        }
//     alt_cal = (1.0 - powf(press_fl/101325.0, 0.190295)) * 44330.0;
    alt_set = altitude;
}

float altitude_hold(void)
{
    float desired_vel;

    float alt_e = 0;
    float vel_alt;

    float new_vel, vel_set = 0;
    float vel_e = 0;
    float vel_i, vel_d = 0;

    float dt;
/*
    ah_time = gettime();
    dt = (ah_time - last_ah_time) * 1e-6;       // dt in seconds
    if (dt < 0.025f) return ah_throttle;        // 40Hz
    last_ah_time = ah_time;
*/
    dt = looptime;

    float newrx = rx[3] - 0.5f;           // zero center throttle

    if (fabs(newrx) > 0.05f)
    {
        if (newrx > 0)
        {
            newrx -= 0.05f;
        } else
        {
            newrx += 0.05f;
        }
//         alt_set = altitude;
        desired_vel = newrx * 2.222222f * FULL_THROTTLE_VELOCITY * VELOCITY_THROTTLE_GAIN; // 1/0.45 ~ 2.222222
        alt_set = altitude + desired_vel * 0.1;     // Add +/- 0.1m to altitude for full throttle
    } /*else
    {
        desired_vel = 0;
    }

    desired_vel *= 2.222222f * FULL_THROTTLE_VELOCITY * VELOCITY_THROTTLE_GAIN; // 1/0.45 ~ 2.222222
*/
    // ALT
    alt_e = (alt_set - altitude);                  // m
    constrain(&alt_e, -1.0f, 1.0f);                // Make the leash max 1m long
    vel_alt = ALT_P * alt_e;                       // Add ALT_P m/s velocity for every meter error

    // VEL
//     new_vel = (altitude - last_altitude) / dt;     // in m/s
    vel = (altitude - last_altitude) / dt;     // in m/s
//     lpf(&vel, new_vel, 0.96875f);                  // low-pass noisy vel a bit 16*
//     constrain(&vel, -1.0f, 1.0f); // get rid of ludicrous values of vel, it's probably T-drift anyway

    // PID
    vel_e = desired_vel + vel_alt - vel;        // in m/s
    vel_i += vel_e * dt;                        // in m
    vel_d = (vel_e - last_vel_e) / dt;          // in m/s^2

    float new_vel_set = VEL_P * vel_e + VEL_I * vel_i + VEL_D * vel_d;
    lpf(&vel_set, new_vel_set, lpfcalc(dt, 0.075f));

    ah_throttle = hover_throttle + vel_set * VELOCITY_THROTTLE_GAIN;

    // limit throttle so that it never drops below minimum
    // or emulate a rocket :/
    constrain(&ah_throttle, HOVER_THROTTLE_MIN, HOVER_THROTTLE_MAX);

    lpf(&zaccel, accel[2]-1, lpfcalc(dt, 0.1f)); // gyro is a bit noisy 16* os

//     if ((fabs(zaccel) < 0.1f))
    if ((fabs(vel) < 0.2f))
    {
        lpf(&hover_throttle, ah_throttle, lpfcalc(dt, 5.0f)); // 2s
//         lpf(&hover_throttle, ah_throttle, 0.984375f); // 64*
//         lpf(&hover_throttle, ah_throttle, 0.96875f); // 32*
    }

    last_altitude = altitude;
    last_vel_e = vel_e;

    return ah_throttle;
}

