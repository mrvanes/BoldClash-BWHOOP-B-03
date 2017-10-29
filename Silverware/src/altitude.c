#include "config.h"
#include "control.h"
#include "barometer.h"
#include "altitude.h"
#include "util.h"
#include "drv_time.h"
#include <math.h>

#define ALT_P 1.0

#define VEL_P 1.0
#define VEL_I 0.0
#define VEL_D 0.0

/*
#define ACC_P 0.0
#define ACC_I 0.0
#define ACC_D 0.0
*/

extern float looptime;
extern double press_fl, temp_fl;

extern float rx[];
extern float gyro[];
extern float accel[];

float ah_time, last_ah_time;

double altitude = 0;
double last_altitude = 0;
double alt_set = 0;
int neutral = 1;

double alt_cal;

float last_vel_e = 0;
/*
float acc, acc_set = 0;
float acc_e, acc_i, acc_d = 0;
float last_acc_e = 0;
*/
#define HOVER_THROTTLE_DEFAULT  0.1f   // the estimated hover throttle, 0 ~ 1
//#define HOVER_THROTTLE_TC       10.0f   // time constant used to update estimated hover throttle
#define HOVER_THROTTLE_MIN      0.1f  // minimum possible hover throttle
#define HOVER_THROTTLE_MAX      0.8f // maximum possible hover throttle

// Define maximum velocity for full throttle in m/s
#define FULL_THROTTLE_VELOCITY 1.5
#define VELOCITY_THROTTLE_GAIN 1.0

float ah_throttle = HOVER_THROTTLE_DEFAULT;
float hover_throttle = HOVER_THROTTLE_DEFAULT;

void altitude_read(void)
{
    read_pressure();
    altitude = (1.0f - pow(press_fl/101325.0, 0.190295)) * 44330.0;// - alt_cal; // pressure to altitude @sea-level}
}
float zaccel;
float vel;

void altitude_cal(void)
{
    // prime pressure reading lpf
    for (int y = 0; y < 1000; y++)
        {
            read_pressure();
            delay(300);
        }
//     alt_cal = (1.0 - powf(press_fl/101325.0, 0.190295)) * 44330.0;
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
    volatile static int counter;

    ah_time = gettime();
    dt = (ah_time - last_ah_time) * 1e-6; // dt in seconds
    if (dt < 0.05f) return ah_throttle;
    last_ah_time = ah_time;

//     dt = looptime;
    counter++;

    float newrx = rx[3] - 0.5f;           // zero center throttle

    if (fabs(newrx) > 0.05f)
    {
        alt_set = altitude;
        if (newrx > 0)
        {
            desired_vel = (newrx - 0.05f);
        } else
        {
            desired_vel = (newrx + 0.05f);
        }
    } else
    {
        desired_vel = 0;
    }

    // ALT
    alt_e = (alt_set - altitude);                  // m
    constrain(&alt_e, -0.5f, 0.5f);                // Make the leash max 0.5m long
    vel_alt = ALT_P * alt_e;                       // Add ALT_P m/s velocity for every meter error

    // VEL
    new_vel = (altitude - last_altitude) / dt;     // in m/s
//     constrain(&new_vel, -0.5f, 0.5f); // get rid of ludicrous values of vel, it's probably T-drift anyway
    lpf(&vel, new_vel, 0.96375f);                  // low-pass noisy vel a bit 16*

    desired_vel *= 2.222222f * FULL_THROTTLE_VELOCITY * 2; // 1/0.45 ~ 2.222222

    // PID
    vel_e = desired_vel + vel_alt - vel;           // in m/s
    vel_i += vel_e * dt;                           // in m
    vel_d = (vel_e - last_vel_e) / dt;             // in m/^s

    vel_set = VEL_P * vel_e + VEL_I * vel_i + VEL_D * vel_d;

/*  Skip acc PID for the moment
    acc = vel_set / dt;                       // in m/s^2
    acc_e = acc - accel[3];
    acc_i += acc_e * dt;
    acc_d = (acc_e - last_acc_e) / dt;

    acc_set = ACC_P * acc_e + ACC_I * acc_i + ACC_D * acc_d;
*/
    ah_throttle = hover_throttle + vel_set * VELOCITY_THROTTLE_GAIN;

    // limit throttle so that it never drops below minimum
    // or emulate a rocket :/
    constrain(&ah_throttle, HOVER_THROTTLE_MIN, HOVER_THROTTLE_MAX);

    lpf(&zaccel, accel[2]-1, 0.8f); // gyro is a bit noisy

    if ((fabs(vel) < 0.05f) && (fabs(zaccel) < 0.02f))
    {
        lpf(&hover_throttle, ah_throttle, 0.984375f); // 64*
    }

    last_altitude = altitude;
    last_vel_e = vel_e;
//     last_acc_e = acc_e;

    return ah_throttle;
}

