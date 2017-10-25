#include "config.h"
#include "control.h"
#include "barometer.h"
#include "altitude.h"
#include "util.h"
#include "drv_time.h"
#include <math.h>

#define VEL_P 0.8
#define VEL_I 0.3
#define VEL_D 0.1

#define ACC_P 0.0
#define ACC_I 0.0
#define ACC_D 0.0

extern float looptime;
extern float last_ah_time;
extern double press_fl, temp_fl;
// extern float press_cp; // pressure in centiPascal

extern float rx[];
extern float gyro[3];
extern float accel[3];
extern float GEstG[3];

extern int onground;

double altitude = 0;
double last_altitude = 0;
// float last_temp_fl = 0;
// float press_cal;
// float press_cp_cal;

double alt_cal;

float vel, vel_set = 0;
float vel_e, vel_i, vel_d = 0;
float last_vel_e = 0;
/*
float acc, acc_set = 0;
float acc_e, acc_i, acc_d = 0;
float last_acc_e = 0;
*/
#define HOVER_THROTTLE_DEFAULT  0.1f   // the estimated hover throttle, 0 ~ 1
// #define HOVER_THROTTLE_TC       10.0f   // time constant used to update estimated hover throttle
#define HOVER_THROTTLE_MIN      0.0f  // minimum possible hover throttle
#define HOVER_THROTTLE_MAX      0.8f // maximum possible hover throttle

// Define maximum velocity for full throttle in m/s
#define FULL_THROTTLE_VELOCITY 0.7


float ah_throttle = 0;
float hover_throttle = HOVER_THROTTLE_DEFAULT;

void altitude_read(void)
{
    last_altitude = altitude;
//     last_temp_fl = temp_fl;

    read_pressure();
//     float new_alt = lroundf((1.0f - powf(press_cp/press_cp_cal, 0.190295f)) * 44330000.0f)/1000.0f; // pressure to altitude @sea-level
    double new_alt = (1.0 - pow(press_fl/101325.0, 0.190295)) * 44330.0; // pressure to altitude @sea-level
    dlpf(&altitude, new_alt, 0.99);
}

void altitude_cal(void)
{
//     read_pressure();
//     press_cp_cal = press_cp;
//     alt_cal = (1.0 - powf(press_fl/101325.0, 0.190295)) * 44330.0;

    for (int y = 0; y < 100; y++)
        {
//             altitude_read();
            read_pressure();
//             float new_alt_cal = (1.0f - powf(press_fl/101325.0f, 0.190295f)) * 44330.0f;
//             lpf(&alt_cal, new_alt_cal, 0.9f);
            delay(30000);
        }
    double altitude = (1.0 - pow(press_fl/101325.0, 0.190295)) * 44330.0; // pressure to altitude @sea-level
}

float time;
float last_time = 0;

float altitude_hold(void)
{
    float desired_vel;
    float dt;
    time = gettime();

    dt = (time - last_time) * 1e-6;
    if (dt < 0.03f) return ah_throttle; // 16 times oversampling = 27.6ms/s
    last_time = time;

    float newrx = rx[3] - 0.5f; // zero center throttle

    if (fabs(newrx) > 0.05f)
    {
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

    desired_vel *= 2.222222f * FULL_THROTTLE_VELOCITY;

    vel = (altitude - last_altitude) / dt;    // in m/s
    vel_e = desired_vel - vel;                      // in m/s
//     float new_vel_e = desired_vel - vel;                      // in m/s
//     lpf(&vel_e, new_vel_e, 0.99f);
    vel_i += vel_e * dt;
    vel_d = (vel_e - last_vel_e) / dt;

    vel_set = VEL_P * vel_e + VEL_I * vel_i + VEL_D * vel_d;

/*  Skip acc PID for the moment
    acc = vel_set / dt;                       // in m/s^2
    acc_e = acc - accel[3];
    acc_i += acc_e * dt;
    acc_d = (acc_e - last_acc_e) / dt;

    acc_set = ACC_P * acc_e + ACC_I * acc_i + ACC_D * acc_d;
*/
    ah_throttle = hover_throttle + vel_set;

    // limit throttle so that it never drops below minimum
    // and motors don't stop running
    constrain(&ah_throttle, 0.1f, 1.0f);

//     if ((fabs(desired_vel) < 0.01f) && (fabs(vel) < 0.1) && (fabs(accel[0]) < 0.1) && (fabs(accel[1]) < 0.1))
//     if (fabs(desired_vel < 0.01f) && ((accel[3] - 1) < 0.2f))
    if ((fabs(vel) < 0.05f) && (GEstG[3] < 0.05f))
    {
        lpf(&hover_throttle, ah_throttle, 0.99f);
//         constrain(&hover_throttle, HOVER_THROTTLE_MIN, HOVER_THROTTLE_MAX);
    }

    last_vel_e = vel_e;
//     last_acc_e = acc_e;

    return ah_throttle;
}

void set_hover_throttle()
{
    // Check if we're flying and hovering stable
    if ((vel_e < 0.1) && (fabs(accel[1]) < 0.1) && (fabs(accel[2]) < 0.2))
    {
        lpf(&hover_throttle, ah_throttle, 0.999f);
//         constrain(&hover_throttle, HOVER_THROTTLE_MIN, HOVER_THROTTLE_MAX);
    }

}

