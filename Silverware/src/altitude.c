#include "altitude.h"
#include "util.h"
#include "drv_time.h"
#include <math.h>

float altitude = 0;
float altitude_cal = 0;

void altitude_read(void)
{
    float pressure;
    float new_altitude;

    pressure = read_pressure();
    new_altitude = 44330 * (1.0 - powf(pressure /102430 , 0.1903));
    lpf(&altitude, new_altitude - altitude_cal, 0.96);
}

void altidude_cal(void)
{
    float temp_cal;
    float cal_pressure;

    for (int y = 0; y < 500; y++)
        {
            altitude_cal = 44330 * (1.0 - powf(read_pressure() /102430 , 0.1903));
            lpf(&altitude_cal, altitude_cal, 0.92);
            delay(2000);
        }
}
