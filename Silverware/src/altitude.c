#include "altitude.h"
#include <math.h>

float altitude;

void altitude_read(void)
{
    float pressure;

    pressure = read_pressure();
    altitude = 44330 * (1.0 - powf(pressure /102430 , 0.1903));
}
