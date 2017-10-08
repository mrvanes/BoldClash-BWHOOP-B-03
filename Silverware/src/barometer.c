#include "hardware.h"
#include "barometer.h"
#include "drv_dps310.h"

void barometer_init()
{
#ifdef USE_BARO_DPS310
    dps310_init();
#endif
}

int barometer_check()
{
#ifndef DISABLE_BARO_CHECK
#ifdef USE_BARO_DPS310
    return dps310_check();
#endif

#endif
    return 1;
}

float read_pressure()
{
    float pressure;

#ifdef USE_BARO_DPS310
    pressure = dps310_read_pressure();
    return pressure;
#else
    return 0;
#endif
}
