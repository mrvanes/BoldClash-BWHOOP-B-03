#include "hardware.h"
#include "barometer.h"
#include "drv_dps310.h"

void barometer_init(void)
{
#ifdef USE_BARO_DPS310
    dps310_init();
#endif
}

int barometer_check(void)
{
#ifndef DISABLE_BARO_CHECK
#ifdef USE_BARO_DPS310
    return dps310_check();
#endif

#endif
    return 1;
}

float read_pressure(void)
{
//     float pressure;

#ifdef USE_BARO_DPS310
    dps310_read_pressure();
    dps310_pcomp_lpf();
//     dps310_tcomp_lpf(); //we don't need tcomp for altitude
#else
    return 0;
#endif
}
