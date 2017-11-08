#include "drv_dps310.h"
#include "drv_i2c.h"
#include "drv_time.h"
#include "config.h"
#include "hardware.h"
#include "binary.h"
#include "util.h"
#include <math.h>

#define DPS310_PSR 0x00
#define DPS310_TMP 0x03
#define DPS310_PSR_CFG 0x06
#define DPS310_TMP_CFG 0x07
#define DPS310_MEAS_CFG 0x08
#define DPS310_CFG_REG 0x09
#define DPS310_INT 0x0a
#define DPS310_FIFO 0x0b
#define DPS310_RESET 0x0c
#define DPS310_PRDID 0x0d
#define DPS310_COEF 0x10
#define DPS310_COEF_SRCE 0x28
#define DPS310_ID 0x10

#define T_RDY B00100000
#define P_RDY B00010000

//      199         -259 79830 -51103 -2647 1325 -7843 -102 -788
int32_t  c0, c0Half,  c1,  c00,   c10,  c01, c11,  c20, c21, c30;
double press_raw_sc, temp_raw_sc, press_fl, temp_fl;

void dps310_init(void)
{
    int press_raw, temp_raw = 0;
    int data[3];

    // send reset command
    i2c_writereg( DPS310_I2C_ADDRESS , DPS310_RESET , B10001001 );
    delay(60000); //60ms to warm up

    // blocking - wait for registers
    do
        delay(1000);
    while (!(i2c_readreg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG)&B01000000));

    // blocking wait for coefficients to be ready
    do
        delay(1000);
    while (!(i2c_readreg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG)&B10000000));


    dps310_readcoeffs();

    // Forcing temp_sensor_type to B10000000 results in realistic T readings,
    // but B00000000 from DPS310_COEF_SRCE results in less P drift
    int temp_sensor_type = i2c_readreg(DPS310_I2C_ADDRESS, DPS310_COEF_SRCE)&B10000000;
//     int temp_sensor_type = B10000000; // The sensor on bwhoop seems to report temp_sensor_type wrong? Override with B10000000.

    // pressure config. Sample as quickly as possible, do oversampling in software
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_PSR_CFG, B00000000);// 0 meas/sec | 1 times oversampling, needs no P_SHIFT
    // temp config
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_TMP_CFG, B00000000|temp_sensor_type); // 0 meas/sec / 1 times oversampling, needs no T_SHIFT
    // set CFG_REG
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_CFG_REG, B00000000); // Disable T_SHIFT, P_SHIFT

    // Try some magic
//     i2c_writereg(DPS310_I2C_ADDRESS, 0x0E, 0xA5);
//     i2c_writereg(DPS310_I2C_ADDRESS, 0x0F, 0x96);
//     i2c_writereg(DPS310_I2C_ADDRESS, 0x62, 0x02);
//     i2c_writereg(DPS310_I2C_ADDRESS, 0x0E, 0x00);
//     i2c_writereg(DPS310_I2C_ADDRESS, 0x0F, 0x00);

    // Prime P readout with valid P&T values
    // Request next T sample
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG, B00000010); // New T sample

    delay(100000);
    dps310_read_pressure(); // T
    delay(100000);
    dps310_read_pressure(); // P
//     delay(100000);
//     dps310_read_pressure(); // T
}

int dps310_check(void)
{
    // Get product ID
    int id = i2c_readreg(DPS310_I2C_ADDRESS, DPS310_PRDID);
    return (DPS310_ID==id);
}

void dps310_read_pressure(void)
{
    int32_t press_raw, temp_raw = 0;
//     float new_temp_fl, new_press_fl;
    int rdy = 0;
    int data[3];

    // Check if any sample is ready
    rdy = i2c_readreg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG);

    if (rdy & T_RDY) { // New T sample ready?
        // read out new temp_raw
        i2c_readdata(DPS310_I2C_ADDRESS, DPS310_TMP, data, 3);
        temp_raw = ((data[0]<<16 | data[1]<<8 | data[2])<<8)>>8;
        double temp_raw_sc_new  = (double) temp_raw / 524288;
//         dlpf(&temp_raw_sc, temp_raw_sc_new, 0.9921875); // 128*
        lpfd(&temp_raw_sc, temp_raw_sc_new, 0.984375); // 64*
//         dlpf(&temp_raw_sc, temp_raw_sc_new, 0.96875); // 32*
//         dlpf(&temp_raw_sc, temp_raw_sc_new, 0.9375); // 16*
//         dlpf(&temp_raw_sc, temp_raw_sc_new, 0.875); // 8*

        // Request new P sample
        i2c_writereg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG, B00000001);

    } else if (rdy & P_RDY) { // New P sample ready?
        // read out new press_raw
        i2c_readdata(DPS310_I2C_ADDRESS, DPS310_PSR, data, 3);
        press_raw = ((data[0]<<16 | data[1]<<8 | data[2])<<8)>>8;
        double press_raw_sc_new =  (double) press_raw / 524288;
//         dlpf(&press_raw_sc, press_raw_sc_new, 0.9921875); // 128*
//         dlpf(&press_raw_sc, press_raw_sc_new, 0.984375); // 64*
//         dlpf(&press_raw_sc, press_raw_sc_new, 0.96375); // 32*
        lpfd(&press_raw_sc, press_raw_sc_new, 0.9375); // 16*
//         dlpf(&press_raw_sc, press_raw_sc_new, 0.875); // 8*

        // Request new T sample
        i2c_writereg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG, B00000010);

    }
}

void dps310_tcomp_lpf(void) {
    temp_fl = c0Half + c1 * temp_raw_sc;
//     double new_temp_fl = c0 * 0.5f + c1 * temp_raw_sc;
//     dlpf(&temp_fl, new_temp_fl, 0.96875f);
}

void dps310_pcomp_lpf(void) {
    press_fl =        c00 + press_raw_sc * (c10 + press_raw_sc * (c20 + press_raw_sc * c30)) + (c01 * temp_raw_sc) + temp_raw_sc * press_raw_sc * (c11 + c21 * press_raw_sc);
//     float new_press_fl = c00 + press_raw_sc * (c10 + press_raw_sc * (c20 + press_raw_sc * c30)) + (c01 * temp_raw_sc) + temp_raw_sc * press_raw_sc * (c11 + c21 * press_raw_sc);
//     dlpf(&press_fl,new_press_fl, 0.96875f); // 32*
}


void dps310_readcoeffs(void)
{
    int buffer[18];

    i2c_readdata(DPS310_I2C_ADDRESS, 0x10, buffer, 18);

    c0Half = ((buffer[0]<<8 | buffer[1] )<<12 )>>12; // sign extend by shifting to 32bits and back
    c0Half = c0Half / 2U;
    c1 = ((buffer[1]<<8 | buffer[2] )<<16 )>>16;
    c00 = ((buffer[3]<<16 | buffer[4]<<8 | buffer[5] )<<8 )>>12;
    c10 = ((buffer[5]<<16 | buffer[6]<<8 | buffer[7] )<<12 )>>12;
    c01 = ((buffer[8]<<8 | buffer[9] )<<16 )>>16;
    c11 = ((buffer[10]<<8 | buffer[11] )<<16 )>>16;
    c20 = ((buffer[12]<<8 | buffer[13] )<<16 )>>16;
    c21 = ((buffer[14]<<8 | buffer[15] )<<16 )>>16;
    c30 = ((buffer[16]<<8 | buffer[17] )<<16 )>>16;
}
