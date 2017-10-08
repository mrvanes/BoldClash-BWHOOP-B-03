#include "drv_dps310.h"
// #include "drv_softi2c.h"
#include "drv_i2c.h"
#include "drv_time.h"
#include "config.h"
#include "hardware.h"
#include "binary.h"

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

float c0, c1, c00, c10, c01, c11, c20, c21, c30;
float press_raw_sc, temp_raw_sc, press_fl;

void dps310_init(void)
{
    int press_raw, temp_raw = 0;
    int read[3];

    // send reset command
    i2c_writereg( DPS310_I2C_ADDRESS , DPS310_RESET , B10001001 );

    // blocking - wait for registers
    do
        delay(10000);
    while (!(i2c_readreg(DPS310_I2C_ADDRESS , DPS310_MEAS_CFG)&B11000000));

    // Wait for coefficients to be ready
    do
        delay(10000);
    while (!i2c_readreg( DPS310_I2C_ADDRESS , DPS310_MEAS_CFG )&B10000000);

    dps310_readcoeffs();

    //int temp_sensor_type = i2c_readreg(DPS310_I2C_ADDRESS, DPS310_COEF_SRCE)&B10000000;
    int temp_sensor_type = B10000000; // The sensor on bwhoop seems to report temp_sensor_type wrong? Override with B10000000.

    // pressure config
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_PSR_CFG, B00110110);// 8 meas/sec | 64 times oversampling, needs P_SHIFT
    // temp config
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_TMP_CFG, B00110110|temp_sensor_type); // 8 meas/sec // 64 times oversampling, needs T_SHIFT
    // set CFG_REG
//     i2c_writereg(DPS310_I2C_ADDRESS, DPS310_CFG_REG, B00001110); // Enable T_SHIFT, P_SHIFT, FIFO
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_CFG_REG, B00001100); // Enable T_SHIFT, P_SHIFT
    // set MEAS_CFG
//     i2c_writereg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG, B00000111); // Enable continuous T and P measurements via FIFO

    // Request intial samples
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG, B00000010); // New T sample
    do
        delay(10000);
    while(!i2c_readreg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG)&B00100000); // T sample ready
//     int temp_raw = i2c_readregbytes(DPS310_I2C_ADDRESS, DPS310_TMP, 3);
    i2c_readdata(DPS310_I2C_ADDRESS, DPS310_TMP, read, 3);
    temp_raw = (read[0]<<16) + (read[1]<<8) + read[2];
    temp_raw<<=8; temp_raw>>=8;
    temp_raw_sc  = (float) temp_raw / 1040384;
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG, B00000001); // New P sample
    do
        delay(10000);
    while(!i2c_readreg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG)&B00010000); // P sample ready
    i2c_readdata(DPS310_I2C_ADDRESS, DPS310_PSR, read, 3);
    press_raw = (read[0]<<16) + (read[1]<<8) + read[2];
    press_raw<<=8; press_raw>>=8;
    press_raw_sc  = (float) press_raw / 1040384;

    // Request next T samples
    i2c_writereg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG, B00000010); // New T sample

}

int dps310_check(void)
{
    // Get product ID
    int id = i2c_readreg(DPS310_I2C_ADDRESS, DPS310_PRDID);
    return (DPS310_ID==id);
}

float dps310_read_pressure(void)
{
    int press_raw, temp_raw = 0;
    int meas = 0;
    int read[3];

    // Check if any sample is ready
    meas = i2c_readreg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG);

    // Get latest P or T data
    if (meas&B00100000) { // New T samples ready?
        // read out new temp_raw
        i2c_readdata(DPS310_I2C_ADDRESS, DPS310_TMP, read, 3);
        temp_raw = (read[0]<<16) + (read[1]<<8) + read[2];
        temp_raw<<=8; temp_raw>>=8;
        temp_raw_sc  = (float) temp_raw / 1040384;

        // Request new P sample
        i2c_writereg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG, B00000001);

    } else if (meas&B00010000) { // New P samples ready?

        // read out new press_raw
        i2c_readdata(DPS310_I2C_ADDRESS, DPS310_PSR, read, 3);
        press_raw = (read[0]<<16) + (read[1]<<8) + read[2];
        press_raw<<=8; press_raw>>=8;
        press_raw_sc =  (float) press_raw / 1040384;

        // Request new T sample
        i2c_writereg(DPS310_I2C_ADDRESS, DPS310_MEAS_CFG, B00000010);

    } else { // Return old value
        return press_fl;
    }

    press_fl = c00 + press_raw_sc * (c10 + press_raw_sc * (c20 + press_raw_sc * c30)) + (c01 * temp_raw_sc) + temp_raw_sc * press_raw_sc * (c11 + c21 * press_raw_sc);

    return press_fl;
}
void dps310_readcoeffs(void)
{
    c0  = dps310_readcoeff_number(0x10);
    c1  = dps310_readcoeff_number(0x11);
    c00 = dps310_readcoeff_number(0x13);
    c10 = dps310_readcoeff_number(0x15);
    c01 = dps310_readcoeff_number(0x18);
    c11 = dps310_readcoeff_number(0x1a);
    c20 = dps310_readcoeff_number(0x1c);
    c21 = dps310_readcoeff_number(0x1e);
    c30 = dps310_readcoeff_number(0x20);
}

int dps310_readcoeff_number(int num)
{
    int read[3];
    int ans = 0;

    // c01  c11 c20 c21 c30
    if (num >= 0x18) {
        i2c_readdata(DPS310_I2C_ADDRESS, num, read, 2);
        ans = (read[0]<<8) + read[1];
        ans<<=16; ans>>=16; // sign extend
    }
    // c10
    if (num == 0x15) {
        i2c_readdata(DPS310_I2C_ADDRESS, num, read, 3);
        ans = (read[0]<<16) + (read[1]<<8) + read[2];
        ans<<=12; ans>>=12; // sign extend
    }
    //c00
    if (num == 0x13) {
        i2c_readdata(DPS310_I2C_ADDRESS, num, read, 3);
        ans = (read[0]<<16) + (read[1]<<8) + read[2];
        ans<<=8;ans>>=12;
    }
    //c0
    if (num == 0x10) {
        i2c_readdata(DPS310_I2C_ADDRESS, num, read, 2);
        ans = (read[0]<<8) + read[1];
        ans<<=16;  ans>>=20;
    }
    //c1
    if (num == 0x11) {
        i2c_readdata(DPS310_I2C_ADDRESS, num, read, 2);
        ans = (read[0]<<8) + read[1];
        ans<<=20; ans>>=20;
    }
    return ans;
}
