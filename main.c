/* Name: main.c
 * Author: Zhyhariev MIkhail
 * License: MIT
 */

// Default libraries
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

// Custom libraries
#include "lcd/lcd.h"
#include "mpu6050/mpu6050.h"
#include "bmp180/bmp180.h"
#include "telemetry-processor/telemetry_processor.h"
// #include "filters/complimentary/complimentary_filter.h"
// #include "filters/exponential/exponential_filter.h"

// Amount of stream elements
#define COUNT               11

// Stream identifiers
#define MPU_TEMP_REG        1
#define MPU_ACCEL           2
#define MPU_GYRO            3
#define MPU_TEMP            4
#define MPU_ACCEL_ANGLE     5
#define MPU_GYRO_ANGLE      6
#define BMP_TEMP_REG        7
#define BMP_PRES_REG        8
#define BMP_TEMP            9
#define BMP_PRES            10
#define BMP_HEIGHT          11

int main(void)
{
    // Initialize all libraries
    Telemetry_init();
    LCDInit(8);
    LCDClear();
    MPU6050_Init();
    BMP180_Init();

    // Callback functions
    getter functions[COUNT] = {
        (getter)MPU6050_getTemp,
        (getter)MPU6050_getAccel,
        (getter)MPU6050_getGyro,
        (getter)MPU6050_countTemp,
        (getter)MPU6050_getGyroAngles,
        (getter)MPU6050_getAccelAngles,
        (getter)BMP180_readUncompTemp,
        (getter)BMP180_readUncompPressure,
        (getter)BMP180_getTemp,
        (getter)BMP180_getPressure,
        (getter)BMP180_getHeight
    };

    // Types
    u8 types[COUNT] = {
        TWO_BYTE,
        ARRAY,
        ARRAY,
        FLOAT,
        ARRAY,
        ARRAY,
        FOUR_BYTE,
        FOUR_BYTE,
        ONE_BYTE,
        FOUR_BYTE,
        FLOAT
    };

    // Identifiers
    u8 ids[COUNT] = {
        MPU_TEMP_REG,
        MPU_ACCEL,
        MPU_GYRO,
        MPU_TEMP,
        MPU_ACCEL_ANGLE,
        MPU_GYRO_ANGLE,
        BMP_TEMP_REG,
        BMP_PRES_REG,
        BMP_TEMP,
        BMP_PRES,
        BMP_HEIGHT
    };

    // Information about array (type and length)
    u8 arr_type[] = {TWO_BYTE, TWO_BYTE, ONE_BYTE, ONE_BYTE};
    u8 arr_len[] = {3, 3, 3, 3};

    // Initialize stream items
    telemetry_item* items = Telemetry_getItems(COUNT, ids, functions, types, arr_len, arr_type);

    LCDWriteStringXY(0, 0, "Waiting for id..");

    while (1) {
        u8 id = Telemetry_streamData(items, COUNT);
        LCDWriteStringXY(0, 1, "id=")
        LCDWriteIntXY(3, 1, id, 2);
        _delay_ms(50);
    }
    return 0;
}
