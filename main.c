/* Name: main.c
 * Author: Zhyhariev MIkhail
 * License: MIT
 */

// Default libraries
#include <avr/io.h>
#include <util/delay.h>

// Custom libraries
#include "lcd/lcd.h"
#include "mpu6050/mpu6050.h"
#include "bmp180/bmp180.h"
#include "telemetry-processor/telemetry_processor.h"
#include "filters/complimentary/complimentary_filter.h"
#include "filters/exponential/exponential_filter.h"

int main(void)
{
    // Initialize all libraries
    LCDInit(8);
    LCDClear();
    MPU6050_Init();
    BMP180_Init();
    Telemetry_init();

    LCDWriteStringXY(0, 0, "Hello");

    while (1) {

    }
    return 0;
}
