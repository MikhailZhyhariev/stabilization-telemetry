#ifndef I2C
#define I2C

#include <avr/io.h>

#define SDA			1
#define SCL			0
#define I2C_PORT	PORTC
#define I2C_PIN		PINC
#define I2C_DDR		DDRC
#define I2C_DELAY	_delay_us(4);

void i2c_init(void);

void i2c_start_cond(void);

void i2c_restart_cond(void);

void i2c_stop_cond(void);

unsigned char i2c_send_byte(unsigned char data);

unsigned char i2c_get_byte(unsigned char last_byte);

#endif
