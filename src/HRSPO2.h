/*
 * HRSPO2.h
 *
 *  Created on: 05-Mar-2023
 *      Author: Windows
 */

#ifndef SRC_HRSPO2_H_
#define SRC_HRSPO2_H_

#include <stdint.h>

#define PULSEOX_ADDR 0x55 // I2C address of the SEN15219 sensor
// I2C peripheral and device addresses
//#define I2C_ADDRESS 0x57
#define I2C_PORT gpioPortC
#define SDA_PIN 11
#define SCL_PIN 10

#define READ_DATA_OUTPUT 0x12
#define READ_DATA 0x00

#define MAXFAST_ARRAY_SIZE 6
#define MAXFAST_EXTENDED_DATA 5

// Register addresses for HR and SPO2 values
#define HR_VALUE_MSB 0x05
#define HR_VALUE_LSB 0x04
#define SPO2_VALUE_MSB 0x07
#define SPO2_VALUE_LSB 0x06

// Register addresses for mode control and LED power
#define MODE_CONTROL 0x09
#define LED_POWER 0x0C

// BMP mode configuration values
#define BMP_MODE 0x02
#define LED_PW_2MA 0x1F
int i2c_recvdata_noP_array(uint8_t devaddr, uint8_t *data_write, uint8_t size, uint8_t *data);
void pulseox_write(uint8_t familyByte, uint8_t indexByte,uint8_t writebyte);
uint8_t pulseox_simple_read(uint8_t reg);
void pulseox_read_array(uint8_t familyByte, uint8_t indexByte, uint8_t numOfReads, uint8_t data[] );
void i2c_recvdata_noP(uint8_t devaddr, void *data, uint8_t size);
void i2c_senddata(uint8_t devaddr, const void *data, uint8_t size);

void read_hrspo2value(void);

void i2c_init(void);

void hrspo2_init(void);



#endif /* SRC_HRSPO2_H_ */
