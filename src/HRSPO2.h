/*
 * HRSPO2.h
 *
 *  Created on: 05-Mar-2023
 *      Author: Windows
 */

#ifndef SRC_HRSPO2_H_
#define SRC_HRSPO2_H_

#include <stdint.h>

#define I2C_ADDRESS 0x55 // I2C address of the SEN15219 sensor
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
void i2c_init(void);
bool writeCommand(uint8_t command);
//bool readValue(uint8_t command, uint16_t* value);
bool readValue(uint8_t command);
void hrspo2_init();
void read_hrspo2value();

#endif /* SRC_HRSPO2_H_ */
