/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.

   Edited by : Ganesh KM for Assignment 3

 */


// *****************************************************************************
// Students:
// We will be creating additional functions that configure and manipulate GPIOs.
// For any new GPIO function you create, place that function in this file.
// *****************************************************************************

#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>

#include "gpio.h"

// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

#define LED0_port  gpioPortF // change to correct ports and pins
#define LED0_pin   4
#define LED1_port  gpioPortF
#define LED1_pin   5

#define SI7021_PORT gpioPortD
#define SI7021_PIN  15

#define I2C0_SCL_PORT  gpioPortC
#define I2C0_SCL_PIN   10
#define I2C0_SDA_PORT  gpioPortC
#define I2C0_SDA_PIN   11

#define RESET_PORT_HRSPO2 gpioPortD
#define RESET_PIN_HRSPO2 10
#define MFIO_PORT_HRSPO2 gpioPortD
#define MFIO_PIN_HRSPO2 11




// Set GPIO drive strengths and modes of operation
void gpioInit()
{

  // Student Edit:

	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);

	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

	GPIO_PinModeSet( SI7021_PORT, SI7021_PIN, gpioModePushPull, true ); //SI7021 sensor initialization

	GPIO_DriveStrengthSet(RESET_PORT_HRSPO2, gpioDriveStrengthStrongAlternateStrong);
	  GPIO_PinModeSet(RESET_PORT_HRSPO2, RESET_PIN_HRSPO2, gpioModePushPull, false);

	  GPIO_DriveStrengthSet(MFIO_PORT_HRSPO2, gpioDriveStrengthStrongAlternateStrong);
	      GPIO_PinModeSet(MFIO_PORT_HRSPO2, MFIO_PIN_HRSPO2, gpioModePushPull, false);




} // gpioInit()


void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}


void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}

void gpioSi7021Enable()
{
  GPIO_PinOutSet(SI7021_PORT,SI7021_PIN);
}

void gpioSi7021Disable()
{
  GPIO_PinOutClear(SI7021_PORT,SI7021_PIN);
}

void hrspo2_reset_enable()
{
  GPIO_PinOutSet(RESET_PORT_HRSPO2,RESET_PIN_HRSPO2);
}
void hrspo2_reset_disable()
{
  GPIO_PinOutClear(RESET_PORT_HRSPO2,RESET_PIN_HRSPO2);
}
void hrspo2_mfio_enable()
{
  GPIO_PinOutSet(MFIO_PORT_HRSPO2,MFIO_PIN_HRSPO2);
}
void hrspo2_mfio_disable()
{
  GPIO_PinOutClear(MFIO_PORT_HRSPO2,MFIO_PIN_HRSPO2);
}

/* need to check --> not sure about disabling I2C after reading measurement
void gpioI2cSdaDisable() //
{
    GPIO_PinOutClear( I2C0_SDA_PORT, I2C0_SDA_PIN );
}


void gpioI2cSclDisable()
{
    GPIO_PinOutClear( I2C0_SCL_PORT, I2C0_SCL_PIN );
}
*/



