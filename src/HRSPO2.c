/*
 * HRSPO2.c
 *
 *  Created on: 06-Mar-2023
 *      Author: Windows
 */
#include <stdio.h>
#include <stdint.h>
#include "em_device.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "HRSPO2.h"
#include "gpio.h"



I2C_TypeDef *i2c = I2C0;

void i2c_init(void)
{
  CMU_ClockEnable(cmuClock_I2C0, true);
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
  i2cInit.freq = I2C_FREQ_STANDARD_MAX;
  I2C_Init(i2c, &i2cInit);
}

uint8_t buf[2]; // Buffer to store configuration data
uint8_t rxBuffer[MAXFAST_ARRAY_SIZE];
uint8_t txBuffer[2]={0};


bool readValue(uint8_t command)
{
    I2C_TransferSeq_TypeDef seq;
    txBuffer[0]=command;
    seq.addr = I2C_ADDRESS<<1;
    seq.flags = I2C_FLAG_WRITE;
    seq.buf[0].data = txBuffer;
    seq.buf[0].len = 1;
    seq.buf[1].data = rxBuffer;
    seq.buf[1].len = 6;
    I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &seq);
          while (status == i2cTransferInProgress)
            {

              status = I2C_Transfer(I2C0);
//              printf("Error writing to BME680\n in Write= status %d \n\r",status);
            if(status == i2cTransferDone)
              {
                printf(" I2C write done in read \n\r");
                printf(" rxbuffer values = %d \n\r",rxBuffer[0]);
                return true;
              }

            }
          return false;
}
bool writeCommand(uint8_t command)
{
  I2C_TransferSeq_TypeDef seq;
  txBuffer[0]=command;
  seq.addr = I2C_ADDRESS<<1;
  seq.flags = I2C_FLAG_WRITE;
  seq.buf[0].data = txBuffer;
    seq.buf[0].len = 2;
    I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &seq);
      while (status == i2cTransferInProgress)
        {

          status = I2C_Transfer(I2C0);
//          printf("Error writing to BME680\n in Write= status %d \n\r",status);
        if(status == i2cTransferDone)
          {
            printf(" I2C write done in write \n\r");
            return true;
          }

        }
      return false;
}

static void delayApprox(int delay)
{
  volatile int i;

  for (i = 0; i < delay; ) {
      i=i+1;
  }

} // delayApprox()




void hrspo2_init()
{

  hrspo2_mfio_enable();
  hrspo2_reset_disable();
  delayApprox(4000);
  hrspo2_reset_enable();
  delayApprox(3500000);
//  hrspo2_mfio_disable();
  delayApprox(20000);

  printf("init donme \n\r");
}






void read_hrspo2value()
{

  //read the device mode
  int device_mode = readValue(0x02);
  printf(" device mode = %d \n\r",device_mode);

  readValue(0x00);
//  if (!writeCommand(0x0D) || !writeCommand(0x03)) {
//      printf("error in config\n\r");
//    }
  writeCommand(0x01);
   readValue(0X12);
   writeCommand(0x12);
      // Read heart rate data
            uint16_t heartRate = 0;
            uint16_t confidence = 0;
            uint16_t oxygen = 0;
            uint16_t status = 0;
      // Heart Rate formatting
           heartRate = ((uint16_t)(rxBuffer[0])) << 8;
          heartRate |= (rxBuffer[1]);
          heartRate /= 10;

          // Confidence formatting
           confidence = rxBuffer[2];

          //Blood oxygen level formatting
          oxygen = ((uint16_t)(rxBuffer[3])) << 8;
          oxygen |= rxBuffer[4];
          oxygen /= 10;

          //"Machine State" - has a finger been detected?
           status = rxBuffer[5];

      printf(" SPO2 = %d , Heart rate = %d confidence = %d status = %d \n\r",oxygen, heartRate, confidence, status);


}
