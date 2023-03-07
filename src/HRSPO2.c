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


void i2c_recvdata_noP(uint8_t devaddr, void *data, uint8_t size)
{
    I2C_TransferSeq_TypeDef seq;
    uint8_t *udata = (uint8_t*)data;    //Make sure data is uint8_t
    seq.addr = devaddr<<1;
    seq.flags = I2C_FLAG_WRITE;
    seq.buf[0].data = data;
    seq.buf[0].len = size;
    I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &seq);
              while (status == i2cTransferInProgress)
                {

                  status = I2C_Transfer(I2C0);
    //              printf("Error writing to BME680\n in Write= status %d \n\r",status);
                if(status == i2cTransferDone)
                  {
                    printf(" I2C write done in i2c_recvdata_noP \n\r");

                  }

                }
       seq.addr = PULSEOX_ADDR<<1;
       seq.flags = I2C_FLAG_READ;
       seq.buf[0].data = udata;
       seq.buf[0].len = size;
       status = I2C_TransferInit(i2c, &seq);
       while (status == i2cTransferInProgress)
           {

            status = I2C_Transfer(I2C0);
            //printf("Error writing to BME680\n in Write= status %d \n\r",status);
           if(status == i2cTransferDone)
              {
                printf(" I2C read done in i2c_recvdata_noP \n\r");

              }

             }


}
void i2c_senddata(uint8_t devaddr, const void *data, uint8_t size)
{
  I2C_TransferSeq_TypeDef seq;
  uint8_t *udata = (uint8_t*)data;    //Make sure data is uint8_t
  seq.addr = devaddr<<1;
  seq.flags = I2C_FLAG_WRITE;
  seq.buf[0].data = udata;
  seq.buf[0].len = size;
    I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &seq);
      while (status == i2cTransferInProgress)
        {

          status = I2C_Transfer(I2C0);
          printf("Error writing to hrspo2\n in i2c_senddata= status %d \n\r",status);
        if(status == i2cTransferDone)
          {
            printf(" I2C write done in i2c_senddata \n\r");
          }

        }

}
int i2c_recvdata_noP_array(uint8_t devaddr, uint8_t *data_write, uint8_t size, uint8_t *data)
{
  I2C_TransferSeq_TypeDef seq;
      uint8_t *udata = (uint8_t*)data_write;
      uint8_t *ureg = (uint8_t*)data;    //Make sure data is uint8_t//Make sure data is uint8_t
      seq.addr = devaddr<<1;
      seq.flags = I2C_FLAG_WRITE;
      seq.buf[0].data = udata;
      seq.buf[0].len = 2;
      I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &seq);
                while (status == i2cTransferInProgress)
                  {

                    status = I2C_Transfer(I2C0);
      //              printf("Error writing to BME680\n in Write= status %d \n\r",status);
                  if(status == i2cTransferDone)
                    {
//                      printf(" I2C write done in i2c_recvdata_noP_array\n\r");

                    }

                  }
         seq.addr = PULSEOX_ADDR<<1;
         seq.flags = I2C_FLAG_READ;
         seq.buf[0].data = ureg;
         seq.buf[0].len = size;
        status = I2C_TransferInit(i2c, &seq);
         while (status == i2cTransferInProgress)
             {

              status = I2C_Transfer(I2C0);
//              printf("Error writing to BME680\n in Write= status %d \n\r",status);
             if(status == i2cTransferDone)
                {
//                  printf(" I2C read done in i2c_recvdata_noP_array \n\r");

                }

               }
         printf(" ureg values %d %d %d %d %d %d\n\r",ureg[0],ureg[1],ureg[2],ureg[3],ureg[4],ureg[5]);

  return 0;
}
//============================================================================
// PULSEOX_WRITE
//  * Send data to the pulse-oximeter
//  * Used mainly for configuration
//============================================================================
void pulseox_write(uint8_t familyByte, uint8_t indexByte,uint8_t writebyte) {
  uint8_t pulseox_writedata[3] = {familyByte,indexByte,writebyte};
  i2c_senddata(PULSEOX_ADDR, pulseox_writedata,3);
}

//============================================================================
// PULSEOX_SIMPLE_READ
//  * Read a single byte from the pulse oximeter.
//============================================================================
uint8_t pulseox_simple_read(uint8_t reg) {
    uint8_t pulseox_data[1] = {reg};
    i2c_recvdata_noP(PULSEOX_ADDR, pulseox_data, 1);
    return 0;
}

//============================================================================
// PULSEOX_READ_ARRAY
//  * Read individual data from the pulse oximeter.
//  * Result is stored into the data buffer
//============================================================================
void pulseox_read_array(uint8_t familyByte, uint8_t indexByte, uint8_t numOfReads, uint8_t *data ) {
    uint8_t write_data[2] = {familyByte, indexByte };
    i2c_recvdata_noP_array(PULSEOX_ADDR,write_data,numOfReads,data);              //Read data
}

static void delayApprox(int delay)
{
  volatile int i;

  for (i = 0; i < delay; ) {
      i=i+1;
  }

} // delayApprox()

//============================================================================
// PULSEOX_SETUP
//  * Resets pulse ox
//  * Sets to average 32 samples per FIFO sample, roll-over data
//============================================================================
void hrspo2_init(void) {

  hrspo2_mfio_enable();
    hrspo2_reset_disable();
    delayApprox(4000);
    hrspo2_reset_enable();
    delayApprox(3500000);
  //  hrspo2_mfio_disable();
    delayApprox(20000);



  pulseox_write(0x10,0x00,0x02);//set output mode
  pulseox_write(0x10,0x01,0x01);//set fifo threshold
  pulseox_write(0x52,0x00,0x01);//set agcalgocontrol
  pulseox_write(0x44,0x03,0x01);//enable max30101
  pulseox_write(0x52,0x02,0x01);// set maxim fast algo control
  delayApprox(3500000);
  printf(" hrpso2_init done \n\r");
}





void read_hrspo2value()
{
//  uint8_t rxBuffer[MAXFAST_ARRAY_SIZE] = {};
      // Read heart rate data
            uint16_t heartRate = 0;
            uint16_t confidence = 0;
            uint16_t oxygen = 0;
            uint16_t status = 0;

            pulseox_read_array(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE, rxBuffer);

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
