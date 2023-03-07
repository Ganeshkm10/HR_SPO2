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
//          printf("Error writing to hrspo2\n in i2c_senddata= status %d \n\r",status);
        if(status == i2cTransferDone)
          {
            printf(" I2C write done in i2c_senddata \n\r");
          }

        }

}
int i2c_recvdata_noP_array(uint8_t devaddr, void *data, uint8_t size, void *reg)
{
  I2C_TransferSeq_TypeDef seq;
      uint8_t *udata = (uint8_t*)data;
      uint8_t *ureg = (uint8_t*)reg;    //Make sure data is uint8_t//Make sure data is uint8_t
      seq.addr = devaddr<<1;
      seq.flags = I2C_FLAG_WRITE;
      seq.buf[0].data = ureg;
      seq.buf[0].len = 1;
      I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &seq);
                while (status == i2cTransferInProgress)
                  {

                    status = I2C_Transfer(I2C0);
      //              printf("Error writing to BME680\n in Write= status %d \n\r",status);
                  if(status == i2cTransferDone)
                    {
                      printf(" I2C write done in i2c_recvdata_noP_array\n\r");

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
                  printf(" I2C read done in i2c_recvdata_noP_array \n\r");

                }

               }

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
void pulseox_read_array(uint8_t loc, char data[], uint8_t len) {
    uint8_t reg[1] = { loc };
    i2c_recvdata_noP_array(PULSEOX_ADDR,data,len,reg);              //Read data
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



  pulseox_write(0x09,0x40);   //Reset
  pulseox_write(0x08,0x70);   //Average 8 samples per FIFO sample, roll-over data
  pulseox_write(0x09,0x03);   //Set to SpO2 mode (RED and IR) -> 2 active LEDs
  pulseox_write(0x0a,0x27);   //Set to 4096nA range, 100 samples/second, and 411us width
  pulseox_write(0x0c,0x1f); //Set LEDs to 6.2mA power (about ~4" detection)
  pulseox_write(0x0d,0x1f);

  pulseox_write(0x11,0x21);   //Set Time Slots (Slot 1 -> Red; Slot 2 -> Infrared)

  //Clear FIFO
  pulseox_write(0x04,0x00);
  pulseox_write(0x05,0x00);
  pulseox_write(0x06,0x00);

  printf(" hrpso2_init done \n\r");
}





int led_arr[30*5*2];
float r;

//============================================================================
// PULSEOX_CHECK
//  * THEORETICALLY checks data from the FIFO.
//  * Adapted from https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
//    (SPARKFUN's OPEN SOURCE ARDUINO LIBRARY)
//============================================================================
void pulseox_check(void)
{
    //Read register FIDO_DATA in (3-byte * number of active LED) chunks
    //Until FIFO_RD_PTR = FIFO_WR_PTR
    uint8_t readPointer  = pulseox_simple_read(0x06);
    uint8_t writePointer = pulseox_simple_read(0x04);

    int numberOfSamples = 0;

    //Do we have new data?
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    //3 bytes per sample, 2 LEDs
    char pulseox_buf[6];
    pulseox_read_array(0x07, pulseox_buf,6);

    int IR = (pulseox_buf[0] << 16) | (pulseox_buf[1] << 8) | (pulseox_buf[2]);
    IR &= 0x3ffff;
    int Rd = (pulseox_buf[3] << 16) | (pulseox_buf[4] << 8) | (pulseox_buf[5]);
    Rd &= 0x3ffff;
    printf("R%6d  I%6d\n",Rd,IR);

    printf("buffer values %d %d %d %d %d %d\n\r",pulseox_buf[0],pulseox_buf[1],pulseox_buf[2],pulseox_buf[3],pulseox_buf[4],pulseox_buf[5]);

    for(int i = 300; i > 0; i -= 2) {
      led_arr[i]   = led_arr[i-2];
      led_arr[i-1] = led_arr[i-3];
    }
    led_arr[0] = Rd;
    led_arr[1] = IR;
}

int red_avg;
int red_min;
//============================================================================
// GET_SPO2
//  * Gives user SpO2 values from a regression curve calibrated to a
//    commercial pulse ox.
//============================================================================
int get_spo2(void) {
    //Calculate R Value for SpO2
    int ir_min = 16777216;
    int ir_max = 0;
    red_min = 16777216;
    int red_max = 0;
    red_avg = 0;
    //red & ir min and max
    for(int i = 0; i < 300; i++) {
      if(i%2 == 0){
        red_avg += led_arr[i];
        if(led_arr[i] < red_min)
          red_min = led_arr[i];
        if(led_arr[i] > red_max)
          red_max = led_arr[i];
      }
      else{
            if(led_arr[i] < ir_min)
                ir_min = led_arr[i];
            if(led_arr[i] > ir_max)
                ir_max = led_arr[i];
      }
    }
    red_avg /= 150;

    if(ir_min < 1500 || red_min < 1500) {
        printf("Wrist Not Detected\n");
        //return(-1);
    }

    printf("%d %d %d\n",red_min,red_avg,red_max);

    float r_AC = ((float)(ir_max - ir_min) / (float)(red_max - red_min));
    float r_DC = ((float)red_min / (float)(ir_min));
    r = r_AC * r_DC;
    int spo2 = (float)(-3.1339*r + 99);
    printf("%.4f\n\r",r);
    printf("%d\n\r",spo2);
    return(spo2);
    printf("%.4f\n",spo2);
}

//
int count = 0;
int sampl_hr[7] = {70,70,70,70,70,70,70};
int hr_avg;
//============================================================================
// GET_HR
//  * Gives user HR. Works by looking for a peak (one value surrounded by
//    several smaller values. Then, takes average of last few samples to
//    consider noise and bad measurements.
//============================================================================
int get_HR(void) {
  //Check if at a peak
  if(led_arr[8] > led_arr[0] && led_arr[8] > led_arr[2]
    && led_arr[8] > led_arr[4]
    && led_arr[8] > led_arr[6]
    && led_arr[8] > led_arr[14]
    && led_arr[8] > led_arr[16]
    && led_arr[8] > red_avg) {

    //Check for realistic pulses
    // [36BPM to 120BPM]
    // Realistically, people are unlikely to have values that surpass
    // these without being in the hospital.
    if(count > 15 && count < 50 && red_min > 1500) {

      //Find average HR for last 7 samples
      hr_avg = 0;
      for(int i = 6; i > 0; i--) {
        hr_avg += sampl_hr[i];
        sampl_hr[i] = sampl_hr[i-1];
      }
      sampl_hr[0] = 1800/count;
      hr_avg += sampl_hr[0];
      printf("Heart Rate: %d\n\r",hr_avg/10);
    }
      printf("Time: %d\n\r",count);
    count = 0;
    }

  //Reset the counter and return the final heartrate
    count++;
  return(hr_avg/7);
}
