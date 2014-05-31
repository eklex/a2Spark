/*
*  Name:      a2Spark
*  Author:    Alexandre Boni
*  Created:   2013/10/26
*  Modified:  2013/11/17
*  Version:   0.7
*  IDE:       Arduino 1.0.4 Digispark
*  Hardware:  Digispark (ATtiny85)
*
*  Pins:     P0: I2C SDA
*            P1: AL1
*            P2: I2C SCK
*            P3: AL2
*            P4: AN2
*            P5: AN1
*
*  Release:
*    0.7
*          Adding comments and layout
*
*    0.2 to 0.6
*          First design of a2Spark
*
*    0.1
*          Creation of this code.
*
*
*
*/

// library TinyWireS used for I2C slave device
// modified for ATtiny85 by rambo on github.com
#include <TinyWireS.h>

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

/*
*****************************************************************
*****************************************************************
*****************************************************************
***                     GLOBAL DEFINES                        ***
*****************************************************************
*****************************************************************
*****************************************************************
*/

/*
*****************************************************************
***                    Global Variables                       ***
*****************************************************************
*/
// Current Firmware Version
#define FIRMWARE_VERSION    (0x07)
// I2C Address
// this address should change to use more than one a2Spark device
// address = [0x03 : 0x77]
#define I2C_ADDRESS_DEVICE  (0x42)

/*
*****************************************************************
***                      I2C Registers                        ***
***                      Offset & Mask                        ***
*****************************************************************
*/
// I2C Address Reg
#define ADDR                  (0x00)
// Firmware Version Register
#define FWVR                  (0x01)
// Analog Configuration Register
#define AN1CR                 (0x04)
#define AN2CR                 (0x14)
#define ANEN                  (0x01<<0)
#define AVEN                  (0x01<<1)
// Average Analog Register
#define AVG1R                 (0x05)
#define AVG2R                 (0x15)
#define AG                    (0x0F<<0)
// Analog Data Register
#define AN1DRL                (0x06)
#define AN2DRL                (0x16)
#define AN1DRH                (0x07)
#define AN2DRH                (0x17)
#define AN_L                  (0xFF<<0)
#define AN_H                  (0x03<<0)
// Alarm Configuration Register
#define AL1CR                 (0x08)
#define AL2CR                 (0x18)
#define ALEN                  (0x01<<0)
#define ALO                   (0x01<<1)
#define ALM                   (0x07<<2)
#define ALM_FE                (0x01<<0)
#define ALM_RE                (0x01<<1)
#define ALM_DE                (ALM_RE | ALM_FE)
#define ALM_LL                (0x01<<2)
#define ALM_HL                (0x05<<0)
// Alarm Set Value Register
#define AL1SVL                (0x09)
#define AL2SVL                (0x19)
#define AL1SVH                (0x0A)
#define AL2SVH                (0x1A)
#define ALS_L                 (0xFF<<0)
#define ALS_H                 (0x03<<0)
// Alarm Clear Value Register
#define AL1CVL                (0x0B)
#define AL2CVL                (0x1B)
#define AL1CVH                (0x0C)
#define AL2CVH                (0x1C)
#define ALC_L                 (0xFF<<0)
#define ALC_H                 (0x03<<0)
// Temperature Register
#define TCR                   (0x10)
#define TEN                   (0x01<<0)
#define TUS                   (0x03<<1)
#define TUS_C                 (0x00<<1)
#define TUS_F                 (0x01<<1)
#define TUS_K                 (0x01<<2)
#define TDRL                  (0x11)
#define TDRH                  (0x12)
#define TP_L                  (0xFF<<0)
#define TP_H                  (0x03<<0)

/*
*****************************************************************
***                      I2C Registers                        ***
***                      Default Values                       ***
*****************************************************************
*/
// General Registers
#define RESERVED              (0x00)
#define ADDR_DEF              (I2C_ADDRESS_DEVICE)
#define FWVR_DEF              (FIRMWARE_VERSION)
// Analog1 & Alarm1 Registers
#define AN1CR_DEF             (ANEN)
#define AVG1R_DEF             (0x00)
#define AN1DRL_DEF            (0x00)
#define AN1DRH_DEF            (0x00)
#define AL1CR_DEF             (0x00)
#define AL1SVL_DEF            (0x00)
#define AL1SVH_DEF            (0x00)
#define AL1CVL_DEF            (0x00)
#define AL1CVH_DEF            (0x00)
// Temperature Registers
#define TCR_DEF               (TEN | TUS_C)
#define TDRL_DEF              (0x00)
#define TDRH_DEF              (0x00)
// Analog1 & Alarm2 Registers
#define AN2CR_DEF             (0x00)
#define AVG2R_DEF             (AVG1R_DEF)
#define AN2DRL_DEF            (AN1DRL_DEF)
#define AN2DRH_DEF            (AN1DRH_DEF)
#define AL2CR_DEF             (AL1CR_DEF)
#define AL2SVL_DEF            (AL1SVL_DEF)
#define AL2SVH_DEF            (AL1SVH_DEF)
#define AL2CVL_DEF            (AL1CVL_DEF)
#define AL2CVH_DEF            (AL1CVH_DEF)

/*
*****************************************************************
***                Buffer Size & Pin Defines                  ***
*****************************************************************
*/
// Number of Analog Input
// = number of alarm output
#define ANALOG_INPUT          (2)
// Analog Averaging Buffer Size
#define ANALOG_BUFF_SIZE      (16)
// Temperature Buffer Size
#define TEMPERATURE_BUFF_SIZE (20)
// Analog1 Input Pin (AN0 = PB5)
#define ANALOG_AN1_ADC        (0)
// Analog2 Input Pin (AN2 = PB4)
#define ANALOG_AN2_ADC        (2)
// Alarm1 Output Pin (PB1)
#define ALARM_AL1_PIN         (1)
// Alarm2 Output Pin (PB3)
#define ALARM_AL2_PIN         (3)

/*
*****************************************************************
***                          Macro                            ***
*****************************************************************
*/
// Check if __n__ is between 0 and ANALOG_INPUT-1
// return a boolean
#define SELECT_CHECK(__n__)  \
        ((__n__ >= 0) && (__n__ < ANALOG_INPUT))

/*
*****************************************************************
*****************************************************************
*****************************************************************
***                    GLOBAL VARIABLES                       ***
*****************************************************************
*****************************************************************
*****************************************************************
*/

/*
*****************************************************************
***                    Analog Variables                       ***
***             ANALOG_INPUT=0 => Analog1 (AN1)               ***
***             ANALOG_INPUT=1 => Analog2 (AN2)               ***
*****************************************************************
*/
// Analog Value in ANnDRH:L 
uint16_t analogVal[ANALOG_INPUT];
// Analog Accumulator for Averaging process
uint16_t analogAcc[ANALOG_INPUT];
// Analog Index for Averaging process
uint8_t  analogIndex[ANALOG_INPUT];
// Analog Buffer Size selected by AVGnR.AG3:0
uint8_t  analogBuffSize[ANALOG_INPUT];
// Analog Buffer for Averaging process
uint16_t analogBuff[ANALOG_INPUT][ANALOG_BUFF_SIZE];

/*
*****************************************************************
***                 Temperature Variables                     ***
*****************************************************************
*/
// Temperature Value in TDRH:L
// this value is signed
int16_t  temperatureVal;
// Temperature Accumulator for Averaging process
uint16_t temperatureAcc;
// Temperature Index for Averaging process
uint8_t  temperatureIndex;
// Temperature Buffer Size defined by TEMPERATURE_BUFF_SIZE
uint8_t  temperatureBuffSize;
// Temperature Buffer for Averaging process
uint16_t temperatureBuff[TEMPERATURE_BUFF_SIZE];

/*
*****************************************************************
***                      Alarm Variables                      ***
*****************************************************************
*/
// Alarm State in ALnCR.ALO
uint8_t  alarmOutput[ANALOG_INPUT];

/*
*****************************************************************
***                  I2C Protocol Variables                   ***
*****************************************************************
*/
// I2C Register Index 
// track the current register pointer position
volatile uint8_t i2cRegIndex;
// I2C Core Registers
volatile uint8_t i2cRegs[] =
{
  ADDR_DEF,      //0x00
  FWVR_DEF,      //0x01
  RESERVED,      //0x02
  RESERVED,      //0x03
  AN1CR_DEF,     //0x04
  AVG1R_DEF,     //0x05
  AN1DRL_DEF,    //0x06
  AN1DRH_DEF,    //0x07
  AL1CR_DEF,     //0x08
  AL1SVL_DEF,    //0x09
  AL1SVH_DEF,    //0x0A
  AL1CVL_DEF,    //0x0B
  AL1CVH_DEF,    //0x0C
  RESERVED,      //0x0D
  RESERVED,      //0x0E
  RESERVED,      //0x0F
  TCR_DEF,       //0x10
  TDRL_DEF,      //0x11
  TDRH_DEF,      //0x12
  RESERVED,      //0x13
  AN2CR_DEF,     //0x14
  AVG2R_DEF,     //0x15
  AN2DRL_DEF,    //0x16
  AN2DRH_DEF,    //0x17
  AL2CR_DEF,     //0x18
  AL2SVL_DEF,    //0x19
  AL2SVH_DEF,    //0x1A
  AL2CVL_DEF,    //0x1B
  AL2CVH_DEF,    //0x1C
  RESERVED,      //0x1D
  RESERVED,      //0x1E
  RESERVED       //0x1F
};

/*
*****************************************************************
*****************************************************************
*****************************************************************
***                  FUNCTION PROTOTYPES                      ***
*****************************************************************
*****************************************************************
*****************************************************************
*/

/*
*****************************************************************
***                  I2C Protocol Functions                   ***
*****************************************************************
*/
/*
*****************************************************************
*  Procedure:    delay
*  Description:  This function overrides the common delay()
*                provided by the standard Arduino Library.
*                In this code, the delay is provided by tws_delay().
*  Input:        Delay value in millisecond.
*****************************************************************
*/
void delay(int);
/*
*****************************************************************
*  Procedure:    sendEvent
*  Description:  This function is called when the I2C master
*                request a read cycle. One byte is sent from the
*                i2cRegs[i2cRegIndex], then the I2C index is
*                incremented.
*****************************************************************
*/
void sendEvent(void);
/*
*****************************************************************
*  Procedure:    receiveEvent
*  Description:  This function is called when the I2C master
*                send data. The FIFO buffer size is defined by
*                TWI_RX_BUFFER_SIZE, then the buffer is able to
*                store (TWI_RX_BUFFER_SIZE/8) bytes.
*  Input:        Number of byte in the FIFO buffer.
*****************************************************************
*/
void receiveEvent(uint8_t);
/*
*****************************************************************
*  Procedure:    updateRegs
*  Description:  This function is called at every main cycle in
*                order to keep read-only bits and registers
*                cleared.
*****************************************************************
*/
void updateRegs(void);

/*
*****************************************************************
***                 Initialization Functions                  ***
*****************************************************************
*/
/*
*****************************************************************
*  Procedure:    initSensor
*  Description:  This function is called in the setup function
*                in order to initialize the selected analog
*                channel.
*  Input:        Analog channel. Shall be 0 or 1 for AN1 or AN2.
*****************************************************************
*/
void initSensor(uint8_t);
/*
*****************************************************************
*  Procedure:    initAlarm
*  Description:  This function is called in the setup function
*                in order to initialize the selected alarm
*                channel.
*  Input:        Alarm channel. Shall be 0 or 1 for AL1 or AL2.
*****************************************************************
*/
void initAlarm(uint8_t);
/*
*****************************************************************
*  Procedure:    initTemperature
*  Description:  This function is called in the setup function
*                in order to initialize the internal temperature
*                sensor.
*****************************************************************
*/
void initTemperature(void);

/*
*****************************************************************
***                 Sensor & Alarm Functions                  ***
*****************************************************************
*/
/*
*****************************************************************
*  Procedure:    readSensor
*  Description:  This function reads the selected analog channel,
*                with averaging or not, then updates the ANnDRH:L
*                registers.
*  Input:        Analog channel. Shall be 0 or 1 for AN1 or AN2.
*****************************************************************
*/
void readSensor(uint8_t);
/*
*****************************************************************
*  Procedure:    checkAlarm
*  Description:  This function checks the selected alarm channel
*                by comparing the analog value with the set value
*                (ALnSVH:L) and the clear value (ALnCVH:L). Then
*                it updates the ALnCR.ALO bit.
*  Input:        Alarm channel. Shall be 0 or 1 for AL1 or AL2.
*****************************************************************
*/
void checkAlarm(uint8_t);
/*
*****************************************************************
*  Procedure:    updateAlarm
*  Description:  This function updates the digital alarm output
*                according to the selected mode in ALnCR.ALM2:0
*                and the alarm state (ALnCR.ALO).
*  Input:        Alarm channel. Shall be 0 or 1 for AL1 or AL2.
*****************************************************************
*/
void updateAlarm(uint8_t);
/*
*****************************************************************
*  Procedure:    readTemperature
*  Description:  This function reads the internal temperature
*                sensor. The raw value (temperature in kelvin)
*                is smoothed with the TEMPERATURE_BUFF_SIZE
*                previous values. Then TDRH:L is updated with
*                the temperature value formatted according to the
*                selected temperature unit in TCR.TUS1:0 (C, F, K).
*****************************************************************
*/
void readTemperature(void);

/*
*****************************************************************
*****************************************************************
*****************************************************************
***                 FUNCTION DEFINITIONS                      ***
*****************************************************************
*****************************************************************
*****************************************************************
*/

/*
*****************************************************************
***                  I2C Protocol Functions                   ***
*****************************************************************
*/

/*
*****************************************************************
*  Procedure:    delay
*  Description:  This function overrides the common delay()
*                provided by the standard Arduino Library.
*                In this code, the delay is provided by tws_delay().
*  Input:        Delay value in millisecond.
*****************************************************************
*/
void delay(int ms)
{
  tws_delay(ms);
}

/*
*****************************************************************
*  Procedure:    sendEvent
*  Description:  This function is called when the I2C master
*                request a read cycle. One byte is sent from the
*                i2cRegs[i2cRegIndex], then the I2C index is
*                incremented.
*****************************************************************
*/
void sendEvent()
{
  // send the selected byte in register to the I2C master  
  TinyWireS.send(i2cRegs[i2cRegIndex]);
  // increment the register position after sending
  i2cRegIndex = (i2cRegIndex+1) % sizeof(i2cRegs);
}

/*
*****************************************************************
*  Procedure:    receiveEvent
*  Description:  This function is called when the I2C master
*                send data. The FIFO buffer size is defined by
*                TWI_RX_BUFFER_SIZE, then the buffer is able to
*                store (TWI_RX_BUFFER_SIZE/8) bytes.
*  Input:        Number of byte in the FIFO buffer.
*****************************************************************
*/
void receiveEvent(uint8_t byteInBuff)
{
  // to process, the FIFO buffer must be filled
  // by one byte at least or by TWI_RX_BUFFER_SIZE bytes max
  if (byteInBuff < 1)                   return;
  if (byteInBuff > TWI_RX_BUFFER_SIZE)  return;

  // let's receive the first byte which sets the register index
  i2cRegIndex = TinyWireS.receive();
  // remove the byte just received from the buffer
  byteInBuff--;
  // if the buffer is now empty, it means that the event was just
  // to set the register index for the next master read
  if (byteInBuff == 0)  return;

  // get received byte and store it in the I2C register
  while(byteInBuff--)
  {
    // override the current value in the register
    i2cRegs[i2cRegIndex % sizeof(i2cRegs)] = TinyWireS.receive();
    // increment the register index for the next received byte
    i2cRegIndex++;
  }
}

/*
*****************************************************************
*  Procedure:    updateRegs
*  Description:  This function is called at every main cycle in
*                order to keep read-only bits and registers
*                cleared.
*****************************************************************
*/
void updateRegs()
{
  i2cRegs[ADDR]    = ADDR_DEF;
  i2cRegs[FWVR]    = FWVR_DEF;
  i2cRegs[AN1CR]  &= (ANEN | AVEN);
  i2cRegs[AN2CR]  &= (ANEN | AVEN);
  i2cRegs[AVG1R]  &= AG;
  i2cRegs[AVG2R]  &= AG;
  i2cRegs[AN1DRL] &= AN_L;
  i2cRegs[AN2DRL] &= AN_L;
  i2cRegs[AN1DRH] &= AN_H;
  i2cRegs[AN2DRH] &= AN_H;
  i2cRegs[AL1CR]  &= (ALEN | ALO | ALM);
  i2cRegs[AL2CR]  &= (ALEN | ALO | ALM);
  i2cRegs[AL1SVL] &= ALS_L;
  i2cRegs[AL2SVL] &= ALS_L;
  i2cRegs[AL1SVH] &= ALS_H;
  i2cRegs[AL2SVH] &= ALS_H;
  i2cRegs[AL1CVL] &= ALC_L;
  i2cRegs[AL2CVL] &= ALC_L;
  i2cRegs[AL1CVH] &= ALC_H;
  i2cRegs[AL2CVH] &= ALC_H;
  i2cRegs[TCR]    &= (TEN | TUS);
  i2cRegs[TDRL]   &= TP_L;
  i2cRegs[TDRH]   &= TP_H;
}

/*
*****************************************************************
***                 Initialization Functions                  ***
*****************************************************************
*/

/*
*****************************************************************
*  Procedure:    initSensor
*  Description:  This function is called in the setup function
*                in order to initialize the selected analog
*                channel.
*  Input:        Analog channel. Shall be 0 or 1 for AN1 or AN2.
*****************************************************************
*/
void initSensor(uint8_t an)
{
  uint16_t j;
  uint8_t tmpReg;

  // check if 'an' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(an))  return;
  
  // get the right average analog reg
  if(an == 0)       tmpReg = AVG1R;
  else if(an == 1)  tmpReg = AVG2R;
  
  // init analog variables
  analogVal[an]      = 0;
  analogAcc[an]      = 0;
  analogIndex[an]    = 0;
  analogBuffSize[an] = (i2cRegs[tmpReg] & AG) + 1;
  
  // init the analog buff
  for(j=0; j<ANALOG_BUFF_SIZE; j++)
  {
    analogBuff[an][j] = 0;
  }
  
  // get the right analog config reg
  if(an == 0)       tmpReg = AN1CR;
  else if(an == 1)  tmpReg = AN2CR;
  if(i2cRegs[tmpReg] & AVEN)
  {
    // init the analog averaging
    for(j=0; j<analogBuffSize[an]; j++)
    {
      readSensor(an);
    }
  } 
}

/*
*****************************************************************
*  Procedure:    initAlarm
*  Description:  This function is called in the setup function
*                in order to initialize the selected alarm
*                channel.
*  Input:        Alarm channel. Shall be 0 or 1 for AL1 or AL2.
*****************************************************************
*/
void initAlarm(uint8_t al)
{
  uint8_t tmpReg;
  
  // check if 'al' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(al))  return;
  
  alarmOutput[al] = 0;
  checkAlarm(al);
  updateAlarm(al);
  
  // get the right alarm output pin
  if(al == 0)       tmpReg = ALARM_AL1_PIN;
  else if(al == 1)  tmpReg = ALARM_AL2_PIN;
  // set the alarm pin in output status
  pinMode(tmpReg, OUTPUT);
}

/*
*****************************************************************
*  Procedure:    initTemperature
*  Description:  This function is called in the setup function
*                in order to initialize the internal temperature
*                sensor.
*****************************************************************
*/
void initTemperature()
{
  uint16_t j;
  
  temperatureVal      = 0;
  temperatureAcc      = 0;
  temperatureIndex    = 0;
  temperatureBuffSize = TEMPERATURE_BUFF_SIZE;
  
  // check if the temperature sensor read is enabled
  if(i2cRegs[TCR] & TEN)
  {
    // init the temperature buffer
    for(j=0; j<temperatureBuffSize; j++)
    {
      readTemperature();
    }
  } 
}

/*
*****************************************************************
***                 Sensor & Alarm Functions                  ***
*****************************************************************
*/

/*
*****************************************************************
*  Procedure:    readSensor
*  Description:  This function reads the selected analog channel,
*                with averaging or not, then updates the ANnDRH:L
*                registers.
*  Input:        Analog channel. Shall be 0 or 1 for AN1 or AN2.
*****************************************************************
*/
void readSensor(uint8_t an)
{
  uint8_t i;
  uint8_t tmpBuffSize;
  uint8_t tmpReg;
  
  // check if 'an' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(an))  return;
  
  // get the right analog config reg
  if(an == 0)       tmpReg = AN1CR;
  else if(an == 1)  tmpReg = AN2CR;
  
  // check if sampling is enabled
  if( !(i2cRegs[tmpReg] & ANEN) )  return;
  
  // check if averaging is enabled
  if( !(i2cRegs[tmpReg] & AVEN) )
  {
    // get the right analog input
    if(an == 0)       tmpReg = ANALOG_AN1_ADC;
    else if(an == 1)  tmpReg = ANALOG_AN2_ADC;
    // averging disabled then just capture one sample
    analogVal[an] = analogRead(tmpReg);
  }
  else
  {
    // get the right average analog reg
    if(an == 0)       tmpReg = AVG1R;
    else if(an == 1)  tmpReg = AVG2R;
    // save the last buffer size
    tmpBuffSize = analogBuffSize[an];
    // read the AVGnR register because averaging enabled
    analogBuffSize[an] = (i2cRegs[tmpReg] & AG) + 1;
    // get the right analog input
    if(an == 0)       tmpReg = ANALOG_AN1_ADC;
    else if(an == 1)  tmpReg = ANALOG_AN2_ADC;
    // load the buffer with relevant values when the size is increased
    if(analogBuffSize[an] > tmpBuffSize)
    {
      for(i=0; i<analogBuffSize[an]-tmpBuffSize; i++)
        analogBuff[an][tmpBuffSize+i] = analogRead(tmpReg);
    }
    // smooth the analog value
    analogAcc[an] = analogAcc[an] - analogBuff[an][analogIndex[an]];
    analogBuff[an][analogIndex[an]] = analogRead(tmpReg);
    analogAcc[an] = analogAcc[an] + analogBuff[an][analogIndex[an]];
    analogIndex[an] ++;
    if (analogIndex[an] >= analogBuffSize[an])  analogIndex[an] = 0;
    analogVal[an] = analogAcc[an]/analogBuffSize[an];
  }
  
  // get the right analog data reg
  if(an == 0)       tmpReg = AN1DRL;
  else if(an == 1)  tmpReg = AN2DRL;
  
  // set the analog data reg with the new value
  i2cRegs[tmpReg]   = analogVal[an] & AN_L;
  i2cRegs[tmpReg+1] = (analogVal[an] >> 8) & AN_H;
}

/*
*****************************************************************
*  Procedure:    checkAlarm
*  Description:  This function checks the selected alarm channel
*                by comparing the analog value with the set value
*                (ALnSVH:L) and the clear value (ALnCVH:L). Then
*                it updates the ALnCR.ALO bit.
*  Input:        Alarm channel. Shall be 0 or 1 for AL1 or AL2.
*****************************************************************
*/
void checkAlarm(uint8_t al)
{
  uint8_t  tmpReg;
  uint16_t setValue;
  uint16_t clearValue;
  
  // check if 'al' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(al))  return;
  
  // get the right alarm config reg
  if(al == 0)       tmpReg = AL1CR;
  else if(al == 1)  tmpReg = AL2CR;
  // check if alarm is enabled
  if( !(i2cRegs[tmpReg] & ALEN) )
  {
    alarmOutput[al] = 0;
    goto checkAlarm_updateALO;
  }
  
  // get the right alarm set value reg
  if(al == 0)       tmpReg = AL1SVL;
  else if(al == 1)  tmpReg = AL2SVL;
  setValue = ( (i2cRegs[tmpReg] & ALS_L) | ((i2cRegs[tmpReg+1] & ALS_H) << 8) );
  
  // get the right alarm clear value reg
  if(al == 0)       tmpReg = AL1CVL;
  else if(al == 1)  tmpReg = AL2CVL;
  clearValue = ( (i2cRegs[tmpReg] & ALC_L) | ((i2cRegs[tmpReg+1] & ALC_H) << 8) );
  
  // check the alarm status
  if(setValue > clearValue)
  {
    if(analogVal[al] > setValue)         alarmOutput[al] = 1;
    else if(analogVal[al] < clearValue)  alarmOutput[al] = 0;
  }
  else
  {
    if(analogVal[al] < setValue)         alarmOutput[al] = 1;
    else if(analogVal[al] > clearValue)  alarmOutput[al] = 0;
  }
  
  // get the right alarm config reg
  if(al == 0)       tmpReg = AL1CR;
  else if(al == 1)  tmpReg = AL2CR;
  
  // update ALnCR.ALO bit
  checkAlarm_updateALO:
  if(alarmOutput[al]==0) i2cRegs[tmpReg] &= ~ALO;
  else                   i2cRegs[tmpReg] |= ALO;
}

/*
*****************************************************************
*  Procedure:    updateAlarm
*  Description:  This function updates the digital alarm output
*                according to the selected mode in ALnCR.ALM2:0
*                and the alarm state (ALnCR.ALO).
*  Input:        Alarm channel. Shall be 0 or 1 for AL1 or AL2.
*****************************************************************
*/
void updateAlarm(uint8_t al)
{
  uint8_t tmpReg;
  uint8_t tmpMode;
  uint8_t tmpAlarm;
  // previous alarm state
  static uint8_t outputState[ANALOG_INPUT];
  
  // check if 'al' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(al))  return;
  
  // get the right alarm config reg
  if(al == 0)       tmpReg = AL1CR;
  else if(al == 1)  tmpReg = AL2CR;
  
  // get alarm mode
  tmpMode = (i2cRegs[tmpReg] & ALM) >> 2;
  
  // check if alarm is enabled and mode valid
  if( !(i2cRegs[tmpReg] & ALEN) || (tmpMode==0x0) )
  {
    tmpAlarm = 0;
  }
  // level mode
  else if(tmpMode & ALM_LL)  // check ALM2
  {
    // mask to keep ALM2 and ALM0 only
    tmpMode &= ALM_HL;
    if(tmpMode == ALM_LL)       tmpAlarm = !alarmOutput[al];
    else if(tmpMode == ALM_HL)  tmpAlarm = alarmOutput[al];
    else                        tmpAlarm = 0;
  }
  // pulse mode
  else
  {
    if(outputState[al] != alarmOutput[al])
    {
      // falling edge
      if(outputState[al] && !alarmOutput[al])
      {
        if(tmpMode == ALM_FE || tmpMode == ALM_DE) tmpAlarm = 1;
        else tmpAlarm = 0;
      }
      // raising edge
      else
      {
        if(tmpMode == ALM_RE || tmpMode == ALM_DE)  tmpAlarm = 1;
        else tmpAlarm = 0;
      }
    }
    else tmpAlarm = 0;
  }
  
  // save the current alarm state
  outputState[al] = alarmOutput[al];
  // get the right alarm output pin
  if(al == 0)       tmpReg = ALARM_AL1_PIN;
  else if(al == 1)  tmpReg = ALARM_AL2_PIN;
  // update the output
  digitalWrite(tmpReg, tmpAlarm);
}

/*
*****************************************************************
*  Procedure:    readTemperature
*  Description:  This function reads the internal temperature
*                sensor. The raw value (temperature in kelvin)
*                is smoothed with the TEMPERATURE_BUFF_SIZE
*                previous values. Then TDRH:L is updated with
*                the temperature value formatted according to the
*                selected temperature unit in TCR.TUS1:0 (C, F, K).
*****************************************************************
*/
void readTemperature()
{
  // check if temperature is enabled
  if(!(i2cRegs[TCR] & TEN))  return;
  
  // change the default analog reference for internal temperature
  // this reference is reset to default after an analogRead()
  analogReference(INTERNAL1V1);
  // smooth the Kelvin temperature value
  temperatureAcc -= temperatureBuff[temperatureIndex];
  temperatureBuff[temperatureIndex] = analogRead(A0+15);
  temperatureAcc += temperatureBuff[temperatureIndex];
  temperatureIndex ++;
  if (temperatureIndex >= temperatureBuffSize)  temperatureIndex = 0;
  temperatureVal = (int16_t)(temperatureAcc/temperatureBuffSize);
  // Kelvin unit not selected
  if(!(i2cRegs[TCR] & TUS_K))
  {
    // convert Kelvin to Fahrenheit    //
    // (simplification of calculation) //
    // F = 9/5*K - 459.67              //
    // F = (9*K - 5*459.67)/5          //
    // F = (9*K - 2298.35)/5           //
    // F = (9*K - 2298)/5              //
    if(i2cRegs[TCR] & TUS_F)
    {
      temperatureVal *= 9;
      temperatureVal -= 2298;
      temperatureVal /= 5;
    }
    // convert Kelvin to Celsius       //
    // (simplification of calculation) //
    // C = K - 273.15                  //
    // C = K - 273                     //
    else
    {
      temperatureVal -= 273;
    }
  }
  // update the temperature data reg with the new value
  i2cRegs[TDRL] = temperatureVal & TP_L;
  i2cRegs[TDRH] = (temperatureVal >> 8) & TP_H;
}

/*
*****************************************************************
*  Procedure:    setup
*  Description:  This function is executed once for every
*                startup initialization of the main function.
*****************************************************************
*/
void setup()
{
  uint8_t i;
  
  updateRegs();
  initTemperature();
  for(i=0; i<ANALOG_INPUT; i++)
  {
    initSensor(i);
    initAlarm(i);
  }
  // initialize the I2C bus
  TinyWireS.begin(i2cRegs[ADDR]);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(sendEvent);
}

/*
*****************************************************************
*  Procedure:    loop
*  Description:  This function is the main function executed
*                as an infinite loop.
*****************************************************************
*/
void loop()
{
  static uint8_t i;
  
  updateRegs();
  readTemperature();
  for(i=0; i<ANALOG_INPUT; i++)
  {
    readSensor(i);
    checkAlarm(i);
    updateAlarm(i);
  }
  // handle any I2C interruption
  TinyWireS_stop_check();
}

