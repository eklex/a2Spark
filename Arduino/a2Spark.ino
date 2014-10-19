/*
*  Name:      a2Spark
*  Author:    Alexandre Boni
*  Created:   2013/10/26
*  Modified:  2014/06/05
*  Version:   1.0
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
*    1.0
*          Set the default configuration for 
*          the RaspLapse project.
*          
*    0.9
*          Freeze current alarm output status
*          when alarm is disbaled, and reset
*          alarm output when alarm mode is disabled
*          Correct alarm mode mask
*    0.8
*          Adding temperature feature
*          Adding Low-Pass filter
*          Removing weighted average
*          Adding reverse mode in alarm mode
*    0.7
*          Adding comments and layout
*    0.2 to 0.6
*          First design of a2Spark
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
#define FIRMWARE_VERSION          (0x10)
// I2C Address
// this address should change to use more than one a2Spark device
// address = [0x03 : 0x77]
#define I2C_ADDRESS_DEVICE        (0x42)
// Internal Temperature sensor
// this temperature offset sensor is used to tune the Kelvin value
// 0 <= TEMPERATURE_OFFSET <= 15
#define TEMPERATURE_OFFSET        (4)
// this bit sets the sign of TEMPERATURE_OFFSET
// positive sign is 0, and negative sign is 1
#define TEMPERATURE_OFFSET_SIGN   (0)

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
#define LPEN                  (0x01<<1)
// Average Analog Register
#define LPF1R                 (0x05)
#define LPF2R                 (0x15)
#define LP                    (0x0F<<0)
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
#define ALME                  (0x01<<4)
#define ALM                   (0x07<<5)
#define ALM_INV               (0x01<<7)
#define ALM_FE                (0x01<<5)
#define ALM_RE                (0x01<<6)
#define ALM_DE                (ALM_RE | ALM_FE)
#define ALM_L                 (0x03<<5)
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
#define TSNG_SHIFT            (3)
#define TSNG                  (0x01<<TSNG_SHIFT)
#define TOS_SHIFT             (4)
#define TOS                   (0x0F<<TOS_SHIFT)
#define TDRL                  (0x11)
#define TDRH                  (0x12)
#define TP_L                  (0xFF<<0)
#define TP_H                  (0xFF<<0)

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
#define AN1CR_DEF             (ANEN | LPEN)
#define LPF1R_DEF             (0x0F)
#define AN1DRL_DEF            (0x00)
#define AN1DRH_DEF            (0x00)
#define AL1CR_DEF             (ALEN | ALME | ALM_FE | ALM_INV)
#define AL1SVL_DEF            (0xBC)
#define AL1SVH_DEF            (0x02)
#define AL1CVL_DEF            (0x20)
#define AL1CVH_DEF            (0x03)
// Temperature Registers
#define TOS_DEF               (TEMPERATURE_OFFSET<<TOS_SHIFT)
#define TSGN_DEF              (TEMPERATURE_OFFSET_SIGN<<TSNG_SHIFT)
#define TCR_DEF               (TEN | TUS_C | TSGN_DEF | TOS_DEF)
#define TDRL_DEF              (0x00)
#define TDRH_DEF              (0x00)
// Analog1 & Alarm2 Registers
#define AN2CR_DEF             (ANEN | LPEN)
#define LPF2R_DEF             (0x08)
#define AN2DRL_DEF            (0x00)
#define AN2DRH_DEF            (0x00)
#define AL2CR_DEF             (0x00)
#define AL2SVL_DEF            (0x00)
#define AL2SVH_DEF            (0x00)
#define AL2CVL_DEF            (0x00)
#define AL2CVH_DEF            (0x00)

/*
*****************************************************************
***                Buffer Size & Pin Defines                  ***
*****************************************************************
*/
// Number of Analog Input
// = number of alarm output
#define ANALOG_INPUT          (2)
// Analog1 Input Pin (AN0 = PB5)
#define ANALOG_AN1_ADC        (0)
// Analog2 Input Pin (AN2 = PB4)
#define ANALOG_AN2_ADC        (2)
// Alarm1 Output Pin (PB1)
#define ALARM_AL1_PIN         (1)
// Alarm2 Output Pin (PB3)
#define ALARM_AL2_PIN         (3)
// Low-Pass Temperature Filter
// Parameter k
#define TEMPERATURE_FILTER_K  (15)

/*
*****************************************************************
***                          Macro                            ***
*****************************************************************
*/
// check if __n__ is between 0 and ANALOG_INPUT-1
// return a boolean
#define SELECT_CHECK(__n__) \
        ((__n__ >= 0) && (__n__ < ANALOG_INPUT))

// initialize the low-pass filter with the current
// analog value
// return an uint32_t
#define LOWPASS_FILTER_INIT(__k__, __in__) \
        (__in__ << __k__)
        

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
// Low-Pass Filter
uint32_t lpFilter[ANALOG_INPUT];

/*
*****************************************************************
***                 Temperature Variables                     ***
*****************************************************************
*/
// Temperature Value in TDRH:L
// this value is signed
int16_t temperatureVal;
// Low-Pass Filter for Temperature
uint32_t temperatureFilter;

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
  LPF1R_DEF,     //0x05
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
  LPF2R_DEF,     //0x15
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
  i2cRegs[AN1CR]  &= (ANEN | LPEN);
  i2cRegs[AN2CR]  &= (ANEN | LPEN);
  i2cRegs[LPF1R]  &= LP;
  i2cRegs[LPF2R]  &= LP;
  i2cRegs[AN1DRL] &= AN_L;
  i2cRegs[AN2DRL] &= AN_L;
  i2cRegs[AN1DRH] &= AN_H;
  i2cRegs[AN2DRH] &= AN_H;
  i2cRegs[AL1CR]  &= (ALEN | ALO | ALM | ALME);
  i2cRegs[AL2CR]  &= (ALEN | ALO | ALM | ALME);
  i2cRegs[AL1SVL] &= ALS_L;
  i2cRegs[AL2SVL] &= ALS_L;
  i2cRegs[AL1SVH] &= ALS_H;
  i2cRegs[AL2SVH] &= ALS_H;
  i2cRegs[AL1CVL] &= ALC_L;
  i2cRegs[AL2CVL] &= ALC_L;
  i2cRegs[AL1CVH] &= ALC_H;
  i2cRegs[AL2CVH] &= ALC_H;
  i2cRegs[TCR]    &= (TEN | TUS | TSNG | TOS);
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
  uint8_t tmpRegCR, tmpRegLP, tmpRegAN;

  // check if 'an' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(an))  return;
  
  // get the right analog config reg, and average analog reg
  if(an == 0)
  {
    tmpRegCR = AN1CR;
    tmpRegLP = LPF1R;
    tmpRegAN = ANALOG_AN1_ADC;
  }
  else if(an == 1)
  {
    tmpRegCR = AN2CR;
    tmpRegLP = LPF2R;
    tmpRegAN = ANALOG_AN2_ADC;
  }
  
  // init analog variables
  analogVal[an]      = 0;
  lpFilter[an]       = 0;
  
  // init the LP filter if feature is enabled
  if(i2cRegs[tmpRegCR] & LPEN)
  {
    lpFilter[an] = analogRead(tmpRegAN);
    lpFilter[an] = LOWPASS_FILTER_INIT(
                      (i2cRegs[tmpRegLP] & LP) + 1,
                      lpFilter[an]
                      );
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
  uint8_t tmpRegAL;
  
  // check if 'al' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(al))  return;
  
  alarmOutput[al] = 0;
  checkAlarm(al);
  updateAlarm(al);
  
  // get the right alarm output pin
  if(al == 0)       tmpRegAL = ALARM_AL1_PIN;
  else if(al == 1)  tmpRegAL = ALARM_AL2_PIN;
  // set the alarm pin in output status
  pinMode(tmpRegAL, OUTPUT);
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
  // init temperature variable
  temperatureVal      = 0;
  temperatureFilter   = 0;
  
  // check if the temperature sensor is enabled
  if(i2cRegs[TCR] & TEN)
  {
    // change the default analog reference for internal temperature
    analogReference(INTERNAL1V1);
    // capture a temperature sample
    temperatureFilter = analogRead(A0+15);
    // restore the default analog reference to VCC as reference
    analogReference(DEFAULT);

    // init the low-pass temperature filter
    temperatureFilter  = LOWPASS_FILTER_INIT(
                              TEMPERATURE_FILTER_K,
                              temperatureFilter
                              );
  } 
}

/*
*****************************************************************
***                 Sensor & Alarm Functions                  ***
*****************************************************************
*/

/*
*****************************************************************
*  Procedure:    lowPassFilter
*  Description:  This function filters as a low-pass
*                the input_value.
*  Input:        Pointer on the filter buffer
*                Parameter k (shift value)
*                Analog input value
*****************************************************************
*/
uint16_t unsignedLowPassFilter(
  uint32_t  *filter,
  uint8_t   k_shift,
  uint16_t  input_value
  )
{
  // update filter with current sample
  *filter +=  input_value - (*filter >> k_shift);
  // scale output for unity gain. 
  return (*filter >> k_shift);
}

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
  static uint8_t currentShift[ANALOG_INPUT]  = {0};
  static uint8_t previousShift[ANALOG_INPUT] = {0};
  uint8_t i;
  uint8_t tmpRegCR, tmpRegLP, tmpRegAN, tmpRegAD;
  
  // check if 'an' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(an))  return;
  
  // get the right analog config reg, average analog reg,
  // analog input, and analog data reg
  if(an == 0)
  {
    tmpRegCR = AN1CR;
    tmpRegLP = LPF1R;
    tmpRegAN = ANALOG_AN1_ADC;
    tmpRegAD = AN1DRL;
  }
  else if(an == 1)
  {
    tmpRegCR = AN2CR;
    tmpRegLP = LPF2R;
    tmpRegAN = ANALOG_AN2_ADC;
    tmpRegAD = AN2DRL;
  }
  
  // check if sampling is enabled
  if( !(i2cRegs[tmpRegCR] & ANEN) )  return;
  
  // check if filter is enabled
  if( !(i2cRegs[tmpRegCR] & LPEN) )
  {
    // filter disabled then just capture one sample
    analogVal[an] = analogRead(tmpRegAN);
  }
  else
  {    
    // save the last k parameter (shift value)
    previousShift[an] = currentShift[an];
    // read the LPFnR register because LP filter enabled
    currentShift[an] = (i2cRegs[tmpRegLP] & LP) + 1;
    // re-initialize the LP filter if the k parameter change
    if(currentShift[an] != previousShift[an])
    {
      lpFilter[an] = analogRead(tmpRegAN);
      lpFilter[an] = LOWPASS_FILTER_INIT(
                        currentShift[an],
                        lpFilter[an]
                        );
    }
    // filter the current analog sample
    analogVal[an] = unsignedLowPassFilter(
                      &(lpFilter[an]),
                      currentShift[an],
                      analogRead(tmpRegAN)
                      );
  }
  
  // set the analog data reg with the new value
  i2cRegs[tmpRegAD]   = analogVal[an] & AN_L;
  i2cRegs[tmpRegAD+1] = (analogVal[an] >> 8) & AN_H;
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
  uint16_t setValue;
  uint16_t clearValue;
  uint8_t  tmpRegCR, tmpRegSV, tmpRegCV;
  
  // check if 'al' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(al))  return;
  
  // get the right alarm config reg, alarm set value reg,
  // and alarm clear value reg
  if(al == 0)
  {
    tmpRegCR = AL1CR;
    tmpRegSV = AL1SVL;
    tmpRegCV = AL1CVL;
  }
  else if(al == 1)
  {
    tmpRegCR = AL2CR;
    tmpRegSV = AL2SVL;
    tmpRegCV = AL2CVL;
  }
  // check if alarm is enabled
  if( !(i2cRegs[tmpRegCR] & ALEN) )
  {
    goto checkAlarm_updateALO;
  }
  
  // read the set value 
  setValue = ( (i2cRegs[tmpRegSV] & ALS_L) | ((i2cRegs[tmpRegSV+1] & ALS_H) << 8) );
  
  // read the clear value 
  clearValue = ( (i2cRegs[tmpRegCV] & ALC_L) | ((i2cRegs[tmpRegCV+1] & ALC_H) << 8) );
  
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
  
  // update ALnCR.ALO bit
  checkAlarm_updateALO:
  if(alarmOutput[al]==0) i2cRegs[tmpRegCR] &= ~ALO;
  else                   i2cRegs[tmpRegCR] |= ALO;
}

/*
*****************************************************************
*  Procedure:    updateAlarm
*  Description:  This function updates the digital alarm output
*                according to the selected mode in ALnCR.ALM2:0,
*                and the alarm state (ALnCR.ALO).
*  Input:        Alarm channel. Shall be 0 or 1 for AL1 or AL2.
*****************************************************************
*/
void updateAlarm(uint8_t al)
{
  uint8_t tmpRegCR, tmpRegAL;
  uint8_t tmpMode;
  uint8_t tmpAlarm;
  // previous alarm state
  static uint8_t outputState[ANALOG_INPUT];
  
  // check if 'al' is between 0 and ANALOG_INPUT-1
  if(!SELECT_CHECK(al))  return;
  
  // get the right alarm config reg, and alarm output pin
  if(al == 0)
  {
    tmpRegCR = AL1CR;
    tmpRegAL = ALARM_AL1_PIN;
  }
  else if(al == 1)
  {
    tmpRegCR = AL2CR;
    tmpRegAL = ALARM_AL2_PIN;
  }
  
  // get alarm mode config
  tmpMode = (i2cRegs[tmpRegCR] & ALM);
  
  // check if alarm is enabled
  if( !(i2cRegs[tmpRegCR] & ALEN) )
  {
    // freeze current alarm output
    return;
  }
  // check if alarm mode is enabled
  else if( !(i2cRegs[tmpRegCR] & ALME) )
  {
    // disable output
    tmpAlarm = 0;
  }
  // alarm mode enable
  else
  {
    // level mode
    if(!(tmpMode & ALM_L)) // ALM6:5={0,0}
    {
      tmpAlarm = alarmOutput[al];
    }
    // pulse mode
    else
    {
      if(outputState[al] != alarmOutput[al])
      {
        // falling edge
        if(outputState[al] && !alarmOutput[al])
        {
          if(tmpMode & ALM_FE) tmpAlarm = 1;
          else tmpAlarm = 0;
        }
        // raising edge
        else
        {
          if(tmpMode & ALM_RE)  tmpAlarm = 1;
          else tmpAlarm = 0;
        }
      }
      else tmpAlarm = 0;
    }
    // reverse bit
    if(tmpMode & ALM_INV)  tmpAlarm = !tmpAlarm;
  }
  
  // save the current alarm state
  outputState[al] = alarmOutput[al];

  // update the output
  digitalWrite(tmpRegAL, tmpAlarm);
}

/*
*****************************************************************
*  Procedure:    readTemperature
*  Description:  This function reads the internal temperature
*                sensor. The raw value (temperature in kelvin)
*                is smoothed with the low-pass filter configured
*                with a parameter k equal to TEMPERATURE_FILTER_K.
*                Then TDRH:L is updated with the temperature value
*                formatted according to the selected temperature
*                unit in TCR.TUS1:0 (C, F, K).
*****************************************************************
*/
void readTemperature()
{
  uint16_t  tmpTemp;
  
  // check if temperature is enabled
  if(!(i2cRegs[TCR] & TEN))  return;
  
  // change the default analog reference for internal temperature
  // this reference should be reset to default before exiting the function
  analogReference(INTERNAL1V1);
  // capture a temperature sample
  tmpTemp = analogRead(A0+15);
  // restore the default analog reference to VCC as reference
  analogReference(DEFAULT);
  
  // filter with a low-pass the Kelvin temperature value
  // then cast to signed integer
  temperatureVal = unsignedLowPassFilter(
                        (&temperatureFilter),
                        TEMPERATURE_FILTER_K,
                        tmpTemp
                        );
                        
   if(i2cRegs[TCR] & TSNG)
     temperatureVal -= ((i2cRegs[TCR] & TOS) >> TOS_SHIFT);
   else
     temperatureVal += ((i2cRegs[TCR] & TOS) >> TOS_SHIFT);   
                                
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

