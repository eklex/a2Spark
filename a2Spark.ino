/**
 * Pin notes by Suovula, see also http://hlt.media.mit.edu/?p=1229
 *
 * DIP and SOIC have same pinout, however the SOIC chips are much cheaper, especially if you buy more than 5 at a time
 * For nice breakout boards see https://github.com/rambo/attiny_boards
 *
 * Basically the arduino pin numbers map directly to the PORTB bit numbers.
 *
// I2C
arduino pin 0 = not(OC1A) = PORTB <- _BV(0) = SOIC pin 5 (I2C SDA, PWM)
arduino pin 2 =           = PORTB <- _BV(2) = SOIC pin 7 (I2C SCL, Analog 1)
 */
#define FIRMWARE_VERSION  0x01

#include <TinyWireS.h>

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

//I2C Address Reg
#define ADDR        (0x00)
//Firmware Version Reg
#define FWVR        (0x01)
//Analog Configuration Reg
#define AN1CR       (0x04)
#define AN2CR       (0x14)
#define ANEN        (0x01<<0)
#define AVEN        (0x01<<1)
//Average Analog Reg
#define AVG1R       (0x05)
#define AVG2R       (0x15)
#define AG          (0x0F<<0)
//Analog Data Reg
#define AN1DRL      (0x06)
#define AN2DRL      (0x16)
#define AN1DRH      (0x07)
#define AN2DRH      (0x17)
#define AN_L        (0xFF<<0)
#define AN_H        (0x03<<0)
//Alarm Configuration Reg
#define AL1CR       (0x08)
#define AL2CR       (0x18)
#define ALEN        (0x01<<0)
#define ALO         (0x01<<1)
#define ALM         (0x07<<2)
#define ALM_FE      (0x01<<0)
#define ALM_RE      (0x01<<1)
#define ALM_DE      (ALM_RE | ALM_FE)
#define ALM_LL      (0x01<<2)
#define ALM_HL      (0x05<<0)
//Alarm Set Value Reg
#define AL1SRL      (0x09)
#define AL2SRL      (0x19)
#define AL1SRH      (0x0A)
#define AL2SRH      (0x1A)
#define ALS_L       (0xFF<<0)
#define ALS_H       (0x03<<0)
//Alarm Clear Value Reg
#define AL1CRL      (0x0B)
#define AL2CRL      (0x1B)
#define AL1CRH      (0x0C)
#define AL2CRH      (0x1C)
#define ALC_L       (0xFF<<0)
#define ALC_H       (0x03<<0)

//Default Values
#define RESERVED    (0x00)
#define ADDR_DEF    (0x42)
#define FWVR_DEF    (FIRMWARE_VERSION)

#define AN1CR_DEF   (ANEN | AVEN)
#define AVG1R_DEF   (0x00)
#define AN1DRL_DEF  (0x00)
#define AN1DRH_DEF  (0x00)
#define AL1CR_DEF   (0x00)
#define AL1SRL_DEF  (0x00)
#define AL1SRH_DEF  (0x00)
#define AL1CRL_DEF  (0x00)
#define AL1CRH_DEF  (0x00)

#define AN2CR_DEF   (0x00)
#define AVG2R_DEF   (AVG1R_DEF)
#define AN2DRL_DEF  (AN1DRL_DEF)
#define AN2DRH_DEF  (AN1DRH_DEF)
#define AL2CR_DEF   (AL1CR_DEF)
#define AL2SRL_DEF  (AL1SRL_DEF)
#define AL2SRH_DEF  (AL1SRH_DEF)
#define AL2CRL_DEF  (AL1CRL_DEF)
#define AL2CRH_DEF  (AL1CRH_DEF)



#define ANALOG_BUFF_SIZE  16
uint8_t  analogPin;
uint16_t analogVal;
uint16_t analogAcc;
uint8_t  analogIndex;
uint8_t  analogBuffSize;
uint16_t analogBuff[ANALOG_BUFF_SIZE];

// Tracks the current register pointer position
volatile byte reg_position;
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
  AL1SRL_DEF,    //0x09
  AL1SRH_DEF,    //0x0A
  AL1CRL_DEF,    //0x0B
  AL1CRH_DEF,    //0x0C
  RESERVED,      //0x0D
  RESERVED,      //0x0E
  RESERVED,      //0x0F
  RESERVED,      //0x10
  RESERVED,      //0x11
  RESERVED,      //0x12
  RESERVED,      //0x13
  AN2CR_DEF,     //0x14
  AVG2R_DEF,     //0x15
  AN2DRL_DEF,    //0x16
  AN2DRH_DEF,    //0x17
  AL2CR_DEF,     //0x18
  AL2SRL_DEF,    //0x19
  AL2SRH_DEF,    //0x1A
  AL2CRL_DEF,    //0x1B
  AL2CRH_DEF,    //0x1C
  RESERVED,      //0x1D
  RESERVED,      //0x1E
  RESERVED       //0x1F
};


/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
void requestEvent()
{  
    TinyWireS.send(i2cRegs[reg_position]);
    // Increment the reg position on each read, and loop back to zero
    reg_position = (reg_position+1) % sizeof(i2cRegs);
}

/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
 */
void receiveEvent(uint8_t howMany)
{
    if (howMany < 1)
    {
        // Sanity-check
        return;
    }
    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }

    reg_position = TinyWireS.receive();
    howMany--;
    if (!howMany)
    {
        // This write was only to set the buffer for next read
        return;
    }
    while(howMany--)
    {
        i2cRegs[reg_position%sizeof(i2cRegs)] = TinyWireS.receive();
        reg_position++;
    }
}

void initSensor()
{
  int i;
  analogPin      = 0;
  analogVal      = 0;
  analogAcc      = 0;
  analogIndex    = 0;
  analogBuffSize = (i2cRegs[AVG1R] & AG) + 1;
  for(i=0; i<ANALOG_BUFF_SIZE; i++)
  {
    analogBuff[i] = 0;
  }
  for(i=0; i<analogBuffSize; i++)
  {
    readSensor();
  }
}

void readSensor()
{
  uint8_t i;
  uint8_t tmpBuffSize = analogBuffSize;
  // check if sampling is enabled
  if( !(i2cRegs[AN1CR] & ANEN) )  return;
  // check if averaging is enabled
  if( !(i2cRegs[AN1CR] & AVEN) )
  {
    // averging disabled then just capture one sample
    analogVal = analogRead(0);
  }
  else
  {
    // read the AVGR register because averaging enabled
    analogBuffSize = (i2cRegs[AVG1R] & AG) + 1;
    // load the buffer with relevant values when the size is increased
    if(analogBuffSize > tmpBuffSize)
    {
      for(i=0; i<analogBuffSize-tmpBuffSize; i++)
        analogBuff[tmpBuffSize+i] = analogRead(analogPin);
    }
    // smooth the analog value
    analogAcc = analogAcc - analogBuff[analogIndex];
    analogBuff[analogIndex] = analogRead(analogPin);
    analogAcc = analogAcc + analogBuff[analogIndex];
    analogIndex ++;
    if (analogIndex >= analogBuffSize)  analogIndex = 0;
    analogVal = analogAcc/analogBuffSize;
  }
  
  i2cRegs[AN1DRL] = analogVal & AN_L;
  i2cRegs[AN1DRH] = (analogVal >> 8) & AN_H;
}

void updateRegs()
{
  i2cRegs[AN1CR]  &= (ANEN | AVEN);
  i2cRegs[AVG1R]  &= AG;
  i2cRegs[AN1DRL] &= AN_L;
  i2cRegs[AN1DRH] &= AN_H;
}

int checkAlarm(int value, int alarm, int setValue, int clearValue)
{
  if(setValue > clearValue)
  {
    if(!alarm)  alarm = (value > setValue) ? 1:0;
    else        alarm = (value < clearValue) ? 0:1;
  }
  else
  {
    if(!alarm)  alarm = (value < setValue) ? 1:0;
    else        alarm = (value > clearValue) ? 0:1;
  }
  return alarm;
}

void setup()
{
  updateRegs();
  initSensor();
  TinyWireS.begin(i2cRegs[ADDR]);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
}

void loop()
{
  updateRegs();
  readSensor();
  TinyWireS_stop_check();
}
