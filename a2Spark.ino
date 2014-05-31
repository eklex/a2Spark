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

#include <TinyWireS.h>

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

#define ADDR        (0x00)
#define ANCR        (0x01)
#define ANCR_ANEN   (0x01<<0)
#define ANCR_AVEN   (0x01<<1)
#define AVGR        (0x02)
#define AVGR_AG     (0x0F<<0)
#define ANDRL       (0x03)
#define ANDRL_AN    (0xFF<<0)
#define ANDRH       (0x04)
#define ANDRH_AN    (0x03<<0)

#define ADDR_DEF    (0x42)
#define ANCR_DEF    (ANCR_ANEN | ANCR_AVEN)
#define AVGR_DEF    (0x00)
#define ANDRL_DEF   (0x00)
#define ANDRH_DEF   (0x00)

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
    ADDR_DEF,  //0
    ANCR_DEF,  //1
    AVGR_DEF,  //2
    ANDRL_DEF, //3
    ANDRH_DEF, //4
    0x00
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
  analogBuffSize = (i2cRegs[AVGR] & AVGR_AG) + 1;
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
  if( !(i2cRegs[ANCR] & ANCR_ANEN) )  return;
  // check if averaging is enabled
  if( !(i2cRegs[ANCR] & ANCR_AVEN) )
  {
    // averging disabled then just capture one sample
    analogVal = analogRead(0);
  }
  else
  {
    // read the AVGR register because averaging enabled
    analogBuffSize = (i2cRegs[AVGR] & AVGR_AG) + 1;
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
  
  i2cRegs[ANDRL] = analogVal & ANDRL_AN;
  i2cRegs[ANDRH] = (analogVal >> 8) & ANDRH_AN;
}

void updateRegs()
{
  i2cRegs[ANCR]  &= (ANCR_ANEN | ANCR_AVEN);
  i2cRegs[AVGR]  &= AVGR_AG;
  i2cRegs[ANDRL] &= ANDRL_AN;
  i2cRegs[ANDRH] &= ANDRH_AN;
}

int checkAlarm(int value, int alarm, int setValue, int clearValue)
{
  if(setValue > clearValue)
  {
    if(!alarm)	alarm = (value > setValue) ? 1:0;
	else		alarm = (value < clearValue) ? 0:1;
  }
  else
  {
    if(!alarm)	alarm = (value < setValue) ? 1:0;
	else		alarm = (value > clearValue) ? 0:1;
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
