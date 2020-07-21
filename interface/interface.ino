#include "digitalWriteFast.h"
#include "profiler.h"
#include "SPI.h"

uint32_t startTime;

#define SLAVESELECT 8

int RXLED = 17;

void setup() {

  pinMode(SLAVESELECT, OUTPUT);
  digitalWrite(SLAVESELECT, HIGH);

  SPI.begin();

  // SPI mode	
  // Clock polarity (CPOL/CKP)
  // Clock phase (CPHA)
  // Clock edge (CKE/NCPHA)
  // Mode CPOL CPHA CKE
  //  0	   0	  0	   1
  //  1	   0	  1	   0
  //  2	   1	  0	   1
  //  3	   1	  1	   0

  // interrupt disabled,spi enabled,lsb 1st,master,clk high when idle,
  // sample on leading edge of clk,system clock/8 rate (500Kbit/S)  

  SPISettings settings(500000, LSBFIRST, SPI_MODE3);
  SPI.beginTransaction(settings);

  // Debug pins and alive blink
  initDebugPin(DEBUG_INTERRUPT_TIME);
  pinMode(5, OUTPUT);
  pinMode(RXLED, OUTPUT);  // Set RX LED as an output
  startTime = millis();

  // initialize the serial communication:
  Serial.begin(9600);
  Serial.println("Start");
}

byte PS_Data[2];
#define PS_DIGI_BUTTONS_0 0
#define PS_DIGI_BUTTONS_1 1

// Get one data packet via SPI
void Poll_PSX(byte Big_Motor_Value, boolean Small_Motor_Value)
{
    byte Loop_Cnt;
    byte clr;
    // Set the gamepad into pressure mode. Note that from this point on, we
    // must sample the pad at least once every two seconds or it will leave
    // pressure mode.
    digitalWrite(SLAVESELECT, LOW); // Select the pad.

    delayMicroseconds(10);
    clr = SPI.transfer(0x01);

    delayMicroseconds(20);
    clr = SPI.transfer(0x42);

    delayMicroseconds(20);
    clr = SPI.transfer(0x00);

    for (Loop_Cnt = 0; Loop_Cnt < 2; Loop_Cnt++)
    {
        delayMicroseconds(20);
        PS_Data[Loop_Cnt] = SPI.transfer(0x00);
    }
    digitalWrite(SLAVESELECT, HIGH); // de-select the pad.
    // Invert the digital buttons (they are active low in the PSX!?!)
    PS_Data[PS_DIGI_BUTTONS_0] ^= 255;
    PS_Data[PS_DIGI_BUTTONS_1] ^= 255;
}


void PrintJoystick()
{
  uint8_t ps0 = PS_Data[0];
  uint8_t ps1 = PS_Data[1];  

  if((ps0 & 0x10)) 
  {
    Serial.println("Up");
  }
  if((ps0 & 0x40)) 
  {
    Serial.println("Down");
  }
  if((ps0 & 0x80)) 
  {
    Serial.println("Left");
  }
  if((ps0 & 0x20)) 
  {
    Serial.println("Right");
  }  
  if((ps0 & 0x01)) 
  {
    Serial.println("Right");
  }  
  if((ps0 & 0x08)) 
  {
    Serial.println("Right");
  }  
  if((ps1 & 0x10)) 
  {
    Serial.println("Right");
  }  
}

void aliveBlink()
{
  static int ledState = 1;
  uint32_t curTime = millis();
  if ((curTime - startTime ) > 500)
  {
    ledState ^= 1;
    if (ledState)
    { digitalWriteFast(RXLED, HIGH); }
    else 
    { digitalWriteFast(RXLED, LOW);}
    startTime = curTime;
  }
}

void loop() 
{
  beginDebugEvent(DEBUG_INTERRUPT_TIME);

  aliveBlink();

  PS_Data[0] = 0;
  PS_Data[1] = 0;
  
  Poll_PSX(0,0);

  uint8_t ps0 = PS_Data[0];
  uint8_t ps1 = PS_Data[1];

  if((ps0 & 0x10)) { digitalWriteFast(7,HIGH); } else { digitalWriteFast(7,LOW); } //up
  if((ps0 & 0x40)) { digitalWriteFast(6,HIGH); } else { digitalWriteFast(6,LOW); } //down
  if((ps0 & 0x80)) { digitalWriteFast(5,HIGH); } else { digitalWriteFast(5,LOW); } //left
  if((ps0 & 0x20)) { digitalWriteFast(4,HIGH); } else { digitalWriteFast(4,LOW); } //right
  if((ps0 & 0x01)) { digitalWriteFast(3,HIGH); } else { digitalWriteFast(3,LOW); } //select
  if((ps0 & 0x08)) { digitalWriteFast(3,HIGH); } else { digitalWriteFast(3,LOW); } //start
  if((ps1 & 0x10)) { digitalWriteFast(7,HIGH); } else { digitalWriteFast(7,LOW); } //triangle 
  
  endDebugEvent(DEBUG_INTERRUPT_TIME);

  delay(8); // Don't poll the gamepad too often...

}
