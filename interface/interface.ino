#include "digitalWriteFast.h"
#include "profiler.h"
#include "SPI.h"

#define pinA 2    //The pins that the rotary encoder's A and B terminals are connected to.
#define pinB 3


//The previous state of the AB pins
volatile byte  previousReading = 0;

//Keeps track of how much the encoder has been moved
volatile int rotPosition = 0;
volatile int lastRotPosition = -1;


volatile byte  aState = 0;
volatile byte  aLastState = 0;

uint32_t startTime;


typedef unsigned char u8;
typedef unsigned short u16;
typedef char s8;
typedef short s16;

#define SLAVESELECT 7

// Exchange one byte over SPI (with wait for completion)


// u8 transaction(u8 s) {
// 	SPDR = s;
// 	while(!(SPSR & 0x80));
// 	return SPDR;
// }


//#define digitalReadFast digitalRead
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

  SPISettings settings(500000, LSBFIRST, SPI_MODE3);
  SPI.beginTransaction(settings);

  // interrupt disabled,spi enabled,lsb 1st,master,clk high when idle,
  // sample on leading edge of clk,system clock/8 rate (500Kbit/S)  
  //SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << CPOL) | (1 << CPHA) | (1 << DORD);
  //SPSR = (1 << SPI2X);

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

void loop() 
{

	//asm("sei");

  beginDebugEvent(DEBUG_INTERRUPT_TIME);
  // alive blink
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

  delay(8); // Don't poll the gamepad too often...
  PS_Data[0] = 0;
  PS_Data[1] = 0;
  
  Poll_PSX(0,0);

  if((PS_Data[PS_DIGI_BUTTONS_0] & 0x10)) 
  {
    Serial.println("Up");
    //joybits |= 0x20;	// up
  }
  if((PS_Data[PS_DIGI_BUTTONS_0] & 0x40)) 
  {
    Serial.println("Down");
    //joybits |= 0x10;	// down
  }
  if((PS_Data[PS_DIGI_BUTTONS_0] & 0x80)) 
  {
    Serial.println("Left");
     //joybits |= 0x08;	// left
  }
  if((PS_Data[PS_DIGI_BUTTONS_0] & 0x20)) 
  {
    Serial.println("Right");
    //joybits |= 0x04;	// right
  }

  // if(!(data4 & 0x01)) joybits |= 0x01;	// select -> fire
  // if(!(data4 & 0x08)) joybits |= 0x01;	// start -> fire

  // if(!(data5 & 0x10)) joybits |= 0x10;	// triangle -> down


  endDebugEvent(DEBUG_INTERRUPT_TIME);

}
