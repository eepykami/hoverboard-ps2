#include <PsxControllerBitBang.h>
#include <DigitalIO.h>
#include <SoftwareSerial.h>

const byte oRx = 8; // rx and tx pins for hoverboard controller
const byte oTx = 9;
                     
const byte ATT = 10; // pin 6   // pins for ps2 controller 
const byte CMD = 11; // pin 2
const byte DAT = 12; // pin 1
const byte CLK = 13; // pin 7

PsxControllerBitBang<ATT, CMD, DAT, CLK> psx;
SoftwareSerial oSerial(oRx, oTx);

boolean haveController = false;

typedef struct {
   int16_t steer;
   int16_t speed;
   uint32_t crc; } Serialcommand;
Serialcommand oCmd;

typedef struct {
   int16_t iSpeedL; // 100* km/h
   int16_t iSpeedR; // 100* km/h
   uint16_t iHallSkippedL;
   uint16_t iHallSkippedR;
   uint16_t iTemp;  // °C
   uint16_t iVolt;  // 100* V
   int16_t iAmpL;  // 100* A
   int16_t iAmpR;  // 100* A
   uint32_t crc;
} SerialFeedback;
SerialFeedback oFeedback;

byte lx, ly, rx, ry, slx, sly, srx, sry;

uint32_t crc32_for_byte(uint32_t r) 
{
  for(int j = 0; j < 8; ++j)
    r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
  return r ^ (uint32_t)0xFF000000L;
}

void crc32(const void *data, size_t n_bytes, uint32_t* crc) {
  static uint32_t table[0x100];
  if(!*table)
    for(size_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}

void Send(int16_t iSpeed,int16_t iSteer)
{
  oCmd.steer = iSteer;
  oCmd.speed = iSpeed;

  uint32_t crc = 0;
  crc32((const void *)&oCmd, sizeof(Serialcommand)-4,   &crc);
  oCmd.crc = crc;
  
  oSerial.write((uint8_t *) &oCmd, sizeof(oCmd)); 
}

int iFailedRec = 0;

boolean Receive()
{
  while (oSerial.available()) {Serial.print(" ");Serial.print(oSerial.read(),HEX);}return false;

  if (oSerial.available()<  sizeof(SerialFeedback))
    return false;

  SerialFeedback oNew;
  byte* p = (byte*)&oNew;
  for (unsigned int i=0; i < sizeof(SerialFeedback); i++)
    *p++ = oSerial.read();;

  uint32_t crc = 0;
  crc32((const void *)&oNew, sizeof(SerialFeedback)-4,   &crc);

    if (oNew.crc == crc)
  {
    memcpy(&oFeedback,&oNew,sizeof(SerialFeedback));
    return true;    
  }
return false;
}

void readAnalogSticks() { // left analogue = forward+backward, right analogue = turn left/right
  psx.getLeftAnalog(lx, ly); // left analogue stick
  if(ly != sly) { // only update if value has changed
    sly = ly;
  }

  psx.getRightAnalog(rx, ry); // right analogue stick
  if(rx != srx) { // only update if value has changed
    srx = rx;
  }
}

void setup() {
  
  Serial.begin(9600); // serial init
  Serial.println("Hello!");
  
  oSerial.begin(9600); // hoverboard serial init

}

void loop() {

  if(!haveController) { // if controller not detected, begin connecting to controller
    
    if(psx.begin ()) {
      
      Serial.println("Controller found.");
      delay(300);

      if(!psx.enableAnalogSticks ()) {
        Serial.println("Cannot enable analogue sticks.");
      }
      
    }
    
    haveController = true;
    
  } else {
    
    if (!psx.read ()) { // if controller not detected, keep trying to find it until it is
      
      Serial.println ("Controller lost.");

      haveController = false;
      
    } else { // controller is found, run the code
      
      readAnalogSticks();

      if(psx.buttonPressed (PSB_R1) == 1) {
 
        int steer = map(rx, 0, 256, -250, 250); // limited to 500 for now cause its unusable at 1000
        int speed = map(ly, 0, 256, -250, 250);
        
        Serial.println(steer);
        Serial.println(speed);
        
        Send(speed,steer);
        
       }
      }
    }
     delay (1000 / 60);  
  }

  

  
  
