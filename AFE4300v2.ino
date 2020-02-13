#include <SPI.h>
#include "pwm_lib.h"
//#include "tc_lib.h"
using namespace arduino_due::pwm_lib;

#define PERIODS 16 
// pins
uint32_t periods[PERIODS]= // hundredths of usecs (1e-8 secs.)
{
  10,      // 0.1 usecs. 
  100,     // 1 usec.
  1000,    // 10 usecs. 
  10000,   // 100 usecs. 
  100000,  // 1000 usecs.
  1000000, // 10000 usecs.
  10000000,// 100000 usecs.
  50000000,// 500000 usecs.
  50000000,// 500000 usecs.
  10000000,// 100000 usecs.
  1000000, // 10000 usecs.
  100000,  // 1000 usecs.
  10000,   // 100 usecs. 
  1000,    // 10 usecs. 
  100,     // 1 usec.
  10,      // 0.1 usecs. 
};

uint32_t period=1;

//#define CAPTURE_TIME_WINDOW 1000000 // usecs
//#define DUTY_KEEPING_TIME 1500 // msecs

#define PIN_RESET  9
#define PIN_DRDY   2
#define PIN_LED     13
#define SS 10
#define SlaveSelectPin 10

// defining pwm object using pin 35, pin PC3 mapped to pin 35 on the DUE
// this object uses PWM channel 0
pwm<pwm_pin::PWMH0_PC3> pwm_pin35;
int valAFE;
bool flag;

void resetAFE4300();
void writeRegister(unsigned char address, unsigned int data); //Escribir en un registro del AFE4300
int readRegister(unsigned char address); //Leer desde el registro del  AFE4300
void initAFE4300()                                                                                                                     ; //Inicializar el AFE4300
void initWeighScale(); // Initializes the Weigh Scale Module
void initBCM(); // Initializes the BCM Module

void readData(); //  Leer del ADC_DATA_RESULT

/***************************
AFE4300 Definimos la direccion de los registros
****************************/
#define ADC_DATA_RESULT         0x00
#define ADC_CONTROL_REGISTER    0x01
#define MISC1_REGISTER          0x02
#define MISC2_REGISTER          0x03
#define DEVICE_CONTROL_1        0x09
#define ISW_MATRIX              0x0A
#define VSW_MATRIX              0x0B
#define IQ_MODE_ENABLE          0x0C
#define WEIGHT_SCALE_CONTROL    0x0D
#define BCM_DAC_FREQ            0x0E
#define DEVICE_CONTROL_2        0x0F
#define ADC_CONTROL_REGISTER_2  0x10
#define MISC3_REGISTER          0x1A

int inbyte;
long getHEXSerial()
{
  unsigned long hexData = 0;
  while (inbyte != '\n')
  {
    inbyte = Serial.read();  
    if (inbyte >= 0 && inbyte != '\n')
    { 
      switch(inbyte){
        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'E':
        case 'F':
        {
          inbyte = inbyte - 'A' + 10;
          break;  
        }
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
        {
          inbyte = inbyte - 'a' + 10;
          break;  
        }
        default:
        {
          inbyte = inbyte - '0';  
          break;  
        }
      }
      hexData = (hexData <<4) + inbyte;
      
    }
    inbyte = 0;
  }
  
  Serial.println(hexData,HEX);
  return hexData;
}
void setup()
{


pinMode(PIN_LED,   OUTPUT);
pinMode(PIN_RESET, OUTPUT);
pinMode(SS, OUTPUT);
//pinMode(PIN_DRDY, INPUT);
// ==== Interupt for data conversion from AFE4300 ====
pinMode(PIN_DRDY, INPUT);
//pinMode(slaveAPin, OUTPUT);
attachInterrupt(digitalPinToInterrupt(PIN_DRDY), readData, RISING);


 Serial.begin(115200);
 Serial.flush();
   

SPI.begin();
SPI.setClockDivider(SlaveSelectPin,21); // 12MHz clock generation default 4Mhz (21)
SPI.setDataMode(SlaveSelectPin, SPI_MODE1); // as Mr dlloyd  suggested . MODE 0 not working .

 resetAFE4300();
 initAFE4300();
 //initWeighScale();
 initBCM();
 //SPI.endTransaction();
 //========== set 1 MHZ for AFE4300 =============
 pwm_pin35.start(periods[period],(periods[period]>>1));         
}
void loop()
 {
    
    int valAFE;
    if(Serial.available()){
    
      Serial.println("Idle State. Enter Address Regiter want to read");
      long reg = getHEXSerial();
    
      valAFE = readRegister((unsigned char)reg);
      Serial.println(valAFE,HEX);
    }
//Read data out when Ready signal 
     if(flag)
     {
      valAFE =  readRegister(ADC_DATA_RESULT);//ADC_DATA_RESULT
      //Serial.print("Data AFE: ");
      Serial.println(valAFE);
      flag = 0;  
     }
      
// if(!pwm_pin35.set_period_and_duty(periods[period],(periods[period]>>1)))
// {
//        Serial.print("[ERROR] set_period_and_duty(");
//        Serial.print(periods[period]);
//        Serial.print(",");
//        Serial.print((periods[period]>>1));
//        Serial.println(") failed!");
// }

 }
/**
* @brief  Resets the AFE4300 device
*/
void resetAFE4300()
{
     digitalWrite(PIN_RESET ,LOW);
     delay(100);
     digitalWrite(PIN_RESET ,HIGH);
     writeRegister(MISC1_REGISTER,0x0000);
     writeRegister(MISC2_REGISTER,0xFFFF);
     writeRegister(MISC3_REGISTER,0x0030);
}

/**
* @brief  Writes to a register on the AFE4300
**/
void writeRegister(unsigned char address, unsigned int data)
{
unsigned char firstByte = (unsigned char)(data >> 8); //LA CARA ES UN OCHO)
unsigned char secondByte = (unsigned char)data;
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

digitalWrite(SS,LOW);
SPI.transfer(address);
//Send 2 bytes to be written
SPI.transfer(firstByte);
SPI.transfer(secondByte);
digitalWrite(SS,HIGH);

SPI.endTransaction();
}      

/**
* @brief  Reads from a register on the AFE4300
*/
int readRegister(unsigned char address)
{
   
   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
     
   int spiReceive = 0;
   unsigned char spiReceiveFirst = 0;
   unsigned char spiReceiveSecond = 0;
     
   address = address & 0x1F; //Ultimos 5 bits especifican la direccion
   address = address | 0x20; //Los primeros 3 bits tienen que ser 001 por código de operación de lectura
     //SPI.begin();
     digitalWrite(SS,LOW);  
     SPI.transfer(address);
     spiReceiveFirst  = SPI.transfer(0x00);
     spiReceiveSecond = SPI.transfer(0x00);
     digitalWrite (SS, HIGH);
     //SPI.end();
    SPI.endTransaction();
     
       //Combine the two received bytes into a signed int
    spiReceive = (spiReceiveFirst << 8);  //LA CARA ES UN OCHO)
    spiReceive |= spiReceiveSecond;
    return spiReceive;
}

/**
* @brief  Initializes the AFE4300 device
*/
void initAFE4300()
{
         writeRegister(ADC_CONTROL_REGISTER,0x5140);
         writeRegister(DEVICE_CONTROL_1,0x6004);//,0x6006
         writeRegister(ISW_MATRIX,0x0000);//0x0804
         writeRegister(VSW_MATRIX,0x0000);//0x0804
         writeRegister(IQ_MODE_ENABLE,0x0000);
         writeRegister(WEIGHT_SCALE_CONTROL,0x0000);
         writeRegister(BCM_DAC_FREQ,0x0010);//0x0010
         writeRegister(DEVICE_CONTROL_2,0x0000);
         writeRegister(ADC_CONTROL_REGISTER_2,0x0011);//0x0063
         writeRegister(MISC1_REGISTER,0x0000);
         writeRegister(MISC2_REGISTER,0xFFFF);
         writeRegister(MISC3_REGISTER,0x0030);//0x00C0//0x0030
         
}
/**
* @brief  Initializes the Weigh Scale Module
*/
void initWeighScale()
{
  writeRegister(ADC_CONTROL_REGISTER,0x4120); //Differential measurement mode, 32 SPS
  writeRegister(DEVICE_CONTROL_1,0x0005); //Power up weigh scale signal chain
  writeRegister(ADC_CONTROL_REGISTER_2,0x0000); //ADC selects output of weigh scale
  writeRegister(WEIGHT_SCALE_CONTROL,0x603F); //Gain = 4 DAC Offset = -1
  writeRegister(BCM_DAC_FREQ,0x0040); //Frequency = default
  writeRegister(IQ_MODE_ENABLE,0x0000); //Disable IQ mode
  writeRegister(ISW_MATRIX,0x0000); //Channels IOUTP1 and IOUTN0
  writeRegister(VSW_MATRIX,0x0000); //Channels VSENSEP1 and VSENSEN0
}

/**
* @brief  Initializes the BCM Module
*/
void initBCM()
{
  writeRegister(ADC_CONTROL_REGISTER,0x4120); //Differential measurement mode, 32 SPS //0x4120 4130: 64SPS
  writeRegister(DEVICE_CONTROL_1,0x0006); //Power up BCM signal chain
  writeRegister(ISW_MATRIX,0x0408); //Channels IOUTP1 and IOUTN0 0x0804
  writeRegister(VSW_MATRIX,0x0408); //Channels VSENSEP1 and VSENSEN0
  writeRegister(ADC_CONTROL_REGISTER_2,0x0063); //ADC selects output of BCM-I output
  writeRegister(WEIGHT_SCALE_CONTROL,0x0000); //Gain = 1 DAC Offset = 0
  writeRegister(BCM_DAC_FREQ,0x0010);//0x0010
  writeRegister(DEVICE_CONTROL_2,0x0006);
} 

/**
* @brief  Reads the ADC data register
*/
void readData()
{
  //int valAFE;
  //delayMicroseconds(8);
  //valAFE =  readRegister(ADC_DATA_RESULT);//ADC_DATA_RESULT
  //Serial.println(valAFE);
  flag = 1;
  //return valAFE;
}     
