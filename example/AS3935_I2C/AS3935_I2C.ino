/* VMA343 AS3935 lightning sensor - I2C example program by PSI @ Velleman NV 
   Based on the example by J. Steinlage @ https://github.com/PlayingWithFusion/PWFusion_AS3935_I2C/blob/master/Examples/as3935_lightning_i2c_nocal/as3935_lightning_i2c_nocal.ino
*/ 

//libraries
#include "I2C.h"                                  //I2C library for AS3935 lib can be found here => https://github.com/rambo/I2C
#include "AS3935_I2C_VMA212mod.h"     // Specific lib for AS3935 sensor (mod by PSI to choose IRQ pin myself)

//AS3935 sensor defenitions 
#define intPIN 2        //interrupt pin
boolean interrupted=0;
#define AS3935_ADD           0x03  // Default I2C Address of AS3935 sensor: x03 <-- you can use i2C scanner to see the i2c address
#define AS3935_CAPACITANCE   72    // <-- SET THIS VALUE TO THE NUMBER LISTED ON YOUR BOARD (calculation between the caps)
                                   // 72 pF default, 56pF for PlayingWithFusion board, 112pF for other
#define AS3935_IRQPIN        2    // interrupt pin on board (18 or 19 on MEGA, 2 or 3 on UNO)

// defines for general chip settings
boolean AS3935_OUTDOORS;            // Set to 1 to enabled for Outdoor Used, use 0 for Indoor Use
int AS3935_EEPROMaddr = 0;          // address to store this value in long term memory (EEPROM)
#define AS3935_DIST_DIS      0
#define AS3935_DIST_EN       1

// define Lightning sensor on specific address
AS3935_I2C lightX((uint8_t)AS3935_ADD);


void setup() 
{
  Serial.begin(9600);
  Serial.println("AS3935 Lightning Sensor Test");
  Serial.println("Starting setup procedure ...");
  delay(2000);
  
  I2c.begin(); // Set i2C Libr Enable pullups set to 400kHz speed
  I2c.pullup(true); //Set i2C pUll up 
  I2c.setSpeed(1);  //Set speed to 1
  delay(10); // Delay 
  
  lightX.AS3935_DefInit();   // set registers to default  
  // now update sensor cal for your application and power up chip
  lightX.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUTDOORS, AS3935_DIST_EN); //
                             //   AS3935_ManualCal Parameters:
                             //   --> capacitance, in pF (marked on package)
                             //   --> indoors/outdoors (AS3935_INDOORS:0 / AS3935_OUTDOORS:1)
                             //   --> disturbers (AS3935_DIST_EN:1 / AS3935_DIST_DIS:2)
                             // function also powers up the chip

  // enable interrupt (hook IRQ pin to Arduino Uno/Mega interrupt to AS3935_IRQPIN -> 19)
  pinMode(AS3935_IRQPIN, INPUT_PULLUP);   // See http://arduino.cc/en/Tutorial/DigitalPins
  attachInterrupt(digitalPinToInterrupt(AS3935_IRQPIN), interruptFunction, RISING);
  
  Serial.println("Printing out Reg vals:");
  lightX.AS3935_PrintAllRegs(); // Print all Regs
  delay(250);
  interrupted=0;
  Serial.println("End of setup procedure.");
}

void loop() 
{
  if(interrupted !=0)
  {
    Serial.println("Interrupt from lightining sensor!");
    delay(5); //wait so not to overflow the bus
    
    //get interrupt source
    uint8_t int_src = lightX.AS3935_GetInterruptSrc();
    if(0 == int_src)
    {
      Serial.println("IRQ source result not expected...");
    }
    else if(1 == int_src)
    {
      uint8_t lightning_dist_km = lightX.AS3935_GetLightningDistKm();
      Serial.print("Lightning detected! Distance to strike: ");
      Serial.print(lightning_dist_km);
      Serial.println("KM");
    }
    else if(2 == int_src)
    {
      Serial.println("Disturber detected!");
    }
    else if(3 == int_src)
    {
      Serial.println("Noise level too high!");
    }
    //lightX.AS3935_PrintAllRegs(); // Set for debug
    interrupted = 0;
  }
}

void interruptFunction() 
{
  interrupted = 1;
}
