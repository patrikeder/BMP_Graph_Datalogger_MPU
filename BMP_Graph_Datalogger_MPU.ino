/*
  SD card datalogger
 
 This example shows how to log data from three analog sensors 
 to an SD card using the SD library.
 	
 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 
 MEGA:
 ** MOSI - pin 51
 ** MISO - pin 50
 ** CLK - pin 52
 ** CS - pin 4
 
 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 	 
 */

#include <SD.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


#include <avr/sleep.h>
#include <avr/wdt.h>

#define SLEEP_8S          bit (WDP3) | bit (WDP0)
#define SLEEP_1S          bit (WDP2) | bit (WDP1)
#define SLEEP_125mS       bit (WDP1) | bit (WDP0)
#define SLEEP_64mS        bit (WDP1)

#include <LCD5110_Graph.h>

LCD5110 myGLCD(7,8,9,10,6);
extern uint8_t SmallFont[];

#define base_logfile "data"
char  logfile[8+3];

const unsigned int  uArray_Size = 83;
String dataString_array[uArray_Size];
int ap = 0;

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 4;
uint16_t pval,xpos = 0;

void LCD_update(long time, uint32_t val)
{
  xpos ++;

  pval = (uint16_t) map(val, -180, 180, 47, 25);
  myGLCD.setPixel(xpos, pval);

  myGLCD.print("Sec=  ",LEFT,0);
  myGLCD.printNumI(time/1000,RIGHT,0);
  myGLCD.print("ax=  ",LEFT,8);
  myGLCD.print(String((long)aa.x),RIGHT,8);
  myGLCD.print("gx=  ",LEFT,16);
  myGLCD.print(String((long)gravity.x),RIGHT,16);
  myGLCD.print("ay=  ",LEFT,24);
  myGLCD.print(String((long)aa.y),RIGHT,24);
  myGLCD.print("gy=  ",LEFT,32);
  myGLCD.print(String((long)gravity.y),RIGHT,32);

  myGLCD.update();       

  if (xpos >= 83) 
  {
    xpos = 0;
    myGLCD.clrScr();
  }
}

void LCD_msg_out(String msg,int time = 500)
{
  myGLCD.clrScr();
  myGLCD.print("             ",LEFT,15);
  myGLCD.print(msg,LEFT,15);
  myGLCD.update(); 
  delay(time);
  myGLCD.clrScr();
}

int meas_log()
{  // make a string for assembling the data to log:
  String dataString = "";

  long time = millis();

  //LCD_update(time,aa.z);

  dataString += String(time) + String(";");
  dataString += String((long)aa.x) + String(";") + String((long)aa.y)+ String(";") + String((long)aa.z)+ String(";");
  dataString += String((long)gravity.x) + String(";") + String((long)gravity.y)+ String(";") + String((long)gravity.z)+ String(";");
  dataString += String((long)(ypr[0] * 180/M_PI)) + String(";") + String((long)(ypr[1] * 180/M_PI))+ String(";") + String((long)(ypr[2] * 180/M_PI))+ String(";");

  dataString_array[ap++] = dataString;

  if (ap == uArray_Size){
    ap = 0;  
    file_write(dataString_array);
  }
  return 0;
}

int make_header()
{
  if (file_write("Time;aX;aY;aZ;gX;gY;gZ;yaw;pitch;roll;") == 0)
  {
    LCD_msg_out("Header done",250);
  }
  return 0;
}

int file_write(String strng)
{
  int ret = 1;
  File dataFile = SD.open(logfile, FILE_WRITE);
  if(dataFile){
    dataFile.println(strng);
    dataFile.close();
    ret = 0;
  }
  else{
    LCD_msg_out("Error on write");
    ret = 1;
  }
  return ret;
}

int file_write(String strng[])
{
  int ret = 1;
  File dataFile = SD.open(logfile, FILE_WRITE);
  if(dataFile){
    for (int i = 0; i<= (uArray_Size-1);i++){
      dataFile.println(strng[i]);
    }
    dataFile.close();
    ret = 0;
  }
  else{
    LCD_msg_out("Error on write");
    ret = 1;
  }
  return ret;
}


int file_write(String strng1,String strng2)
{
  int ret = 1;
  File dataFile = SD.open(logfile, FILE_WRITE);
  if(dataFile){
    dataFile.print(strng1);
    dataFile.println(strng2);
    dataFile.close();
    ret = 0;
  }
  else{
    LCD_msg_out("Error on write",500);
    ret = 1;
  }
  return ret;
}

// watchdog interrupt
ISR (WDT_vect) 
{
  wdt_disable();  // disable watchdog
}  // end of WDT_vect

void wdt_sleep(int time)
{
  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and an interval 
  WDTCSR = bit (WDIE) | time;    // set WDIE, and 8 seconds delay
  wdt_reset();  // pat the dog

  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_enable();


  sleep_cpu ();  

  // cancel sleep as a precaution
  sleep_disable();   
}


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
int measREADY = 0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{

  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setFont(SmallFont);

  pinMode(53, OUTPUT); // MEGA

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    LCD_msg_out("Card failed");
    // don't do anything more:
    while(1);
  }

  for (int i = 0;i<100;i++)
  {
    String tmp_logfile = String(base_logfile) + String(i);
    tmp_logfile += String(".txt");
    tmp_logfile.toCharArray(logfile, 16) ;
    Serial.println(logfile);
    // if the file is available, next number:
    if (SD.exists(logfile)) {
      LCD_msg_out("File exists, next",250);
      continue;      
    }
    else{
      LCD_msg_out("File new");
      if(file_write("logging:",logfile) == 0){
        make_header();
        break;
      }
      else{
        LCD_msg_out("Error on creation",2000);
        while(1);
      }
    }
  }

  ap = 0;
  mpu.initialize();

  LCD_msg_out(mpu.testConnection() ? "MPU6050 ok" : "MPU6050 failed",1000);

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); 

  if (devStatus == 0) {    
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    LCD_msg_out(String("DMP failed code ")+ String(devStatus));
    while (1);
  }
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  measREADY = 0;
}

void loop()
{
  uint32_t counter;

  LCD_msg_out("Logging");

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (measREADY == 1)
    {
      meas_log();
      measREADY = 0;
    }
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    LCD_msg_out("FIFO overflow!");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    measREADY = 1;
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

}





















