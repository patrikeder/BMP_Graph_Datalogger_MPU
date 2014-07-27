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


#define LCD_verbose

#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#include <stdlib.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

#define MPU_POWER 48 //MPU power source pin


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[128]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
int measREADY = 0;
#define HEADER "Time;arX;arY;arZ;aX;aY;aZ;gX;gY;gZ;yaw;pitch;roll;"

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#define SLEEP_8S          bit (WDP3) | bit (WDP0)
#define SLEEP_1S          bit (WDP2) | bit (WDP1)
#define SLEEP_125mS       bit (WDP1) | bit (WDP0)
#define SLEEP_64mS        bit (WDP1)

const unsigned long RUNTIME_MAX = 5*60*1000; //run 5 Minutes and restart

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

void(* resetFunc) (void) = 0;//declare reset function at address 0

void wdt_reset_trigger()
{
  //cli();                  // Clear interrupts
  //wdt_enable(WDTO_15MS);      // Set the Watchdog to 15ms
  resetFunc();
  while(1);            // Enter an infinite loop
}


#include <LCD5110_Graph.h>

LCD5110 myGLCD(7,8,9,10,6);
extern uint8_t SmallFont[];

#define base_logfile "data"
char  logfile[8+3];

const unsigned int  uArray_Size = 20;
String dataString_array[uArray_Size];
int ap = 0;

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 4;

#ifdef LCD_verbose
uint16_t xpos = 0;

void LCD_update(long time)
{
  char tmp[8];
  xpos ++;

  myGLCD.print("Sec=  ",LEFT,0);
  myGLCD.printNumI(time/1000,RIGHT,0);

  myGLCD.print("ax=  ",LEFT,8);
  myGLCD.print(String((int)aa.x),RIGHT,8);  

  myGLCD.print("gz=  ",LEFT,16);
  dtostrf(gravity.z,1, 3,tmp);
  myGLCD.print(tmp,RIGHT,16);    

  myGLCD.print("yaw=  ",LEFT,24);
  dtostrf(ypr[0],1, 3,tmp);
  myGLCD.print(tmp,RIGHT,24);

  myGLCD.print("pitch=  ",LEFT,32);
  dtostrf(ypr[1],1, 3,tmp);
  myGLCD.print(tmp,RIGHT,32);

  myGLCD.print("rol=  ",LEFT,40);
  dtostrf(ypr[2],1, 3,tmp);
  myGLCD.print(tmp,RIGHT,40);

  myGLCD.update();       

  if (xpos >= 83) 
  {
    xpos = 0;
    myGLCD.clrScr();
  }
}
#endif

void LCD_msg_out(String msg,int time = 500)
{
  myGLCD.clrScr();
  myGLCD.print("             ",LEFT,15);
  myGLCD.print(msg,LEFT,15);
  myGLCD.update(); 
  wdt_sleep(SLEEP_125mS);
  wdt_sleep(SLEEP_125mS);
  if (time > 500){
    wdt_sleep(SLEEP_125mS);
    wdt_sleep(SLEEP_125mS);
    wdt_sleep(SLEEP_125mS);
    wdt_sleep(SLEEP_125mS);
  }
  myGLCD.clrScr();
}

int meas_log()
{  // make a string for assembling the data to log:
  char tmp[3][8];
  String dataString = "";

  long time = millis();
  // blink LED to indicate activity
  //  blinkState = !blinkState;
  //  digitalWrite(LED_PIN, blinkState);
  dataString += String(time) + String(";");
  dataString += String((int)aaReal.x) + String(";") + String((int)aaReal.y)+ String(";") + String((int)aaReal.z)+ String(";");
  dataString += String((int)aa.x) + String(";") + String((int)aa.y)+ String(";") + String((int)aa.z)+ String(";");

  dtostrf(gravity.x,1, 3,tmp[0]);
  dtostrf(gravity.y,1, 3,tmp[1]);
  dtostrf(gravity.z,1, 3,tmp[2]);
  dataString += String(tmp[0]) + String(";") + String(tmp[1])+ String(";") + String(tmp[2])+ String(";");

  dtostrf(ypr[0],1, 3,tmp[0]);
  dtostrf(ypr[1],1, 3,tmp[1]);
  dtostrf(ypr[2],1, 3,tmp[2]);  
  dataString += String(tmp[0]) + String(";") + String(tmp[1])+ String(";") + String(tmp[2])+ String(";");

  dataString_array[ap++] = dataString;

  if (ap == uArray_Size){
    ap = 0;  
    digitalWrite(LED_PIN, HIGH);
    file_write(dataString_array);
#ifdef LCD_verbose
    LCD_update(time);
#endif
    digitalWrite(LED_PIN, LOW);
  }

  if (time > RUNTIME_MAX){
    LCD_msg_out("Restart !");
    wdt_reset_trigger();
    while (1);
  }

  return 0;
}

int make_header()
{
  if (file_write(HEADER) == 0)
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


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setFont(SmallFont);

  //Serial.begin(115200);

  // configure pins for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(53, OUTPUT); // MEGA
  pinMode(MPU_POWER,OUTPUT);
  digitalWrite(MPU_POWER, LOW);

  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    // don't do anything more:
    while(1){
      LCD_msg_out("Card failed");
    }
  }

  SPI.setClockDivider(SPI_CLOCK_DIV2); //Speed up SD

  for (int i = 0;i<500;i++)
  {
    String tmp_logfile = String(base_logfile) + String(i);
    tmp_logfile += String(".csv");
    tmp_logfile.toCharArray(logfile, 16);
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

    if (i==499){      
      while(1){
        LCD_msg_out("Out of files",2000); // no file available
      }
    }
  }

  digitalWrite(MPU_POWER, HIGH); //enable power source for MPU
  delay(100);
  ap = 0;
  mpu.initialize();

  LCD_msg_out(mpu.testConnection() ? "MPU6050 ok" : "MPU6050 failed",1000);

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(59);
  mpu.setYGyroOffset(62);
  mpu.setZGyroOffset(1);
  mpu.setZAccelOffset(0); 

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
    LCD_msg_out(String("DMP failed: ")+ String(devStatus));
    while (1);
  }
  measREADY = 0;
  LCD_msg_out("Logging");
}

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;


  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
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
  if  (fifoCount == 1024) {
#ifdef LCD_verbose
    LCD_msg_out("FIFO overflow!");
#endif 
    file_write("FIFO overflow!");

    // reset so we can continue cleanl
    mpu.resetFIFO();	  
  }
  else if(mpuIntStatus & 0x10){
#ifdef LCD_verbose
    LCD_msg_out("FIFO intOvr!");
#endif
    file_write("FIFO intOvr!");
    mpu.resetFIFO();	  
  }  
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    measREADY = 1; 
  }
  if (measREADY == 1)
  {
    meas_log();
    measREADY = 0;
  }

}




































