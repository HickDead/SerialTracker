
/*
 *  SerialTracker v0.1
 *  
 *  Copyright (c) 2019 HickDead (https://github.com/HickDead)
 *  
 *  Includes code taken from:
 *    MPU6050_DMP6_ESPWiFi.ino
 *      Copyright (c) 2012 Jeff Rowberg
 *    STM32_BluePill.ino
 *      Copyright (c) 2019 Relativty - Sarahghz
 *
 *  Needs Jeff Rowberg's i2cdevlib, 
 *  with the included MPU6050 driver.
 *  https://www.i2cdevlib.com/usage
 *  https://github.com/jrowberg/i2cdevlib/
 *  
 */



#define SENSOR_MPU6050
//#define SENSOR_MPU9150   // untested
#define MPU_AD0 0


/*  ==================================
 *  supply your own gyro offsets here, 
 *  scaled for min sensitivity
 *  follow procedure here to calibrate offsets 
 *  https://github.com/kkpoon/CalibrateMPU6050
 *  ==================================
 */


#define ACCEL_OFFSET_X     0
#define ACCEL_OFFSET_Y     0
#define ACCEL_OFFSET_Z     0
#define GYRO_OFFSET_X      0
#define GYRO_OFFSET_Y      0
#define GYRO_OFFSET_Z      0

// after filling out the numbers above, uncomment this
//#define CALIBRATE

// uncomment these for more periodic output on the serial console
#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

// I2C speed in KHz, 100 - 400
#define WIRE_CLOCK 100  
//#define WIRE_CLOCK 400


// We'd like to send 4 floats (max 7 bytes each) 3 commas plus CRLF (another 5 bytes)
// 100 times every second, so that's (4*7+5) * 8bits * 100/sec = 26400 bps


#ifdef ESP32
/* 
 *  ESP32 specific code
 */

# include "BluetoothSerial.h"
# if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#   error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
# endif
BluetoothSerial SerialBT;

# define CONS_BAUD_RATE "SerialTracker"
# define SerialData Serial
# define SerialCons SerialBT

// I2C pins
# define INT_PIN GPIO_NUM_17
# define SDA_PIN GPIO_NUM_18
# define SCL_PIN GPIO_NUM_19
# define VCC_PIN GPIO_NUM_5

// workaround for bug in MPU6050_6Axis_MotionApps20.h
# define __PGMSPACE_H_ 1

#elif defined( ARDUINO_SAM_DUE )
/* 
 *  Arduino Due specific code (untested)
 */

# define SerialData SerialUSB
# define SerialCons Serial1    // TX 18, RX 19 ?

// I2C pins
# define INT_PIN 2       // arbitrarely chosen, any digital pin should work
///# define SDA_PIN 20    // for reference, not used, fixed?
///# define SCL_PIN 21    // for reference, not used, fixed?

#elif defined( ARDUINO_BLUEPILL_F103C6 ) || defined( ARDUINO_BLUEPILL_F103C8 ) || defined( ARDUINO_BLACKPILL_F103C8 ) || defined( ARDUINO_BLACKPILL_F303CC )
/*
 *  Black-/Blue-pill specific code (untested)
 */

# define SerialData Serial
# define SerialCons Serial1    // TX PB6, RX PB7 ?

// I2C pins
# define INT_PIN PA1
# define SCL_PIN PB8    // default, black = PB6 ?? (or PB10)
# define SDA_PIN PB9    // default, black = PB7 ?? (or PB11)

#else // ESP32 || ARDUINO_SAM_DUE || ARDUINO_BLACKPILL_F103C8 || ARDUINO_BLUEPILL_F103C6
/*
 *  Everything else (untested)
 */

// Arduino Uno?
# include <SoftwareSerial.h>
SoftwareSerial SerialSW( 10, 11);

# define CONS_BAUD_RATE  19200
# define SerialData Serial
# define SerialCons SerialSW

// I2C pins
# define INT_PIN 2
///#  define SDA_PIN A4   // for reference, not used, fixed
///#  define SCL_PIN A5   // for reference, not used, fixed
# define VCC_PIN 5

#endif // ESP32 || ARDUINO_SAM_DUE || ARDUINO_BLUEPILL_F103C6 || ARDUINO_BLUEPILL_F103C8 || ARDUINO_BLACKPILL_F103C8 || ARDUINO_BLACKPILL_F303CC


#ifndef DATA_BAUD_RATE
# define DATA_BAUD_RATE 115200
#endif // DATA_BAUD_RATE
#ifndef CONS_BAUD_RATE
# define CONS_BAUD_RATE 115200
#endif // CONS_BAUD_RATE


// Somehow older IDE can't find Wire.h otherwise...
//# if ARDUINO < 10800 && I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//# include <Wire.h>
//# endif // ARDUINO < 10800 && I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
////#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif // I2CDEV_ARDUINO_WIRE

// class default I2C address for the MPU6050 and MPU9150 is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

#ifdef SENSOR_MPU6050
# include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu( 0x68 + MPU_AD0);
#elif defined( SENSOR_MPU9150 )
# include "MPU9150_9Axis_MotionApps41.h"
MPU9150 mpu( 0x68 + MPU_AD0);
#else
# error No sensor selected!
#endif

uint16_t   packetSize;        // expected DMP packet size (default is 42 bytes)

volatile bool mpuInterrupt = false;     // has MPU interrupt pin gone high?

/*
 *  Handle interrupts from the motion sensor
 */
void dmpDataReady()
{
    mpuInterrupt = true;
}



/*
 *  Initialize the serial ports
 */
void setup_serial()
{
  
#ifdef DEBUG
  Serial.begin( CONS_BAUD_RATE);    // for when DEBUG is defined in MPU6050_6Axis_MotionApps20.h
#endif // DEBUG

  SerialData.begin( DATA_BAUD_RATE);
  SerialCons.begin( CONS_BAUD_RATE);
# ifdef ARDUINO_SAM_DUE
//  while( ! SerialData )   { ; }   // Needed for native USB port only
# endif // ARDUINO_SAM_DUE

}



/* 
 *  Initialize the i2c bus
 */
void setup_wire()
{

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
# ifdef ESP32
  Wire.begin( SDA_PIN, SCL_PIN, WIRE_CLOCK*1000UL);
# else // ESP32
#   if defined( ARDUINO_BLUEPILL_F103C6 ) || defined( ARDUINO_BLUEPILL_F103C8 ) || defined ( ARDUINO_BLACKPILL_F103C8 ) || defined ( ARDUINO_BLACKPILL_F303CC )
  Wire.setSDA( SDA_PIN);
  Wire.setSCL( SCL_PIN);
#   endif // ARDUINO_BLUEPILL_F103C6 || ARDUINO_BLUEPILL_F103C8 || ARDUINO_BLACKPILL_F103C8 || ARDUINO_BLACKPILL_F303CC
  Wire.begin();
  Wire.setClock(WIRE_CLOCK*1000UL);
# endif // ESP32
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(WIRE_CLOCK, true);
#endif // I2CDEV_ARDUINO_WIRE || I2CDEV_BUILTIN_FASTWIRE

}


/*
 *  Initialize the motion sensor
 */
bool setup_mpu()
{
  bool     value = false;
  uint8_t  devStatus;         // return status after each device operation (0 = success, !0 = error)
  uint8_t  mpuIntStatus;      // holds actual interrupt status byte from MPU

  
  // initialize device
  SerialCons.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  SerialCons.println(F("Testing device connections..."));
  if( mpu.testConnection() )
    SerialCons.println( F("MPU connection successful"));
  else
    SerialCons.println( F("MPU connection failed"));

  // load and configure the DMP
  SerialCons.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

#ifdef CALIBRATE
  mpu.setXAccelOffset(ACCEL_OFFSET_X);
  mpu.setYAccelOffset(ACCEL_OFFSET_Y);
  mpu.setZAccelOffset(ACCEL_OFFSET_Z);
  mpu.setXGyroOffset(GYRO_OFFSET_X);
  mpu.setYGyroOffset(GYRO_OFFSET_Y);
  mpu.setZGyroOffset(GYRO_OFFSET_Z);
#endif // CALIBRATE

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
  {
    // turn on the DMP, now that it's ready
    SerialCons.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // configure interrupt pin as input
    pinMode( INT_PIN, INPUT);
    // activate pull-up on interrupt pin
    digitalWrite( INT_PIN, HIGH);
    // enable Arduino interrupt detection
    SerialCons.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    SerialCons.println(F("DMP ready! Waiting for first interrupt..."));
    value= true;
  }
  else 
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    SerialCons.print(F("DMP Initialization failed (code "));
    SerialCons.print(devStatus);
    SerialCons.println(F(")"));

    // wait a second before retrying
    delay( 900);
  }
  
  return value;
}



/*
 *  The setup function that gets called once at startup
 */
void setup()
{
  bool       success;

  
  setup_serial();

  SerialCons.print( "Compiled with Arduino IDE version: ");
  SerialCons.println( ARDUINO);

  success= false;
  while( ! success )
  {
#ifdef VCC_PIN    // MPU VCC pin hooked to IO pin?  powercycle it!
    pinMode( VCC_PIN, OUTPUT);
    digitalWrite( VCC_PIN, LOW);
    delay( 100);
    digitalWrite( VCC_PIN, HIGH);
#endif // VCC_PIN

    setup_wire();
    success=setup_mpu();
  }

#ifdef ESP32
  SerialCons.println( "The device started, now you can pair it with bluetooth!");
#endif // ESP32

}



/*
 *  Spit out quaternion data to the PC
 */
void output_relativty(Quaternion q)
{
  const int          accuracy = 4;

  // send our data X,Y,Z,W !!!
  SerialData.print(q.x, accuracy);
  SerialData.print(",");
  SerialData.print(q.y, accuracy);
  SerialData.print(",");
  SerialData.print(q.z, accuracy);
  SerialData.print(",");
  SerialData.print(q.w, accuracy);
  SerialData.println();
  SerialData.flush();

}



/*
 *  The loop function that keeps on getting called ad infinitum
 */
void loop()
{
  uint8_t            mpuIntStatus;      // holds actual interrupt status byte from MPU
  static uint16_t    fifoCount;         // count of all bytes currently in FIFO
  static uint8_t     fifoBuffer[64];    // FIFO storage buffer
  
  static uint8_t     counter = 0;
  Quaternion         q;           // [w, x, y, z]         quaternion container
  VectorInt16        aa;          // [x, y, z]            accel sensor measurements


  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize)
    return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    SerialCons.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // process all packets currently in the FIFO
    while( fifoCount >= packetSize )
    {
  
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
  
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);


      // report result
      output_relativty( q);


      // about every second?
      counter %= 100;
      if( ! counter++ )
      {
        
#if defined( OUTPUT_READABLE_REALACCEL ) || defined( OUTPUT_READABLE_WORLDACCEL )
        mpu.dmpGetAccel(&aa, fifoBuffer);
#endif // OUTPUT_READABLE_REALACCEL || OUTPUT_READABLE_WORLDACCEL

        // display values in readable form
        output_readable( q, aa);
      }
    }
  }

}



/*
 *  More trivial output stuff below...
 */
void output_quarernion(Quaternion q)
{

  // display quaternion values in easy matrix form: w x y z
  SerialCons.print("quaternion(w,x,y,z):\t");
  SerialCons.print(q.w);
  SerialCons.print("\t");
  SerialCons.print(q.x);
  SerialCons.print("\t");
  SerialCons.print(q.y);
  SerialCons.print("\t");
  SerialCons.println(q.z);

}



void output_euler(Quaternion q)
{
  float              euler[3];    // [psi, theta, phi]    Euler angle container

      
  mpu.dmpGetEuler(euler, &q);

  // display Euler angles in degrees
  SerialCons.print("euler(psi,theta,phi):\t\t");
  SerialCons.print(euler[0] * 180/M_PI);
  SerialCons.print("\t");
  SerialCons.print(euler[1] * 180/M_PI);
  SerialCons.print("\t");
  SerialCons.println(euler[2] * 180/M_PI);
  
}



void output_yawPitchRoll(Quaternion q)
{
  VectorFloat        gravity;     // [x, y, z]            gravity vector
  float              ypr[3];      // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // display Euler angles in degrees
  SerialCons.print("angles(yaw,pitch,roll):\t\t");
  SerialCons.print(ypr[0] * 180/M_PI);
  SerialCons.print("\t");
  SerialCons.print(ypr[1] * 180/M_PI);
  SerialCons.print("\t");
  SerialCons.println(ypr[2] * 180/M_PI);

}



void output_realAccel(Quaternion q,VectorInt16 aa)
{
  VectorFloat        gravity;     // [x, y, z]            gravity vector
  VectorInt16        aaReal;      // [x, y, z]            gravity-free accel sensor measurements


  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  // display real acceleration, adjusted to remove gravity
  SerialCons.print("realAccel(x,y,z):\t\t");
  SerialCons.print(aaReal.x);
  SerialCons.print("\t");
  SerialCons.print(aaReal.y);
  SerialCons.print("\t");
  SerialCons.println(aaReal.z);
}



void output_worldAccel(Quaternion q,VectorInt16 aa)
{
  VectorFloat        gravity;     // [x, y, z]            gravity vector
  VectorInt16        aaReal;      // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16        aaWorld;     // [x, y, z]            world-frame accel sensor measurements


  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  SerialCons.print("worldAccel(x,y,z):\t\t");
  SerialCons.print(aaWorld.x);
  SerialCons.print("\t");
  SerialCons.print(aaWorld.y);
  SerialCons.print("\t");
  SerialCons.println(aaWorld.z);
  
}



void output_readable(Quaternion q,VectorInt16 aa)
{

#ifdef OUTPUT_READABLE_QUATERNION
  output_quarernion( q);
#endif // OUTPUT_READABLE_QUATERNION
#ifdef OUTPUT_READABLE_EULER
  output_euler( q);
#endif // OUTPUT_READABLE_EULER
#ifdef OUTPUT_READABLE_YAWPITCHROLL
  output_yawPitchRoll( q);
#endif
#ifdef OUTPUT_READABLE_REALACCEL
  output_realAccel( q, aa);
#endif // OUTPUT_READABLE_REALACCEL
#ifdef OUTPUT_READABLE_WORLDACCEL
  output_worldAccel( q, aa);
#endif // OUTPUT_READABLE_WORLDACCEL
#if defined( OUTPUT_READABLE_QUATERNION ) \
  || defined( OUTPUT_READABLE_EULER ) \
  || defined( OUTPUT_READABLE_YAWPITCHROLL ) \
  || defined( OUTPUT_READABLE_REALACCEL ) \
  || defined( OUTPUT_READABLE_WORLDACCEL )
  SerialCons.println();
#endif // OUTPUT_READABLE_*
  
}
