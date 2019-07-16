
/*
 *  SerialTracker v0.2
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
//#define SENSOR_MPU9150

// set to 1 if AD0 on the sensor is connected to VCC
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
//#define OUTPUT_READABLE_QUATERNION
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
// non-data on seperate serial?
//# define SerialCons SerialBT

// I2C pins
# define INT_PIN GPIO_NUM_17  //
# define SDA_PIN GPIO_NUM_18  //
# define SCL_PIN GPIO_NUM_19  //
# define VCC_PIN GPIO_NUM_5   // arbitrarely chosen, any digital output pin should work

// workaround for bug in MPU6050_6Axis_MotionApps20.h
# define __PGMSPACE_H_ 1

#elif defined( ARDUINO_SAM_DUE )
/* 
 *  Arduino Due specific code (untested)
 */

# define SerialData SerialUSB
// non-data on seperate serial?
//# define SerialCons Serial1   // TX 18, RX 19 ?

// I2C pins
# define INT_PIN 2             // arbitrarely chosen, any digital pin should work
# define SDA_PIN 20            // for reference, not used, fixed?
# define SCL_PIN 21            // for reference, not used, fixed?
# define VCC_PIN 5             // arbitrarely chosen, any digital output pin should work

# define LED_PIN 13
# define LED_ON HIGH
# define LED_OFF LOW

#elif defined( ARDUINO_ARCH_STM32 )
/*
 *  Black-/Blue-pill specific code
 */

# define SerialData Serial
// non-data on seperate serial?
//# define SerialCons Serial1     // TX PA9, RX PA10

// I2C pins
# define INT_PIN PA1            //
# define SCL_PIN PB6            // for reference, unused, default
# define SDA_PIN PB7            // for reference, unused, default
# define VCC_PIN PB5            // arbitrarely chosen, any output pin should work

#define LED_PIN PC13
#define LED_ON LOW
#define LED_OFF HIGH


#else // ESP32 || ARDUINO_SAM_DUE || ARDUINO_ARCH_STM32
/*
 *  Everything else (untested)
 */

// Arduino Uno?
# include <SoftwareSerial.h>
SoftwareSerial SerialSW( 10, 11);

# define CONS_BAUD_RATE  19200
# define SerialData Serial
// non-data on seperate serial?
//# define SerialCons SerialSW

// I2C pins
# define INT_PIN 2              // or 3
# define SDA_PIN A4             // for reference, unused, =fixed
# define SCL_PIN A5             // for reference, unused, =fixed
# define VCC_PIN 5              // arbitrarely chosen

#define LED_PIN LED_BUILTIN
#define LED_ON HIGH
#define LED_OFF LOW

#endif // ESP32 || ARDUINO_SAM_DUE || ARDUINO_ARCH_STM32


#ifndef DATA_BAUD_RATE
# define DATA_BAUD_RATE 115200
#endif // DATA_BAUD_RATE

#ifndef SerialCons
# undef CONS_BAUD_RATE
# define SerialCons SerialData
#elif ! defined( CONS_BAUD_RATE )
# define CONS_BAUD_RATE 115200
#endif // CONS_BAUD_RATE


// Somehow older IDE can't find Wire.h otherwise...
//#if ARDUINO < 10800
//#include <Wire.h>
//#endif // ARDUINO < 10800


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#ifdef SENSOR_MPU6050
# include "MPU6050_6Axis_MotionApps20.h"
MPU6050 sensor( 0x68 + MPU_AD0);
#elif defined( SENSOR_MPU9150 )
# include "MPU9150_9Axis_MotionApps41.h"
MPU9150 sensor( 0x68 + MPU_AD0);
#else
# error No sensor selected!
#endif

uint16_t   packetSize;                    // expected DMP packet size (default is 42 bytes)

volatile bool sensorInterrupt = false;    // has sensor interrupt pin gone high?

/*
 *  Handle interrupts from the motion sensor
 */
void dmpDataReady()
{
    sensorInterrupt= true;
}



/*
 *  Initialize the serial ports
 */
void setup_serial()
{
  
# ifdef DEBUG
  Serial.begin( CONS_BAUD_RATE);    // just in case DEBUG is defined in MPU6050_6Axis_MotionApps20.h
# endif // DEBUG

  SerialData.begin( DATA_BAUD_RATE);
# ifdef CONS_BAUD_RATE
  SerialCons.begin( CONS_BAUD_RATE);
# endif // CONS_BAUD_RATE

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
bool setup_sensor()
{
  bool     value = false;
  uint8_t  devStatus;           // return status after each device operation (0 = success, !0 = error)

  
  // initialize device
  SerialCons.println(F("Initializing I2C devices..."));
  sensor.initialize();

  // verify connection
  SerialCons.println(F("Testing device connections..."));
  if( sensor.testConnection() )
    SerialCons.println( F("sensor connection successful"));
  else
  {
    SerialCons.print( F("sensor connection failed, DeviceID="));
    SerialCons.print( sensor.getDeviceID(), HEX);
    SerialCons.println();
  }

    // load and configure the DMP
    SerialCons.println(F("Initializing DMP..."));
    devStatus= sensor.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if( devStatus == 0 )
    {

      // turn on the DMP, now that it's ready
      SerialCons.println(F("Enabling DMP..."));
      sensor.setDMPEnabled(true);

      // configure interrupt pin as input
      pinMode( INT_PIN, INPUT);
      // activate pull-up on interrupt pin?
//      digitalWrite( INT_PIN, HIGH);
      // enable Arduino interrupt detection
      SerialCons.print(F("Enabling interrupt detection (Arduino external interrupt "));
      SerialCons.print(digitalPinToInterrupt(INT_PIN));
      SerialCons.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);

      // get expected DMP packet size for later comparison
      packetSize= sensor.dmpGetFIFOPacketSize();

#     ifdef CALIBRATE
      sensor.setXAccelOffset(ACCEL_OFFSET_X);
      sensor.setYAccelOffset(ACCEL_OFFSET_Y);
      sensor.setZAccelOffset(ACCEL_OFFSET_Z);
      sensor.setXGyroOffset(GYRO_OFFSET_X);
      sensor.setYGyroOffset(GYRO_OFFSET_Y);
      sensor.setZGyroOffset(GYRO_OFFSET_Z);
#     endif // CALIBRATE

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

#     ifdef LED_PIN
      digitalWrite( LED_PIN, LED_OFF);
#     endif // LED_PIN

      // wait a bit before retrying
      delay( 800);
    }

  
  return value;
}



/*
 *  The setup function that gets called once at startup
 */
void setup()
{
  bool       success;

  
# ifdef LED_PIN
  pinMode( LED_PIN, OUTPUT);
# endif // LED_PIN

  setup_serial();

  SerialCons.print( "Compiled with Arduino IDE version: ");
  SerialCons.println( ARDUINO);

  success= false;
  while( ! success )
  {
#   ifdef LED_PIN
    digitalWrite( LED_PIN, LED_ON);
#   endif // LED_PIN
#   ifdef VCC_PIN    // sensor VCC pin hooked to IO pin?  powercycle it!
    pinMode( VCC_PIN, OUTPUT);
    digitalWrite( VCC_PIN, LOW);
    delay( 100);
    digitalWrite( VCC_PIN, HIGH);
    delay( 100);
#   endif // VCC_PIN

    setup_wire();
    success=setup_sensor();
  }

  SerialCons.println( "The device has been started!");

#   ifdef LED_PIN
    digitalWrite( LED_PIN, LED_OFF);
#   endif // LED_PIN

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
  uint8_t            sensorIntStatus;   // holds actual interrupt status byte from sensor
  static uint16_t    fifoCount;         // count of all bytes currently in FIFO
  static uint8_t     fifoBuffer[64];    // FIFO storage buffer
  
  static uint8_t     counter = 0;
  Quaternion         q;                 // [w, x, y, z]         quaternion container
  VectorInt16        aa;                // [x, y, z]            accel sensor measurements


  // wait for sensor interrupt or extra packet(s) available
  if( ! sensorInterrupt && fifoCount < packetSize )
    return;

  // get current FIFO count
  fifoCount= sensor.getFIFOCount();

  if( fifoCount < packetSize )
    return;

  // reset interrupt flag 
  sensorInterrupt= false;

  // get INT_STATUS byte
  sensorIntStatus= sensor.getIntStatus();

  // check for overflow (this should never happen unless our code is too inefficient)

  if( (sensorIntStatus & 0x10) || fifoCount == 1024 )
  {
#   ifdef LED_PIN
    digitalWrite( LED_PIN, LED_ON);
#   endif // LED_PIN

    // reset so we can continue cleanly
    sensor.resetFIFO();
    SerialCons.println( F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if( sensorIntStatus & 0x02 )
  {
#   ifdef LED_PIN
    digitalWrite( LED_PIN, LED_OFF);
#   endif // LED_PIN

    // process all packets currently in the FIFO
    while( fifoCount >= packetSize )
    {
  
      // read a packet from FIFO
      sensor.getFIFOBytes( fifoBuffer, packetSize);
  
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount-= packetSize;

    }

    sensor.dmpGetQuaternion( &q, fifoBuffer);


    // report result
    output_relativty( q);


    // about every second?
    counter %= 100;
    if( ! counter++ )
    {
      sensor.dmpGetAccel( &aa, fifoBuffer);

      // display values in readable form
      output_readable( q, aa);
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

      
  sensor.dmpGetEuler(euler, &q);

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

  sensor.dmpGetGravity(&gravity, &q);
  sensor.dmpGetYawPitchRoll(ypr, &q, &gravity);

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


  sensor.dmpGetGravity(&gravity, &q);
  sensor.dmpGetLinearAccel(&aaReal, &aa, &gravity);

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


  sensor.dmpGetGravity(&gravity, &q);
  sensor.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  sensor.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

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

# ifdef OUTPUT_READABLE_QUATERNION
  output_quarernion( q);
# endif // OUTPUT_READABLE_QUATERNION
# ifdef OUTPUT_READABLE_EULER
  output_euler( q);
# endif // OUTPUT_READABLE_EULER
# ifdef OUTPUT_READABLE_YAWPITCHROLL
  output_yawPitchRoll( q);
# endif
# ifdef OUTPUT_READABLE_REALACCEL
  output_realAccel( q, aa);
# endif // OUTPUT_READABLE_REALACCEL
# ifdef OUTPUT_READABLE_WORLDACCEL
  output_worldAccel( q, aa);
# endif // OUTPUT_READABLE_WORLDACCEL
# if defined( OUTPUT_READABLE_QUATERNION ) \
  || defined( OUTPUT_READABLE_EULER ) \
  || defined( OUTPUT_READABLE_YAWPITCHROLL ) \
  || defined( OUTPUT_READABLE_REALACCEL ) \
  || defined( OUTPUT_READABLE_WORLDACCEL )
  SerialCons.println();
#endif // OUTPUT_READABLE_*
  
}
