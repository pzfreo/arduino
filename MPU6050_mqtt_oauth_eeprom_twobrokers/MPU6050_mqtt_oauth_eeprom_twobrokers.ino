
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

//#include <avr/eeprom.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;

#include <avr/eeprom.h>

struct settings_t
{
  char refresh[40];
} settings;


//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


char clientid[] = "ard0123456";
int reconnect=0;
boolean publish = false;

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

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { 
//  '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// Update these with values suitable for your network.
byte mac[]    = {  
  0x40, 0x6c, 0x8f, 0x5a, 0xa4, 0x70 };
byte server[] = { 10,254,254,2 };
byte ip[]     = { 10,254,254,3 };


char bearer[] = "012345678901234567890123456789012345";
void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  memcpy(bearer, payload, 35);
  reconnect=1;
}

EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);
PubSubClient refreshclient(server, 1884, callback, ethClient);
char dpb[15]; // somewhere to store strings from doubles before outputting
char y[15];
char p[15];
char r[15];

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));  

  Ethernet.begin(mac, ip);
  if (client.connect(clientid, bearer, "d")) {
    publish = 1;
    setup_i2c();
    // bearer is still valid
  }
  else 
  {
    if (refreshclient.connect(clientid, "r", "r")) {
      char topic[15];
      strcpy(topic, "/c/");
      strcat(topic,clientid);
      refreshclient.subscribe(topic);

      // publish two strings (255 delimited) - first client id, second refresh token
      char message[80];
      strcpy(message,clientid);
      strcat(message, "\xFF");
      strcat(message, settings.refresh);
      refreshclient.publish("/r", message);
    }
  }
}

void setup_i2c() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  mpu.testConnection();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
//  mpu.setXGyroOffset(220);
//  mpu.setYGyroOffset(76);
//  mpu.setZGyroOffset(-85);
//  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  if (!publish) {
    if (reconnect==2) {
      delay(40000);
      if (client.connect(clientid, bearer, "dummy")) {
        publish = true;
        setup_i2c();
        reconnect=0;
      }
    } 
    else
      if (reconnect==1) {
        reconnect=2;
        refreshclient.disconnect();
        delay(500);
      }
    refreshclient.loop(); // wait for new bearer token
    return;
  }



  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
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
    delay(50);
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

//#ifdef OUTPUT_READABLE_QUATERNION
//    // display quaternion values in easy matrix form: w x y z
//    mpu.dmpGetQuaternion(&q, fifoBuffer);
//    String quat = "quat\t"+q.w+"\t"+q.x+"\t"+q.y+"\t"+q.z;
//    client.publish("/pzf/quat",quat);
//#endif

    //        #ifdef OUTPUT_READABLE_EULER
    //            // display Euler angles in degrees
    //            mpu.dmpGetQuaternion(&q, fifoBuffer);
    //            mpu.dmpGetEuler(euler, &q);
    //            Serial.print("euler\t");
    //            Serial.print(euler[0] * 180/M_PI);
    //            Serial.print("\t");
    //            Serial.print(euler[1] * 180/M_PI);
    //            Serial.print("\t");
    //            Serial.println(euler[2] * 180/M_PI);
    //        #endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    dtostrf(ypr[0] * 180/M_PI, 10, 10, y);
    dtostrf(ypr[1] * 180/M_PI, 10, 10, p);
    dtostrf(ypr[2] * 180/M_PI, 10, 10, r);
    char buff[70];
    sprintf(buff, "ypr\t%s\t%s\t%s",y,p,r);
    client.publish("/pzf/ypr", buff);

    //            Serial.print("ypr\t");
    //           Serial.print(ypr[0] * 180/M_PI);
    //           Serial.print("\t");
    //           Serial.print(ypr[1] * 180/M_PI);
    //           Serial.print("\t");
    //           Serial.println(ypr[2] * 180/M_PI);
#endif

      //        #ifdef OUTPUT_READABLE_REALACCEL
    //            // display real acceleration, adjusted to remove gravity
    //            mpu.dmpGetQuaternion(&q, fifoBuffer);
    //            mpu.dmpGetAccel(&aa, fifoBuffer);
    //            mpu.dmpGetGravity(&gravity, &q);
    //            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //            Serial.print("areal\t");
    //            Serial.print(aaReal.x);
    //            Serial.print("\t");
    //            Serial.print(aaReal.y);
    //            Serial.print("\t");
    //            Serial.println(aaReal.z);
    //        #endif
    //
    //        #ifdef OUTPUT_READABLE_WORLDACCEL
    //            // display initial world-frame acceleration, adjusted to remove gravity
    //            // and rotated based on known orientation from quaternion
    //            mpu.dmpGetQuaternion(&q, fifoBuffer);
    //            mpu.dmpGetAccel(&aa, fifoBuffer);
    //            mpu.dmpGetGravity(&gravity, &q);
    //            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    //            Serial.print("aworld\t");
    //            Serial.print(aaWorld.x);
    //            Serial.print("\t");
    //            Serial.print(aaWorld.y);
    //            Serial.print("\t");
    //            Serial.println(aaWorld.z);
    //        #endif
    //    
    //        #ifdef OUTPUT_TEAPOT
    //            // display quaternion values in InvenSense Teapot demo format:
    //            teapotPacket[2] = fifoBuffer[0];
    //            teapotPacket[3] = fifoBuffer[1];
    //            teapotPacket[4] = fifoBuffer[4];
    //            teapotPacket[5] = fifoBuffer[5];
    //            teapotPacket[6] = fifoBuffer[8];
    //            teapotPacket[7] = fifoBuffer[9];
    //            teapotPacket[8] = fifoBuffer[12];
    //            teapotPacket[9] = fifoBuffer[13];
    //            Serial.write(teapotPacket, 14);
    //            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
    //        #endif

      // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

  }
}




