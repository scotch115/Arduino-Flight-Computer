#include <Servo.h>
#include "Wire.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <RH_RF95.h>
#include <avr/dtostrf.h>


// ------------ Globals

//LoRa
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 6

#define SEALEVELPRESSURE_HPA (1022.6)

Adafruit_BME280 bme; // I2C

int altitude, lastAlt, apogee;

// Buzzer
unsigned int numTones = 6;
unsigned int tones[] = {523, 587, 659, 739, 830, 880};
////            upper    C    D    E    F#   G#   A


// --------------------- Servo Definitions
int servo0Pin = 10;
int servo1Pin = 11;
Servo servo0;
Servo servo1;

// Text
boolean chutesFired = false;
boolean landed = false;

char packetBuffer[100];
char flightData[60] = "{""\"status\":""\"Flight CPU sent data to server\"""}";
char chutes[50] = "{""\"status\":""\"Chutes Deployed!\"""}";
char liftoff[50] = "{""\"status\":""\"Liftoff!\"""}";
char bmeDetected[60] = "{""\"status\":""\"BME 280 Detected.\"""}";
char mpuDetected[60] = "{""\"status\":""\"MPU 6050 Detected.\"""}";
char startupCompleted[60] = "{""\"status\":""\"Startup process complete.\"""}";
char flightCPU[50] = "{""\"status\":""\"Flight CPU ready\"""}";
char vehicleLanded[50] = "{""\"status\":""\"Touchdown!\"""}";



// ------------------------ MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// ------------------------ orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ---------------------- IMU interruption detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ------------ Setup

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(115200);
  for (unsigned int i = 0; i < numTones; i++)
  {
    tone(A1, tones[i]);
    delay(50);
  }
  noTone(A1);
  delay(2000);  // time to get serial running
  unsigned status;

  Serial.println("Feather LoRa TX Test!");

  // Reset LoRa Module
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

   rf95.setTxPower(23, false);
 

  // default settings
  status = bme.begin();
//  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1) delay(10);
  } else {
    if (status) {
      String bmeMsg = "BME280 Initialized";
      char bMessage[200];
      int b = 0;
      while((b<200))
      {
         bMessage[b]=bmeMsg[b];  
         b++;
      }
      rf95.send((uint8_t *)bMessage, 200);
      delay(10);
      rf95.waitPacketSent();
      tone(A1, 523);
      delay(50);
      noTone(A1);
      delay(1000);
    }
  }
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  String mpuMsg = "MPU6050 Initialized";
  char mMessage[200];
  int m = 0;
  while((m<200))
    {
       mMessage[m]=mpuMsg[m];  
       m++;
    }
  rf95.send((uint8_t *)mMessage, 200);
  delay(10);
  rf95.waitPacketSent();
  tone(A1, 523);
  delay(50);
  noTone(A1);
  delay(1000);

  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-578);
    mpu.setYAccelOffset(-3277);
    mpu.setZAccelOffset(1804);
    mpu.setXGyroOffset(62);
    mpu.setYGyroOffset(105);
    mpu.setZGyroOffset(39);

  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    servo0.attach(servo0Pin);
    servo1.attach(servo1Pin);
  
    analogWrite(A0, 150);
    delay(200);
  
    String readyMsg = "Flight Computer Primed and Ready";
    char rMessage[200];
    int r = 0;
    while((r<200))
    {
       rMessage[r]=readyMsg[r];  
       r++;
    }
    rf95.send((uint8_t *)rMessage, 200);
    delay(10);
    rf95.waitPacketSent();
  
    tone(A1, 1760);
    delay(500);
    noTone(A1);
  
  //  delay(1000);
  //  analogWrite(A0, 150);
  
  // Relay
  //  pinMode(A2, OUTPUT);
  //  digitalWrite(A2, LOW);  
}

// ------------ Main loop

void loop() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  }

  
//  // Check rocket has not surpassed apogee 
//  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
//  if (altitude - lastAlt <= -1) {
//    delay(100);
//    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
//    if (altitude - lastAlt <= -2) {
//      delay(100);
//      altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
//      if (altitude - lastAlt <= -3) {
//        apogee = lastAlt - 3;
//        delay(150);
//        deploy();
//      } else {
//      lastAlt = altitude;
//      }
//    } else {
//      lastAlt = altitude;
//      } 
//   } else {
//    lastAlt = altitude;
//   }


//   if (chutesFired == true) {
//    // BEGIN DESCENT // 
//        analogWrite(A0,150);
//        delay(100);
//        analogWrite(A0, 0);
//        delay(100);
//        analogWrite(A0,150);
//        delay(100);
//        analogWrite(A0, 0);
//        delay(100);
//        touchdown();
//   }

   
    ypr[0] = ypr[0] * 180/M_PI;
    ypr[1] = ypr[1] * 180/M_PI;
    ypr[2] = ypr[2] * 180/M_PI;

    // Complementary filter - combine acceleromter and gyro angle values
    ypr[1] = 0.96 * ypr[1] + 0.04 * aaReal.y;
    ypr[2] = 0.96 * ypr[2] + 0.04 * aaReal.x;

    // Map the values of the MPU6050 sensor from -90 to 90 to values suitable for the servo control from 0 to 180
    int servo0Value = map(ypr[1], -90, 90, 0, 180);
    int servo1Value = map(ypr[1], -90, 90, 0, 180);
    
    // Control the servos according to the MPU6050 orientation
    servo0.write(servo0Value);
    servo1.write(servo1Value);

    // ------------ Convert collected data to JSON
    String Payload = "{""\"Temperature\":";
    Payload += bme.readTemperature();
    Payload += ",""\"Pressure\":";
    Payload += (bme.readPressure() / 100.0F);
    Payload += ",""\"Altitude\":";
    Payload += bme.readAltitude(SEALEVELPRESSURE_HPA);
    Payload += ",""\"Humidity\":";
    Payload += bme.readHumidity();
    Payload += ",""\"Yaw\":";
    Payload += ypr[0];
    Payload += ",""\"Pitch\":";
    Payload += ypr[1];
    Payload += ",""\"Roll\":";
    Payload += ypr[2];
    Payload += ",""\"X\":";
    Payload += aaReal.x;
    Payload += ",""\"Y\":";
    Payload += aaReal.y;
    Payload += ",""\"Z\":";
    Payload += aaReal.x;
    Payload += "}";
    
    char packet[200];
    int i = 0;
    int sizeOf = 200;
    int offset = 0;
    
    while((i<sizeOf))
    {
       packet[i]=Payload[i];  
       i++;
    }
    
    Serial.println(packet);
    // LoRa Packet transmission
    rf95.send((uint8_t *)packet, 200);
    delay(10);
    rf95.waitPacketSent();
}


//void deploy() {
//  chutesFired = true;
//  // Fire relay
//  digitalWrite(A2, HIGH);
//  delay(1000);
//  digitalWrite(A2, LOW);
//  delay(100);
//}
//
//void touchdown() {
//  if (altitude - lastAlt == 0) {
//    delay(100);
//    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
//    if (altitude - lastAlt == 0) {
//      delay(100);
//      altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
//      if (altitude - lastAlt == 0) {
//        landed = true;
//        delay(150);
//      } else {
//        lastAlt = altitude;
//      }
//    } else {
//      lastAlt = altitude;
//    }
//   } else {
//    lastAlt = altitude;
//   }
//}
