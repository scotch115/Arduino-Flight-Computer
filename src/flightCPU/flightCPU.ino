#include <Servo.h>
#include "Wire.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <RH_RF95.h>
#include <avr/dtostrf.h>
#include <SimpleKalmanFilter.h>


/*************************************************************************************************
 ******************************************** Globals ********************************************
 *************************************************************************************************/

// LoRa Pinouts
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Radio communication frequency
#define RF95_FREQ 915.0

// LoRa Radio Module instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Barometric Altimeter instance
Adafruit_BME280 bme; // I2C

// Inertial Measurement Unit (IMU) instance
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 6

// Current Atmospheric Pressure (UPDATE BEFORE FLIGHT)
#define SEALEVELPRESSURE_HPA (1022.6)

// Battery Voltage pinout (for collecting battery status)
#define VBATPIN A7

// LED Pinout
#define LEDPIN A3

int altitude, lastAlt, apogee;

// Buzzer Sounds
unsigned int numTones = 6;
unsigned int tones[] = {523, 587, 659, 739, 830, 880};
/************* upper    C    D    E    F#   G#   A   */

/************************
 *** Servo Definitions ***
 ************************/
int servo0Pin = 10;
int servo1Pin = 11;
Servo servo0;
Servo servo1;

/************************************
 *  Data Payload (String) Variables *
 ************************************/
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


/***************************
 * MPU control/status vars *
 ***************************/
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

/***************************
 * orientation/motion vars *
 ***************************/
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]     Euler angle container
float yrp[3];           // [yaw, roll, pitch]    yaw/roll/pitch container and gravity vector


// Define base position values as zero, to convey the "starting" position
float xPos = 0;
float yPos = 0;
float zPos = 0;


/******************************
 * IMU interruption detection *
 ******************************/
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


/********************** KALMAN FILTERS**********************/
/***************** SimpleKalmanFilter(e_mea, e_est, q); ****/
/***************** e_mea: Measurement Uncertainty **********/
/***************** e_est: Estimation Uncertainty ***********/
/****************** q: Process Noise  **********************/
  SimpleKalmanFilter xKalmanFilter(2, 1, 0.001);
  SimpleKalmanFilter yKalmanFilter(2, 1, 0.001);
  SimpleKalmanFilter zKalmanFilter(2, 1, 0.001);
  SimpleKalmanFilter yawKalmanFilter(1, 1, 0.01);
  SimpleKalmanFilter pitchKalmanFilter(1, 1, 0.01);
  SimpleKalmanFilter rollKalmanFilter(1, 1, 0.01);


  /*************************/
  /* Launch State Handling */
  /*************************/
  String launchState = "OFF";


/*********************************************************************************
 ************************************ Startup ************************************
 *********************************************************************************/

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif

  /**********************************
   *    ? Prep RMF95 for flight ?
   * ! REMOVE WHEN TESTING OFFLINE  !
   **********************************/
  // pinMode(RFM95_RST, OUTPUT);
  // digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(115200);

  while (!Serial) {
    // Wait for serial to be available (only necessary for current stage of software development)
  }

  preCheck();
}

/*************************************************************************
 ******************************* Main loop *******************************
 *************************************************************************/

void loop() {
/**
 * @note - The flight CPU should have the following "commands", receivable via 900mHz radio from the launch pad controller
 * 
 * @param - STARTUP
 * @param - ARMED
 * @param - START CAMERA
 * @param - STOP CAMERA
 * @param - IGNITION / LAUNCH
 * @param - DEPLOY / CHUTE
 * @param - REBOOT
 */
  if (launchState == "ARMED"
    || launchState == "LIFTOFF"
    || launchState == "IDLE"
    || launchState == "FLIGHT") {
      if (!dmpReady) return;
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);


      /**
       * ! (WIP)
       * Rocket camera feed will be handled by raspberry pi zero,
       * using a configuration similar to what is outlined here:
       * https://automaticaddison.com/2-way-communication-between-raspberry-pi-and-arduino/
       * 
       * The only expected obstacle at this time is going to be streaming the video to the ground computer,
       * because I don't think sending video signals over radio is going to be a good idea (or work at all).
       */

      
      /******************************************
       * ! (WIP)
       *  Check rocket has not surpassed apogee * 
       ******************************************/
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

      // --TODO: Fix the yrp mapping due to orientation (MPU is not oriented the same as the CPU - x and y axes are swapped)--
      // Convert Yaw/Roll/Pitch to readable values   
      yrp[0] = yrp[0] * 180/M_PI;
      yrp[1] = yrp[1] * 180/M_PI;
      yrp[2] = yrp[2] * 180/M_PI;

      // Complementary filter - combine acceleromter and gyro angle values
      //  yrp[1] = 0.96 * yrp[1] + 0.04 * aaReal.y;
      //  yrp[2] = 0.96 * yrp[2] + 0.04 * aaReal.x;
      yrp[0] = yawKalmanFilter.updateEstimate(yrp[0]);
      yrp[1] = pitchKalmanFilter.updateEstimate(yrp[1]);
      yrp[2] = rollKalmanFilter.updateEstimate(yrp[2]);

      if (launchState == "LIFTOFF" || launchState == "FLIGHT") {
        // Map the values of the MPU6050 sensor from -90 to 90 to values suitable for the servo control from 0 to 180
        int servo0Value = map(yrp[1], -90, 90, 0, 180);
        int servo1Value = map(yrp[1], -90, 90, 0, 180);
        
        // Control the servos according to the MPU6050 orientation
        servo0.write(servo0Value);
        servo1.write(servo1Value);
      }

    /***************************************
     *    ? Get current battery level ?
     * ! REMOVE WHEN BATTERY IS UNPLUGGED  !
     **************************************/
    // // Get current battery level
    // float measuredvbat = analogRead(VBATPIN);
    // measuredvbat *= 2;
    // measuredvbat *= 3.3;
    // measuredvbat /= 1024;

    /**********************************
     * Convert collected data to JSON *
     **********************************/
    String Payload = "\"{";
    Payload += "\"Temperature\":";
    Payload += bme.readTemperature();
    Payload += ",""\"Pressure\":";
    Payload += (bme.readPressure() / 100.0F);
    Payload += ",""\"Altitude\":";
    /**
     * TODO
     * Attempting a multi-data kalman filter to reduce altitude bias, but will need to remap x/y values when IMU
     * is installed on custom PCB due to change in board mount orientation
     */
    Payload += calculateAltitude(bme.readAltitude(SEALEVELPRESSURE_HPA), aaReal.y);
    Payload += ",""\"Humidity\":";
    Payload += bme.readHumidity();
    Payload += ",""\"Yaw\":";
    Payload += yrp[0];
    Payload += ",""\"Pitch\":";
    Payload += yrp[2];
    Payload += ",""\"Roll\":";
    Payload += yrp[1];
    Payload += ",""\"X\":";
    Payload += calculateXDisplacement(aaReal.x);
    Payload += ",""\"Y\":";
    Payload += calculateYDisplacement(aaReal.y);
    Payload += ",""\"Z\":";
    Payload += calculateZDisplacement(aaReal.z);

    //  ! (WIP - Battery Status)
    // Payload += ",""\"Battery Voltage\":";
    // Payload += measuredvbat;

    /* ! (WIP - Servo Angle of Attack tracking) */
    // Payload += ",""\"Servo0 AoA\":";
    // Payload += servo0Value;
    // Payload += ",""\"Servo1 AoA\":";
    // Payload += servo1Value;
    //  Payload += "}";
      
    char packet[200];
    int i = 0;
    int sizeOf = 200;
    int offset = 0;
    
    while((i<sizeOf))
    {
        packet[i]=Payload[i];  
        i++;
    }
    
    /*********************************
     * ! FOR OFFLINE TESTING ONLY !  *
     * !! REMOVE WHEN USING RF !!    *
     *********************************/
    Serial.println(packet);

    /*********************************
     *  ? LoRa Payload transmission ?
     * ! REMOVE WHEN TESTING OFFLINE !
      *******************************/
    // rf95.send((uint8_t *)packet, 200);
    // delay(10);
    // rf95.waitPacketSent();
    } 
  } else if (launchState == "REBOOT") {
    preCheck();
  } else if (launchState == "STANDBY"
    || launchState == "ERROR"
    || launchState =="STARTUP") {
    return;
  }
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

// XYZ Position Calculations
float calculateXDisplacement(float accelX) {
  xPos = xKalmanFilter.updateEstimate(accelX);
//  xPos = calculateDistance(xPos, accelX, millis());
  return xPos;
}

float calculateYDisplacement(float accelY) {
  yPos = yKalmanFilter.updateEstimate(accelY);
//  yPos = calculateDistance(yPos, accelY, millis());
  return yPos;
}

float calculateZDisplacement(float accelZ) {
  zPos = zKalmanFilter.updateEstimate(accelZ);
//  zPos = calculateDistance(zPos, accelZ, millis());
  return zPos;
}

float calculateAltitude(float alt, float accel) {
  // Create a new kalman filter
  SimpleKalmanFilter altitudeKalmanFilter(1, 1, 0.01);
  // Set altitude to accelerometer 
  //  float altitude = bme.seaLevelForAltitude(accel, (bme.readPressure() / 100.0F));
  float altitude = altitudeKalmanFilter.updateEstimate(alt);
  return altitude;
}


float calculateDistance(float v, float accel, float t) {
  float velocity = v + accel * t;
  float distance = velocity * t + 0.5 * accel * pow(t, 2);
  return distance;
}

/***********************************************************************************************/
/********************************* Pre-Flight Startup Sequence *********************************/
/***********************************************************************************************/
void preCheck() {
  Serial.println();Serial.println();
  Serial.println("Beginning Pre-Flight Software check.");
  Serial.println();Serial.println();
  delay(2000);

  launchState = "STARTUP";
  Serial.println();
  Serial.print("Flight Computer Status: ");Serial.println(launchState);

  pinMode(LEDPIN, OUTPUT);

  // Play bootup sequence sound
  for (unsigned int i = 0; i < numTones; i++) {
    tone(A1, tones[i]);
    delay(50);
  }
  noTone(A1);
  delay(2000);  // time to get serial running
  unsigned status;

  /**********************************
   *     ? Boot up LoRa module ?
   * ! REMOVE WHEN TESTING OFFLINE  !
   **********************************/
  // Serial.println("Feather LoRa TX Test!");

  // // Reset LoRa Module
  // digitalWrite(RFM95_RST, LOW);
  // delay(10);
  // digitalWrite(RFM95_RST, HIGH);
  // delay(10);

  // while (!rf95.init()) {
  //   Serial.println("LoRa radio init failed");
  //   Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
  //   while (1);
  // }
  // Serial.println("LoRa radio init OK!");

  // if (!rf95.setFrequency(RF95_FREQ)) {
  //   Serial.println("setFrequency failed");
  //   while (1);
  // }
  // Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // rf95.setTxPower(23, false);

  Serial.println("First let's see if the BME280 Barometer is connected. Standby...");
 

  // default settings
  status = bme.begin();
//  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    while (1) delay(10);
  } else {
    if (status) {
      String bmeMsg = "Found the BME280 sensor! Here's some data...";
      char bMessage[200];
      int b = 0;
      while((b<200)) {
        bMessage[b]=bmeMsg[b];  
        b++;
      }

      /*********************************
       * ! FOR OFFLINE TESTING ONLY !  *
       * !! REMOVE WHEN USING RF !!    *
       *********************************/
      Serial.println(bMessage);
      delay(2000);
      tone(A1, 523);
      digitalWrite(LEDPIN, HIGH);
      delay(50);
      noTone(A1);
      digitalWrite(LEDPIN, LOW);
      Serial.print("Temperature = ");Serial.print(bme.readTemperature());Serial.println(" *C");
      Serial.print("Pressure = ");Serial.print((bme.readPressure() / 100.0F));Serial.println(" Pa");
      Serial.print("Approx Altitude = ");Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));Serial.println(" m");
      Serial.print("Humidity = ");Serial.print(bme.readHumidity());Serial.println(" %");
      Serial.println();Serial.println();

      /**********************************
       * ? Send status update over LoRa ?
       * ! REMOVE WHEN TESTING OFFLINE  !
       **********************************/
      // rf95.send((uint8_t *)bMessage, 200);
      // delay(10);
      // rf95.waitPacketSent();
    }
  }

  Serial.println("Now we'll look for the MPU6050 IMU. Standby...");
  
  delay(2000);
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  String mpuMsg = "Found it! MPU6050 Connection Successful.";
  char mMessage[200];
  int m = 0;
  while((m<200)) {
    mMessage[m]=mpuMsg[m];  
    m++;
  }

  /*********************************
   * ! FOR OFFLINE TESTING ONLY !  *
   * !! REMOVE WHEN USING RF !!    *
   *********************************/
  Serial.println(mMessage);

  /**********************************
   * ? Send status update over LoRa ?
   * ! REMOVE WHEN TESTING OFFLINE  !
   **********************************/
  // rf95.send((uint8_t *)mMessage, 200);
  // delay(10);
  // rf95.waitPacketSent();

  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
  /*******************************
   * ! May require recalibration *
   *******************************/
    mpu.setXAccelOffset(-613);
    mpu.setYAccelOffset(-3312);
    mpu.setZAccelOffset(1793);
    mpu.setXGyroOffset(54);
    mpu.setYGyroOffset(86);
    mpu.setZGyroOffset(39);

  if (devStatus == 0) {
  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling Digital Motion Processor (DMP). Standby..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP Calibration complete!"));
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

  delay(2000);
  tone(A1, 523);
  digitalWrite(LEDPIN, HIGH);
  delay(50);
  noTone(A1);
  digitalWrite(LEDPIN, LOW);
  Serial.println();Serial.println();
  Serial.println("Here's a little bit of data!");
  for (int i = 0; i < 15; i++) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    }
    delay(100);
    Serial.print("a/g:\t");
    Serial.print(aaReal.x); Serial.print("\t");
    Serial.print(aaReal.y); Serial.print("\t");
    Serial.print(aaReal.z); Serial.print("\t");
    Serial.print(yrp[0]); Serial.print("\t");
    Serial.print(yrp[1]); Serial.print("\t");
    Serial.println(yrp[2]);
  }
  Serial.println();Serial.println();

  delay(2000);
  Serial.println("Now let's test the fin control! Connecting to servos, standby...");
  Serial.println();Serial.println();

  servo0.attach(servo0Pin);
  servo1.attach(servo1Pin);
  
  // Test servo functionality
  servo0.write(180);
  servo1.write(-180);
  delay(1000);
  servo0.write(-180);
  servo1.write(180);
  delay(1000);
  servo0.write(90);
  servo1.write(90);
  delay(1000);


  digitalWrite(LEDPIN, HIGH);
  analogWrite(A0, 150);
  delay(50);
  digitalWrite(LEDPIN, LOW);
  noTone(A1);

  String readyMsg = "Flight Computer is Armed and Ready!";
  char rMessage[200];
  int r = 0;
  while((r<200)) {
    rMessage[r]=readyMsg[r];  
    r++;
  }

  delay(1000);
  launchState = "ARMED";
  Serial.println();
  Serial.print("Flight Computer Status: ");Serial.println(launchState);

  /*********************************
   * ! FOR OFFLINE TESTING ONLY !  *
   * !! REMOVE WHEN USING RF !!    *
   *********************************/
  Serial.println();Serial.println();
  Serial.println(rMessage);
  Serial.println();Serial.println();

  /**********************************
   * ? Send status update over LoRa ?
   * ! REMOVE WHEN TESTING OFFLINE  !
   **********************************/
  // rf95.send((uint8_t *)rMessage, 200);
  // delay(10);
  // rf95.waitPacketSent();

  tone(A1, 1760);
  digitalWrite(LEDPIN, HIGH);
  delay(500);
  noTone(A1);

  delay(1000);
  
  //  delay(1000);
  //  analogWrite(A0, 150);
  
  // Relay
  //  pinMode(A2, OUTPUT);
  //  digitalWrite(A2, LOW);

  // readyForLaunch();
}


/************************************************************************/
/************************ CPU Armed Hype Song ;) ************************/
/************************************************************************/
#define NOTE_B4  494
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_FS5 740
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_B5  988
#define REST      0


int tempo = 169;
int buzzer = 15;

int melody[] = {
  NOTE_FS5,8, NOTE_FS5,8,NOTE_D5,8, NOTE_B4,8, REST,8, NOTE_B4,8, REST,8, NOTE_E5,8, 
  REST,8, NOTE_E5,8, REST,8, NOTE_E5,8, NOTE_GS5,8, NOTE_GS5,8, NOTE_A5,8, NOTE_B5,8,
  NOTE_A5,8, NOTE_A5,8, NOTE_A5,8, NOTE_E5,8, REST,8, NOTE_D5,8, REST,8, NOTE_FS5,8, 
  REST,8, NOTE_FS5,8, REST,8, NOTE_FS5,8, NOTE_E5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,8,

  NOTE_FS5,8, NOTE_FS5,8,NOTE_D5,8, NOTE_B4,8, REST,8, NOTE_B4,8, REST,8, NOTE_E5,8, 
  REST,8, NOTE_E5,8, REST,8, NOTE_E5,8, NOTE_GS5,8, NOTE_GS5,8, NOTE_A5,8, NOTE_B5,8,
  NOTE_A5,8, NOTE_A5,8, NOTE_A5,8, NOTE_E5,8, REST,8, NOTE_D5,8, REST,8, NOTE_FS5,8, 
  REST,8, NOTE_FS5,8, REST,8, NOTE_FS5,8, NOTE_E5,8, NOTE_E5,8, NOTE_FS5,8, NOTE_E5,8,
};

int notes = sizeof(melody) / sizeof(melody[0]) / 2;

int wholenote = (60000 * 4) / 169;

int divider = 0, noteDuration = 0;

void readyForLaunch() {
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    divider = melody[thisNote + 1];
    if (divider > 0) {
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      noteDuration = (wholenote) / abs(divider);
    }
    tone(buzzer, melody[thisNote], noteDuration * 0.9);
    delay(noteDuration);
    noTone(buzzer);
  }
}
