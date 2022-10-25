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
#include "variables.h"
#include "flightConfiguration.h"

/****************************************************************************************************************************************************************************/
/****************************************************************************************************************************************************************************/
/****************************************************************************************** Startup *************************************************************************/
/****************************************************************************************************************************************************************************/
/****************************************************************************************************************************************************************************/

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif

  /*   Prep RMF95 for flight   */
  if (TELEMETRY == "ACTIVE") {
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
  }
  
  Serial.begin(115200);

  // Wait for serial to be available if board is in 'DEBUG' mode
  if (debugState == "ON") {
    while (!Serial) {
    }
  }

  // Run pre-flight check
  preCheck();
}

/*****************************************************************************************************************************************************************************/
/*****************************************************************************************************************************************************************************/
/************************************************************************* flightControl loop *********************************************************************************/
/*****************************************************************************************************************************************************************************/
/*****************************************************************************************************************************************************************************/

void loop() {
/**
 * @note - The flight CPU should have the following states, some 
 * of which are receivable via 900mHz radio from the launch pad controller
 * 
 * @param - STARTUP
 * @param - STANDBY
 * @param - IDLE
 * @param - ARMED
 * @param - START CAMERA
 * @param - STOP CAMERA
 * @param - LIFTOFF
 * @param - CHUTE
 * @param - ERROR
 * @param - REBOOT
 * @param - TOUCHDOWN
 * @param - DEBUG
 */
  if (launchState == "ARMED"
  || launchState == "STANDBY") {
    missionBlep(interval, 1200);
    flightControl();
    if (TELEMETRY == "ACTIVE") {
      // Check to see if groundControl is available
      sync();
    }
    // Reduce power consumption of loop frequency by limiting data collection to 2Hz
    delay(500);
   } else if (launchState == "IDLE") {
    missionBlep(interval, 1000);
    flightControl();
    if (TELEMETRY == "ACTIVE") {
      // Check to see if groundControl is available
      sync();
    }
    // If flight computer is in 'IDLE' mode, lower to 0.3Hz to further improve battery performance
    delay(3000);
   } else if (launchState == "LIFTOFF"
   || launchState == "FLIGHT"
   || launchState == "CHUTE") {
    /**
     * Blep once every second to indicate that the flight computer is armed and ready
     */
    unsigned long missionTime = millis();
    // Blep twice as in flight to indicate a change in state
    missionBlep((interval / 2), 1200);
    flightControl();
  } else if (launchState == "REBOOT") {
    preCheck();
  } else if (launchState =="STARTUP") {
    return;
  } else if (launchState == "ERROR") {
    /**
     * Crash the program and force engineer(s) to follow standard operating procedures
     * and power down rocket for safety
     * */
    Serial.println("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR");
    Serial.println("!!!!! POWER DOWN THE ROCKET AND BE SURE TO DISENGAGE THE MOTOR !!!!!");
    Serial.println("!!!!! EVALUATE FLIGHT COMPUTER AND CIRCUITRY BEFORE CONTINUING !!!!!");
    Serial.println("!!!!! ENSURE CONTINUITY ON ALL COMPONENTS BEFORE ATTEMPTING FLIGHT !!!!!");
    Serial.println("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR");
    blep(BUZZER_PIN, 311, 1000);

    delay(10000);
  } else if (launchState == "TOUCHDOWN") {
    /* Play a deep, drone note */
    tone(BUZZER_PIN, 300);
    /* Beep periodically to help recovery crew find vehicle */
    blep(BUZZER_PIN, 523, 100);
  } else if (launchState == "DEBUG") {
    /* Do not limit data stream for efficient debugging */
    missionBlep(interval, 1200);
    flightControl();
  }
}

/******************************************************************/
/****************** Main In-Flight Control Logic ******************/
/******************************************************************/
void flightControl() {
  if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

      /**
       * (WIP) *
       * Rocket camera feed will be handled by raspberry pi zero,
       * using a configuration similar to what is outlined here:
       * https://automaticaddison.com/2-way-communication-between-raspberry-pi-and-arduino/
       * 
       * The only expected obstacle at this time is going to be streaming the video to the ground computer,
       * because I don't think sending video signals over radio is going to be a good idea (or work at all).
       */

      
      /*************************************************
       * (WIP) *
       *  Check if rocket has reached or passed apogee * 
       *************************************************/
      getVehicleAltitude();

      if (chutesFired) {
        touchdown();
      }

      /******************************************************
       * Calculate Reaction Control offsets using IMU data, *
       * and pass values to servos to correct flight         *
       ******************************************************/
      if (RCS) {
        rcs(ypr);
      }

      /***************************************
       *    ? Get current battery level ?
       * ! REMOVE WHEN BATTERY IS UNPLUGGED  !
       **************************************/
      // Get current battery level
      // float measuredvbat = analogRead(VBATPIN);
      // measuredvbat *= 2;
      // measuredvbat *= 3.3;
      // measuredvbat /= 1024;

      transmitData();
    } 
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
    tone(BUZZER_PIN, tones[i]);
    delay(50);
  }
  noTone(BUZZER_PIN);
  delay(2000);  // time to get serial running
  unsigned status;

  /**********************************
   *     ? Boot up LoRa module ?
   * ! REMOVE WHEN TESTING OFFLINE  !
   **********************************/
  if (TELEMETRY == "ACTIVE") {
    Serial.println();
    Serial.println("LoRa Radio module found!");

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
    Serial.println("LoRa radio initialized!");

    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("setFrequency failed");
      while (1);
    }
    Serial.print("Frequency set to: "); Serial.println(RF95_FREQ);

    rf95.setTxPower(23, false);
  }

  Serial.println("First let's see if the BME280 Barometer is connected. Standby...");
 

  // Initialize and test BME 280
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    launchState = "ERROR";
  } else {
    if (status) {
      String bmeMsg = "Found the BME280 sensor! Here's some data...";
      char bMessage[200];
      int b = 0;
      while((b<200)) {
        bMessage[b]=bmeMsg[b];  
        b++;
      }

      Serial.println(bMessage);
      delay(2000);

      blep(BUZZER_PIN, 523, 50);

      Serial.print("Temperature = ");Serial.print(bme.readTemperature());Serial.println(" *C");
      Serial.print("Pressure = ");Serial.print((bme.readPressure() / 100.0F));Serial.println(" Pa");
      Serial.print("Approx Altitude = ");Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));Serial.println(" m");
      Serial.print("Humidity = ");Serial.print(bme.readHumidity());Serial.println(" %");
      Serial.println();Serial.println();

      /**********************************
       * ? Send status update over LoRa ?
       * ! REMOVE WHEN TESTING OFFLINE  !
       **********************************/
      if (TELEMETRY == "ACTIVE") {
        rf95.send((uint8_t *)bMessage, 200);
        delay(10);
        rf95.waitPacketSent();
      }
    }
  }

  Serial.println("Now we'll look for the MPU6050 IMU. Standby...");
  
  delay(2000);
  
  // Initialize and test IMU (MPU 6050)
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  String mpuMsg = "Found it! MPU6050 Connection Successful.";
  char mMessage[200];
  int m = 0;
  while((m<200)) {
    mMessage[m]=mpuMsg[m];  
    m++;
  }

  Serial.println(mMessage);

  /**********************************
   * ? Send status update over LoRa ?
   * ! REMOVE WHEN TESTING OFFLINE  !
   **********************************/
  if (TELEMETRY == "ACTIVE") {
    rf95.send((uint8_t *)mMessage, 200);
    delay(10);
    rf95.waitPacketSent();
  }

  devStatus = mpu.dmpInitialize();


  /*******************************
   * ! May require recalibration *
   *******************************/
    mpu.setXAccelOffset(XACCELOFFSET);
    mpu.setYAccelOffset(YACCELOFFSET);
    mpu.setZAccelOffset(ZACCELOFFSET);
    mpu.setXGyroOffset(XGYRO_OFFSET);
    mpu.setYGyroOffset(YGYRO_OFFSET);
    mpu.setZGyroOffset(ZGYRO_OFFSET);

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

    // set our DMP Ready flag so the flightControl() function knows it's okay to use it
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
    launchState = "ERROR";
  }

  delay(2000);

  blep(BUZZER_PIN, 523, 50);
  
  if (launchState != "ERROR") {
    Serial.println();Serial.println();
    Serial.println("Here's a little bit of data!");
    for (int i = 0; i < 20; i++) {
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      }
      delay(10);
      Serial.print("a/g:\t");
      Serial.print(aaReal.x); Serial.print("\t");
      Serial.print(aaReal.y); Serial.print("\t");
      Serial.print(aaReal.z); Serial.print("\t");
      Serial.print(ypr[0]); Serial.print("\t");
      Serial.print(ypr[1]); Serial.print("\t");
      Serial.println(ypr[2]);
    }
    Serial.println();Serial.println();
  }

  delay(2000);
  
  if (DEFLECTION_TEST == "ACTIVE") {
    Serial.println("Now let's test the fins! Connecting to servos to perform Deflection Test, standby...");
    Serial.println();Serial.println();
    deflectionTest();
  }


  blep(BUZZER_PIN, 523, 50);

  if (launchState != "ERROR") {

    String readyMsg = "Flight Computer is Armed and Ready!";
    char rMessage[200];
    int r = 0;
    while((r<200)) {
      rMessage[r]=readyMsg[r];  
      r++;
    }

    if (STARTLAUNCHSTATE != "DEBUG") {
      launchState = "ARMED";
    } else {
      launchState = STARTLAUNCHSTATE;
    }

    Serial.println();
    Serial.print("Flight Computer Status: ");Serial.println(launchState);
    
    delay(1000);

    Serial.println();Serial.println();
    Serial.println(rMessage);
    Serial.println();Serial.println();

    /**********************************
     * ? Send status update over LoRa ?
     * ! REMOVE WHEN TESTING OFFLINE  !
     **********************************/
    if (TELEMETRY == "ACTIVE") {
      rf95.send((uint8_t *)rMessage, 200);
      delay(10);
      rf95.waitPacketSent();
    }
  }

  blep(BUZZER_PIN, 1760, 500);
  
  delay(1000);
  
  // Prepare Pyro Channel 1 (PYRO_1) for chute charge ignition
   if (PYRO1) {
    pinMode(PYRO_1, OUTPUT);
    digitalWrite(PYRO_1, LOW);
   }

  // readyForLaunch();
}

/********************************************************************************************************************************************************************/
/********************************************************************************************************************************************************************/
/********************************************************************* Sensor Data Assimilation *********************************************************************/
/********************************************************************************************************************************************************************/
/********************************************************************************************************************************************************************/


// XYZ Position Calculations
float calculateXDisplacement(float accelX) {
  // Filter noise from accelerometer
  float filteredX = xKalmanFilter.updateEstimate(accelX);
//  xVel = xVel + (filteredX * time);
//  xPos = calculateDistance(xPos, xVel, filteredX, time);
  // Calculate rate of change in acceleration
//  float dxAccel = (filteredX - xAccel);
//  if (dxAccel == 0) return xPos; // If the acceleration hasn't changed, then we are not moving
//
//  dxAccel = dxAccel * 9.8; // Cancel out gravity?
//
//  xPos = 0.5 * dxAccel * sq(time);

  xPos = filteredX;
  

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

float calculateDistance(float pos, float velocity, float accel, float t) {
  // Using the second integration of the acceleration function:
  float distance = pos + (velocity * time) + (0.5 * accel * sq(time));
  return distance;
}

void getVehicleAltitude() {
  unsigned long missionTime = millis();
  if (missionTime - prevDeployTime >= 3000) {
    prevDeployTime = missionTime;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    if (altitude > lastAlt) {
      lastAlt = altitude;
    } else if (altitude < lastAlt && altitude - lastAlt <= -10) {
      apogee = altitude;
      delay(150);
      deploy();
    } 
  }
}

/*********************************************************************************************************************************************************************/
/*********************************************************************************************************************************************************************/
/************************************************************************ Auxillary Functions ************************************************************************/
/*********************************************************************************************************************************************************************/
/*********************************************************************************************************************************************************************/

void blep(uint8_t buzzerPin, int pitch, int length) {
  if (!mute) {
    tone(buzzerPin, pitch);
    digitalWrite(LEDPIN, HIGH);
    delay(length);
    noTone(buzzerPin);
    digitalWrite(LEDPIN, LOW);
  }
}

/***********************************************************************************************/
/*********************************** REACTION CONTROL SYSTEM ***********************************/
/***********************************************************************************************/
void rcs(float *ypr) {
  // Convert Yaw/Pitch/Roll to readable values   
  ypr[0] = ypr[0] * 180 / M_PI;
  ypr[1] = ypr[1] * 180 / M_PI;
  ypr[2] = ypr[2] * 180 / M_PI;

  // Add Yaw/Pitch/Roll values to Kalman Filter(s)
  ypr[0] = yawKalmanFilter.updateEstimate(ypr[0]);
  ypr[1] = pitchKalmanFilter.updateEstimate(ypr[1]);
  ypr[2] = rollKalmanFilter.updateEstimate(ypr[2]);

  {/********************************/}
  {/* !! SOMETHING IS WRONG HERE !!*/}
  {/********************************/}

  // if (launchState == "LIFTOFF" || launchState == "FLIGHT") {
  // Map the values of the MPU6050 sensor from -90 to 90 to values suitable for the servo control from 0 to 180
  int servo0Value = map(ypr[2], -90, 90, 0, 180);
  int servo1Value = map(ypr[2], -90, 90, 0, 180);
  int servo2Value = map(ypr[2], -90, 90, 0, 180);
  int servo3Value = map(ypr[2], -90, 90, 0, 180);

  // Control the servos according to the MPU6050 orientation
  servo0.write(servo0Value);
  servo1.write(servo1Value);
  servo2.write(servo2Value);
  servo3.write(servo3Value);
  // }
}

void deflectionTest() {
  servo0.attach(servo0Pin);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  
  // Test servo functionality
  servo0.write(180);
  servo1.write(-180);
  servo2.write(180);
  servo3.write(-180);
  delay(2000);
  servo0.write(-180);
  servo1.write(180);
  servo2.write(-180);
  servo3.write(180);
  delay(2000);
  servo0.write(90);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  delay(2000);
}

void deploy() {
 chutesFired = true;
 // Fire relay
 digitalWrite(PYRO_1, HIGH);
 delay(1000);
 digitalWrite(PYRO_1, LOW);
 delay(100);
}

void touchdown() {
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
 if (altitude - lastAlt == 0) {
    landed = true;
    launchState = "TOUCHDOWN";
  }
}

void transmitData() {
  /**********************************
   * Convert collected data to JSON *
   **********************************/
  String Payload = "";
  if (debugState == "ON") {
    Payload += "{ ";
    Payload += "\"Temperature\": ";
    Payload += bme.readTemperature();
    Payload += ", ""\"Pressure\": ";
    Payload += (bme.readPressure() / 100.0F);
    Payload += ", ""\"Altitude\": ";
    Payload += calculateAltitude(bme.readAltitude(SEALEVELPRESSURE_HPA), aaReal.y);
    Payload += ", ""\"Humidity\": ";
    Payload += bme.readHumidity();
    Payload += ", ""\"Yaw\": ";
    Payload += ypr[0];
    Payload += ", ""\"Pitch\": ";
    Payload += ypr[1];
    Payload += ", ""\"Roll\": ";
    Payload += ypr[2];
    Payload += ", ""\"X\": ";
    Payload += calculateXDisplacement(aaReal.x);
    Payload += ", ""\"Y\": ";
    Payload += calculateYDisplacement(aaReal.y);
    Payload += ", ""\"Z\": ";
    Payload += calculateZDisplacement(aaReal.z);
    //  ! (WIP - Battery Status)
    // Payload += ",""\"Battery Voltage\":";
    // Payload += measuredvbat;
    Payload += ", ""\"LaunchState\": ";
    Payload += launchState;
    Payload += ", ""\"TELEMETRY\": ";
    Payload += TELEMETRY;
    Payload += " }";
  } else {
    Payload += "/* BLEPFLIGHTDATA,";
    Payload += ",";
    Payload += (bme.readPressure() / 100.0F);
    Payload += ",";
    Payload += calculateAltitude(bme.readAltitude(SEALEVELPRESSURE_HPA), aaReal.y);
    Payload += ",";
    Payload += bme.readHumidity();
    Payload += ",";
    Payload += ypr[0];
    Payload += ",";
    Payload += ypr[1];
    Payload += ",";
    Payload += ypr[2];
    Payload += ",";
    Payload += calculateXDisplacement(aaReal.x);
    Payload += ",";
    Payload += calculateYDisplacement(aaReal.y);
    Payload += ",";
    Payload += calculateZDisplacement(aaReal.z);
    //  ! (WIP - Battery Status)
    // Payload += ",";
    // Payload += measuredvbat;
    Payload += "*/";
  }
      
  char packet[250];
  int i = 0;
  int sizeOf = 250;
  int offset = 0;
  
  while((i<sizeOf))
  {
      packet[i]=Payload[i];  
      i++;
  }

  Serial.println(packet);

  /*******************************/
  /*  LoRa Payload transmission  */
  /*******************************/
  if (TELEMETRY == "ACTIVE") {
    rf95.send((uint8_t *)packet, 200);
    delay(10);
    rf95.waitPacketSent();
  }
}

void sync() {
  if (rf95.available())
  {
    // Fetch message from ground control
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      
      // Temporary pitch value to audibly distinguish status
      blep(BUZZER_PIN, 800, 50); 

      // Send a reply
      uint8_t data[] = "TELEMETRY GOOD";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("TELEMETRY GOOD");
    }
    else
    {
      Serial.println("Failed to fetch data from ground control. Trying again, standby...");
    }
  }
}

// Blink and beep to indicate vehicle status after startup
void missionBlep(float intervalSkip, int pitch) {
  unsigned long missionTime = millis();
    if (missionTime - prevTime >= intervalSkip) {
      prevTime = missionTime;
      blep(BUZZER_PIN, pitch, 50);
    }

    /** 
     * If flight computer has not launched or changed state after two minutes of being active,
     * set the launch state to 'IDLE'
     */
    if (missionTime >= 120000 && launchState == "ARMED") {
      launchState = "IDLE";
    }

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
