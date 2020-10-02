/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal.h> 
#include <SoftwareSerial.h>


#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1016.9)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
const byte numChars = 3; // X, Y, Z data points
int received[numChars];
boolean newData = false;

SoftwareSerial serialIn(3, 2); // RX, TX
void setup() {
    Serial.begin(9600);
    serialIn.begin(57600);
    while(!Serial);    // time to get serial running
//    Serial.println(F("BME280 test"));

    unsigned status;
    
    // default settings
    status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
//    Serial.println("-- Default Test --");
    delayTime = 1000;

//    Serial.println();

    lcd.begin(16, 2);
    lcd.print("BME280 Sensor");
    lcd.setCursor(0, 1);
    lcd.print("Scan");
    delay(delayTime);
}


void loop() { 
  // Multiple calls to serialValues() between calls to printValues() to create manual delay on lcd without delaying serial output. Bad implementation but works for now
    
    printValues(1);
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    printValues(2);
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
    serialValues();
//    printValues(3); // For LCD Testing Purposes only
//    delay(delayTime); 


   // Attempting to connect Arduino Pro Micro with LIS3DH accelerometer to Arduino Nano with LCD and BME280 Temperature sensor  
//    Serial.println("==Checking for Wire.==");
//    recvWithEndMarker();
//    showNewNumber();
//    delay(delayTime);
}

void recvWithEndMarker() {
    static byte ndx = 0;
    static byte rCounter = 0;
    char endMarker = '\n';
    byte rc;
    
    while (serialIn.available() > 0 && newData == false) {
//        Serial.println("==ProMicro detected!==");
        rc = serialIn.read();
//        Serial.println(serialIn.read());
        

        if (rCounter != 3) {
            received[ndx] = rc;
            ndx++;
            rCounter++;
        }
        else {
//            received[ndx] = '\0'; // terminate the string
            ndx = 0;
            rCounter = 0;
            newData = true;
        }
    }
}

void showNewNumber() {
  if (newData == true) {
    Serial.println("Data received: ");
    Serial.print("X: ");
    Serial.println(received[0]);
    Serial.print("Y: ");
    Serial.println(received[1]);
    Serial.print("Z: ");
    Serial.println(received[2]);
    lcd.clear();
    lcd.print("X: ");
    lcd.print(received[0]);
    lcd.print(" Y: ");
    lcd.print(received[1]);
    lcd.setCursor(0,1);
    lcd.print("Z: ");
    lcd.print(received[2]); 
    newData = false;
    memset(received, 0, sizeof(received));
  }
}

void serialValues() {
   Serial.print("{\"Temperature\":");
    Serial.print(bme.readTemperature());
    Serial.print(",");

    Serial.print("\"Pressure\":");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.print(",");

    Serial.print("\"Altitude\":");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.print(",");

    Serial.print("\"Humidity\":");
    Serial.print(bme.readHumidity());
    Serial.println("}");
}



void printValues(int i) {
  serialValues();
    if (i == 1) {
      lcd.clear();
      lcd.print("Temp: ");
      lcd.print(bme.readTemperature());
      lcd.print("*C");
      lcd.setCursor(0, 1);
      lcd.print("Humd: ");
      lcd.print(bme.readHumidity());
      lcd.print("%");
    } else if (i == 2) {
      lcd.clear();
      lcd.print("Press: ");
      lcd.print(bme.readPressure() / 100.0F);
      lcd.print("hPa");
      lcd.setCursor(0, 1);
      lcd.print("Alt: ");
      lcd.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      lcd.print("m");
    } else if ( i == 3) {
      lcd.clear();
      lcd.print("T:");
      lcd.print(bme.readTemperature(), 0);
      lcd.print("*C/H:");
      lcd.print(bme.readHumidity(), 0);
      lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("P:");
      lcd.print((bme.readPressure() / 100.0F), 0);
      lcd.print("/A:");
      lcd.print(bme.readAltitude(SEALEVELPRESSURE_HPA),0);
      lcd.print("m");
    }
}