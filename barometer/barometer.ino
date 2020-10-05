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
#include <Adafruit_LIS3DH.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>


#define LIS3DH_CLK 10
#define LIS3DH_MISO 9
#define LIS3DH_MOSI 6
// Used for hardware & software SPI
#define LIS3DH_CS 5

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);


#define BME_SCK 12
#define BME_MISO 11
#define BME_MOSI 10
#define BME_CS 9

#define SEALEVELPRESSURE_HPA (1016.9)

#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    13   // Chip select pin
#define ESP32_RESETN  12   // Reset pin
#define SPIWIFI_ACK   11   // a.k.a BUSY or READY pin
#define ESP32_GPIO0   -1

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
const byte numChars = 3; // X, Y, Z data points
int received[numChars];
boolean newData = false;


unsigned int localPort = 2391;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
//char  ReplyBuffer[];       // a string to send back

WiFiUDP Udp;


void setup() {
  Serial.begin(9600);
  while (!Serial);   // time to get serial running
  //    Serial.println(F("BME280 test"));

  unsigned status;

  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID aof 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  status = WL_IDLE_STATUS;

  delayTime = 1000;

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }

  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  //  Serial.write("Data rate set to: ");
  //  switch (lis.getDataRate()) {
  //    case LIS3DH_DATARATE_1_HZ: Serial.write("1 Hz"); break;
  //    case LIS3DH_DATARATE_10_HZ: Serial.write("10 Hz"); break;
  //    case LIS3DH_DATARATE_25_HZ: Serial.write("25 Hz"); break;
  //    case LIS3DH_DATARATE_50_HZ: Serial.write("50 Hz"); break;
  //    case LIS3DH_DATARATE_100_HZ: Serial.write("100 Hz"); break;
  //    case LIS3DH_DATARATE_200_HZ: Serial.write("200 Hz"); break;
  //    case LIS3DH_DATARATE_400_HZ: Serial.write("400 Hz"); break;
  //
  //
  //   case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
  //   case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
  //   case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  //  }

  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);

  Serial.println("Attempting to connect to WiFi ");
  do {
    status = WiFi.begin("ilovelucy", "JordanTorri108");
    delay(100);
  } while (status != WL_CONNECTED);
  printWifiStatus();
  Udp.begin(localPort);
  

//  lcd.begin(16, 2);
//  lcd.print("BME280 Sensor");
//  lcd.setCursor(0, 1);
//  lcd.print("Scan");
  delay(delayTime);
}

char strx[4];
char stry[4];
char strz[4];

void loop() {
  // Multiple calls to serialValues() between calls to printValues() to create manual delay on lcd without delaying serial output. Bad implementation but works for now
  lis.read();
//  printValues(1);
//  serialValues();
  delay(100);
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  printValues(2);
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
//  serialValues();
String ReplyBuffer = "{""\"Temperature\":";
ReplyBuffer += bme.readTemperature();
ReplyBuffer += ",""\"Pressure\":";
ReplyBuffer += (bme.readPressure() / 100.0F);
ReplyBuffer += ",""\"Altitude\":";
ReplyBuffer += bme.readAltitude(SEALEVELPRESSURE_HPA);
ReplyBuffer += ",""\"Humidity\":";
ReplyBuffer += bme.readHumidity();
ReplyBuffer += ",""\"X\":";
ReplyBuffer += lis.x;
ReplyBuffer += ",""\"Y\":";
ReplyBuffer += lis.y;
ReplyBuffer += ",""\"Z\":";
ReplyBuffer += lis.z;
ReplyBuffer += "}";

char reply[300];
int i = 0;
int sizeOf = 120;
int offset = 0;

while((i<sizeOf))
{
   reply[i+offset]=ReplyBuffer[i];

   i++;
}
IPAddress remoteIp(192, 168, 1, 65);
Udp.beginPacket(remoteIp, 2931);
Udp.write(reply);
Udp.endPacket();
Serial.println(reply);
 
}

void serialValues() {
  lis.read();
  Serial.print("{");
  Serial.print("\"Temperature\":");
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
  Serial.print(",");

  Serial.print("\"X\":");
  Serial.print(lis.x);
  Serial.print(",");

  Serial.print("\"Y\":");
  Serial.print(lis.y);
  Serial.print(",");

  Serial.print("\"Z\":");
  Serial.print(lis.z);
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
    lcd.print(bme.readAltitude(SEALEVELPRESSURE_HPA), 0);
    lcd.print("m");
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
