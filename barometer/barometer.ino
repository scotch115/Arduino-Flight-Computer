#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_LIS3DH.h>
#include <LiquidCrystal.h>
//#include <SoftwareSerial.h>
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

#define SEALEVELPRESSURE_HPA (1016.3)

#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    13   // Chip select pin
#define ESP32_RESETN  12   // Reset pin
#define SPIWIFI_ACK   11   // a.k.a BUSY or READY pin
#define ESP32_GPIO0   -1

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

//LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

IPAddress remoteIp(192, 168, 1, 65);
IPAddress groundIp(192, 168, 1, 153);
unsigned int localPort = 2390;      // local port to listen on
WiFiUDP Udp;

char packetBuffer[255];
char wait[100] = "Flight Computer waiting to establish connection to Ground Control";
char flightData[100];

void setup() {
  Serial.begin(9600);
  delay(2000);  // time to get serial running
  Serial.println(F("Power on"));

  unsigned status;
  unsigned lStatus;
  unsigned wStatus;
  unsigned cpuReady;
  char bmeDetected[100], lisDetected[100], airDetected[100], startupCompleted[100], continuityPass[80], continuityFail[80], flightCPU[100];
  int j = 0;
  int sizeOf = 100;
  int offset = 0;
  String bStr = "BME 280 Detected.";
  String lStr = "LIS3DH Detected.";
  String aStr = "AirLift Featherwing Detected.";
  String sStr = "Startup process completed, beginning sensor scans.";
  String cPStr = "Flight Computer has continuity on tested A0";
  String cFStr = "Flight Computer does not have continuity on A0";
  String fStr = "Flight CPU ready";
  String dataStr = "Flight CPU sent data to server";
  
  while((j<sizeOf))
  {
     bmeDetected[j+offset]=bStr[j];
     lisDetected[j+offset]=lStr[j];
     airDetected[j+offset]=aStr[j];
     startupCompleted[j+offset]=sStr[j];
     continuityPass[j+offset]=cPStr[j];
     continuityFail[j+offset]=cFStr[j];
    flightCPU[j+offset]=fStr[j];
    flightData[j+offset]=dataStr[j];
     
     j++;
  }

  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);  
  while (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("AirLift Featherwing not detected!"));
    delay(1000);
  }

  Serial.println(F("AirLift Featherwing detected."));
  
  Serial.println(F("Attempting to connect to WiFi "));
  do {
    wStatus = WiFi.begin("ilovelucy", "JordanTorri108");
    delay(100);
  } while (wStatus != WL_CONNECTED);
  printWifiStatus();
  Udp.begin(localPort);

  Udp.beginPacket(remoteIp, 2931);
  if (wStatus) {
    Udp.write(airDetected);
  }
  Udp.endPacket();
  delay(2000);

  // default settings
  status = bme.begin();
//  // You can also pass in a Wire library object like &Wire2
//  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    cpuReady == false;
    Serial.println(F("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID aof 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  } else {
    Udp.beginPacket(remoteIp, 2931);
    if (status) {
      Udp.write(bmeDetected);
    }
    Udp.endPacket();
    delay(2000);
  }

  wStatus = WL_IDLE_STATUS;

  delayTime = 1000;
  lStatus = lis.begin(0x18);

  if (!lStatus) {   // change this to 0x19 for alternative i2c address
    Serial.println(F("Couldnt start"));
    cpuReady == false;
    while (1) yield();
  } else { Serial.println(F("LIS3DH Detected.")); }

  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  
  Udp.beginPacket(remoteIp, 2931);
  if (lStatus) {
    Udp.write(lisDetected);
  }
  Udp.endPacket();
  delay(2000);

  pinMode(A0, OUTPUT);
//  digitalWrite(A0, HIGH);
  pinMode(A0, INPUT_PULLUP);
  
  int continuityVal = digitalRead(A0);
  if (continuityVal == HIGH) {
    Udp.beginPacket(remoteIp, 2931);
    Udp.write(continuityPass);
    Udp.endPacket();
  } else {
    Udp.beginPacket(remoteIp, 2931);
    Udp.write(continuityFail);
    Udp.endPacket();
  }

  Serial.println(F("== Startup process completed. =="));
  cpuReady == true;
  
  // Attempting to send confirmation to remote server to show startup has completed, but it's not loading the rest of the code when I do that... hmmmm
  Udp.beginPacket(remoteIp, 2931);
  Udp.write(startupCompleted);
  Udp.endPacket();
  delay(2000);

  Udp.beginPacket(groundIp, 8888);
  Udp.write(flightCPU);
  Udp.endPacket();
  delay(2000);

}

void loop() {
  lis.read();
  sensors_event_t event;
  lis.getEvent(&event);

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
  ReplyBuffer += ",""\"∆X\":";
  ReplyBuffer += event.acceleration.x;
  ReplyBuffer += ",""\"∆Y\":";
  ReplyBuffer += event.acceleration.y;
  ReplyBuffer += ",""\"∆Z\":";
  ReplyBuffer += event.acceleration.z;
  ReplyBuffer += "}";
  
  char reply[300];
  int i = 0;
  int sizeOf = 180;
  int offset = 0;
  
  while((i<sizeOf))
  {
     reply[i+offset]=ReplyBuffer[i];
  
     i++;
  }
  
  Udp.beginPacket(remoteIp, 2931);
  Udp.write(reply);
  Udp.endPacket();
  Serial.println(reply);
  delay(100);
  
  Udp.beginPacket(groundIp, 8888);
  Udp.write(flightData);
  Udp.endPacket();
  delay(100);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print(F("signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));

}
