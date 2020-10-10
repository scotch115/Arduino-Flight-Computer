#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
//#include <SoftwareSerial.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>


#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    10   // Chip select pin
#define ESP32_RESETN   5   // Reset pin
#define SPIWIFI_ACK    7   // a.k.a BUSY or READY pin
#define ESP32_GPIO0    6


unsigned long delayTime;

IPAddress remoteIp(192, 168, 1, 65);
unsigned int localPort = 2391;      // local port to listen on
WiFiUDP Udp;


void setup() {
  Serial.begin(9600);
  delay(2000);  // time to get serial running
  Serial.println(F("Power on"));

  unsigned status;
  unsigned wStatus;

  char groundControlDetected[100];
  int j = 0;
  int sizeOf = 100;
  int offset = 0;
  String gStr = "Ground Control Detected.";
  
  while((j<sizeOf))
  {
     groundControlDetected[j+offset]=gStr[j];
      
     j++;
  }

  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);  
  while (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("AirLift Module not detected!"));
    delay(1000);
  }

  Serial.println(F("AirLift Module detected."));
  
  Serial.println(F("Attempting to connect to WiFi "));
  do {
    wStatus = WiFi.begin("ilovelucy", "JordanTorri108");
    delay(100);
  } while (wStatus != WL_CONNECTED);
  printWifiStatus();
  Udp.begin(localPort);

  Udp.beginPacket(remoteIp, 2931);
  if (wStatus) {
    Udp.write(groundControlDetected);
  }
  Udp.endPacket();
  delay(2000);

}

void loop() {
  unsigned status;
  Serial.print("Awaiting commands");
  while (!status) {
      Serial.print(".");
  }
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
