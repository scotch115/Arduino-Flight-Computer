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


unsigned long delayTime = 2000;

IPAddress remoteIp(192, 168, 1, 65);
unsigned int localPort = 8888;      // local port to listen on
WiFiUDP Udp;

// unsigned int remotePort = 2930;

char packetBuffer[255];
char ReplyBuffer[255];


void setup() {
  Serial.begin(9600);
  delay(2000);  // time to get serial running
  Serial.println(F("Power on"));

  unsigned status;
  unsigned wStatus;

  char groundControlDetected[100], flightCPU[100];
  int j = 0;
  int sizeOf = 100;
  int offset = 0;
  String gStr = "Ground Control Detected.";
  String reply = "Connection established, vehicle should be prepped for launch.";
  
  while((j<sizeOf))
  {
     groundControlDetected[j+offset]=gStr[j];      
     ReplyBuffer[j+offset]=reply[j];
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
  Udp.write(groundControlDetected);
  Udp.endPacket();

  

}

void loop() {
  unsigned status;
  
    
  // Check for packet data from flight cpu
  int packetSize = Udp.parsePacket();
  if (packetSize) {
      IPAddress remote = Udp.remoteIP();
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
          packetBuffer[len] = 0;
      }
      Serial.print("Received: ");
      Serial.println(packetBuffer);

      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ReplyBuffer);
      Udp.endPacket();
  }

  delay(delayTime);
  
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