
#include <SPI.h>
#include <RH_RF95.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
// Feather 32u4 w/Radio

  #define RFM95_CS      10
  #define RFM95_INT     2
  #define RFM95_RST     11


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

boolean launchStatus;

char packetBuffer[255];
char ReplyBuffer[255];

char countdown5[10], countdown4[10], countdown3[10], countdown2[10], countdown1[10];

//char startlaunch = "startlaunch";

char launch[100], ignition[100];

// ------------ Setup

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);

//  Serial.println("Feather RFM69 RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  if (!rf95.init()) {
    Serial.println("RFM95 radio init failed");
    while (1);
  }
//  Serial.println("RFM95 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf95.setTxPower(20, false);  // range from 14-20 for power, 2nd arg must be true for 69HCW
  
  unsigned status;
  unsigned wStatus;

  launchStatus = false;
  
  char groundControlDetected[100], flightCPU[100];
  int j = 0;
  int sizeOf = 100;
  int offset = 0;
  String gStr = "{""\"status\": ""\"Ground Control Detected.\"""}";
  String reply = "{""\"status\":""\"Connection established, vehicle will launch.\"""}";
  String go4launch = "{""\"status\":""\"Launch initiated.\"""}";
  String launchComplete = "{""\"status\":""\"Ignition\"""}";

  while ((j < sizeOf))
  {
    groundControlDetected[j + offset] = gStr[j];
    ReplyBuffer[j + offset] = reply[j];
    launch[j + offset] = go4launch[j];
    ignition[j + offset] = launchComplete[j];
    j++;
  }

  Serial.println(groundControlDetected);
  
  // May also save to sdcard


}

// ------------ Main loop

void loop() {
  if (rf95.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
//      Serial.print("Received [");
//      Serial.print(len);
//      Serial.print("]: ");
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);

      if (strstr((char *)buf, "BME280 Initialized") || strstr((char *)buf, "MPU6050 Initialized") || strstr((char *)buf, "Flight Computer Primed and Ready")) {
        String statusMsg = "{""\"status\": ""\"";
        statusMsg += (char *)buf;
        statusMsg += "\"""}";
        int i = 0;
        char message[200];
        while ((i<200)) {
          message[i]=statusMsg[i];
          i++;
        }
        Serial.println(statusMsg);
        
      }
      Serial.println((char*)buf);
    } else {
      Serial.println("Receive failed");
    }
  }
  // OLD CODE VVVVVV
//  analogWrite(A0, 0);
  // Check for packet data from flight cpu
  
  // NOTE - once radio is receiving data, write code to create packet buffer for serial
  
//    Serial.print("Received: ");
//    Serial.println(packetBuffer);

//    analogWrite(A0, 150);

//    if (launchStatus == false) {
//      if (strcmp(packetBuffer, "startlaunch") == 0) {
//        Serial.println("Received go for launch from server!");
//        beginLaunch();
//      }
//    }

      /* In theory, I would want an indication on the ground that the chutes successfully deployed*/
//    if (packetBuffer == '{"status":"Deploy chutes!"}') {
//      // Confirmation of deployed chutes
//      Serial.println("CONFIRM");
//      analogWrite(A0, 0);
//      delay(500);
//      analogWrite(A0, 150);
//      delay(500);
//    }
  }


  // Flight computer startup has completed at this point, the next step would be to begin the launch sequence,
  // and ignite the motor using analogWrite(). -- Find more features to add to ground control

//}

  // -------------- Need to add relay to board and/or develop code to actually ignite motor after countdown
  // -------------- Would also be cool to display countdown timer on LED Display, start with Serial
