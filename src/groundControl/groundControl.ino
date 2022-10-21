#include "Wire.h"
#include <SPI.h>
#include <RH_RF95.h>
#include "config.h"

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int16_t packetnum = 0;  // packet counter, we increment per transmission

String launchState = "OFF";

/*****************************************************************************************************************************************************************************
 *********************************************************************************   Setup   *********************************************************************************
 *****************************************************************************************************************************************************************************/

void setup() {
  Serial.begin(115200);
  while (!Serial) {

  }

  preCheck();
}


// !WIP - Output data to Serial Studio as:
// `/*BLEP,x,x,x,x,x,...x*/` string 


/**
 * TODO:--
 * * Connect LoRa featherwing and get RF breakout working
 * * Create direct, one-to-one RF communication between ground and blep
 * * Install additional components (buzzer, LED, relayWing, etc)
 * * Design launch pad
 * * Wire new (cool) flip switch to prevent accidental launch
 * * Format serial output for SerialStudio collection and analysis
 * * Create a countdown loop that can be cancelled in case of emergency on the pad
 * * 
 */

/*****************************************************************************************************************************************************************************
 ********************************************************************************* Main loop *********************************************************************************
 *****************************************************************************************************************************************************************************/
void loop() {
  /**
   * @note - The flight CPU should have the following "commands", receivable via 900mHz radio from the launch pad controller
   * 
   * @param - STARTUP
   * @param - TELEM
   * @param - ARMED
   * @param - IGNITION / LAUNCH
   * @param - DEPLOY / CHUTE
   * @param - REBOOT
   */
    digitalWrite(LEDPIN, HIGH);
    delay(50);
    digitalWrite(LEDPIN, LOW);
    delay(500);
    Serial.println("Waiting for data...");
    delay(1000);
    if (TELEMETRY == "ACTIVE") {
      if (rf95.available()) {
        // Should be a message for us now   
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        if (rf95.recv(buf, &len)) {
          if (!len) return;
          buf[len] = 0;
        Serial.print("Received [");
        Serial.print(len);
        Serial.print("]: ");
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);
          Serial.println((char*)buf);
        } else {
          Serial.println("Receive failed");
        }
      }
    } else {
      Serial.println('Radio module disabled. Connect the antenna and turn on telemetry in the config.h file.');
    }
}

void preCheck() {
  Serial.println();Serial.println();
  Serial.println("Beginning Pre-Flight Software check.");
  Serial.println();Serial.println();
  delay(2000);

  launchState = "STARTUP";
  Serial.println();
  Serial.print("Ground Control Computer Status: ");Serial.println(launchState);

  pinMode(LEDPIN, OUTPUT);

//  // Play bootup sequence sound
//  for (unsigned int i = 0; i < numTones; i++) {
//    tone(A1, tones[i]);
//    delay(50);
//  }
//  noTone(A1);
//  delay(2000);  // time to get serial running

/*********************************************************
 * Send data to the flight computer to see if it's awake. *
 * Check if the flight computer responded.                *
 *********************************************************/

  // <Some RF95 code>
  // Serial.println("Blep Flight Computer Online!");
  // launchState = "TELEM"; // This means we have a direct radio connection to the flight computer -- congrats!



}