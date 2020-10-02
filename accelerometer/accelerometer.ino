// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>

// Used for software SPI
#define LIS3DH_CLK 9
#define LIS3DH_MISO 8
#define LIS3DH_MOSI 7
// Used for hardware & software SPI
#define LIS3DH_CS 6

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();

SoftwareSerial sout(2, 3); // RX, TX

void setup(void) {
  sout.begin(9600);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("LIS3DH test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
   sout.write("LIS3DH found!");

   lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

    Serial.print("Range = ");
    Serial.print(2 << lis.getRange());
    Serial.print("G");

  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  sout.write("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: sout.write("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: sout.write("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: sout.write("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: sout.write("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: sout.write("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: sout.write("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: sout.write("400 Hz"); break;


   case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
   case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
   case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }
}

void loop() {
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); 
  Serial.print(lis.x);
  Serial.print("  \tY:  "); 
  Serial.print(lis.y);
  Serial.print("  \tZ:  "); 
  Serial.print(lis.z);
  
  
  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);

  sout.write(lis.x);
  sout.write(lis.y);
  sout.write(lis.z);

  /* Display the results (acceleration is measured in m/s^2) */
  /* For testing purposes */
//  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
//  Serial.print(" \tY: "); Serial.print(event.acceleration.y);
//  Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
//  Serial.println(" m/s^2 ");

  Serial.println();

  delay(50);
}