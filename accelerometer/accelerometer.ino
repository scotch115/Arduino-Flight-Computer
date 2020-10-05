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

//SoftwareSerial sout(2, 3); // RX, TX

void setup(void) {
  Serial.begin(9600);
  Wire.begin();
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("LIS3DH test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
   Serial.print("LIS3DH found!");

   lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

    Serial.print("Range = ");
    Serial.print(2 << lis.getRange());
    Serial.print("G");

  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.write("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.write("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.write("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.write("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.write("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.write("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.write("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.write("400 Hz"); break;


   case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
   case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
   case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }
}



void loop() {
  char strx[4];
  char stry[4];
  char strz[4];
  
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); 
  Serial.print(lis.x);
  Serial.print("  \tY:  "); 
  Serial.print(lis.y);
  Serial.print("  \tZ:  "); 
  Serial.print(lis.z);

  itoa(lis.x, strx, 10);
  itoa(lis.y, stry, 10);
  itoa(lis.z, strz, 10);
  Serial.write(strx);
  Serial.write(stry);
  Serial.write(strz);

//  Wire.write(lis.x);
//  Wire.write(lis.y);
//  Wire.write(lis.z);
//  Wire.endTransmission();
  
  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);

  
  /* Display the results (acceleration is measured in m/s^2) */
  /* For testing purposes */
//  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
//  Serial.print(" \tY: "); Serial.print(event.acceleration.y);
//  Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
//  Serial.println(" m/s^2 ");

  Serial.println();

  delay(50);
}
