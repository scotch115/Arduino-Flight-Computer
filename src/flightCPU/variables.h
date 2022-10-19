/*************************************************************************************************************************************************************************/
/*************************************************************************************************************************************************************************/
/******************************************************************************** Globals ********************************************************************************/
/*************************************************************************************************************************************************************************/
/*************************************************************************************************************************************************************************/

// LoRa Pinouts
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Radio communication frequency
#define RF95_FREQ 915.0

// LoRa Radio Module instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Barometric Altimeter instance
Adafruit_BME280 bme; // I2C

// Inertial Measurement Unit (IMU) instance
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 6

// Battery Voltage pinout (for collecting battery status)
#define VBATPIN A7

#define BUZZER_PIN A1

// LED Pinout
#define LEDPIN A3
unsigned long prevTime = 0;
unsigned long prevDeployTime = 0;
const long interval = 1000;

// PYRO CHANNELS
#define PYRO_1 A2

int altitude, lastAlt, apogee = 0;

// Buzzer Sounds
unsigned int numTones = 6;
unsigned int tones[] = {523, 587, 659, 739, 830, 880};
/************* upper    C    D    E    F#   G#   A   */

/************************
 *** Servo Definitions ***
 ************************/
int servo0Pin = 10;
int servo1Pin = 11;
int servo2Pin = 12;
int servo3Pin = 13;
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;

/************************************
 *  Data Payload (String) Variables *
 ************************************/
bool chutesFired = false;
bool landed = false;

char packetBuffer[100];
char flightData[60] = "{""\"status\":""\"Flight CPU sent data to server\"""}";
char chutes[50] = "{""\"status\":""\"Chutes Deployed!\"""}";
char liftoff[50] = "{""\"status\":""\"Liftoff!\"""}";
char bmeDetected[60] = "{""\"status\":""\"BME 280 Detected.\"""}";
char mpuDetected[60] = "{""\"status\":""\"MPU 6050 Detected.\"""}";
char startupCompleted[60] = "{""\"status\":""\"Startup process complete.\"""}";
char flightCPU[50] = "{""\"status\":""\"Flight CPU ready\"""}";
char vehicleLanded[50] = "{""\"status\":""\"Touchdown!\"""}";


/***************************
 * MPU control/status vars *
 ***************************/
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

/***************************
 * orientation/motion vars *
 ***************************/
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// Define base position values as zero, to convey the "starting" position
float xPos = 0;
float yPos = 0;
float zPos = 0;

float time = millis();
float xAccel = 0;


/******************************
 * IMU interruption detection *
 ******************************/
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


/********************** KALMAN FILTERS**********************/
/***************** SimpleKalmanFilter(e_mea, e_est, q); ****/
/***************** e_mea: Measurement Uncertainty **********/
/***************** e_est: Estimation Uncertainty ***********/
/****************** q: Process Noise  **********************/
SimpleKalmanFilter xKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter yKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter zKalmanFilter(1, 1, 0.01);

SimpleKalmanFilter yawKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter pitchKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter rollKalmanFilter(1, 1, 0.01);


/******************/
/* State Handling */
/******************/
String launchState = "OFF";

bool descent = false;

bool RCS = true;