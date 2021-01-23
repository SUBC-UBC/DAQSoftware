// DAQ 2020
// GPL v3
// SUBC: the UBC Submarine Design Team

#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include "src/lib/I2Cdev.h"
//#include "lib/MPU6050.h"
#include "src/lib/MPU6050_6Axis_MotionApps20.h"
#include "src/lib/MS5837.h"
#include "src/lib/helper_3dmath.h"

#include "daqSoftware.h"

// For VS Code Linting support
#ifndef Serial
HardwareSerial Serial(PIN_SERIAL_RX, PIN_SERIAL_TX);
#endif
// End serial linting

// IMU variables and objects
MPU6050 IMU;
bool IMUWorking = false;
int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile int changes = 0;
unsigned long startTime;
unsigned long endTime;
unsigned long recordTime;
int duration;
int oldFall = 0;
boolean running = false;
// boolean oldRunning=false;
boolean error = false;

///////////////////////////////////////////////////////////
// Following code from Jeff Rowberg's I2Cdev example, licensed under MIT
// license. MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus;   // return status after each device operation (0 = success,
                     // !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;  // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16
    aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float
    ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt =
    false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

// End Rowberg code
///////////////////////////////////////////////////////////

#ifdef DEPTHSENSOR
bool endBuoyancy;

double setDepth;
double depth;
#endif

#ifdef SDON
File outputFile;
#endif

/*// Debounce stuff
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an
int. unsigned long lastDebounceTime = 0;  // the last time the output pin was
toggled unsigned long debounceDelay = 50;    // the debounce time; increase if
the output flickers*/

///////////////////////////////////////////////////////////
// SubSee Serial JSON output objects
HardwareSerial Serial3(SUBSEERIAL_RX, SUBSEERIAL_TX);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // Wait for Serial to connect
  }

// SD Initialization
#ifdef SDON
  init_SD();
#endif

  // Initialize IMU -- always on
  init_IMU();

// Initialize bar02(MS5837) pressure sensor
#ifdef DEPTHSENSOR
  init_Depth();
#endif

#ifdef TACHO
  attachInterrupt(digitalPinToInterrupt(tachoPin), tachoChange, CHANGE);
#endif

#ifdef SUBSEERIAL
  // Initialize SubSee serial output
  Serial3.begin(SUBSEERIAL_BAUD);
#endif
  // Button stuff
  pinMode(BUTTONPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), buttonPress, FALLING);

  pinMode(INDICATORPIN, OUTPUT);
  digitalWrite(INDICATORPIN, LOW);
}

void loop() {

  // Before we acquire data, make sure DAQ is not errored, is supposed to be
  // running, and we're not in a delay period
  if (!(error) && running && (millis() - startTime > MEASUREDELAY)) {
    endTime = millis();
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    getDMPData();
    Serial.print("Time: ");
    duration = (endTime - recordTime);

#ifdef SDON
    print_data_to_file();
#endif

    print_data_to_serial();

    Serial.print(duration);

    changes = 0;
    startTime = millis();
  } else {
    delay(DELAYTIME);
  }
}

void buttonPress(void) {
  Serial.println("Button falling!");
  if (millis() - oldFall > DEBOUNCETIME) {
    oldFall = millis();
    if (running == true && !(error)) {
#ifdef SDON
      // Stuff to close file, etc.
      outputFile.close();
      Serial.println("Closing file...");
#endif

      running = false;
      digitalWrite(INDICATORPIN, LOW);
      delay(100);
    } else if (running == false && !(error)) {
#ifdef SDON
      // Stuff to open file, etc.
      // create a new file
      char filename[] = "DATA00.CSV";
      for (uint8_t i = 0; i < 100; i++) {
        filename[4] = i / 10 + '0';
        filename[5] = i % 10 + '0';
        if (!SD.exists(filename)) {
          // only open a new file if it doesn't exist
          outputFile = SD.open(filename, FILE_WRITE);
          break; // leave the loop!
        }
      }
      // Order of outputs: Time, Yaw, Pitch, Roll, ax, ay, az, gx, gy, gz, RPM
      outputFile.print("Time(ms),Yaw,Pitch,Roll,ax,ay,az,gx,gy,gz");
#ifdef TACHO
      outputFile.print(",RPM");
#endif
#ifdef DEPTHSENSOR
      outputFile.print(",depth");
#endif
      outputFile.println("");
      Serial.print(filename);
      Serial.println(" successfully initialized!");
#endif

      digitalWrite(INDICATORPIN, HIGH);
      running = true;

      changes = 0;
      startTime = millis();
      recordTime = startTime;
    } else {
#ifdef SDON
      Serial.println("Something went wrong. Trying to close file.");
      outputFile.close();
#endif
    }
  } else {
    Serial.println(
        "Not enough time has passed since the last event. Debouncing.");
  }
}

void tachoChange(void) { changes++; }

void init_DMP(void) {
  // Start Jeff Rowberg's code, MIT
  Serial.println("Initializing DMP...");
  devStatus = IMU.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  IMU.setXGyroOffset(220);
  IMU.setYGyroOffset(76);
  IMU.setZGyroOffset(-85);
  IMU.setZAccelOffset(1788);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    IMU.CalibrateAccel(6);
    IMU.CalibrateGyro(6);
    IMU.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println("Enabling DMP...");
    IMU.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print("Enabling interrupt detection (Arduino external interrupt ");
    Serial.print(digitalPinToInterrupt(IMUINTPIN));
    Serial.println(")...");
    attachInterrupt(digitalPinToInterrupt(IMUINTPIN), dmpDataReady, RISING);
    mpuIntStatus = IMU.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to
    // use it
    Serial.println("DMP ready! Waiting for first interrupt...");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = IMU.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    error = true;
  }
  // End Jeff Rowberg's code
  return;
}

void init_SD(void) {
  Serial.print("Initializing SD card...");
  if (!SD.begin(PA15)) {
    Serial.println("SD initialization failed!");
    error = true;
  }
  Serial.println("initialization done.");
  return;
}

void init_IMU(void) {
  Wire.setSDA(SDAPIN);
  Wire.setSCL(SCLPIN);
  Wire.begin();
  Serial.println("Initializing the sensor");
  IMU.initialize();
  IMUWorking = IMU.testConnection();
  pinMode(IMUINTPIN, INPUT);
  if (IMUWorking == true) {
    Serial.println("Successfully Connected");
    Serial.println("Taking Values from the sensor");
    delay(250);

    init_DMP();
  } else {
    Serial.println("Connection failed");
    error = true;
  }
  return;
}

void init_Depth(void) {
#ifdef DEPTHSENSOR
  MS5837 bar_02; // bar_02 pressure sensor

  bar_02.setModel(MS5837::MS5837_02BA);
  if (!bar_02.init()) {
    Serial.println("Pressure sensor initialization failed!");
    error = true;
    delay(1000);
  }

  bar_02.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
  bar_02.read();               // read pressure sensor
  setDepth = bar_02.depth();   // get the set depth value
  // Initialize buoyancy compensator variable
  endBuoyancy = LOW;
  return;
#endif
}

void getDMPData(void) {
  // Begin Jeff Rowberg code (MIT License)
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = IMU.getIntStatus();

  // get current FIFO count
  fifoCount = IMU.getFIFOCount();
  if (fifoCount < packetSize) {
    // Lets go back and wait for another interrupt. We shouldn't be here, we got
    // an interrupt from another event
    // This is blocking so don't do it   while (fifoCount < packetSize)
    // fifoCount = mpu.getFIFOCount();
  }
  // check for overflow (this should never happen unless our code is too
  // inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) ||
           fifoCount >= 1024) {
    // reset so we can continue cleanly
    IMU.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to
    //  ask
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen
    // frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // read a packet from FIFO
    while (fifoCount >= packetSize) { // Lets catch up to NOW, someone is using
                                      // the dreaded delay()!
      IMU.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }
    // display Euler angles in degrees
    IMU.dmpGetQuaternion(&q, fifoBuffer);
    IMU.dmpGetGravity(&gravity, &q);
    IMU.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
  }
  // End Jeff Rowberg code
  return;
}

void print_data_to_file(void) {
#ifdef SDON
  // Order of outputs: Time, Yaw, Pitch, Roll, ax, ay, az, gx, gy, gz, RPM,
  // depth
  outputFile.print(duration);
  outputFile.print(",");
  outputFile.print(ypr[0]);
  outputFile.print(",");
  outputFile.print(ypr[1]);
  outputFile.print(",");
  outputFile.print(ypr[2]);
  outputFile.print(",");
  outputFile.print(ax);
  outputFile.print(",");
  outputFile.print(ay);
  outputFile.print(",");
  outputFile.print(az);
  outputFile.print(",");
  outputFile.print(gx);
  outputFile.print(",");
  outputFile.print(gy);
  outputFile.print(",");
  outputFile.print(gz);
#ifdef TACHO
  outputFile.print(",");
  outputFile.print(rpm);
#endif

#ifdef DEPTHSENSOR
  outputFile.print(",");
  outputFile.print(depth);
#endif
  outputFile.println("");
#endif
  return;
}

void print_data_to_serial() {
  // Order of outputs: Time, Yaw, Pitch, Roll, ax, ay, az, gx, gy, gz, RPM,
  // depth
  Serial.print(duration);
  Serial.print(",");
  Serial.print(ypr[0]);
  Serial.print(",");
  Serial.print(ypr[1]);
  Serial.print(",");
  Serial.print(ypr[2]);
  Serial.print(",");
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
#ifdef TACHO
  Serial.print(",");
  Serial.print(rpm);
#endif

#ifdef DEPTHSENSOR
  Serial.print(",");
  Serial.print(depth);
#endif
  Serial.println("");
  return;
}

void send_subsee_data(void) {
#ifdef SUBSEERIAL
  // We send data as JSON, here manually constructed
  {
	"time": time,
	"Yaw": ypr[0]
	"Pitch": ypr[1],
	"Roll": ypr[2],
	"gx": gx,
	"gy": gy,
	"gz": gz,
	"RPM": RPM,
	"depth": depth,
	//"battery": batterylevel,  // note that these are commented out as their variables do not exist yet
	//"motor": motor
 }
#endif
  return;
}
