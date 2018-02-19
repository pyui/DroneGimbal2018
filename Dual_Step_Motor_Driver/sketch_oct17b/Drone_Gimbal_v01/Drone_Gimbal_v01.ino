/* =====================================================================================
    EM401 Drone Gimbal
    Copyright (c) Paul Yuile 2018 University of Strathclyde
    Student Number: 201416902

    This code accepts the input of an MPU6050 IMU (ypr format) and merges it with the
    cylindrical output coordinates of the Leica AT901B laser system to orient
    the drone-mounted reflecter by correcting unwanted motion in the yaw and
    pitch axes.

    The code uses the following sources:
    Jeff Rowberg: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
    Forum user batata004: https://forum.arduino.cc/index.php?topic=408980.0
    Edgar Bonet: https://arduino.stackexchange.com/questions/20384/controlling-3-stepper-motors-simultaneously
   =====================================================================================
*/
// ================================================================
// ===               IMU DEFINITIONS                            ===
// ================================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;            // class default I2C address is 0x68
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypPREV[2] = {0, 0};
float netYaw;
float netPitch;

// ================================================================
// ===               WIFI DEFINITIONS                            ===
// ================================================================

#include <CytronWiFiShield.h>
#include <CytronWiFiClient.h>
#include <CytronWiFiServer.h>
#include <SoftwareSerial.h>

const char *ssid = "TALKTALK1BB74D";
const char *pass = "CPDJUDEE";
IPAddress ip(192, 168, 1 ,242);
ESP8266Server server(80);

const char htmlHeader[] = "HTTP/1.1 200 OK\r\n"
                        "Content-Type: text/html\r\n"
                        "Connection: close\r\n\r\n"
                        "<!DOCTYPE HTML>\r\n"
                        "<html>\r\n";
                        
// ================================================================
// ===               STEPPER DEFINITIONS                        ===
// ================================================================

int dirPinY = 3;
int stepperPinY = 2;
int dirPinP = 7;
int stepperPinP = 6;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  Wire.begin();                  // join I2C bus
  TWBR = 24;                     // 400kHz I2C clock (200kHz if CPU is 8MHz)
  mpu.initialize();              // initialise device
  mpu.dmpInitialize();           // load and configure DMP
  mpu.setXAccelOffset(-1343);
  mpu.setYAccelOffset(-1155);
  mpu.setZAccelOffset(1033);
  mpu.setXGyroOffset(19);
  mpu.setYGyroOffset(-27);
  mpu.setZGyroOffset(16);
  mpu.setDMPEnabled(true);      // turn on DMP
  packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
  fifoCount = mpu.getFIFOCount();

  Serial.begin(115200); // initialize serial communication

  // define stepper control outputs
  pinMode(dirPinY, OUTPUT);
  pinMode(stepperPinY, OUTPUT);
  pinMode(dirPinP, OUTPUT);
  pinMode(stepperPinP, OUTPUT);
}
// ================================================================
// ===                    STEP FUNCTION                         ===
// ================================================================
/* By looping to the max steps of yaw/pitch motor at a set speed,
   the motor that moves the least distance will sit and wait until
   the other motor is finished.
*/
void step(boolean dirY, int stepsY,
          boolean dirP, int stepsP) {
  digitalWrite(dirPinY, dirY);
  digitalWrite(dirPinP, dirP);
  delay(45);
  int max_steps = max(stepsY, stepsP);
  for (int i = 0; i < max_steps; i++) {
    if (i < stepsY) digitalWrite(stepperPinY, HIGH);
    if (i < stepsP) digitalWrite(stepperPinP, HIGH);
    delayMicroseconds(75); //defines motor speed
    if (i < stepsY) digitalWrite(stepperPinY, LOW);
    if (i < stepsP) digitalWrite(stepperPinP, LOW);
    delayMicroseconds(75);
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  while (fifoCount < packetSize) {
//    ypPREV[0] = ypr[0];
//    ypPREV[1] = ypr[1];
    step(true, 400, true, 1600);
    //    if (ypr[0] > 0) {
    //      step(true, 400, false, 0);
    //    }
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless code is too inefficient)
    if (fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
    }
    else {

      if (fifoCount % packetSize != 0) {

        mpu.resetFIFO();

      }
      else {

        while (fifoCount >= packetSize) {

          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;

        }
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//        netYaw = (ypr[0]) - (ypPREV[0]);
//        netPitch = (ypr[1]) - (ypPREV[1]);
//        Serial.print("yp\t");
//        Serial.print(netYaw * 180 / M_PI);
//        Serial.print("\t");
//        Serial.print(netPitch * 180 / M_PI);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
      }
    }
  }
}
