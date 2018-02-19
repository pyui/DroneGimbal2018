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
int yawStep;
int pitchStep;
bool yawDir;
bool pitchDir;

// ================================================================
// ===               WIFI DEFINITIONS                            ===
// ================================================================

#include <CytronWiFiShield.h>
#include <CytronWiFiClient.h>
#include <CytronWiFiServer.h>
#include <SoftwareSerial.h>

const char *ssid = "TALKTALK1BB74D";
const char *pass = "CPDJUDEE";
IPAddress ip(192, 168, 1 , 242);
ESP8266Server server(80);

const char htmlHeader[] = "HTTP/1.1 200 OK\r\n"
                          "Content-Type: text/html\r\n"
                          "Connection: close\r\n\r\n"
                          "<!DOCTYPE HTML>\r\n"
                          "<html>\r\n";

// ================================================================
// ===               STEPPER DEFINITIONS                        ===
// ================================================================

int dirPinP = 3;
int stepperPinP = 2;
int dirPinY = 7;
int stepperPinY = 6;

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

  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
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
// ===                    MOTOR INTERRUPT                       ===
// ================================================================
ISR(TIMER1_COMPA_vect) { //change the 0 to 1 for timer1 and 2 for timer2
  //   while (fifoCount < packetSize) {
  //    ypPREV[0] = ypr[0];
  //    ypPREV[1] = ypr[1];
  //    Serial.println("topwhileloop");
  //    // get current FIFO count
  //    fifoCount = mpu.getFIFOCount();
  //
  //    // check for overflow (this should never happen unless code is too inefficient)
  //    if (fifoCount == 1024) {
  //      // reset so we can continue cleanly
  //      mpu.resetFIFO();
  //      Serial.println(F("FIFO overflow!"));
  //    }
  //    else {
  //
  //      while (fifoCount >= packetSize) {
  //
  //        mpu.getFIFOBytes(fifoBuffer, packetSize);
  //        fifoCount -= packetSize;
  //        Serial.println("whileloop\n");
  //
  //      }
  // display Euler angles in degrees
  ypPREV[0] = ypr[0];
  ypPREV[1] = ypr[1];
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  netYaw = (ypr[0]) - (ypPREV[0]);
  netPitch = (ypr[1]) - (ypPREV[1]);
  //Serial.print("breakpoint0");
  //Serial.print("yp\t");
  //Serial.print("breakpoint1");
  Serial.print(netYaw * 180 / M_PI);
  Serial.print("\t");
  Serial.print(netPitch * 180 / M_PI);
  Serial.print("\n");
  mpu.resetFIFO();
  Serial.println("elseresetFIFO\n");


  if (((netYaw * 180 / M_PI) > -1.8) && ((netYaw * 180 / M_PI) < 1.8)) {
    yawStep = 0;
    Serial.print("yawStepIF\n");
  }
  else {
    //yawStep = 400;
    yawStep = (netYaw * 180 / M_PI) * (360 / 1600);
    if (yawStep < 0) {
      yawDir = false;
    }
    else {
      yawDir = true;
    }
    Serial.print("yawStepELSE\n");
  }

  if (((netPitch * 180 / M_PI) > -1.8) && ((netPitch * 180 / M_PI) < 1.8)) {
    pitchStep = 0;
    Serial.print("pitchStepIF\n");
  }
  else {
    //pitchStep = 400;
    pitchStep = (netPitch * 180 / M_PI) * (360 / 1600);
    Serial.print("pitchStepELSE\n");
    if (pitchStep < 0) {
      pitchDir = false;
    }
    else {
      pitchDir = true;
    }
    step(yawDir, yawStep, pitchDir, pitchStep);
    Serial.print("turn motors\n");
  }
}




// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  Serial.print("main program loop\n");
}
