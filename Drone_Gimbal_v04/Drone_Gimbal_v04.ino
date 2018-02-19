/*
  Example Timer1 Interrupt
  Flash LED every second
*/

#define ledPin 13
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;            // class default I2C address is 0x68
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int timer1_counter;
bool flag = 0;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypPREV[2] = {0, 0};
float netYaw;
float netPitch;
float pitchStep;
float yawStep;
bool yawDir;
bool pitchDir;

// ================================================================
// ===               STEPPER DEFINITIONS                        ===
// ================================================================

int dirPinP = 3;
int stepperPinP = 2;
int dirPinY = 7;
int stepperPinY = 6;

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
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200); // initialize serial communication

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

  // define stepper control outputs
  pinMode(dirPinY, OUTPUT);
  pinMode(stepperPinY, OUTPUT);
  pinMode(dirPinP, OUTPUT);
  pinMode(stepperPinP, OUTPUT);

  // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

// ================================================================
// ===                    MOTOR INTERRUPT                       ===
// ================================================================

ISR(TIMER1_OVF_vect)        // interrupt service routine
{
  TCNT1 = timer1_counter;   // preload timer
  //digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
  Serial.print("interrupt\n");
  flag = 1;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
  if (flag == 1) {
    flag = 0;
    Serial.println("loop");

    while (fifoCount < packetSize) {
      ypPREV[0] = ypr[0];
      ypPREV[1] = ypr[1];
      Serial.println("topwhileloop");
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless code is too inefficient)
      if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
      }
      else {
        while (fifoCount >= packetSize) {

          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
          Serial.println("whileloop");

        }
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        netYaw = (ypr[0]) - (ypPREV[0]);
        netPitch = (ypr[1]) - (ypPREV[1]);
        Serial.print("yp\t");
        Serial.print(netYaw * 180 / M_PI);
        Serial.print("\t");
        Serial.print(netPitch * 180 / M_PI);
        Serial.print("\n");
        mpu.resetFIFO();
        Serial.println("elseresetFIFO\n");

        if (((netYaw * 180 / M_PI) > -1.8) && ((netYaw * 180 / M_PI) < 1.8)) {
          yawStep = 0;
          Serial.println("yawStepIF");
        }
        else {
          Serial.println("yawStepELSE");
          //yawStep = 400;
          yawStep = ((int)(netYaw * 180 / M_PI) * (1600 / 360));
        }
        if (yawStep < 0) {
          yawDir = false;
          Serial.println("yawDirFALSE");
        }
        else {
          yawDir = true;
          Serial.println("yawDirTRUE");
        }



        if (((netPitch * 180 / M_PI) > -1.8) && ((netPitch * 180 / M_PI) < 1.8)) {
          pitchStep = 0;
          Serial.println("pitchStepIF");
        }
        else {
          //pitchStep = 400;
          pitchStep = ((int)(netPitch * 180 / M_PI) * (1600 / 360));
          Serial.println("pitchStepELSE");
        }
        if (pitchStep < 0) {
          pitchDir = false;
          Serial.println("pitchDirFALSE");
        }
        else {
          pitchDir = true;
          Serial.println("pitchDirTRUE");
        }
        Serial.print("yaw step\t");
        Serial.print(yawStep);
        Serial.print("\n");
        Serial.print("pitch step\t");
        Serial.print(pitchStep);
        Serial.print("\n");
        Serial.print("yaw dir\t");
        Serial.print(yawDir);
        Serial.print("\n");
        Serial.print("pitch dir\t");
        Serial.print(pitchDir);
        Serial.print("\n");
        step(yawDir, abs(yawStep), pitchDir, abs(pitchStep));
        Serial.println("turn motors");
      }

    }

  }
  Serial.println("outwith interrupt within main");
}




