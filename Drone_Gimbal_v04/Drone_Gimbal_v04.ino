/* =====================================================================================
    EM401 Drone Gimbal
    Copyright (c) Paul Yuile 2018 University of Strathclyde
    Student Number: 201416902

    This code accepts the input of an MPU6050 IMU (ypr format) and merges it with the
    output coordinates of the Leica AT901B laser system to orient
    the drone-mounted reflecter by correcting unwanted motion in the yaw and
    pitch axes.

    The code uses the following sources:
    Jeff Rowberg: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
    Forum user batata004: https://forum.arduino.cc/index.php?topic=408980.0
    Edgar Bonet: https://arduino.stackexchange.com/questions/20384/controlling-3-stepper-motors-simultaneously
    Phillip Schmidt: https://github.com/daPhoosa/MedianFilter
   =====================================================================================
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "MedianFilter.h"

MPU6050 mpu;            // class default I2C address is 0x68, AD0 pin not in use
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int timer1_counter;     // defines timer frequency
bool flag = 0;          // timer flag
float ypr[3];// = {0, 0, 0};           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypPREV[2] = {0, 0}; // used to store previous ypr values
float netYaw;           // ypr[x1]-ypPREV[y1]
float netPitch;         // ypr[x2]-ypPREV[y2]
float pitchStep;        // defines number of steps for pitch motor
float yawStep;          // defines number of steps for yaw motor
bool yawDir;            // yaw motor direction, T/F
bool pitchDir;          // pitch motor direction, T/F
MedianFilter yawmed(3, 0);
MedianFilter pitmed(3, 0);
int y = 0;
int p = 0;
float y_out_f = 0;
float p_out_f = 0;
// ================================================================
// ===               STEPPER DEFINITIONS                        ===
// ================================================================

int dirPinY = 3;        // X motor on shield
int stepperPinY = 2;    // X motor on shield
int dirPinP = 7;        // Y motor on shield
int stepperPinP = 6;    // Y motor on shield

// ================================================================
// ===                    STEP FUNCTION                         ===
// ================================================================
/* By looping to the max steps of yaw/pitch motor at a set speed,
   the motor that moves the least distance will sit and wait until
   the other motor is finished.
*/
void step(boolean dirY, int stepsY,
          boolean dirP, int stepsP) {
  digitalWrite(dirPinY, dirY);  // set yaw motor direction based on boolean input
  digitalWrite(dirPinP, dirP);  // same for pitch motor
  delay(7);
  int max_steps = max(stepsY, stepsP); // which steps variable is biggest?
  for (int i = 0; i < max_steps; i++) {   // ...count to largest steps variable
    if (i < stepsY) digitalWrite(stepperPinY, HIGH); // stop motor when it reaches relevant steps value
    if (i < stepsP) digitalWrite(stepperPinP, HIGH);
    delayMicroseconds(75);      // defines motor speed, 50us is fast, 500us is slow
    if (i < stepsY) digitalWrite(stepperPinY, LOW);
    if (i < stepsP) digitalWrite(stepperPinP, LOW);
    delayMicroseconds(75);      // should be the same as the above
  }
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Serial.begin(9600);//115200); // initialize serial communication


  Wire.begin();                  // join I2C bus
  TWBR = 24;                     // 400kHz I2C clock (200kHz if CPU is 8MHz)
  mpu.initialize();              // initialise device
  mpu.dmpInitialize();           // load and configure DMP
  mpu.setXAccelOffset(-1343);    // ignore these offsets - not important to the project
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
  timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 59286;   // preload timer 65536-16MHz/256/10Hz
  //timer1_counter = 53036; // preload timer 65536-16MHz/256/5Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

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
  //Serial.print("interrupt\n");
  flag = 1;                 // set flag high
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{

  // check for Leica data via serial
  //  Leica_Prev = Leica_Data; //store previous Leica co-ordinates
  //  if (Serial.available())
  //  {
  //    char Leica_Data = Serial.read(); // receive data from VS code; this should be a vector with r,z,theta in m and deg respectively
  //  Serial.println(Leica_Data);
  //  yawLeica = Leica_Data[0];
  //  pitchLeica = Leica_Data[1];
  // netYawLeica = yawLeica - Leica_Prev[0];
  // netPitchLeica = pitchLeica - Leica_Prev[1];
  //  }

  if (flag == 1) { // interrupt is active
    flag = 0; // reset the interrupt flag


    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount(); // get current FIFO count
    }

    //}

    if (fifoCount == 1024) { // check for overflow
      mpu.resetFIFO(); // reset if overflow
      //Serial.println("FIFO overflow!");
    }
    else {
      while (fifoCount >= packetSize) {
        //Serial.println("loop");
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize; // subtract packetSize from fifoCount
        //Serial.println("whileloop");

      }

      float y_out_prev = y_out_f;
      float p_out_prev = p_out_f;


      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.resetFIFO();
      //Serial.println("elseresetFIFO\n");
      Serial.print("Yaw\t");
      Serial.println(ypr[0]*180/PI);
      Serial.print("Pitch\t");
      Serial.println(ypr[1]*180/PI);
      Serial.print("Roll\t");
      Serial.println(ypr[2]*180/PI);
      Serial.println(" ");



      y = int((ypr[0] * 180 / M_PI) * 100); // yaw angle in degrees, multiplied by 100 to reduce errors when converting float to int
      yawmed.in(y); // input yaw value to median filter
      int y_out = yawmed.out(); // obtain median filter output
      y_out_f = float(y_out) / 100; // convert output to degree float value, remove 100x multiplier
      float netYaw = (y_out_f) - (y_out_prev); // obtain net yaw since last measurement
//      Serial.print("netYaw\t");
//      Serial.println(netYaw);

      p = int((ypr[2] * 180 / M_PI) * 100); // pitch angle in degrees, multiplied by 100 to reduce errors when converting float to int
      pitmed.in(p); // input pitch value to median filter
      int p_out = pitmed.out(); // obtain median filter output
      p_out_f = float(p_out) / 100; // convert output to degree float value, remove 100x multiplier
      float netPitch = (p_out_f) - (p_out_prev); // obtain net pitch since last measurement
//      Serial.print("netPitch\t");
//      Serial.println(netPitch);


      // if netYaw is less than equivalent of one step (1.8deg)
      if (((netYaw) > -1.8) && ((netYaw) < 1.8)) {
        yawStep = 0;  // don't step the motor
        //Serial.println("yawStepIF");
      }
      else {    // netYaw is greater than one step

        // motor shield has 1600steps/rev, so convert to this range
        yawStep = ((int)(netYaw) * (1600 / 360));
      }
      if (yawStep < 0) {    // if yawStep is negative
        yawDir = false;     // rotate motor backwards
        //Serial.println("yawDirFALSE");
      }
      else { // yawStep is positive
        yawDir = true; // rotate motor forwards
        //Serial.println("yawDirTRUE");
      }


      // if netPitch is less than equivalent of one step (1.8deg)
      if (((netPitch) > -1.8) && ((netPitch) < 1.8)) {
        pitchStep = 0; // don't step the motor
        //Serial.println("pitchStepIF");
      }
      else {
        // step pitch motor; must be an integer value for step function.
        // motor shield has 1600steps/rev, so convert to this range
        pitchStep = ((int)(netPitch) * (1600 / 360));
        //Serial.println("pitchStepELSE");
      }
      if (pitchStep < 0) {  // if pitchStep is negative
        pitchDir = false;   // rotate motor backwards
        //Serial.println("pitchDirFALSE");
      }
      else {    // pitchStep is positive
        pitchDir = true;  // rotate motor forwards
        //Serial.println("pitchDirTRUE");
      }

//      Serial.print("yaw step\t");
//      Serial.print(yawStep);
//      Serial.print("\n");
//      Serial.print("pitch step\t");
//      Serial.print(pitchStep);
//      Serial.print("\n");
//      Serial.print("yaw dir\t");
//      Serial.print(yawDir);
//      Serial.print("\n");
//      Serial.print("pitch dir\t");
//      Serial.print(pitchDir);
//      Serial.print("\n");
      step(yawDir, abs(yawStep), pitchDir, abs(pitchStep)); // control motors using calculated variables
      //step(yawDir, abs(yawStep), 1, 0); // control motors using calculated variables
      //      step(1, 0, pitchDir, abs(pitchStep*2)); //yaw motor has 0.9deg accuracy therefore needs double the number of steps
//      Serial.println("turn motors");
    }

  }
}





