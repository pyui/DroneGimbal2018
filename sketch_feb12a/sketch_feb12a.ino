
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
int dirPinY = 3;
int stepperPinY = 2;
int dirPinP = 7;
int stepperPinP = 6;

void setup() {

    Wire.begin();
    TWBR = 24;
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(-1343);
    mpu.setYAccelOffset(-1155);
    mpu.setZAccelOffset(1033);
    mpu.setXGyroOffset(19);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(16);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();

    Serial.begin(115200);
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
void loop() {

    while (fifoCount < packetSize) {

        //insert here your code
        Serial.print("motor\n");
        
        //step(true, 400, true, 1600);
     

        fifoCount = mpu.getFIFOCount();

    }

    if (fifoCount == 1024) {
    
        
        Serial.println(F("FIFO overflow!"));
        mpu.resetFIFO();
        
    }
    else{
    
      if (fifoCount % packetSize != 0) {
        
          mpu.resetFIFO();
            
    }
        else{
    
            while (fifoCount >= packetSize) {
            
                mpu.getFIFOBytes(fifoBuffer,packetSize);
                fifoCount -= packetSize;
                
            }    
        
            mpu.dmpGetQuaternion(&q,fifoBuffer);
            mpu.dmpGetGravity(&gravity,&q);
            mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);          
            
            Serial.print("ypr\t");
            Serial.print(ypr[0]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[1]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[2]*180/PI);
            Serial.println();
            
    }
   
    }

}
