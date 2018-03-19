/* =====================================================================================
    EM401 Drone Gimbal Motor Demo
    Copyright (c) Paul Yuile 2018 University of Strathclyde
    Student Number: 201416902

    This code allows 2 stepper motors to be controlled simultaneously in different directions/
    number of steps. 

    The code is setup such that each full rotation is subdivided into 1600 steps. As the motors
    used have 400(yaw) and 200(pitch) steps/rev, the user must be aware of the conversion that
    will take place for each motor when controlling them.

    The code uses some aspects of Edgar Bonet's code for running two motors simultaneously: 
    https://arduino.stackexchange.com/questions/20384/controlling-3-stepper-motors-simultaneously
   =====================================================================================
*/
// ================================================================
// ===                    INITILISE PARAMETERS                  ===
// ================================================================
int dirPinP = 3;
int stepperPinP = 2;
int dirPinY = 7;
int stepperPinY = 6;
// ================================================================
// ===                    SETUP                                 ===
// ================================================================
void setup() {
  pinMode(dirPinY, OUTPUT);
  pinMode(stepperPinY, OUTPUT);
  pinMode(dirPinP, OUTPUT);
  pinMode(stepperPinP, OUTPUT);
}
// ================================================================
// ===                    STEP FUNCTION                         ===
// ================================================================
/* By looping to the max steps of yaw/pitch motor at a set speed,
 * the motor that moves the least distance will sit and wait until
 * the other motor is finished.
 */
void step(boolean dirY, int stepsY,
          boolean dirP, int stepsP) {
    digitalWrite(dirPinY, dirY);
    digitalWrite(dirPinP, dirP);
    delay(50);
    int max_steps = max(stepsY, stepsP);
    for (int i = 0; i < max_steps; i++) {
        if (i < stepsY) digitalWrite(stepperPinY, HIGH);
        if (i < stepsP) digitalWrite(stepperPinP, HIGH);
        delayMicroseconds(150); //defines motor speed
        if (i < stepsY) digitalWrite(stepperPinY, LOW);
        if (i < stepsP) digitalWrite(stepperPinP, LOW);
        delayMicroseconds(150);
    }
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  step(true, 800, false, 800); //true = direction, number = number of steps, 1600/rev
  delay(500); //delay between forwards and backwards, milliseconds
  step(false, 800, true, 800);
  delay(500);
}
