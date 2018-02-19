
int dirPin1 = 3;
int stepperPin1 = 2;
int dirPin2 = 7;
int stepperPin2 = 6;
unsigned long prevperiodY = 0;
unsigned long prevperiodP = 0;
int stepperPin1state = LOW;
int stepperPin2state = LOW;
void setup() {
  pinMode(dirPin1, OUTPUT);
  pinMode(stepperPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepperPin2, OUTPUT);
  //  unsigned long prevperiodY = 0;
  //  unsigned long prevperiodP = 0;
  //  int stepperPin1state = LOW;
  //  int stepperPin2state = LOW;
}
/*
  void step(boolean dir,int steps){
  digitalWrite(dirPin1,dir);
  digitalWrite(dirPin2,dir);
  delay(50);
  for(int i=0;i<steps;i++){
    digitalWrite(stepperPin1, HIGH);
    digitalWrite(stepperPin2, HIGH);
    delayMicroseconds(500); //defines speed
    digitalWrite(stepperPin1, LOW);
    digitalWrite(stepperPin2, LOW);
    delayMicroseconds(500); //defines speed
  }
  }
*/
//// ================================================================
//// ===                      STEP FUNCTION                   ===
//// ================================================================
//void step(boolean dirY, int stepsY, unsigned long periodY, boolean dirP, int stepsP, unsigned long periodP) {
//  digitalWrite(dirPin1, dirY);
//  digitalWrite(dirPin2, dirP);
//  delay(50);
//  for (int i = 0; i < stepsY; i++) {
//    unsigned long currentperiodY = millis();
//    if (currentperiodY - prevperiodY > periodY) {
//      // save the last time you switched poles
//      prevperiodY = currentperiodY;
//
//      // if stepperPin1 is off turn it on and vice-versa:
//      if (stepperPin1state == LOW)
//        stepperPin1state = HIGH;
//      else
//        stepperPin1state = LOW;
//
//      // set stepperPin1 with the state of the variable:
//      digitalWrite(stepperPin1, stepperPin1state);
//    }
//    for (int i = 0; i < stepsP; i++) {
//      unsigned long currentperiodP = millis();
//      if (currentperiodP - prevperiodP > periodP) {
//        // save the last time you switched poles
//        prevperiodP = currentperiodP;
//
//        // if stepperPin2 is off turn it on and vice-versa:
//        if (stepperPin2state == LOW)
//          stepperPin2state = HIGH;
//        else
//          stepperPin2state = LOW;
//
//        // set stepperPin2 with the state of the variable:
//        digitalWrite(stepperPin2, stepperPin2state);
//      }
//      //      digitalWrite(stepperPin1, HIGH);
//      //      delayMicroseconds(periodY); //defines speed, nominally 100(fast) -> 1000(slow)
//      //      digitalWrite(stepperPin1, LOW);
//      //      delayMicroseconds(periodY); //defines speed
//      //    }
//      //    for (int i = 0; i < stepsP; i++) {
//      //      digitalWrite(stepperPin2, HIGH);
//      //      delayMicroseconds(periodP); //defines speed, nominally 100(fast) -> 1000(slow)
//      //      digitalWrite(stepperPin2, LOW);
//      //      delayMicroseconds(periodP); //defines speed
//      //    }
//    }
//  }
//}
void step(boolean dirY, int stepsY,
          boolean dirP, int stepsP) {
    digitalWrite(dirPin1, dirY);
    digitalWrite(dirPin2, dirP);
    delay(50);
    int max_steps = max(stepsY, stepsP);
    for (int i = 0; i < max_steps; i++) {
        if (i < stepsY) digitalWrite(stepperPin1, HIGH);
        if (i < stepsP) digitalWrite(stepperPin2, HIGH);
        delayMicroseconds(75);
        if (i < stepsY) digitalWrite(stepperPin1, LOW);
        if (i < stepsP) digitalWrite(stepperPin2, LOW);
        delayMicroseconds(75);
    }
}
void loop() {
  step(true, 1600, false, 400); //true = direction, number = number of steps, 1600/rev
  delay(50); //delay between forwards and backwards, milliseconds
  step(false, 800, true, 3200);
  delay(50);
}
