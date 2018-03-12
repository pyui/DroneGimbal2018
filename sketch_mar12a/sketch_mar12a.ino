#include "Wire.h"

void setup() {
  Serial.begin(9600);
}

void loop() {
  //=== check for Leica data via serial
    //Leica_Prev = Leica_Data; //store previous Leica co-ordinates
    if (Serial.available()) {
      char Leica_Data = Serial.read(); // receive data from VS code; this should be a vector with r,z,theta in m and deg respectively
    Serial.println(Leica_Data);
    //yawLeica = Leica_Data[0];
    //pitchLeica = Leica_Data[1];
   //netYawLeica = yawLeica - Leica_Prev[0];
   //netPitchLeica = pitchLeica - Leica_Prev[1];
    }

}
