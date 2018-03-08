//#include <Arduino.h>
#include "MedianFilter.h"

MedianFilter test1(5, 0);

int i=0;
int j;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.println("*** Program Start ***");
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly: 
  i = int(random(-9, 9));
  test1.in(i);
  j = test1.out();
  
  Serial.print(i);
  Serial.print("\t");
  Serial.println(j);

  //i++;

  delay(500);
  
}
