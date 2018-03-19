const int green = 11;
const int red = 12;
void setup() {
  // put your setup code here, to run once:

pinMode(red,OUTPUT);
pinMode(green,OUTPUT);
}

void loop() {
digitalWrite(red,HIGH);
delay(500);
digitalWrite(red,LOW);

digitalWrite(green,HIGH);
delay(500);
digitalWrite(green,LOW);

digitalWrite(red,HIGH);
delay(500);
digitalWrite(red,LOW);


}
