
void setup()  
{  
    
  Serial.begin(9600);  
  //pinMode(13, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);  
}  
  
  
  
void loop()  
{  
  if(Serial.available())  
  {  
 char  data=Serial.read();  
  Serial.println(data);  
  switch(data)  
  {  
    case 'O':digitalWrite(LED_BUILTIN, HIGH);  
    break;  
    case 'F':digitalWrite(LED_BUILTIN, LOW);  
    break;  
     
  }  
  }  
}   
