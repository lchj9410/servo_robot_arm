#include <Servo.h>
Servo myservo;
int inByte = 0;
int buf[16]; // 2x7+1+1 .  7 motor 1 start, 1 dump
int num,val;  
int index=15;
float serial_in[7];    
void setup() {
  myservo.attach(9);
  Serial.begin(19200);
  while (!Serial)
  {
    ; 
  }
  delay(1000);
  myservo.write(90);
  
}

void loop()
{
    if (Serial.available()>0)
    {
      num= Serial.read();
      if (num==255) {index=0;}// if byte = 255, start
      else 
      {
        buf[index]=num;
        index+=1;
        if (index>15) index=15;
        }
     for (int i=0;i<7;i++)
     {
      if (index>2*i+1 && index<15) serial_in[i]=buf[2*i]+buf[2*i+1]*0.1; 
      }

     }
     
 
}
