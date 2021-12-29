#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int buf[14],num,index=13; // 2x6+1+1 .  6 motor 1 start, 1 dump
float serial_in[6]={100,172.5,77.5,85,90,50}; 

void motor(float ser_in[6]){
  float pulse_width27,pulse_width;
  for (int i=0;i<6;i++){
    if (i!=1){
    pulse_width=map(ser_in[i], 0,180,102.4,512);
    pulse_width=int(pulse_width);
    pwm.setPWM(i,0,pulse_width);}
    else {
      pulse_width27 = map(ser_in[1], 0,270,102.4,512);
      pulse_width27=int(pulse_width27);
      pwm.setPWM(1,0,pulse_width27);}  }  }

void setup() { ////////////////////SET UP//////////////////
  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.begin(115200);
  while (!Serial){;}
  delay(1000);}
      
void loop(){
  if (Serial.available()>0){
    num= Serial.read();
    if (num==255) {index=0;}// if byte = 255, start
    else {
      buf[index]=num;
      index+=1;
      if (index>13) index=13;}
    for (int i=0;i<6;i++){
      if (index>2*i+1 && index<13) serial_in[i]=buf[2*i]+buf[2*i+1]*0.01; }
  }
  motor(serial_in); 
}
