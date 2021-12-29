
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define hz50 50
#define hz333 333
void setup() {
  pwm.begin();
  pwm.setPWMFreq(hz50);

}
void motor(int ang)
{
 float pulse_width27,pulse_width;
// pulse_wide = map(ang, 0,180,minp,maxp);
// pulse_width = int(float(pulse_wide)/1000000*Feq*4095);

pulse_width = map(ang, 0,180,102.4,512);
pulse_width=int(pulse_width);
pulse_width27 = map(ang, 0,270,102.4,512);
pulse_width27=int(pulse_width27);
//pulse_width3 = map(ang, 0,180,102.4,512);
//pulse_width3=int(pulse_width3);
 pwm.setPWM(0,0,pulse_width);
 pwm.setPWM(1,0,pulse_width);
 pwm.setPWM(2,0,pulse_width);
 pwm.setPWM(3,0,pulse_width);
 pwm.setPWM(4,0,pulse_width);
 pwm.setPWM(5,0,pulse_width);
  pwm.setPWM(6,0,pulse_width);
   pwm.setPWM(7,0,pulse_width);
 pwm.setPWM(8,0,pulse_width);
 pwm.setPWM(15,0,pulse_width27);
  }
  
void loop()
{
 motor(0);

}
