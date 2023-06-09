#include "pin_joy.h"
#include <Wire.h>                     // Подключаем библиотеку Wire
#include <Adafruit_PWMServoDriver.h>  // Подключаем библиотеку Adafruit_PWMServoDriver
#include <ros.h>
#include <sensor_msgs/Joy.h>

sensor_msgs::Joy test_test;

int gototh;
void joy(const sensor_msgs::Joy& joy)
{
  if (joy.buttons[0] == 1)
    gototh = 4;  
  else if (joy.buttons[1] == 1)
    gototh = 2;
  else if (joy.buttons[2] == 1)
    gototh = 6;
  else if (joy.buttons[3] == 1)
    gototh = 8;
  else
    gototh = 5;
}

ros::Subscriber<sensor_msgs::Joy> sub("joy", &joy);
ros::NodeHandle nh;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // Установка адреса I2C 0x40

void setup() 
{
  Wire.begin();
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.subscribe(sub);

  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() 
{
  nh.spinOnce();

  if (gototh == 8) //Вперед
   {
    pwm.setPWM(EnA, 1, 4095);
    pwm.setPWM(In_1, 1, 4095);
    pwm.setPWM(In_2, 4095, 1);
    pwm.setPWM(In_3, 4095, 1);
    pwm.setPWM(In_4, 1, 4095);
    pwm.setPWM(EnB, 1, 4095);
  }

  if (gototh == 5)  // Стоп
  {
    pwm.setPWM(EnA, 0, 0);
    pwm.setPWM(In_1, 0, 0);
    pwm.setPWM(In_2, 0, 0);
    pwm.setPWM(In_3, 0, 0);
    pwm.setPWM(In_4, 0, 0);
    pwm.setPWM(EnB, 0, 0);
  }

  if (gototh == 4) // Влево
  {
    pwm.setPWM(EnA, 1, 4095);
    pwm.setPWM(In_1, 1, 4095);
    pwm.setPWM(In_2, 4095, 1);
    pwm.setPWM(In_3, 1, 4095);
    pwm.setPWM(In_4, 4095, 1);
    pwm.setPWM(EnB, 1, 4095);
  }

  if (gototh == 6) // Вправо
  {
    pwm.setPWM(EnA, 1, 4095);
    pwm.setPWM(In_1, 4095, 1);
    pwm.setPWM(In_2, 1, 4095);
    pwm.setPWM(In_3, 4095, 1);
    pwm.setPWM(In_4, 1, 4095);
    pwm.setPWM(EnB, 1, 4095);
  }

  if (gototh == 2) // Назад
   {
    pwm.setPWM(EnA, 1, 4095);
    pwm.setPWM(In_1, 4095, 1);
    pwm.setPWM(In_2, 1, 4095);
    pwm.setPWM(In_3, 1, 4095);
    pwm.setPWM(In_4, 4095, 1);
    pwm.setPWM(EnB, 1, 4095);
  }
}
