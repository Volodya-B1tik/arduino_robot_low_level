#include <ThreadController.h>
#include <Thread.h>
#include <StaticThreadController.h>
#include "pinouts.h"
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include "RotaryEncoder.h"
#include <sensor_msgs/Imu.h>
#include "Simple_MPU6050.h"
#define MPU6050_DEFAULT_ADDRESS 0x68  // address pin low (GND)

Simple_MPU6050 mpu;

ros::NodeHandle nh;

geometry_msgs::Vector3 sonar_msg;
std_msgs::Float32 enco_left;
sensor_msgs::Imu Imu_msg;
ros::Publisher imu_pub("imu/data", &Imu_msg);
ros::Publisher enco("enco", &enco_left);
ros::Publisher sonar("sonar", &sonar_msg);

Thread LeftThread = Thread();
Thread Midhread = Thread();
Thread RightThread = Thread();
Thread Encoders_right = Thread();
Thread IMU = Thread();

RotaryEncoder encoder(inA1, inB1);

void imu_data(int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);

  Imu_msg.linear_acceleration.x = accel[0];
  Imu_msg.linear_acceleration.y = accel[1];
  Imu_msg.linear_acceleration.z = accel[2];

  Imu_msg.angular_velocity.x = gyro[0];
  Imu_msg.angular_velocity.y = gyro[1];
  Imu_msg.angular_velocity.z = gyro[2];

  Imu_msg.orientation.w = q.w;
  Imu_msg.orientation.x = q.x;
  Imu_msg.orientation.y = q.y;
  Imu_msg.orientation.z = q.z;

  imu_pub.publish(&Imu_msg);
 // nh.spinOnce();
}

void setup() {

  nh.initNode();
  nh.advertise(sonar);
  nh.advertise(enco);
  nh.advertise(imu_pub);
  nh.getHardware()->setBaud(115200);

  mpu.begin();

  mpu.Set_DMP_Output_Rate_Hz(10);       // Set the DMP output rate from 200Hz to 5 Minutes.
  mpu.Set_DMP_Output_Rate_Seconds(10);  // Set the DMP output rate in Seconds
  mpu.Set_DMP_Output_Rate_Minutes(5);   // Set the DMP output rate in Minute
  mpu.CalibrateMPU();                   // Calibrates the MPU.
  mpu.load_DMP_Image();                 // Loads the DMP image into the MPU and finish configuration.
  mpu.on_FIFO(imu_data);

  LeftThread.onRun(Left);
  LeftThread.setInterval(1);
  Midhread.onRun(Mid);
  Midhread.setInterval(1);
  RightThread.onRun(Right);
  RightThread.setInterval(1);
  IMU.onRun(imu_data);
  IMU.setInterval(10000);

  Encoders_right.onRun(Enco_right);
  Encoders_right.setInterval(10);

  pinMode(inA1, INPUT);             // Пины в режим приема INPUT
  pinMode(inB1, INPUT);             // Пины в режим приема INPUT
  attachInterrupt(0, A_1, CHANGE);  // Настраиваем обработчик прерываний по изменению сигнала
  attachInterrupt(1, B_1, CHANGE);  // Настраиваем обработчик прерываний по изменению сигнала

  //Serial.begin(9600);
  pinMode(trigPin_left, OUTPUT);
  pinMode(echoPin_left, INPUT);
  pinMode(trigPin_mid, OUTPUT);
  pinMode(echoPin_mid, INPUT);
  pinMode(trigPin_right, OUTPUT);
  pinMode(echoPin_right, INPUT);
}

void loop() {
  if (LeftThread.shouldRun())
    LeftThread.run();

  if (Midhread.shouldRun())
    Midhread.run();

  if (RightThread.shouldRun())
    RightThread.run();

  if (IMU.shouldRun())
    IMU.run();

  currentMillis = millis();
  if (currentMillis - prevMillis >= 10) {
    prevMillis = currentMillis;
    mpu.dmp_read_fifo(false);
  }


  sonar_msg.x = left;
  sonar_msg.y = mid;
  sonar_msg.z = right;
  sonar.publish(&sonar_msg);

  if (Encoders_right.shouldRun())
    Encoders_right.run();

  if (actualcount != count) {  // Чтобы не загружать ненужным выводом в Serial, выводим состояние
    actualcount = count;
    //count=count/80;               // счетчика только в момент изменения
    enco_left.data = count / 80;
    enco.publish(&enco_left);
    //
  }

  nh.spinOnce();
}

void Left() {
  unsigned long start = millis();
  unsigned long eend = 0;
  unsigned long duration = 0;
  digitalWrite(trigPin_left, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_left, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_left, LOW);
  duration_left = pulseIn(echoPin_left, HIGH, 5000);
  eend = millis();
  duration = eend - start;
  if (duration < 5) {
    distance_left = duration_left / 58;
    left = distance_left;
  } else
    left = 200;
}

void Mid() {
  unsigned long start_m = millis();
  unsigned long eend_m = 0;
  unsigned long duration_m = 0;

  digitalWrite(trigPin_mid, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_mid, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_mid, LOW);
  duration_mid = pulseIn(echoPin_mid, HIGH, 5000);
  eend_m = millis();

  duration_m = eend_m - start_m;

  if (eend_m - start_m < 5) {
    distance_mid = duration_mid / 58;
    mid = distance_mid;
  } else
    mid = 200;
}


void Right() {
  unsigned long start_r = millis();
  unsigned long eend_r = 0;
  unsigned long duration_r = 0;
  digitalWrite(trigPin_right, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_right, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_right, LOW);
  duration_right = pulseIn(echoPin_right, HIGH, 5000);
  eend_r = millis();
  duration_r = eend_r - start_r;
  if (duration_r < 5) {
    distance_right = duration_right / 58;
    right = distance_right;
  } else
    right = 200;
}

void A_1() {
  if (micros() - lastTurn < pause) return;  // Если с момента последнего изменения состояния не прошло
  // достаточно времени - выходим из прерывания
  inA1Value = digitalRead(inA1);  // Получаем состояние пинов A и B
  inB1Value = digitalRead(inB1);

  cli();  // Запрещаем обработку прерываний, чтобы не отвлекаться
  if (state == 0 && !inA1Value && inB1Value || state == 2 && inA1Value && !inB1Value) {
    state += 1;  // Если выполняется условие, наращиваем переменную state
    lastTurn = micros();
  }
  if (state == -1 && !inA1Value && !inB1Value || state == -3 && inA1Value && inB1Value) {
    state -= 1;  // Если выполняется условие, наращиваем в минус переменную state
    lastTurn = micros();
  }
  setCount(state);  // Проверяем не было ли полного шага из 4 изменений сигналов (2 импульсов)
  sei();            // Разрешаем обработку прерываний

  if (inA1Value && inB1Value && state != 0) state = 0;  // Если что-то пошло не так, возвращаем статус в исходное состояние
}
void B_1() {
  if (micros() - lastTurn < pause) return;
  inA1Value = digitalRead(inA1);
  inB1Value = digitalRead(inB1);

  cli();
  if (state == 1 && !inA1Value && !inB1Value || state == 3 && inA1Value && inB1Value) {
    state += 1;  // Если выполняется условие, наращиваем переменную state
    lastTurn = micros();
  }
  if (state == 0 && inA1Value && !inB1Value || state == -2 && !inA1Value && inB1Value) {
    state -= 1;  // Если выполняется условие, наращиваем в минус переменную state
    lastTurn = micros();
  }
  setCount(state);  // Проверяем не было ли полного шага из 4 изменений сигналов (2 импульсов)
  sei();

  if (inA1Value && inB1Value && state != 0) state = 0;  // Если что-то пошло не так, возвращаем статус в исходное состояние
}

void setCount(int state) {          // Устанавливаем значение счетчика
  if (state == 4 || state == -4) {  // Если переменная state приняла заданное значение приращения
    count += (int)(state / 4);      // Увеличиваем/уменьшаем счетчик
    lastTurn = micros();            // Запоминаем последнее изменение
  }
}

void Enco_right() {

  current_state_right = digitalRead(inB);
  if (current_state_right != pre_state_right) {
    if (digitalRead(inA) > pre_state_right) {
      counter_right++;
      Serial.print("Value_Right: ");
      Serial.println(counter_right);
      pre_state_right = current_state_right;
    }
    if (digitalRead(inA) < pre_state_right) {
      counter_right--;
      Serial.print("Value_Right: ");
      Serial.println(counter_right);
      pre_state_right = current_state_right;
    }
  }
}
