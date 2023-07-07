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
std_msgs::Float32 enco_right;
sensor_msgs::Imu Imu_msg;
ros::Publisher imu_pub("imu/data", &Imu_msg);
ros::Publisher lwheel("lwheel", &enco_left);
ros::Publisher rwheel("rwheel", &enco_right);
ros::Publisher sonar("sonar", &sonar_msg);

Thread LeftThread = Thread();
Thread Midhread = Thread();
Thread RightThread = Thread();
Thread IMU = Thread();


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
  nh.advertise(lwheel);
  nh.advertise(rwheel);
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


    pinMode(ENC_A_l, INPUT);
    pinMode(ENC_B_l, INPUT);
    pinMode(ENC_A_r, INPUT);
    pinMode(ENC_B_r, INPUT); 

    state_a_l = (boolean) digitalRead(ENC_A_l);
    state_b_l = (boolean) digitalRead(ENC_B_l);
    state_a_r = (boolean) digitalRead(ENC_A_r);
    state_b_r = (boolean) digitalRead(ENC_B_r);

    attachInterrupt(4, interrupt_enc_a_l, CHANGE);
    attachInterrupt(5, interrupt_enc_b_l, CHANGE);
    attachInterrupt(0, interrupt_enc_a_r, CHANGE);
    attachInterrupt(1, interrupt_enc_b_r, CHANGE);  

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

    enc_pos_change_l = enc_pos_l - enc_pos_prev_l;
    enc_pos_change_l = abs(enc_pos_change_l);
    enc_pos_change_r = enc_pos_r - enc_pos_prev_r;
    enc_pos_change_r = abs(enc_pos_change_r);

    enco_left.data = enc_pos_l;
    lwheel.publish(&enco_left);
    enco_right.data = enc_pos_r;
    rwheel.publish(&enco_right);

    enc_pos_prev_l = enc_pos_l;
    enc_pos_prev_r = enc_pos_r;

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

void interrupt_enc_a_l()
{
    if (!state_a_l) {
        state_b_l ? enc_pos_l++: enc_pos_l--;         
    }
    state_a_l = !state_a_l;
}

void interrupt_enc_b_l()  
{
    state_b_l = !state_b_l;
}

void interrupt_enc_a_r()
{
    if (!state_a_r) {
        state_b_r ? enc_pos_r++: enc_pos_r--;         
    }
    state_a_r = !state_a_r;
}

void interrupt_enc_b_r()  
{
    state_b_r = !state_b_r;
}

