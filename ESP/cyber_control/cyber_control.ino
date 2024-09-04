/*
CyberGearとの通信が確立したら，電流値を一定に保ちながら一定速度で回転させる．
CyberGearの角度を取得し続け，変化量がε以下になったら停止し，その角度を原点とする。
以上の動作をキャリブレーションとする．

その後，CyberGearは減速比に基づいて拡張位置制御を行う。

*/
#include <Arduino.h>
#include <math.h>
#include "src/cybergear_driver.hh"
#include "src/cybergear_can_interface_esp32.hh"

#define INC_POSITION  20.0
#define INC_VELOCITY  0.4
#define INC_TORQUE    0.04

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID = 0x7F;

// init cybergeardriver
CybergearDriver driver = CybergearDriver(MASTER_CAN_ID, MOT_CAN_ID);
MotorStatus motor_status;
CybergearCanInterfaceEsp32 interface;
uint8_t mode = MODE_POSITION;
// uint8_t mode = MODE_SPEED;
// uint8_t mode = MODE_CURRENT;
float target_pos = 0.0;         //!< motor target position
float target_vel = 0.0;         //!< motor target velocity
float target_torque = 0.0;      //!< motor target torque
float dir = 1.0f;               //!< direction for motion mode
float default_kp = 50.0f;       //!< default kp for motion mode
float default_kd = 1.0f;        //!< default kd for motion mode
float init_speed = 30.0f;       //!< initial speed
float slow_speed = 1.0f;        //!< slow speed

void setup()
{
  Serial.begin(115200);
  // init cybergear driver
  interface.init();
  driver.init(&interface);
  driver.init_motor(mode);
  driver.set_limit_speed(init_speed);
  driver.enable_motor();
}

int count = 0;
int count_max = 10;
bool flag = false;
void loop()
{
  count++;
  if (count > count_max) {
    flag = !flag;
    count = 0;
  }
  if (flag) {
    if (mode == MODE_POSITION) {
      target_pos += INC_POSITION / 180.0f * M_PI;

    } else if (mode == MODE_SPEED) {
      target_vel += INC_VELOCITY;

    } else if (mode == MODE_CURRENT) {
      target_torque += INC_TORQUE;
    }
  } else {
    if (mode == MODE_POSITION) {
      target_pos -= INC_POSITION / 180.0f * M_PI;

    } else if (mode == MODE_SPEED) {
      target_vel -= INC_VELOCITY;

    } else if (mode == MODE_CURRENT) {
      target_torque -= INC_TORQUE;
    }
  }

  if (driver.get_run_mode() == MODE_POSITION) {
    // set limit speed when state changed
    if (std::fabs(motor_status.position - target_pos) < 10.0 / 180.0 * M_PI) {
      driver.set_limit_speed(init_speed);
    }
    driver.set_position_ref(target_pos);
  }
  else if (driver.get_run_mode() == MODE_SPEED) {
    driver.set_speed_ref(target_vel);
  }
  else if (driver.get_run_mode() == MODE_CURRENT) {
    driver.set_current_ref(target_torque);
  }
  else {
    target_pos += dir * 10.0 / 180.0 * M_PI;
    if (target_pos > P_MAX) { dir = -1.0; target_pos = P_MAX; }
    else if (target_pos < P_MIN) { dir = 1.0; target_pos = P_MIN; }
    driver.motor_control(target_pos, dir * target_vel, dir * target_torque, default_kd, default_kd);
  }

  // update and get motor data
  if ( driver.process_packet() ) {
    motor_status = driver.get_motor_status();
  }

  delay(200);
}
