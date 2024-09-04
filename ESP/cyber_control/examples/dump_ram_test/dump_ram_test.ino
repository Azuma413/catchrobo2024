#include <Arduino.h>
#include "src/cybergear_driver.hh"
#include "src/cybergear_can_interface_esp32.hh"

uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID = 0x7F; // デフォルトのCyberGearのCAN ID 127

CybergearDriver driver = CybergearDriver(MASTER_CAN_ID, MOT_CAN_ID);
CybergearCanInterfaceEsp32 interface;

void setup()
{
    Serial.begin(115200);
    Serial.printf("Start read write motor param test\n");
    interface.init();
    driver.init(&interface);
    driver.init_motor(MODE_POSITION);
    delay(1000);
    driver.enable_motor();
}

void loop()
{
    // get motor parameter
    driver.set_position_ref(0.0f);
    driver.dump_motor_param();
    driver.process_packet();
    MotorParameter param = driver.get_motor_param();
    Serial.printf("Received motor param\n");
    Serial.printf("  stamp    : %u\n", param.stamp_usec);
    Serial.printf("  run_mode : %d\n", param.run_mode);
    Serial.printf("  iq_ref : %f\n", param.iq_ref);
    Serial.printf("  spd_ref : %f\n", param.spd_ref);
    Serial.printf("  limit_torque : %f\n", param.limit_torque);
    Serial.printf("  cur_kp : %f\n", param.cur_kp);
    Serial.printf("  cur_ki : %f\n", param.cur_ki);
    Serial.printf("  cur_filt_gain : %f\n", param.cur_filt_gain);
    Serial.printf("  loc_ref : %f\n", param.loc_ref);
    Serial.printf("  limit_spd : %f\n", param.limit_spd);
    Serial.printf("  limit_cur : %f\n", param.limit_cur);
    Serial.printf("  mech_pos : %f\n", param.mech_pos);
    Serial.printf("  iqf : %f\n", param.iqf);
    Serial.printf("  mech_vel : %f\n", param.mech_vel);
    Serial.printf("  vbus : %f\n", param.vbus);
    Serial.printf("  rotation : %d\n", param.rotation);
    Serial.printf("  loc_kp : %f\n", param.loc_kp);
    Serial.printf("  spd_kp : %f\n", param.spd_kp);
    Serial.printf("  spd_ki : %f\n", param.spd_ki);
    delay(5000);
}
