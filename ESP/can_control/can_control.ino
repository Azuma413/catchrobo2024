#include "DJIMotorCtrlESP.hpp"
#include "adc_read.hpp"

// PIN
// CAN RX 4
// CAN TX 5
// ADC 34

// CAN ID
// 01: m3508
// 02: CyberGear
// 03: CyberGear
// 04: GM6020
// 05: GM6020
const uint8_t RX_PIN = 5;
const uint8_t TX_PIN = 4;
//                          kp,  ki,  kd,  dead_zone,max_value
pid_param gm6020_1_speed(   10,  0,   0,    1, 16384);
pid_param gm6020_1_location(1.0, 0.1, 0.03, 5, 350);
// pid_param gm6020_1_location(0.2,0.1,0,5,350);
GM6020 gm6020_1(4, gm6020_1_location, gm6020_1_speed);

void setup() {
    Serial.begin(115200);
    while(!Serial);
	adc_setup();
    can_init(RX_PIN, TX_PIN, 100);
    gm6020_1.setup();
    gm6020_1.set_angle(180.0);
}

float target_angle = 0;
float rate = 0.01;
void loop() {
	float degree = get_adc_deg();
    target_angle = target_angle * (1.0 - rate) + degree * rate;
    gm6020_1.set_angle(target_angle);
    Serial.print(target_angle);
    Serial.print(",");
    // float current = gm6020_1.get_curunt_ma();
    // Serial.println(current);
    float angle = gm6020_1.get_angle();
    Serial.println(angle);
    delay(1);
}