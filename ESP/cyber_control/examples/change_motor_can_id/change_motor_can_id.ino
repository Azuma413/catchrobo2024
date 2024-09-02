#include <Arduino.h>
#include <math.h>
#include "cybergear_driver.hh"
#include "cybergear_can_interface_esp32.hh"

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CURRENT_CAN_ID = 0x7F;
uint8_t MOT_NEXT_CAN_ID = 0x7E;

// init cybergeardriver
CybergearDriver driver = CybergearDriver(MASTER_CAN_ID, MOT_CURRENT_CAN_ID);
CybergearCanInterfaceEsp32 interface;

void setup()
{
  Serial.begin(115200);
  // init cybergear driver
  Serial.printf("Start change_motor_can_id\n");
  Serial.printf("Change motor can id from [0x%02x] to [0x%02x]\n", MOT_CURRENT_CAN_ID, MOT_NEXT_CAN_ID);

  interface.init();
  driver.init(&interface);
  driver.init_motor(MODE_POSITION);
  driver.change_motor_can_id(MOT_NEXT_CAN_ID);
}

void loop()
{
}
