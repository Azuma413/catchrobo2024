#ifndef CYBERGEAR_CAN_INTERFACE_HPP
#define CYBERGEAR_CAN_INTERFACE_HPP

#include <inttypes.h>
#include <cstdint>
#include <cstring>
#include "cybergear_driver_utils.hh"
#include "cybergear_driver_defs.hh"
#include "driver/twai.h"
#include <Arduino.h>

/**
 * @brief MotorStatus class
 */
struct MotorStatus
{
  unsigned long stamp_usec;  //< timestamp
  uint8_t motor_id;          //!< motor id
  float position;            //!< encoder position (-4pi to 4pi)
  float velocity;            //!< motor velocity (-30rad/s to 30rad/s)
  float effort;              //!< motor effort (-12Nm - 12Nm)
  float temperature;         //!< temperature
  uint16_t raw_position;     //!< raw position (for sync data)
  uint16_t raw_velocity;     //!< raw velocity (for sync data)
  uint16_t raw_effort;       //!< raw effort (for sync data)
  uint16_t raw_temperature;  //!< raw temperature (for sync data)
};

class CybergearCanInterface
{
public:
  std::vector<MotorStatus> motor_status = std::vector<MotorStatus>(2);
  CybergearCanInterface(){};  // コンストラクタ
  virtual ~CybergearCanInterface(){};  // デストラクタ
  virtual bool send_message(uint32_t id, const uint8_t * data, uint8_t len, bool ext)
  {
    CG_DEBUG_FUNC
    twai_message_t message;
    message.identifier = id;
    message.extd = ext;
    message.rtr = 0;
    message.data_length_code = len;
    memcpy(message.data, data, len);
    esp_err_t ret = twai_transmit(&message,portMAX_DELAY);
    if (ret != ESP_OK){
        if (ret == ESP_ERR_TIMEOUT){
            Serial.println("CAN TX timeout");
        }else if (ret == ESP_FAIL){
            Serial.println("CAN TX failed");
        }
        return false;
    }
    return true;
  }

  virtual bool read_message(unsigned long & id, uint8_t * data, uint8_t & len)
  {
    CG_DEBUG_FUNC
    id = receive_msg.identifier;
    len = receive_msg.data_length_code;
    memcpy(data, receive_msg.data, len);
    return true;
  }

  virtual bool available()
  {
    CG_DEBUG_FUNC
    return (receive_msg.identifier != 0);
  }

  void on_receive(twai_message_t *message)
  {
    if ((message->identifier & 0xff00) >> 8 == 1){
      motor_status[0].position = uint_to_float(message->data[0] << 8 | message->data[1], P_MIN, P_MAX);
      motor_status[0].velocity = uint_to_float(message->data[2] << 8 | message->data[3], V_MIN, V_MAX);
      motor_status[0].effort = uint_to_float(message->data[4] << 8 | message->data[5], T_MIN, T_MAX);
      motor_status[0].temperature = message->data[6] << 8 | message->data[7];
    }else if ((message->identifier & 0xff00) >> 8 == 2){
      motor_status[1].position = uint_to_float(message->data[0] << 8 | message->data[1], P_MIN, P_MAX);
      motor_status[1].velocity = uint_to_float(message->data[2] << 8 | message->data[3], V_MIN, V_MAX);
      motor_status[1].effort = uint_to_float(message->data[4] << 8 | message->data[5], T_MIN, T_MAX);
      motor_status[1].temperature = message->data[6] << 8 | message->data[7];
    }
  }

  float uint_to_float(uint16_t x, float x_min, float x_max)
  {
    uint16_t type_max = 0xFFFF;
    float span = x_max - x_min;
    return (float)x / type_max * span + x_min;
  }
private:
  twai_message_t receive_msg;
};

#endif  // CYBERGEAR_CAN_INTERFACE_HPP