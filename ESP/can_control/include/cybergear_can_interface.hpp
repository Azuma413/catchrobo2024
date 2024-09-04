#ifndef CYBERGEAR_CAN_INTERFACE_HH
#define CYBERGEAR_CAN_INTERFACE_HH

#include <inttypes.h>
#include "cybergear_can_interface.hh"
#include <cstdint>
#include <cstring>
#include "cybergear_driver_utils.hh"
#include "driver/twai.h"
#include "freertos/queue.h"
#include "freertos/task.h"

class CybergearCanInterface
{
public:
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

    if (twai_transmit(&message, pdMS_TO_TICKS(10)) != ESP_OK) {
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
    memcpy(&receive_msg, message, sizeof(twai_message_t));
  }
private:
  twai_message_t receive_msg;
};

#endif  // CYBERGEAR_CAN_INTERFACE_HH