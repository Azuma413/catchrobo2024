#include "cybergear_can_interface_esp32.hh"
#include <RingBuf.h>
#include <cstring>
#include "cybergear_can_interface.hh"
#include "cybergear_driver_utils.hh"
// use ESP-IDF TWAI (CAN) driver
#include "driver/twai.h"

struct CanMessage
{
  uint32_t tx_id;
  uint32_t rx_id;
  bool is_extended;
  bool is_rtr;
  uint8_t dlc;
  uint8_t data[8];
};
static RingBuf<CanMessage, 100> buffer;

// TWAI RX ISR callback
static void on_receive(void *arg)
{
  CanMessage msg;
  twai_message_t twai_msg;

  // Receive message from TWAI driver
  if (twai_receive(&twai_msg, portMAX_DELAY) == ESP_OK) {
    msg.is_extended = twai_msg.extd;
    msg.is_rtr = twai_msg.rtr;
    msg.rx_id = twai_msg.identifier;
    msg.dlc = twai_msg.data_length_code;
    memcpy(msg.data, twai_msg.data, msg.dlc);
    buffer.lockedPushOverwrite(msg);
  }
}

CybergearCanInterfaceEsp32::CybergearCanInterfaceEsp32() : CybergearCanInterface() {}

CybergearCanInterfaceEsp32::~CybergearCanInterfaceEsp32() {}

bool CybergearCanInterfaceEsp32::init(uint8_t rx_pin, uint8_t tx_pin)
{
  // TWAI (CAN) configuration
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  // 1 Mbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();  // Accept all messages

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }

  // Start TWAI driver
  if (twai_start() != ESP_OK) {
    return false;
  }

  // Set receive callback (install ISR)
  twai_register_rx_callback(on_receive, nullptr);

  return true;
}

bool CybergearCanInterfaceEsp32::send_message(
  uint32_t id, const uint8_t * data, uint8_t len, bool ext)
{
  CG_DEBUG_FUNC
  twai_message_t twai_msg = {};
  twai_msg.identifier = id;
  twai_msg.extd = ext;
  twai_msg.data_length_code = len;
  memcpy(twai_msg.data, data, len);

  // Send message using TWAI driver
  if (twai_transmit(&twai_msg, portMAX_DELAY) != ESP_OK) {
    return false;
  }

  return true;
}

bool CybergearCanInterfaceEsp32::read_message(unsigned long & id, uint8_t * data, uint8_t & len)
{
  CG_DEBUG_FUNC
  // check empty
  if (buffer.isEmpty()) return false;

  // get message from buffer
  CanMessage msg;
  if (!buffer.lockedPop(msg)) return false;

  id = msg.rx_id;
  len = msg.dlc;
  memcpy(data, msg.data, len);
  return true;
}

bool CybergearCanInterfaceEsp32::available()
{
  CG_DEBUG_FUNC
  return (buffer.isEmpty() == false);
}
