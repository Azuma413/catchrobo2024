#include "cybergear_can_interface_esp32.hh"
#include <cstdint>
#include <cstring>
#include "cybergear_can_interface.hh"
#include "cybergear_driver_utils.hh"
#include "driver/twai.h"
#include "freertos/queue.h"
#include "freertos/task.h"

struct CanMessage
{
  uint64_t tx_id;
  uint64_t rx_id;
  bool is_extended;
  bool is_rtr;
  uint8_t dlc;
  uint8_t data[8];
};

// キューのサイズを設定します
static const int QUEUE_SIZE = 100;
static QueueHandle_t can_queue;

static void on_receive()
{
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    CanMessage msg;
    msg.is_extended = message.extd;
    msg.is_rtr = message.rtr;
    msg.tx_id = message.identifier;
    msg.rx_id = message.identifier;  // Adjust based on your application logic
    msg.dlc = message.data_length_code;
    memcpy(msg.data, message.data, msg.dlc);

    // キューにメッセージを追加
    xQueueSend(can_queue, &msg, portMAX_DELAY);
  }
}

CybergearCanInterfaceEsp32::CybergearCanInterfaceEsp32() : CybergearCanInterface() {}

CybergearCanInterfaceEsp32::~CybergearCanInterfaceEsp32() {}

bool CybergearCanInterfaceEsp32::init(uint8_t rx_pin, uint8_t tx_pin)
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(tx_pin), gpio_num_t(rx_pin), TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }

  if (twai_start() != ESP_OK) {
    return false;
  }

  // キューを作成
  can_queue = xQueueCreate(QUEUE_SIZE, sizeof(CanMessage));
  if (can_queue == NULL) {
    return false;
  }

  // 受信タスクを作成
  xTaskCreatePinnedToCore([](void*) {
    while (true) {
      on_receive();
    }
  }, "CAN_Receive_Task", 4096, nullptr, 10, nullptr, 1);

  return true;
}

bool CybergearCanInterfaceEsp32::send_message(
  uint32_t id, const uint8_t * data, uint8_t len, bool ext)
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

bool CybergearCanInterfaceEsp32::read_message(unsigned long & id, uint8_t * data, uint8_t & len)
{
  CG_DEBUG_FUNC
  CanMessage msg;
  // キューからメッセージを取得
  if (xQueueReceive(can_queue, &msg, 0) == pdFALSE) {
    return false;
  }

  id = msg.rx_id;
  len = msg.dlc;
  memcpy(data, msg.data, len);
  return true;
}

bool CybergearCanInterfaceEsp32::available()
{
  CG_DEBUG_FUNC
  // キュー内のアイテム数を取得
  return (uxQueueMessagesWaiting(can_queue) > 0);
}
