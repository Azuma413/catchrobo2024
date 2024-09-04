#ifndef CYBERGEAR_CAN_INTERFACE_ESP32_HH
#define CYBERGEAR_CAN_INTERFACE_ESP32_HH
#include "cybergear_can_interface.hh"
#define ESP32_RX_PIN 4
#define ESP32_TX_PIN 5

class CybergearCanInterfaceEsp32 : public CybergearCanInterface
{
public:
  CybergearCanInterfaceEsp32();
  virtual ~CybergearCanInterfaceEsp32();
  bool init(uint8_t rx_pin = ESP32_RX_PIN, uint8_t tx_pin = ESP32_TX_PIN);
  virtual bool send_message(uint32_t id, const uint8_t * data, uint8_t len, bool ext);
  virtual bool read_message(unsigned long & id, uint8_t * data, uint8_t & len);
  virtual bool available();

private:
  uint8_t receive_buffer_[64];  //!< receive buffer
};

#endif  // ESP_CAN_INTERFACE_HH
