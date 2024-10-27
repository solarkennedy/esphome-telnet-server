#pragma once

#ifdef ARDUINO_ARCH_ESP8266
#include <ESPAsyncTCP.h>
#else
// AsyncTCP.h includes parts of freertos, which require FreeRTOS.h header to be included first
//#include <freertos/FreeRTOS.h>
#include <AsyncTCP.h>
#endif

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

#include <set>

namespace esphome {
namespace telnet_server {

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define MAXLINELENGTH 1280
#define DEFAULT_MAX_CLIENTS 3
#define TELNET_PORT 23

class TelnetServer : public Component {
 public:
  TelnetServer(const uint16_t port = TELNET_PORT) : server(port) {
    this->port_ = port;
 }

  float get_setup_priority() const override;

  void set_client_count_sensor(sensor::Sensor *client_count_sensor);
  void set_client_ip_text_sensor(text_sensor::TextSensor *client_ip_text_sensor);

  void set_disconnect_delay(uint32_t disconnect_delay) { this->client_disconnect_delay = disconnect_delay; }

  void setup() override;
  void dump_config() override;
  void loop() override;
  void set_uart(uart::UARTComponent *uart);
  int readBytesUntil(uart::UARTComponent *uart, char terminator, char *buffer, int max_length);
  
  void on_shutdown() override;

 protected:
  void cleanup();

  void readSerial();
  void handleTelnetData(AsyncClient *client, void* data, size_t len);

  void updateClientSensors();

  struct Client {
    Client(AsyncClient *client, uart::UARTComponent *uart);
    ~Client() { delete this->tcp_client; }

  uart::UARTComponent *uart_;
    AsyncClient *tcp_client{nullptr};
    std::string identifier{};
    bool disconnected{false};
    void handleTelnetData(AsyncClient *client, void* data, size_t len);
  };

  AsyncServer server;
  uint16_t port_;
  std::vector<std::unique_ptr<Client>> clients_{};
  std::map<std::string, uint32_t> client_disconnect_times{};
  std::set<std::string> last_published_values{};
  bool clients_updated_ = false;

  uint32_t client_disconnect_delay = 5000; // ms

  //  Set to store received telegram
  char buffer[MAXLINELENGTH];
  uint16_t char_idx = 0;

  sensor::Sensor *client_count_sensor_{nullptr};
  text_sensor::TextSensor *client_ip_text_sensor_{nullptr};
  uart::UARTComponent *uart_{nullptr};
};

}  // namespace telnet_server
}  // namespace esphome

