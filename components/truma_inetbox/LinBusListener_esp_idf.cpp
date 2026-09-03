#ifdef USE_ESP_IDF

#include "LinBusListener.h"
#include "esphome/core/log.h"

//#ifdef CUSTOM_ESPHOME_UART
//#include "esphome/components/uart/truma_uart_component_esp_idf.h"
//#define ESPHOME_UART uart::truma_IDFUARTComponent
//#else
#include "esphome/components/uart/uart_component_esp_idf.h"
#define ESPHOME_UART uart::IDFUARTComponent
//#endif  // CUSTOM_ESPHOME_UART

#include <driver/uart.h>

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

void LinBusListener::setup_framework() {
  auto *uart_comp = static_cast<ESPHOME_UART *>(this->parent_);
  uart_port_t uart_num = static_cast<uart_port_t>(uart_comp->get_hw_serial_number());

  // Keep receive latency low so ESPHome's loop wakeup sees bytes immediately.
  // We no longer create our own UART event task or consume an event queue.
  esp_err_t err = uart_set_rx_full_threshold(uart_num, 1);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "uart_set_rx_full_threshold failed: %s", esp_err_to_name(err));
  }

  err = uart_set_rx_timeout(uart_num, 2);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "uart_set_rx_timeout failed: %s", esp_err_to_name(err));
  }

  // Dedizierter RX-Task: Core 1 (WiFi/BT/Hauptloop laufen auf Core 0),
  // Prioritaet 24 wie der uartEventTask_ vor dem 2026.3-Redesign.
  // Stellt sicher, dass LIN-Frames byte-nah gelesen und Antwort-Slots
  // eingehalten werden, unabhaengig von der Last des Hauptloops.
  xTaskCreatePinnedToCore(LinBusListener::rx_task_trampoline,
                          "truma_lin_rx",          // name
                          4096,                    // stack (bytes)
                          this,                    // param
                          24,                      // priority
                          &this->rx_task_handle_,  // handle
                          1                        // core
  );
  if (this->rx_task_handle_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create LIN RX task, falling back to loop-driven RX");
    ESP_LOGCONFIG(TAG, "UART configured for loop-driven RX processing");
  } else {
    ESP_LOGCONFIG(TAG, "UART configured for task-driven RX processing (core 1, prio 24)");
  }
}

void LinBusListener::rx_task_trampoline(void *param) {
  static_cast<LinBusListener *>(param)->rx_task_loop_();
}

void LinBusListener::rx_task_loop_() {
  while (true) {
    if (!this->check_for_lin_fault_() && this->available() > 0) {
      this->on_receive_();
    } else {
      // 1 Tick = 1 ms (CONFIG_FREERTOS_HZ=1000). Haelt Lese-Gaps unter der
      // 1,49-ms-Break-Schwelle und laesst IDLE1 den Watchdog fuettern.
      vTaskDelay(1);
    }
  }
}

}  // namespace truma_inetbox
}  // namespace esphome

#undef ESPHOME_UART

#endif  // USE_ESP_IDF
