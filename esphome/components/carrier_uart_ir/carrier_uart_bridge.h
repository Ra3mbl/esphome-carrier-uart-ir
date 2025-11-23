#pragma once

#include <vector>
#include <cmath>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"

namespace esphome {
namespace carrier_uart_ir {

class CarrierUartBridge : public climate::Climate,
                          public Component,
                          public uart::UARTDevice {
 public:
  explicit CarrierUartBridge(uart::UARTComponent *parent)
      : uart::UARTDevice(parent) {}

  // ================================================================
  // Setup
  // ================================================================
  void setup() override {
    ESP_LOGI("carrier_bridge", "Carrier UART bridge setup");
    parser_state_ = STATE_WAIT_START;
    last_log_ms_ = millis();
  }

  // ================================================================
  // Loop Ч читаем UART, обновл€ем статистику
  // ================================================================
  void loop() override {
    while (this->available()) {
      uint8_t b = this->read();
      this->uart_byte_count_++;
      this->process_uart_byte_(b);
    }

    // периодический лог каждые 10 секунд
    uint32_t now = millis();
    if (now - this->last_log_ms_ > 10000) {
      this->last_log_ms_ = now;
      ESP_LOGI("carrier_bridge",
               "UART stats: bytes=%u frames_ok=%u (55 90=%u, 55 91=%u)",
               this->uart_byte_count_,
               this->uart_frame_count_,
               this->frame_55_90_count_,
               this->frame_55_91_count_);
    }
  }

  // ================================================================
  // Climate control Ч команды из HA
  // ================================================================
  void control(const climate::ClimateCall &call) override {
    if (call.get_mode().has_value())
      this->mode = *call.get_mode();

    if (call.get_target_temperature().has_value())
      this->target_temperature = *call.get_target_temperature();

    if (call.get_fan_mode().has_value())
      this->fan_mode = *call.get_fan_mode();

    ESP_LOGI("carrier_bridge",
             "Control: mode=%d temp=%.1f fan=%d",
             (int) this->mode,
             this->target_temperature,
             (int) this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO));

    // TODO: отправка IR
    this->send_ir_from_state_();

    this->publish_state();
  }

  // ================================================================
  // Climate capabilities
  // ================================================================
  climate::ClimateTraits traits() override {
    climate::ClimateTraits traits;
    traits.set_supports_current_temperature(false);
    traits.set_supported_modes({
        climate::CLIMATE_MODE_OFF,
        climate::CLIMATE_MODE_AUTO,
        climate::CLIMATE_MODE_COOL,
        climate::CLIMATE_MODE_HEAT,
        climate::CLIMATE_MODE_DRY,
        climate::CLIMATE_MODE_FAN_ONLY,
    });
    traits.set_supported_fan_modes({
        climate::CLIMATE_FAN_AUTO,
        climate::CLIMATE_FAN_LOW,
        climate::CLIMATE_FAN_MEDIUM,
        climate::CLIMATE_FAN_HIGH,
    });
    traits.set_visual_min_temperature(18);
    traits.set_visual_max_temperature(30);
    traits.set_visual_temperature_step(1.0f);
    return traits;
  }

 protected:
  // ================================================================
  // UART parser state machine
  // ================================================================
  enum ParserState {
    STATE_WAIT_START,
    STATE_GOT_START,
    STATE_GOT_PID,
    STATE_GOT_LEN,
    STATE_READING_PAYLOAD,
    STATE_WAIT_CHECKSUM
  };

  ParserState parser_state_{STATE_WAIT_START};
  uint8_t frame_start_{0};
  uint8_t frame_pid_{0};
  uint8_t frame_len_{0};
  uint8_t bytes_read_{0};
  std::vector<uint8_t> frame_payload_;

  // UART statistics
  uint32_t uart_byte_count_{0};
  uint32_t uart_frame_count_{0};
  uint32_t frame_55_90_count_{0};
  uint32_t frame_55_91_count_{0};
  uint32_t last_log_ms_{0};

  // ================================================================
  // Parsing logic
  // ================================================================
  void process_uart_byte_(uint8_t b) {
    switch (parser_state_) {
      case STATE_WAIT_START:
        if (b == 0x55 || b == 0xAA) {
          frame_start_ = b;
          parser_state_ = STATE_GOT_START;
        }
        break;

      case STATE_GOT_START:
        // ждЄм PID
        if (b == 0x90 || b == 0x91) {
          frame_pid_ = b;
          parser_state_ = STATE_GOT_PID;
        } else {
          // не PID Ч сброс, но этот байт может быть новым стартом
          parser_state_ = STATE_WAIT_START;
          if (b == 0x55 || b == 0xAA) {
            frame_start_ = b;
            parser_state_ = STATE_GOT_START;
          }
        }
        break;

      case STATE_GOT_PID:
        // длина
        frame_len_ = b;
        frame_payload_.clear();
        frame_payload_.reserve(frame_len_);
        bytes_read_ = 0;
        if (frame_len_ == 0) {
          parser_state_ = STATE_WAIT_CHECKSUM;
        } else {
          parser_state_ = STATE_READING_PAYLOAD;
        }
        break;

      case STATE_READING_PAYLOAD:
        frame_payload_.push_back(b);
        bytes_read_++;
        if (bytes_read_ >= frame_len_) {
          parser_state_ = STATE_WAIT_CHECKSUM;
        }
        break;

      case STATE_WAIT_CHECKSUM:
        // этот байт Ч checksum
        handle_frame_with_checksum_(b);
        parser_state_ = STATE_WAIT_START;
        break;

      case STATE_GOT_LEN:
      default:
        parser_state_ = STATE_WAIT_START;
        break;
    }
  }

  // контрольна€ сумма, как в логах (двухкомплемент)
  bool check_checksum_(uint8_t recv) {
    uint16_t sum = frame_start_ + frame_pid_ + frame_len_;
    for (uint8_t v : frame_payload_)
      sum += v;
    uint8_t calc = (uint8_t) (-(sum & 0xFF));
    return calc == recv;
  }

  void handle_frame_with_checksum_(uint8_t cks) {
    if (!check_checksum_(cks)) {
      // иногда можно раскомментировать дл€ дебага (но будет шумно)
      // ESP_LOGW("carrier_bridge",
      //          "Bad checksum: start=%02X pid=%02X len=%u",
      //          frame_start_, frame_pid_, frame_len_);
      return;
    }

    // статистика по валидным кадрам
    this->uart_frame_count_++;
    if (frame_start_ == 0x55 && frame_pid_ == 0x90) this->frame_55_90_count_++;
    if (frame_start_ == 0x55 && frame_pid_ == 0x91) this->frame_55_91_count_++;

    if (frame_start_ == 0x55 && frame_pid_ == 0x90)
      this->handle_state_frame_();
    else if (frame_start_ == 0x55 && frame_pid_ == 0x91)
      this->handle_mode_frame_();
    else {
      // AA 90 / AA 91 сейчас игнорируем
    }
  }

  // ================================================================
  // Parsing 55 90 Ч состо€ние кондиционера
  // ================================================================
  void handle_state_frame_() {
    if (frame_payload_.size() < 0x14) return;

    const uint8_t *mode_block = &frame_payload_[16];

    if (!is_all_zero_(mode_block, 8))
      decode_mode_block_(mode_block);
    else
      this->mode = climate::CLIMATE_MODE_OFF;

    this->publish_state();
  }

  // ================================================================
  // Parsing 55 91 Ч команда с пульта
  // ================================================================
  void handle_mode_frame_() {
    if (frame_payload_.size() != 8) return;

    const uint8_t *mode_block = &frame_payload_[0];

    if (!is_all_zero_(mode_block, 8))
      decode_mode_block_(mode_block);
    else
      this->mode = climate::CLIMATE_MODE_OFF;  // 00..00 Ч OFF/LED toggle

    this->publish_state();
  }

  bool is_all_zero_(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++)
      if (p[i] != 0x00) return false;
    return true;
  }

  // ================================================================
  // Decoding mode + temperature from 55 91 block
  // ================================================================
  void decode_mode_block_(const uint8_t *m) {
    uint8_t b0 = m[0];
    uint8_t b1 = m[1];
    uint8_t b4 = m[4];

    float temp = decode_temperature_(b0, b1);
    if (!std::isnan(temp))
      this->target_temperature = temp;

    this->mode = decode_mode_(b0, b1, b4);
    this->fan_mode = climate::CLIMATE_FAN_AUTO;
  }

  float decode_temperature_(uint8_t b0, uint8_t b1) {
    struct Entry { uint8_t a, b, t; };
    static const Entry tbl[] = {
        {0x06, 0x7F, 18},
        {0x06, 0x6F, 19},
        {0x5B, 0x3F, 20},
        {0x5B, 0x06, 21},
        {0x5B, 0x5B, 22},
        {0x5B, 0x4F, 23},
        {0x5B, 0x66, 24},
        {0x5B, 0x6D, 25},
        {0x5B, 0x7D, 26},
        {0x5B, 0x07, 27},
        {0x5B, 0x7F, 28},
        {0x5B, 0x6F, 29},
        {0x4F, 0x3F, 30},
    };
    for (auto &e : tbl)
      if (e.a == b0 && e.b == b1)
        return (float) e.t;
    return NAN;
  }

  climate::ClimateMode decode_mode_(uint8_t b0, uint8_t b1, uint8_t b4) {
    if (b0 == 0x5B && b1 == 0x4F) {
      if (b4 & 0x80) {
        if (b4 & 0x09)
          return climate::CLIMATE_MODE_AUTO;
        else
          return climate::CLIMATE_MODE_HEAT;
      } else {
        return climate::CLIMATE_MODE_COOL;
      }
    }

    if (b0 == 0x06 && b1 == 0x6F) {
      if (b4 == 0x48)
        return climate::CLIMATE_MODE_DRY;
      if (b4 == 0x00)
        return climate::CLIMATE_MODE_FAN_ONLY;
    }

    if (b0 == 0x3F) {
      // TURBO toggle Ч режим не мен€ем
      return this->mode;
    }

    return this->mode;
  }

  // ================================================================
  // IR sender stub
  // ================================================================
  void send_ir_from_state_() {
    ESP_LOGI("carrier_bridge",
             "send_ir_from_state_ (stub): mode=%d temp=%.1f",
             (int) this->mode,
             this->target_temperature);
  }
};

}  // namespace carrier_uart_ir
}  // namespace esphome
