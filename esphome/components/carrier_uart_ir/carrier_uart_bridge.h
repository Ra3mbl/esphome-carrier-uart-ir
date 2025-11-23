// esphome/components/carrier_uart_ir/carrier_uart_bridge.h
#pragma once

#include "esphome.h"

namespace esphome {
namespace carrier_uart_ir {

using namespace esphome;

class CarrierUartBridge : public climate::Climate,
                          public Component,
                          public uart::UARTDevice {
 public:
  CarrierUartBridge(uart::UARTComponent *parent) : UARTDevice(parent) {}

  void setup() override {
    ESP_LOGI("carrier_bridge", "Carrier UART bridge setup");
    parser_state_ = STATE_WAIT_START;
  }

  void loop() override {
    while (available()) {
      uint8_t b = read();
      process_uart_byte_(b);
    }
  }

  // === Climate API ===

  void control(const climate::ClimateCall &call) override {
    if (call.get_mode().has_value())
      this->mode = *call.get_mode();

    if (call.get_target_temperature().has_value())
      this->target_temperature = *call.get_target_temperature();

    if (call.get_fan_mode().has_value())
      this->fan_mode = *call.get_fan_mode();

    ESP_LOGI("carrier_bridge", "Control: mode=%d temp=%.1f fan=%d",
             (int)this->mode, this->target_temperature.value_or(NAN),
             (int)this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO));

    // TODO: тут потом повесим отправку »  (HeatpumpIR / CarrierHeatpumpIR)
    send_ir_from_state_();

    this->publish_state();
  }

  climate::ClimateTraits traits() override {
    climate::ClimateTraits t;
    t.set_supports_current_temperature(false);
    t.set_supported_modes({
        climate::CLIMATE_MODE_OFF,
        climate::CLIMATE_MODE_AUTO,
        climate::CLIMATE_MODE_COOL,
        climate::CLIMATE_MODE_HEAT,
        climate::CLIMATE_MODE_DRY,
        climate::CLIMATE_MODE_FAN_ONLY,
    });
    t.set_supported_fan_modes({
        climate::CLIMATE_FAN_AUTO,
        climate::CLIMATE_FAN_LOW,
        climate::CLIMATE_FAN_MEDIUM,
        climate::CLIMATE_FAN_HIGH,
    });
    t.set_visual_min_temperature(18);
    t.set_visual_max_temperature(30);
    t.set_visual_temperature_step(1.0f);
    return t;
  }

 protected:
  // ===== ѕарсер UART =====

  enum ParserState {
    STATE_WAIT_START,
    STATE_GOT_START,
    STATE_GOT_PID,
    STATE_READING_PAYLOAD
  };

  ParserState parser_state_{STATE_WAIT_START};
  uint8_t frame_start_{0};
  uint8_t frame_pid_{0};
  uint8_t frame_len_{0};
  uint8_t bytes_read_{0};
  std::vector<uint8_t> frame_payload_;

  void process_uart_byte_(uint8_t b) {
    switch (parser_state_) {
      case STATE_WAIT_START:
        if (b == 0x55 || b == 0xAA) {
          frame_start_ = b;
          parser_state_ = STATE_GOT_START;
        }
        break;

      case STATE_GOT_START:
        if (b == 0x90 || b == 0x91) {
          frame_pid_ = b;
          parser_state_ = STATE_GOT_PID;
        } else {
          parser_state_ = STATE_WAIT_START;
          if (b == 0x55 || b == 0xAA) {
            frame_start_ = b;
            parser_state_ = STATE_GOT_START;
          }
        }
        break;

      case STATE_GOT_PID:
        frame_len_ = b;
        frame_payload_.clear();
        frame_payload_.reserve(frame_len_);
        bytes_read_ = 0;
        parser_state_ = STATE_READING_PAYLOAD;
        break;

      case STATE_READING_PAYLOAD:
        if (bytes_read_ < frame_len_) {
          frame_payload_.push_back(b);
          bytes_read_++;
          if (bytes_read_ == frame_len_) {
            // следующий байт будет checksum
            parser_state_ = STATE_GOT_PID;  // будем использовать как "ждЄм checksum"
          }
        } else {
          // этот байт Ч checksum
          handle_frame_with_checksum_(b);
          parser_state_ = STATE_WAIT_START;
        }
        break;
    }
  }

  bool check_checksum_(uint8_t recv_ck) {
    uint16_t sum = frame_start_ + frame_pid_ + frame_len_;
    for (auto v : frame_payload_) sum += v;
    uint8_t calc = (uint8_t)(- (sum & 0xFF));
    return calc == recv_ck;
  }

  void handle_frame_with_checksum_(uint8_t cks) {
    if (!check_checksum_(cks)) {
      ESP_LOGW("carrier_bridge",
               "Bad checksum: start=%02X pid=%02X len=%u",
               frame_start_, frame_pid_, frame_len_);
      return;
    }

    if (frame_start_ == 0x55 && frame_pid_ == 0x90) {
      handle_state_frame_();
    } else if (frame_start_ == 0x55 && frame_pid_ == 0x91) {
      handle_mode_frame_();
    } else {
      // 0xAA 90 / 0xAA 91 Ч ответы диспле€, пока игнорируем
    }
  }

  // ===== –азбор содержимого кадров =====

  void handle_state_frame_() {
    if (frame_payload_.size() < 0x14) return;

    const uint8_t *header = &frame_payload_[0];
    const uint8_t *mode_block = &frame_payload_[16];

    ESP_LOGV("carrier_bridge",
             "STATE: hdr=%02X %02X %02X %02X mode=%02X %02X %02X %02X ...",
             header[0], header[1], header[2], header[3],
             mode_block[0], mode_block[1], mode_block[2], mode_block[3]);

    if (!is_all_zero_(mode_block, 8)) {
      decode_mode_block_(mode_block);
    } else {
      this->mode = climate::CLIMATE_MODE_OFF;
    }

    this->publish_state();
  }

  void handle_mode_frame_() {
    if (frame_payload_.size() != 8) return;
    const uint8_t *mode_block = &frame_payload_[0];

    ESP_LOGV("carrier_bridge",
             "MODE: %02X %02X %02X %02X %02X %02X %02X %02X",
             mode_block[0], mode_block[1], mode_block[2], mode_block[3],
             mode_block[4], mode_block[5], mode_block[6], mode_block[7]);

    if (!is_all_zero_(mode_block, 8)) {
      decode_mode_block_(mode_block);
    } else {
      // 00..00 Ч OFF / LED toggle; пока трактуем как OFF
      this->mode = climate::CLIMATE_MODE_OFF;
    }

    this->publish_state();
  }

  bool is_all_zero_(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++)
      if (p[i] != 0x00) return false;
    return true;
  }

  // ===== ƒекодирование режима/температуры из 8 байт 55 91 =====

  void decode_mode_block_(const uint8_t *m) {
    uint8_t b0 = m[0];
    uint8_t b1 = m[1];
    uint8_t b4 = m[4];

    float temp = decode_temperature_(b0, b1);
    if (!std::isnan(temp))
      this->target_temperature = temp;

    climate::ClimateMode new_mode = decode_mode_(b0, b1, b4);
    this->mode = new_mode;
    this->fan_mode = climate::CLIMATE_FAN_AUTO;  // TODO: вытащить реальные биты
  }

  float decode_temperature_(uint8_t b0, uint8_t b1) {
    struct Entry { uint8_t a, b; uint8_t t; };
    static const Entry table[] = {
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
    for (auto &e : table) {
      if (e.a == b0 && e.b == b1) return (float)e.t;
    }
    return NAN;
  }

  climate::ClimateMode decode_mode_(uint8_t b0, uint8_t b1, uint8_t b4) {
    // груба€ логика из наших логов:
    // 5B 4F .. b4=0x40 => COOL
    // 5B 4F .. b4=0x80 => HEAT
    // 5B 4F .. b4=0x09/0x89 => AUTO
    // 06 6F .. b4=0x48 => DRY
    // 06 6F .. b4=0x00 => FAN
    // 3F xx ..         => TURBO toggle

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

  // ===== «аглушка отправки IR =====

  void send_ir_from_state_() {
    // TODO: тут потом подключаем heatpumpir или пр€мой CarrierHeatpumpIR.
    ESP_LOGI("carrier_bridge", "send_ir_from_state_ (stub): mode=%d temp=%.1f",
             (int)this->mode, this->target_temperature.value_or(NAN));
  }
};

}  // namespace carrier_uart_ir
}  // namespace esphome
