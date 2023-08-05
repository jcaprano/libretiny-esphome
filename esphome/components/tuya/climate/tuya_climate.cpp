#include "tuya_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tuya {

static const char *const TAG = "tuya.climate";

void TuyaClimate::setup() {
  if (this->switch_id_.has_value()) {
    this->parent_->register_listener(*this->switch_id_, [this](const TuyaDatapoint &datapoint) {
      ESP_LOGW(TAG, "MCU reported switch is: %s, switch id is:%d", ONOFF(datapoint.value_bool), *this->switch_id_);
      if (!datapoint.value_bool){
        this->is_on_ = false;
        this->mode = climate::CLIMATE_MODE_OFF;
      }else{
        this->is_on_ = true;
        this->mode = this->active_mode_;
      }
      this->compute_state_();
      this->publish_state();
    });
  }
  this->parent_->register_listener(36, [this](const TuyaDatapoint &datapoint) {
      this->valve_open_=!datapoint.value_enum;
      ESP_LOGW(TAG, "MCU reported valve state is: %d valve open set to:%d", datapoint.value_enum,this->valve_open_);
      this->compute_state_();
      this->publish_state();
    });

  if (this->active_state_id_.has_value()) {
    this->parent_->register_listener(*this->active_state_id_, [this](const TuyaDatapoint &datapoint) {
      ESP_LOGW(TAG, "MCU reported active mode is: %u", datapoint.value_enum);
      if(datapoint.value_enum == 0){
        this->active_mode_ = climate::CLIMATE_MODE_COOL;
      }else if(datapoint.value_enum == 1){
        this->active_mode_ = climate::CLIMATE_MODE_HEAT;
      }else if(datapoint.value_enum == 2){
        this->active_mode_ = climate::CLIMATE_MODE_FAN_ONLY;
      }
      if(this->is_on_){
        this->mode = this->active_mode_;
      }
      this->compute_state_();
      this->publish_state();
    });
  } else {
    if (this->heating_state_pin_ != nullptr) {
      this->heating_state_pin_->setup();
      this->heating_state_ = this->heating_state_pin_->digital_read();
    }
    if (this->cooling_state_pin_ != nullptr) {
      this->cooling_state_pin_->setup();
      this->cooling_state_ = this->cooling_state_pin_->digital_read();
    }
  }
  if (this->target_temperature_id_.has_value()) {
    this->parent_->register_listener(*this->target_temperature_id_, [this](const TuyaDatapoint &datapoint) {
      this->manual_temperature_ = datapoint.value_int * this->target_temperature_multiplier_;
      if (this->reports_fahrenheit_) {
        this->manual_temperature_ = (this->manual_temperature_ - 32) * 5 / 9;
      }

      ESP_LOGV(TAG, "MCU reported manual target temperature is: %.1f", this->manual_temperature_);
      this->compute_target_temperature_();
      this->compute_state_();
      this->publish_state();
    });
  }
  if (this->current_temperature_id_.has_value()) {
    this->parent_->register_listener(*this->current_temperature_id_, [this](const TuyaDatapoint &datapoint) {
      this->current_temperature = datapoint.value_int * this->current_temperature_multiplier_;
      if (this->reports_fahrenheit_) {
        this->current_temperature = (this->current_temperature - 32) * 5 / 9;
      }

      ESP_LOGV(TAG, "MCU reported current temperature is: %.1f", this->current_temperature);
      this->compute_state_();
      this->publish_state();
    });
  }
  if (this->eco_id_.has_value()) {
    this->parent_->register_listener(*this->eco_id_, [this](const TuyaDatapoint &datapoint) {
      this->eco_ = datapoint.value_bool;
      ESP_LOGV(TAG, "MCU reported eco is: %s", ONOFF(this->eco_));
      this->compute_preset_();
      this->compute_target_temperature_();
      this->publish_state();
    });
  }
}

void TuyaClimate::loop() {
  if (this->active_state_id_.has_value())
    return;

  bool state_changed = false;
  if (this->heating_state_pin_ != nullptr) {
    bool heating_state = this->heating_state_pin_->digital_read();
    if (heating_state != this->heating_state_) {
      ESP_LOGV(TAG, "Heating state pin changed to: %s", ONOFF(heating_state));
      this->heating_state_ = heating_state;
      state_changed = true;
    }
  }
  if (this->cooling_state_pin_ != nullptr) {
    bool cooling_state = this->cooling_state_pin_->digital_read();
    if (cooling_state != this->cooling_state_) {
      ESP_LOGV(TAG, "Cooling state pin changed to: %s", ONOFF(cooling_state));
      this->cooling_state_ = cooling_state;
      state_changed = true;
    }
  }

  if (state_changed) {
    this->compute_state_();
    this->publish_state();
  }
}

void TuyaClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    ESP_LOGW(TAG, "Received mode: %d, sending to MCU", *call.get_mode());
    climate::ClimateMode received_mode = *call.get_mode();
    if(received_mode==climate::CLIMATE_MODE_OFF){
      this->parent_->set_boolean_datapoint_value(*this->switch_id_, 0);
    }else{
      this->parent_->set_boolean_datapoint_value(*this->switch_id_, 1);
      if(received_mode==climate::CLIMATE_MODE_COOL){
        this->parent_->set_enum_datapoint_value(*this->active_state_id_, 0);
      }else if(received_mode==climate::CLIMATE_MODE_HEAT){
        this->parent_->set_enum_datapoint_value(*this->active_state_id_, 1);
      }else if(received_mode==climate::CLIMATE_MODE_FAN_ONLY){
        this->parent_->set_enum_datapoint_value(*this->active_state_id_, 2);
      }
    }
  }

  if (call.get_target_temperature().has_value()) {
    float target_temperature = *call.get_target_temperature();
    if (this->reports_fahrenheit_)
      target_temperature = (target_temperature * 9 / 5) + 32;

    ESP_LOGV(TAG, "Setting target temperature: %.1f", target_temperature);
    this->parent_->set_integer_datapoint_value(*this->target_temperature_id_,
                                               (int) (target_temperature / this->target_temperature_multiplier_));
  }

  if (call.get_preset().has_value()) {
    const climate::ClimatePreset preset = *call.get_preset();
    if (this->eco_id_.has_value()) {
      const bool eco = preset == climate::CLIMATE_PRESET_ECO;
      ESP_LOGV(TAG, "Setting eco: %s", ONOFF(eco));
      this->parent_->set_boolean_datapoint_value(*this->eco_id_, eco);
    }
  }
}

climate::ClimateTraits TuyaClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_action(true);
  traits.set_supports_current_temperature(this->current_temperature_id_.has_value());

  std::set<climate::ClimateMode> modes = {climate::CLIMATE_MODE_OFF};
  if (supports_heat_)
    modes.insert(climate::CLIMATE_MODE_HEAT);
  if (supports_cool_)
    modes.insert(climate::CLIMATE_MODE_COOL);
  if(true)
    modes.insert(climate::CLIMATE_MODE_FAN_ONLY);
  traits.set_supported_modes(modes);

  std::set<climate::ClimateFanMode> fan_modes = {};
  if(true)
    fan_modes.insert(climate::CLIMATE_FAN_LOW);
    fan_modes.insert(climate::CLIMATE_FAN_MEDIUM);
    fan_modes.insert(climate::CLIMATE_FAN_HIGH);
    fan_modes.insert(climate::CLIMATE_FAN_AUTO);
  traits.set_supported_fan_modes(fan_modes);

  std::set<climate::ClimatePreset> presets = {};
  if (this->eco_id_.has_value()) {
    presets.insert(climate::CLIMATE_PRESET_NONE);
    presets.insert(climate::CLIMATE_PRESET_ECO);
  }
  traits.set_supported_presets(presets);

  return traits;
}

void TuyaClimate::dump_config() {
  LOG_CLIMATE("", "Tuya Climate", this);
  if (this->switch_id_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Switch has datapoint ID %u", *this->switch_id_);
  }
  if (this->active_state_id_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Active state has datapoint ID %u", *this->active_state_id_);
  }
  if (this->target_temperature_id_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Target Temperature has datapoint ID %u", *this->target_temperature_id_);
  }
  if (this->current_temperature_id_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Current Temperature has datapoint ID %u", *this->current_temperature_id_);
  }
  LOG_PIN("  Heating State Pin: ", this->heating_state_pin_);
  LOG_PIN("  Cooling State Pin: ", this->cooling_state_pin_);
  if (this->eco_id_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Eco has datapoint ID %u", *this->eco_id_);
  }
}

void TuyaClimate::compute_preset_() {
  if (this->eco_) {
    this->preset = climate::CLIMATE_PRESET_ECO;
  } else {
    this->preset = climate::CLIMATE_PRESET_NONE;
  }
}

void TuyaClimate::compute_target_temperature_() {
  if (this->eco_ && this->eco_temperature_.has_value()) {
    this->target_temperature = *this->eco_temperature_;
  } else {
    this->target_temperature = this->manual_temperature_;
  }
}

void TuyaClimate::compute_state_() {
  if (std::isnan(this->current_temperature) || std::isnan(this->target_temperature)) {
    // if any control parameters are nan, go to OFF action (not IDLE!)
    this->switch_to_action_(climate::CLIMATE_ACTION_OFF);
    return;
  }

  if (this->mode == climate::CLIMATE_MODE_OFF) {
    this->switch_to_action_(climate::CLIMATE_ACTION_OFF);
    return;
  }

  climate::ClimateAction target_action = climate::CLIMATE_ACTION_IDLE;
  if (this->active_state_id_.has_value()) {
    if (this->mode == climate::CLIMATE_MODE_FAN_ONLY){
      target_action = climate::CLIMATE_ACTION_FAN;
    }else if(this->mode == climate::CLIMATE_MODE_COOL  && this->valve_open_){
      target_action = climate::CLIMATE_ACTION_COOLING;
    }else if (this->mode == climate::CLIMATE_MODE_HEAT  && this->valve_open_){
      target_action = climate::CLIMATE_ACTION_HEATING;
    }
    // Use state from MCU datapoint
    /*if (this->supports_heat_ && this->active_state_heating_value_.has_value() &&
        this->active_state_ == this->active_state_heating_value_) {
      target_action = climate::CLIMATE_ACTION_HEATING;
    } else if (this->supports_cool_ && this->active_state_cooling_value_.has_value() &&
               this->active_state_ == this->active_state_cooling_value_) {
      target_action = climate::CLIMATE_ACTION_COOLING;
    }*/
  } else if (this->heating_state_pin_ != nullptr || this->cooling_state_pin_ != nullptr) {
    // Use state from input pins
    if (this->heating_state_) {
      target_action = climate::CLIMATE_ACTION_HEATING;
    } else if (this->cooling_state_) {
      target_action = climate::CLIMATE_ACTION_COOLING;
    }
  } else {
    // Fallback to active state calc based on temp and hysteresis
    const float temp_diff = this->target_temperature - this->current_temperature;
    if (std::abs(temp_diff) > this->hysteresis_) {
      if (this->supports_heat_ && temp_diff > 0) {
        target_action = climate::CLIMATE_ACTION_HEATING;
      } else if (this->supports_cool_ && temp_diff < 0) {
        target_action = climate::CLIMATE_ACTION_COOLING;
      }
    }
  }

  this->switch_to_action_(target_action);
}

void TuyaClimate::switch_to_action_(climate::ClimateAction action) {
  // For now this just sets the current action but could include triggers later
  this->action = action;
}

}  // namespace tuya
}  // namespace esphome
