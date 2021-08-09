#include "battery_node.hpp"
#include <cmath>
#include <limits>
#include <thread>
#include <rcutils/logging.h>

namespace phoenix {

const std::string BatteryPublisherNode::PARAM_NAME_DEVICE_PATH = "device_path";
const std::string BatteryPublisherNode::PARAM_NAME_DEVICE_ADDRESS = "device_address";

/// デバイスパスの初期値
static const std::string DEFAULT_DEVICE_PATH = "/dev/i2c-1";

/// I2Cアドレスの初期値
static constexpr uint8_t DEFAULT_DEVICE_ADDRESS = 0x48;

/// 過熱警報の閾値 [℃] (この値はSLG46826に設定した過熱シャットダウンの閾値より低い)
static constexpr float OVER_TEMPERATURE_WARNING_THRESHOLD = 60.0f;

/// バッテリー過電圧警報の閾値 [V]
static constexpr float OVER_VOLTAGE_WARNING_THRESHOLD = 28.0f;

/// バッテリー低電圧警報の閾値 [V] (この値はSLG46826に設定したUVLO電圧より高い)
static constexpr float UNDER_VOLTAGE_WARNING_THRESHOLD = 19.2f;

BatteryPublisherNode::BatteryPublisherNode(const rclcpp::NodeOptions &options) : Node("phoenix_battery"), _adc3() {
    using namespace std::chrono_literals;
    (void)options;

    // パラメータを宣言し値を取得する
    auto device_path = declare_parameter<std::string>(PARAM_NAME_DEVICE_PATH, DEFAULT_DEVICE_PATH);
    auto device_address = declare_parameter<int>(PARAM_NAME_DEVICE_ADDRESS, DEFAULT_DEVICE_ADDRESS);
    if ((device_address < 0x08) || (0x78 < device_address)) {
        RCUTILS_LOG_ERROR("Parameter '%s' is out of range.", PARAM_NAME_DEVICE_ADDRESS.c_str());
        throw;
    }

    // I2Cデバイスを開く
    std::shared_ptr<I2c> i2c(new I2c);
    if (!i2c->openDevice(device_path)) {
        RCUTILS_LOG_FATAL("Cannot open '%s'.", device_path.c_str());
        throw;
    }

    // ADC3(ADS1015)を見つける
    _adc3 = std::make_shared<ADS1015>(i2c, static_cast<uint8_t>(device_address));
    if (!_adc3->initialize()) {
        RCUTILS_LOG_FATAL("ADS1015 is not found.");
        throw;
    }

    // publisherを作成する
    rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    _battery_publisher = create_publisher<sensor_msgs::msg::BatteryState>("battery", qos);
    _timer = create_wall_timer(10ms, std::bind(&BatteryPublisherNode::timerCallback, this));

    // _battery_stateの不明な項目をNaNで初期化する
    _battery_state.charge = std::numeric_limits<float>::quiet_NaN();
    _battery_state.capacity = std::numeric_limits<float>::quiet_NaN();
    _battery_state.design_capacity = std::numeric_limits<float>::quiet_NaN();
    _battery_state.percentage = std::numeric_limits<float>::quiet_NaN();
}

void BatteryPublisherNode::timerCallback(void) {
    int16_t adc_result;
    static constexpr ADS1015::MUX_t mux_table[4] = {ADS1015::MUX_GND_to_AIN0, ADS1015::MUX_GND_to_AIN1, ADS1015::MUX_GND_to_AIN2, ADS1015::MUX_GND_to_AIN3};
    if (doConversion(mux_table[_channel], ADS1015::FSR_2048mV, ADS1015::DR_1600SPS, &adc_result) == false) {
        // 何度か失敗したらノードを終了するようにするといいかもしれない
        return;
    }
    if (_channel == 0) {
        // AIN0 (VBAT_IN_MON)
        _battery_state.voltage = adc_result * (2.048f / 32767 * 23);
    }
    else if (_channel == 1) {
        // AIN1 (VSYS_MON)
        _vsys_voltage = adc_result * (2.048f / 32767 * 23);
    }
    else if (_channel == 2) {
        // AIN2 (ESW_IS)
        _battery_state.current = adc_result * (-2.048f / 32767 / 220 * 9700); // 負荷は放電電流になるので負符号にする
    }
    else if (_channel == 3) {
        // AIN3 (TEMP_MON)
        float ratio = 3.3f / (adc_result * (2.048f / 32767)) - 1.0f;
        _battery_state.temperature = 1.0f / (-log(ratio) / 3380 + 1.0f / (273.15f + 25.0f)) - 273.15f;

        // 測定結果を配信する
        _battery_state.present = (_vsys_voltage <= _battery_state.voltage); // VbatよりVsysが高かったらバッテリー非接続とする
        _battery_state.power_supply_status = _battery_state.present ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
                                                                    : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
        if (OVER_TEMPERATURE_WARNING_THRESHOLD <= _battery_state.temperature) {
            _battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
        }
        else if (!_battery_state.present) {
            _battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        }
        else if (_battery_state.voltage < UNDER_VOLTAGE_WARNING_THRESHOLD) {
            _battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
        }
        else if (OVER_VOLTAGE_WARNING_THRESHOLD < _battery_state.voltage) {
            _battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        }
        else {
            _battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        }
        _battery_publisher->publish(_battery_state);
    }
    _channel = (_channel + 1) % 4;
}

bool BatteryPublisherNode::doConversion(ADS1015::MUX_t mux, ADS1015::FSR_t fsr, ADS1015::DR_t dr, int16_t *result) {
    using namespace std::chrono_literals;
    static constexpr int TIMEOUT = 100;
    if (_adc3->startConversion(mux, fsr, dr) == true) {
        int timeout = TIMEOUT;
        do {
            std::this_thread::sleep_for(1ms);
            bool complete = true;
            if (_adc3->isConversionCompleted(&complete) == false) {
                break;
            }
            if (complete == true) {
                if (_adc3->getConversionResult(result) == false) {
                    break;
                }
                return true;
            }
        } while (0 < --timeout);
    }
    return false;
}

} // namespace phoenix

#if PHOENIX_BUILD_LIBRARY
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(phoenix::BatteryPublisherNode)
#elif PHOENIX_BUILD_STANDALONE
int main(int argc, char *argv[]) {
    // ROS2ノードを起動する
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<phoenix::BatteryPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
#endif
