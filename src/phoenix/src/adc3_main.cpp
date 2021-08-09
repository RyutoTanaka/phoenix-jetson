#include <cmath>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rcutils/logging.h>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include "i2c.hpp"
#include "ads1015.hpp"

namespace phoenix {

/// I2Cのデバイスパス
static const char I2cDeviceName[] = "/dev/i2c-1";

/// ADS1015のI2Cアドレス
static constexpr uint8_t Ads1015Address = 0x48;

class AdcPublisherNode : public rclcpp::Node {
public:
    AdcPublisherNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("phoenix_battery"), _adc3() {
        using namespace std::chrono_literals;
        (void)options;

        // I2Cデバイスを開く
        std::shared_ptr<I2c> i2c(new I2c);
        if (i2c->openDevice(phoenix::I2cDeviceName) == false) {
            RCUTILS_LOG_FATAL("Cannot open '%s'.", I2cDeviceName);
            throw;
        }

        // ADC3(ADS1015)を見つける
        _adc3 = std::shared_ptr<ADS1015>(new ADS1015(i2c, phoenix::Ads1015Address));
        if (_adc3->initialize() == false) {
            RCUTILS_LOG_FATAL("ADS1015 is not found.");
            throw;
        }

        // publisherを作成する
        rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
        _battery_publisher = this->create_publisher<sensor_msgs::msg::BatteryState>("battery", qos);
        _timer = this->create_wall_timer(10ms, std::bind(&AdcPublisherNode::timerCallback, this));

        // _battery_stateの不明な項目をNaNで初期化する
        _battery_state.charge = std::numeric_limits<float>::quiet_NaN();
        _battery_state.capacity = std::numeric_limits<float>::quiet_NaN();
        _battery_state.design_capacity = std::numeric_limits<float>::quiet_NaN();
        _battery_state.percentage = std::numeric_limits<float>::quiet_NaN();
    }

private:
    /**
     * @brief 定期的に1チャンネルずつ変換を行う
     */
    void timerCallback(void) {
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

    /**
     * @brief ADCの変換を行って結果を取得する
     * @param mux マルチプレクサ
     * @param fsr フルスケール電圧
     * @param dr 変換レート
     * @param result 変換結果を格納する変数へのポインタ
     * @return trueなら変換が成功した。falseなら失敗した。
     */
    bool doConversion(ADS1015::MUX_t mux, ADS1015::FSR_t fsr, ADS1015::DR_t dr, int16_t *result) {
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

    /// ADC3
    std::shared_ptr<ADS1015> _adc3;

    /// 測定中のチャンネル番号
    int _channel = 0;

    /// VSYS電圧の測定値
    /// _BatteryStateの中には該当する項目が無いため
    float _vsys_voltage = 0.0f;

    /// バッテリー状態を格納するメッセージ
    sensor_msgs::msg::BatteryState _battery_state;

    /// AD変換を行うタイミングを生成するタイマー
    rclcpp::TimerBase::SharedPtr _timer;

    /// _battery_stateを配信するpublisher
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr _battery_publisher;

    /// 過熱警報の閾値 [℃] (この値はSLG46826に設定した過熱シャットダウンの閾値より低い)
    static constexpr float OVER_TEMPERATURE_WARNING_THRESHOLD = 60.0f;

    /// バッテリー過電圧警報の閾値 [V]
    static constexpr float OVER_VOLTAGE_WARNING_THRESHOLD = 28.0f;

    /// バッテリー低電圧警報の閾値 [V] (この値はSLG46826に設定したUVLO電圧より高い)
    static constexpr float UNDER_VOLTAGE_WARNING_THRESHOLD = 19.2f;
};

} // namespace phoenix

int main(int argc, char *argv[]) {
    // ROS2ノードを起動する
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<phoenix::AdcPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
