#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include "i2c.hpp"
#include "ads1015.hpp"

namespace phoenix {

class BatteryPublisherNode : public rclcpp::Node {
public:
    BatteryPublisherNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /// デバイスパスを指定するパラメータ名
    static const std::string PARAM_NAME_DEVICE_PATH;

    /// デバイスのI2Cアドレスを指定するパラメータ名
    static const std::string PARAM_NAME_DEVICE_ADDRESS;

private:
    /**
     * @brief 定期的に1チャンネルずつ変換を行う
     */
    void timerCallback(void);

    /**
     * @brief ADCの変換を行って結果を取得する
     * @param mux マルチプレクサ
     * @param fsr フルスケール電圧
     * @param dr 変換レート
     * @param result 変換結果を格納する変数へのポインタ
     * @return trueなら変換が成功した。falseなら失敗した。
     */
    bool doConversion(ADS1015::MUX_t mux, ADS1015::FSR_t fsr, ADS1015::DR_t dr, int16_t *result);

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
};

} // namespace phoenix
