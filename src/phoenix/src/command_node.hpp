#pragma once

#include "spi.hpp"
#include "avalon_mm.hpp"
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <phoenix_msgs/srv/clear_error.hpp>
#include <phoenix_msgs/srv/set_speed.hpp>
#include <phoenix_msgs/srv/program_nios.hpp>
#include <phoenix_msgs/srv/program_fpga.hpp>
#include "../include/phoenix/shared_memory.hpp"

namespace phoenix {

/**
 * @brief SPIで制御コマンドを送るノード
 */
class CommandServerNode : public rclcpp::Node {
public:
    CommandServerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /// SPIのデバイスパスを指定するパラメータ名
    static const std::string PARAM_NAME_DEVICE_PATH;

    /// 速度制御の比例ゲインのパラメータ名
    static const std::string PARAM_NAME_SPEED_KP;

    /// 速度制御の積分ゲインのパラメータ名
    static const std::string PARAM_NAME_SPEED_KI;

    /// 姿勢補正制御の比例ゲインのパラメータ名
    static const std::string PARAM_NAME_COMPENSATION_KP;

    /// 姿勢補正制御の積分ゲインのパラメータ名
    static const std::string PARAM_NAME_COMPENSATION_KI;

private:
    /**
     * @brief ClearErrorサービスを処理する
     */
    void clearErrorCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::ClearError::Request> request,
                            const std::shared_ptr<phoenix_msgs::srv::ClearError::Response> response);

    /**
     * @brief SetSpeedサービスを処理する
     */
    void setSpeedCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::SetSpeed::Request> request,
                          const std::shared_ptr<phoenix_msgs::srv::SetSpeed::Response> response);

    /**
     * @brief ProgramNiosサービスを処理する
     */
    void programNiosCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::ProgramNios::Request> request,
                             const std::shared_ptr<phoenix_msgs::srv::ProgramNios::Response> response);

    /**
     * @brief ProgramFpgaサービスを処理する
     */
    void programFpgaCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::ProgramFpga::Request> request,
                             const std::shared_ptr<phoenix_msgs::srv::ProgramFpga::Response> response);

    /**
     * @brief パラメータをfloat型として取得する
     * @param parameter_name パラメータ名
     * @param default_value パラメータの型が不正だった場合に返すデフォルト値
     */
    float getFloatParameter(const std::string &parameter_name, float default_value = 0.0f);

    /// SPI
    std::shared_ptr<Spi> _spi;

    /// Avalon-MMマスター
    std::shared_ptr<AvalonMm> _avalon_mm;

    /// ClearErrorサービス
    rclcpp::Service<phoenix_msgs::srv::ClearError>::SharedPtr _clear_error_service;

    /// SetSpeedサービス
    rclcpp::Service<phoenix_msgs::srv::SetSpeed>::SharedPtr _set_speed_service;

    /// ProgramNiosサービス
    rclcpp::Service<phoenix_msgs::srv::ProgramNios>::SharedPtr _program_nios_service;

    /// ProgramFpgaサービス
    rclcpp::Service<phoenix_msgs::srv::ProgramFpga>::SharedPtr _program_fpga_service;

    /// 共有メモリー
    SharedMemory_t _shared_memory;
    
    /// SPIのビットレート[Hz]
    static constexpr unsigned int SPI_FREQUENCY = 10000000;

    /// FPGAのSPIモード
    static constexpr int FPGA_SPI_MODE = 1;

    /// EPCQのSPIモード
    static constexpr int EPCQ_SPI_MODE = 0;

    /// QoSのキューの長さ
    static constexpr int QOS_DEPTH = 10;
};

} // namespace phoenix
