#pragma once

#include "spi.hpp"
#include "avalon_mm.hpp"
#include <shared_memory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <self_test/test_runner.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <diagnostic_msgs/srv/self_test.hpp>
#include <phoenix_msgs/srv/program_nios.hpp>
#include <phoenix_msgs/srv/program_fpga.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <mutex>

namespace phoenix {

/**
 * @brief SPIで制御コマンドを送るノード
 */
class CommandNode : public rclcpp::Node {
public:
    CommandNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    /**
     * @brief cmd_velトピックを処理する
     */
    void commandVelocityCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

    /**
     * @brief FPGAのセルフテストを実行する
     */
    void doSelfTestFpga(diagnostic_msgs::msg::DiagnosticStatus &diag);

    /**
     * @brief program_niosサービスを処理する
     */
    void programNiosCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::ProgramNios::Request> request,
                             const std::shared_ptr<phoenix_msgs::srv::ProgramNios::Response> response);

    /**
     * @brief program_fpgaサービスを処理する
     */
    void programFpgaCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<phoenix_msgs::srv::ProgramFpga::Request> request,
                             const std::shared_ptr<phoenix_msgs::srv::ProgramFpga::Response> response);

    /**
     * @brief 設定されたパラメータを処理する
     * @param param パラメータのvector
     * @return 結果
     */
    rcl_interfaces::msg::SetParametersResult setParameterCallback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * @brief パラメータをfloatに変換する
     * double型およびint型のパラメータからのみ変換できる。
     * @param parameter 変換するパラメータ
     * @return 変換結果
     */
    static float toFloat(const rclcpp::Parameter &parameter);

    /**
     * @brief パラメータがfloatに変換できて範囲内ならdestinationに代入しtrueを返す。それ以外ならfalseを返す。
     * @param parameter 変換するパラメータ
     * @param lower_bound 下限値
     * @param upper_bound 上限値
     * @param destination 代入先
     * @return trueならdestinationにパラメータを代入した。falseならしなかった。
     */
    static bool writeFloatParameterToMemory(const rclcpp::Parameter &parameter, float lower_bound, float upper_bound, float *destination);

    /// SPI
    std::shared_ptr<Spi> _spi;

    /// Avalon-MMマスター
    std::shared_ptr<AvalonMm> _avalon_mm;

    // cmd_velトピックのSubscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _velocity_subscription;

    // injected_error_flagsトピックのSubscription
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr _injected_error_flags_subscription;

    // injected_fault_flagsトピックのSubscription
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr _injected_fault_flags_subscription;

    /// program_niosサービス
    rclcpp::Service<phoenix_msgs::srv::ProgramNios>::SharedPtr _program_nios_service;

    /// program_fpgaサービス
    rclcpp::Service<phoenix_msgs::srv::ProgramFpga>::SharedPtr _program_fpga_service;

    /// パラメータが設定されたとき呼ばれるコールバックを保持する
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr _parameter_handler;
    
    /// セルフテストの管理
    self_test::TestRunner _test_runner;

    /// 共有メモリー
    SharedMemory _shared_memory;

    /// 故障注入されたエラーフラグ
    uint32_t _injected_error_flags = 0;

    /// 故障注入されたフォルトフラグ
    uint32_t _injected_fault_flags = 0;

    /// SPIのビットレート[Hz]
    static constexpr unsigned int SPI_FREQUENCY = 10000000;

    /// FPGAのSPIモード
    static constexpr int FPGA_SPI_MODE = 1;

    /// EPCQのSPIモード
    static constexpr int EPCQ_SPI_MODE = 0;

    /// QoSのキューの長さ
    static constexpr int QOS_DEPTH = 1;
};

} // namespace phoenix
