#pragma once

#include "avalon_st.hpp"
#include "uart.hpp"
#include <status_flags.hpp>
#include <stream_data.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <phoenix_msgs/msg/stream_data_adc2.hpp>
#include <phoenix_msgs/msg/stream_data_motion.hpp>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <deque>
#include <thread>

namespace phoenix {

struct StreamDataMotionWithTimestamp : public StreamDataMotion {
    rclcpp::Time timestamp;

    StreamDataMotionWithTimestamp(const StreamDataMotion &motion, rclcpp::Time timestamp) : StreamDataMotion(motion), timestamp(timestamp) {}
};

/**
 * @brief UARTで受信したデータをROS2トピックとして配信するノード
 */
class StreamPublisherNode : public rclcpp::Node {
public:
    StreamPublisherNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    virtual ~StreamPublisherNode();

private:
    /**
     * @brief 受信スレッドのエントリポイント
     */
    void receiveThread(void);

    /**
     * @brief 受信したパケットを処理する
     * @param payload パケットのペイロード
     * @param channel パケットのチャンネル番号
     */
    void processPacket(const std::vector<std::uint8_t> &payload, int channel);

    /**
     * @brief 配信スレッドのエントリポイント
     */
    void publishThread(void);

    /// UARTデバイス
    std::shared_ptr<Uart> _uart;

    /// 受信スレッドへの終了リクエスト
    volatile bool _thread_exit_request = false;

    /// 受信スレッド
    std::thread *_receive_thread = nullptr;

    /// 配信スレッド
    std::thread *_publish_thread = nullptr;

    /// キューにデータが追加されたことを受信スレッドが配信スレッドに通知するための状態変数
    std::condition_variable _condition_variable;

    /// キュー操作を制御するミューテックス
    std::mutex _queue_mutex;

    /// StreamDataStatus_tを格納するキュー
    std::deque<StreamDataStatus> _status_queue;

    /// StreamDataAdc2_tを格納するキュー
    std::deque<StreamDataAdc2> _adc2_queue;

    /// StreamDataMotion_tを格納するキュー
    std::deque<StreamDataMotionWithTimestamp> _motion_queue;

    // injected_error_flagsトピックのSubscription
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr _injected_error_flags_subscription;

    // injected_fault_flagsトピックのSubscription
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr _injected_fault_flags_subscription;

    /// StreamDataStatusを元に診断ステータスを配信するPublisher
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr _diag_publisher;

    /// StreamDataAdc2(FPGAに繋がったADC2の測定値)を配信するPublisher
    rclcpp::Publisher<phoenix_msgs::msg::StreamDataAdc2>::SharedPtr _adc2_publisher;

    /// StreamDataMotion(IMU・センサー・モーター制御情報)を配信するPublisher
    rclcpp::Publisher<phoenix_msgs::msg::StreamDataMotion>::SharedPtr _motion_publisher;

    /// オドメトリを配信するPublisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odometry_publisher;

    /// IMUの測定値を配信するPublisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;

    /// 診断ステータスの送信制御
    diagnostic_updater::Updater _diag_updater;

    /// 最後に受信したStreamDataStatus_t
    StreamDataStatus _status;

    /// tf2関連トピックのPublisher
    tf2_ros::TransformBroadcaster _tf2_broadcaster;

    /// 故障注入されたエラーフラグ
    uint32_t _injected_error_flags = 0;

    /// 故障注入されたフォルトフラグ
    uint32_t _injected_fault_flags = 0;

    /// QoSのデータの格納数
    static constexpr int QOS_DEPTH = 10;

    /// キューの長さ
    static constexpr size_t QUEUE_LENGTH = 10;

    /// UARTのボーレート
    static constexpr int BAUDRATE = B4000000;
};

} // namespace phoenix
