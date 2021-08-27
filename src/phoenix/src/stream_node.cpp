#include "stream_node.hpp"
#include "diagnostics.hpp"
#include "../include/phoenix.hpp"
#include <rcutils/logging.h>
#include <chrono>

using namespace std::chrono_literals;

namespace phoenix {

/// UARTのデバイスパスの初期値
static const std::string DEFAULT_DEVICE_PATH = "/dev/ttyTHS1";

/**
 * @brief StreamDataAdc2_tをStreamDataAdc2メッセージに変換する
 * @param data 変換前のデータ
 * @param msg 変換後のメッセージ
 */
static std::unique_ptr<phoenix_msgs::msg::StreamDataAdc2> convertToAdc2(const StreamDataAdc2_t &data) {
    auto msg = std::make_unique<phoenix_msgs::msg::StreamDataAdc2>();
    msg->dc48v_voltage = data.dc48v_voltage;
    msg->dribble_voltage = data.dribble_voltage;
    msg->dribble_current = data.dribble_current;
    return msg;
}

/**
 * @brief StreamDataMotionWithTimestamp_tをStreamDataMotionメッセージに変換する
 * @param data 変換前のデータ
 * @return 変換後のメッセージ
 */
static std::unique_ptr<phoenix_msgs::msg::StreamDataMotion> convertToMotion(const StreamDataMotionWithTimestamp_t &data) {
    auto msg = std::make_unique<phoenix_msgs::msg::StreamDataMotion>();
    for (int axis = 0; axis < 3; axis++) {
        msg->accelerometer[axis] = data.accelerometer[axis];
        msg->gyroscope[axis] = data.gyroscope[axis];
        msg->gravity[axis] = data.gravity[axis];
        msg->body_acceleration[axis] = data.body_acceleration[axis];
        msg->body_velocity[axis] = data.body_velocity[axis];
    }
    for (int index = 0; index < 4; index++) {
        msg->wheel_velocity_meas[index] = data.wheel_velocity_meas[index];
        msg->wheel_current_meas_d[index] = data.wheel_current_meas_d[index];
        msg->wheel_current_meas_q[index] = data.wheel_current_meas_q[index];
        msg->wheel_current_ref[index] = data.wheel_current_ref[index];
        msg->body_ref_accel_unlimit[index] = data.body_ref_accel_unlimit[index];
        msg->body_ref_accel[index] = data.body_ref_accel[index];
    }
    msg->rotation_torque = data.rotation_torque;
    msg->omega_weight = data.omega_weight;
    msg->performance_counter = data.performance_counter;
    return msg;
}

/**
 * @brief StreamDataMotionWithTimestamp_tをOdometryメッセージに変換する
 * @param data 変換前のデータ
 * @return 変換後のメッセージ
 */
static std::unique_ptr<nav_msgs::msg::Odometry> convertToOdometry(const StreamDataMotionWithTimestamp_t &data) {
    auto msg = std::make_unique<nav_msgs::msg::Odometry>();
    msg->header.stamp = data.timestamp;
    msg->header.frame_id = FRAME_ID_ODOMETRY;
    msg->child_frame_id = FRAME_ID_BASE;
    msg->twist.twist.linear.x = data.body_velocity[0];
    msg->twist.twist.linear.x = data.body_velocity[1];
    msg->twist.twist.angular.z = data.body_velocity[2];
    return msg;
}

/**
 * @brief StreamDataMotionWithTimestamp_tをImuメッセージに変換する
 * @param data 変換前のデータ
 * @return 変換後のメッセージ
 */
static std::unique_ptr<sensor_msgs::msg::Imu> convertToImu(const StreamDataMotionWithTimestamp_t &data) {
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.stamp = data.timestamp;
    msg->header.frame_id = FRAME_ID_IMU;
    msg->orientation_covariance[0] = -1.0;
    msg->linear_acceleration.x = data.accelerometer[0];
    msg->linear_acceleration.y = data.accelerometer[1];
    msg->linear_acceleration.z = data.accelerometer[2];
    msg->angular_velocity.x = data.gyroscope[0];
    msg->angular_velocity.y = data.gyroscope[1];
    msg->angular_velocity.z = data.gyroscope[2];
    return msg;
}

StreamPublisherNode::StreamPublisherNode(const rclcpp::NodeOptions &options)
    : Node(stream::NODE_NAME), _uart(new Uart), _diag_updater(this), _tf2_broadcaster(this) {
    using namespace std::placeholders;
    (void)options;

    // パラメータを宣言し値を取得する
    auto device_path = declare_parameter<std::string>(stream::PARAM_NAME_DEVICE_PATH, DEFAULT_DEVICE_PATH);

    // UARTデバイスを開いて設定する
    if (!_uart->openDevice(device_path)) {
        RCUTILS_LOG_FATAL("Cannot open '%s'.", device_path.c_str());
        throw;
    }
    if (!_uart->setBaudrate(BAUDRATE)) {
        RCUTILS_LOG_FATAL("Cannot configure desired baudrate.");
        throw;
    }
    if (!_uart->setIgnoreBreakEnabled(true)) {
        RCUTILS_LOG_FATAL("Cannot configure serial port.");
        throw;
    }

    // ステータスを初期化する
    _status.error_flags = 0xFFFFFFFFUL;
    _status.fault_flags = 0xFFFFFFFFUL;

    // Subscriptionを作成する
    _injected_error_flags_subscription =
        create_subscription<std_msgs::msg::UInt32>(test::TOPIC_NAME_INJECTED_ERROR_FLAGS, 1, [this](std_msgs::msg::UInt32::SharedPtr msg) {
            _injected_error_flags = msg->data;
        });
    _injected_fault_flags_subscription =
        create_subscription<std_msgs::msg::UInt32>(test::TOPIC_NAME_INJECTED_FAULT_FLAGS, 1, [this](std_msgs::msg::UInt32::SharedPtr msg) {
            _injected_fault_flags = msg->data;
        });

    // Publisherを作成する
    _adc2_publisher = create_publisher<phoenix_msgs::msg::StreamDataAdc2>(TOPIC_NAME_ADC2, QOS_DEPTH);
    _motion_publisher = create_publisher<phoenix_msgs::msg::StreamDataMotion>(TOPIC_NAME_MOTION, QOS_DEPTH);
    _odometry_publisher = create_publisher<nav_msgs::msg::Odometry>(TOPIC_NAME_ODOMETRY, QOS_DEPTH);
    _imu_publisher = create_publisher<sensor_msgs::msg::Imu>(TOPIC_NAME_IMU, QOS_DEPTH);

    // 診断ステータスの初期化を行う
    setDefaultHardwareId(get_namespace());
    _diag_updater.setHardwareID(get_namespace());
    _diag_updater.add(getHostName() + DIAGNOSTICS_NAME_SUFFIX_FPGA, [this](diagnostic_msgs::msg::DiagnosticStatus &diag) {
        createFpgaDiagnostics(_status, diag);
    });

    // スレッドを起動する
    _receive_thread = new std::thread([this](void) {
        receiveThread();
    });
    _publish_thread = new std::thread([this](void) {
        publishThread();
    });
}

StreamPublisherNode::~StreamPublisherNode() {
    _thread_exit_request = true;
    if (_receive_thread != nullptr) {
        _receive_thread->join();
        delete _receive_thread;
    }
    if (_publish_thread != nullptr) {
        _publish_thread->join();
        delete _publish_thread;
    }
}

void StreamPublisherNode::receiveThread(void) {
    static constexpr size_t READ_BUFFER_SIZE = 4096;
    static constexpr size_t MAXIMUM_PAYLOAD_LENGTH = 1024;
    std::vector<uint8_t> payload;
    payload.reserve(MAXIMUM_PAYLOAD_LENGTH);
    int channel_number = -1;
    bool payload_overrun = false;
    std::array<uint8_t, READ_BUFFER_SIZE> buffer;
    AvalonStBytesToPacketsConverter converter;

    while (!_thread_exit_request) {
        // UARTからデータを受信する
        size_t read_length;
        if (!_uart->readData(buffer.size(), buffer.data(), &read_length)) {
            RCUTILS_LOG_ERROR("An error was occured while reading UART device.\n");
            return;
        }

        // バイトストリームをパースする
        converter.parse(buffer.data(), read_length, [&](std::uint8_t data, int channel, bool sof, bool eof) {
            if ((channel_number != channel) || sof) {
                if (!payload.empty() && !payload_overrun) {
                    processPacket(payload, channel_number);
                }
                channel_number = channel;
                payload.clear();
                payload_overrun = false;
            }
            if (payload.size() < MAXIMUM_PAYLOAD_LENGTH) {
                payload.push_back(data);
            }
            else {
                payload_overrun = true;
            }
            if (eof) {
                if (!payload.empty() && !payload_overrun) {
                    processPacket(payload, channel_number);
                }
                payload.clear();
                payload_overrun = false;
            }
            return true;
        });
    }
}

void StreamPublisherNode::processPacket(const std::vector<std::uint8_t> &payload, int channel) {
    switch (channel) {
    case StreamIdStatus:
        if (payload.size() == sizeof(StreamDataStatus_t)) {
            _queue_mutex.lock();
            if (QUEUE_LENGTH <= _status_queue.size()) {
                _status_queue.pop_front();
            }
            _status_queue.push_back(*reinterpret_cast<const StreamDataStatus_t *>(payload.data()));
            _queue_mutex.unlock();
            _condition_variable.notify_one();
        }
        break;

    case StreamIdAdc2:
        if (payload.size() == sizeof(StreamDataAdc2_t)) {
            _queue_mutex.lock();
            if (QUEUE_LENGTH <= _adc2_queue.size()) {
                _adc2_queue.pop_front();
            }
            _adc2_queue.push_back(*reinterpret_cast<const StreamDataAdc2_t *>(payload.data()));
            _queue_mutex.unlock();
            _condition_variable.notify_one();
        }
        break;

    case StreamIdMotion:
        if (payload.size() == sizeof(StreamDataMotion_t)) {
            _queue_mutex.lock();
            if (QUEUE_LENGTH <= _motion_queue.size()) {
                _motion_queue.pop_front();
            }
            _motion_queue.emplace_back(*reinterpret_cast<const StreamDataMotion_t *>(payload.data()), rclcpp::Clock().now());
            _queue_mutex.unlock();
            _condition_variable.notify_one();
        }
        break;

    default:
        break;
    }
}

void StreamPublisherNode::publishThread(void) {
    static constexpr auto TIMEOUT = 100ms;
    while (!_thread_exit_request) {
        std::unique_lock<std::mutex> lock(_queue_mutex);
        _condition_variable.wait_for(lock, TIMEOUT);
        if (!_status_queue.empty()) {
            // StreamDataStatus_tをROS2メッセージに変換して配信する

            // 最後のStreamDataStatus_tをコピーする
            StreamDataStatus_t last_status = _status;
            _status = _status_queue.back();
            _status.error_flags |= _injected_error_flags;
            _status.fault_flags |= _injected_fault_flags;
            _status_queue.clear();

            if ((last_status.error_flags != _status.error_flags) || (last_status.fault_flags != _status.fault_flags)) {
                // ステータスに変更があれば診断ステータスを強制的に配信する
                // ここで配信しなくてもdiagnostic_updater::Updaterが定期的に配信する
                lock.unlock();
                _diag_updater.force_update();
                lock.lock();
            }
        }
        if (!_adc2_queue.empty()) {
            // StreamDataAdc2_tをROS2メッセージに変換して配信する
            do {
                auto data = _adc2_queue.front();
                _adc2_queue.pop_front();
                lock.unlock();

                // StreamDataAdc2を配信する
                _adc2_publisher->publish(convertToAdc2(data));

                lock.lock();
            } while (!_adc2_queue.empty());
        }
        if (!_motion_queue.empty()) {
            // StreamDataMotion_tをROS2メッセージに変換して配信する
            do {
                auto data = _motion_queue.front();
                _motion_queue.pop_front();
                lock.unlock();

                // StreamDataMotionとImuを配信する
                _motion_publisher->publish(std::move(convertToMotion(data)));
                _imu_publisher->publish(std::move(convertToImu(data)));

                // OdometryとTransformを配信する
                auto odom_msg = convertToOdometry(data);
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header = odom_msg->header;
                tf_msg.child_frame_id = FRAME_ID_BASE;
                tf_msg.transform.translation.x = odom_msg->pose.pose.position.x;
                tf_msg.transform.translation.y = odom_msg->pose.pose.position.y;
                tf_msg.transform.translation.z = odom_msg->pose.pose.position.z;
                tf_msg.transform.rotation = odom_msg->pose.pose.orientation;
                _odometry_publisher->publish(std::move(odom_msg));
                _tf2_broadcaster.sendTransform(tf_msg);

                lock.lock();
            } while (!_motion_queue.empty());
        }
    }
}

} // namespace phoenix

#if PHOENIX_BUILD_LIBRARY
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(phoenix::StreamPublisherNode)
#elif PHOENIX_BUILD_STANDALONE
int main(int argc, char *argv[]) {
    // ROS2ノードを起動する
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<phoenix::StreamPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
#endif
