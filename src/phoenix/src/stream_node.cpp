#include "stream_node.hpp"
#include "timestamp.hpp"
#include "../include/phoenix.hpp"
#include <rcutils/logging.h>
#include <chrono>

using namespace std::chrono_literals;

namespace phoenix {

/// UARTのデバイスパスの初期値
static const std::string DEFAULT_DEVICE_PATH = "/dev/ttyTHS1";

/// IMUのframe_id
static const std::string IMU_FRAME_ID = "";

StreamPublisherNode::StreamPublisherNode(const rclcpp::NodeOptions &options) : Node(stream::NODE_NAME), _uart(new Uart) {
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

    // Publisherを作成する
    _status_publisher = create_publisher<phoenix_msgs::msg::StreamDataStatus>(TOPIC_NAME_STATUS, QOS_DEPTH);
    _adc2_publisher = create_publisher<phoenix_msgs::msg::StreamDataAdc2>(TOPIC_NAME_ADC2, QOS_DEPTH);
    _motion_publisher = create_publisher<phoenix_msgs::msg::StreamDataMotion>(TOPIC_NAME_MOTION, QOS_DEPTH);
    _imu_publisher = create_publisher<sensor_msgs::msg::Imu>(TOPIC_NAME_IMU, QOS_DEPTH);

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
                _status_queue.pop();
            }
            _status_queue.push(*reinterpret_cast<const StreamDataStatus_t *>(payload.data()));
            _queue_mutex.unlock();
            _condition_variable.notify_one();
        }
        break;

    case StreamIdAdc2:
        if (payload.size() == sizeof(StreamDataAdc2_t)) {
            _queue_mutex.lock();
            if (QUEUE_LENGTH <= _adc2_queue.size()) {
                _adc2_queue.pop();
            }
            _adc2_queue.push(*reinterpret_cast<const StreamDataAdc2_t *>(payload.data()));
            _queue_mutex.unlock();
            _condition_variable.notify_one();
        }
        break;

    case StreamIdMotion:
        if (payload.size() == sizeof(StreamDataMotion_t)) {
            _queue_mutex.lock();
            if (QUEUE_LENGTH <= _motion_queue.size()) {
                _motion_queue.pop();
            }
            _motion_queue.push(*reinterpret_cast<const StreamDataMotion_t *>(payload.data()));
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
            // StreamDataStatus_tをコピーする
            std::array<phoenix_msgs::msg::StreamDataStatus, QUEUE_LENGTH> buffer;
            int count = 0;
            do {
                convertStatus(_status_queue.front(), &buffer[count]);
                count++;
                _status_queue.pop();
            } while (!_status_queue.empty());

            // StreamDataStatusを配信する
            lock.unlock();
            for (int index = 0; index < count; index++) {
                _status_publisher->publish(buffer[index]);
            }
            lock.lock();
        }
        if (!_adc2_queue.empty()) {
            // StreamDataAdc2_tをコピーする
            std::array<phoenix_msgs::msg::StreamDataAdc2, QUEUE_LENGTH> buffer;
            int count = 0;
            do {
                convertAdc2(_adc2_queue.front(), &buffer[count]);
                count++;
                _adc2_queue.pop();
            } while (!_adc2_queue.empty());

            // StreamDataAdc2を配信する
            lock.unlock();
            for (int index = 0; index < count; index++) {
                _adc2_publisher->publish(buffer[index]);
            }
            lock.lock();
        }
        if (!_motion_queue.empty()) {
            // StreamDataMotion_tをコピーする
            std::array<phoenix_msgs::msg::StreamDataMotion, QUEUE_LENGTH> motion_buffer;
            std::array<sensor_msgs::msg::Imu, QUEUE_LENGTH> imu_buffer;
            int count = 0;
            do {
                convertMotion(_motion_queue.front(), &motion_buffer[count], &imu_buffer[count]);
                count++;
                _motion_queue.pop();
            } while (!_motion_queue.empty());

            // StreamDataMotionとImuを配信する
            lock.unlock();
            for (int index = 0; index < count; index++) {
                _motion_publisher->publish(motion_buffer[index]);
                _imu_publisher->publish(imu_buffer[index]);
            }
            lock.lock();
        }
    }
}

void StreamPublisherNode::convertStatus(const StreamDataStatus_t &data, phoenix_msgs::msg::StreamDataStatus *msg) {
    uint32_t error_flags = data.error_flags;
    uint32_t fault_flags = data.fault_flags;
    if (error_flags != 0) {
        msg->error_module_sleep = (error_flags & ErrorCauseModuleSleep);
        msg->error_fpga_stop = (error_flags & ErrorCauseFpgaStop);
        msg->error_dc48v_uv = (error_flags & ErrorCauseDc48vUnderVoltage);
        msg->error_dc48v_ov = (error_flags & ErrorCauseDc48vOverVoltage);
        msg->error_motor_oc[0] = (error_flags & ErrorCauseMotor1OverCurrent);
        msg->error_motor_oc[1] = (error_flags & ErrorCauseMotor2OverCurrent);
        msg->error_motor_oc[2] = (error_flags & ErrorCauseMotor3OverCurrent);
        msg->error_motor_oc[3] = (error_flags & ErrorCauseMotor4OverCurrent);
        msg->error_motor_oc[4] = (error_flags & ErrorCauseMotor5OverCurrent);
        msg->error_motor_hall_sensor[0] = (error_flags & ErrorCauseMotor1HallSensor);
        msg->error_motor_hall_sensor[1] = (error_flags & ErrorCauseMotor2HallSensor);
        msg->error_motor_hall_sensor[2] = (error_flags & ErrorCauseMotor3HallSensor);
        msg->error_motor_hall_sensor[3] = (error_flags & ErrorCauseMotor4HallSensor);
        msg->error_motor_hall_sensor[4] = (error_flags & ErrorCauseMotor5HallSensor);
    }
    if (fault_flags != 0) {
        msg->fault_adc2_timeout = (fault_flags & FaultCauseAdc2Timeout);
        msg->fault_imu_timeout = (fault_flags & FaultCauseImuTimeout);
        msg->fault_motor_ot[0] = (fault_flags & FaultCauseMotor1OverTemperature);
        msg->fault_motor_ot[1] = (fault_flags & FaultCauseMotor2OverTemperature);
        msg->fault_motor_ot[2] = (fault_flags & FaultCauseMotor3OverTemperature);
        msg->fault_motor_ot[3] = (fault_flags & FaultCauseMotor4OverTemperature);
        msg->fault_motor_ot[4] = (fault_flags & FaultCauseMotor5OverTemperature);
        msg->fault_motor_oc[0] = (fault_flags & FaultCauseMotor1OverCurrent);
        msg->fault_motor_oc[1] = (fault_flags & FaultCauseMotor2OverCurrent);
        msg->fault_motor_oc[2] = (fault_flags & FaultCauseMotor3OverCurrent);
        msg->fault_motor_oc[3] = (fault_flags & FaultCauseMotor4OverCurrent);
        msg->fault_motor_oc[4] = (fault_flags & FaultCauseMotor5OverCurrent);
        msg->fault_motor_load_switch[0] = (fault_flags & FaultCauseMotor1LoadSwitch);
        msg->fault_motor_load_switch[1] = (fault_flags & FaultCauseMotor2LoadSwitch);
        msg->fault_motor_load_switch[2] = (fault_flags & FaultCauseMotor3LoadSwitch);
        msg->fault_motor_load_switch[3] = (fault_flags & FaultCauseMotor4LoadSwitch);
        msg->fault_motor_load_switch[4] = (fault_flags & FaultCauseMotor5LoadSwitch);
    }
}

void StreamPublisherNode::convertAdc2(const StreamDataAdc2_t &data, phoenix_msgs::msg::StreamDataAdc2 *msg) {
    msg->dc48v_voltage = data.dc48v_voltage;
    msg->dribble_voltage = data.dribble_voltage;
    msg->dribble_current = data.dribble_current;
}

void StreamPublisherNode::convertMotion(const StreamDataMotion_t &data, phoenix_msgs::msg::StreamDataMotion *motion, sensor_msgs::msg::Imu *imu) {
    // StreamDataMotionに変換する
    for (int axis = 0; axis < 3; axis++) {
        motion->accelerometer[axis] = data.accelerometer[axis];
        motion->gyroscope[axis] = data.gyroscope[axis];
    }
    for (int index = 0; index < 4; index++) {
        motion->wheel_velocity_meas[index] = data.wheel_velocity_meas[index];
        motion->wheel_current_meas_d[index] = data.wheel_current_meas_d[index];
        motion->wheel_current_meas_q[index] = data.wheel_current_meas_q[index];
    }
    for (int index = 0; index < 4; index++) {
        motion->wheel_velocity_ref[index] = data.wheel_velocity_ref[index];
        motion->wheel_current_ref[index] = data.wheel_current_ref[index];
        motion->wheel_current_limit[index] = data.wheel_current_limit[index];
    }
    for (int index = 0; index < 3; index++) {
        motion->machine_velocity[index] = data.machine_velocity[index];
    }
    motion->slip_flags = data.slip_flags;
    motion->performance_counter = data.performance_counter;

    // IMU測定値をImuメッセージに変換する
    imu->header.stamp = getTimeStamp();
    if (!IMU_FRAME_ID.empty()) {
        imu->header.frame_id = IMU_FRAME_ID;
    }
    imu->orientation_covariance[0] = -1.0;
    imu->linear_acceleration.x = data.accelerometer[0];
    imu->linear_acceleration.y = data.accelerometer[1];
    imu->linear_acceleration.z = data.accelerometer[2];
    imu->angular_velocity.x = data.gyroscope[0];
    imu->angular_velocity.y = data.gyroscope[1];
    imu->angular_velocity.z = data.gyroscope[2];
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
