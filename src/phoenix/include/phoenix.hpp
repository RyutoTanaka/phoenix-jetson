#pragma once

namespace phoenix {



/// 速度指令値を購読するトピック名
static constexpr char TOPIC_NAME_COMMAND_VELOCITY[] = "cmd_vel";

/// バッテリ状態を配信するトピック名
static constexpr char TOPIC_NAME_BATTERY[] = "battery_state";

/// IMU測定値を配信するトピック名
static constexpr char TOPIC_NAME_IMU[] = "imu";

/// オドメトリを配信するトピック名
static constexpr char TOPIC_NAME_ODOMETRY[] = "odom";

/// エラーフラグを配信するトピック名
static constexpr char TOPIC_NAME_STATUS[] = "status";

/// ADC2の測定値を配信するトピック名
static constexpr char TOPIC_NAME_ADC2[] = "adc2";

/// モーションに関する情報を配信するトピック名
static constexpr char TOPIC_NAME_MOTION[] = "motion";

/// エラーフラグをクリアするサービス名
static constexpr char SERVICE_NAME_CLEAR_ERROR[] = "clear_error";

/// Nios IIのプログラムを書き換えるサービス名
static constexpr char SERVICE_NAME_PROGRAM_NIOS[] = "program_nios";

/// FPGAを書き換えるサービス名
static constexpr char SERVICE_NAME_PROGRAM_FPGA[] = "program_fpga";



namespace battery {

/// ノード名
static constexpr char NODE_NAME[] = "phoenix_battery";

/// I2Cのデバイスパスのパラメータ名
static constexpr char PARAM_NAME_DEVICE_PATH[] = "device_path";

/// デバイスのI2Cアドレスを指定するパラメータ名
static constexpr char PARAM_NAME_DEVICE_ADDRESS[] = "device_address";

} // namespace battery



namespace command {

/// ノード名
static constexpr char NODE_NAME[] = "phoenix_command";

/// SPIのデバイスパスのパラメータ名
static constexpr char PARAM_NAME_DEVICE_PATH[] = "device_path";

/// 速度制御の比例ゲインのパラメータ名
static constexpr char PARAM_NAME_SPEED_KP[] = "speed_kp";

/// 速度制御の積分ゲインのパラメータ名
static constexpr char PARAM_NAME_SPEED_KI[] = "speed_ki";

/// 姿勢補正制御の比例ゲインのパラメータ名
static constexpr char PARAM_NAME_COMPENSATION_KP[] = "compensation_kp";

/// 姿勢補正制御の積分ゲインのパラメータ名
static constexpr char PARAM_NAME_COMPENSATION_KI[] = "compensation_ki";

} // namespace command



namespace stream {

/// ノード名
static constexpr char NODE_NAME[] = "phoenix_stream";

/// UARTのデバイスパスのパラメータ名
static constexpr char PARAM_NAME_DEVICE_PATH[] = "device_path";

} // namespace stream



} // namespace phoenix
