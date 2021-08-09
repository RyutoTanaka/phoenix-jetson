#include "command_node.hpp"
#include "gpio.hpp"
#include "avalon_st.hpp"
#include "epcq.hpp"
#include <chrono>
#include <thread>
#include <cstring>

using namespace std::chrono_literals;

namespace phoenix {

const std::string CommandServerNode::PARAM_NAME_DEVICE_PATH("device_path");
const std::string CommandServerNode::PARAM_NAME_SPEED_KP("speed_kp");
const std::string CommandServerNode::PARAM_NAME_SPEED_KI("speed_ki");
const std::string CommandServerNode::PARAM_NAME_COMPENSATION_KP("compensation_kp");
const std::string CommandServerNode::PARAM_NAME_COMPENSATION_KI("compensation_ki");

static const std::string DEFAULT_DEVICE_PATH("/dev/spidev1.0");
static constexpr double DEFAULT_SPEED_KP = 5.0;
static constexpr double DEFAULT_SPEED_KI = 0.05;
static constexpr double DEFAULT_COMPENSATION_KP = 2.0;
static constexpr double DEFAULT_COMPENSATION_KI = 0.02;

/// Nios IIの共有メモリーのベースアドレス
static constexpr uint32_t NIOS_SHARED_RAM_BASE = 0x0u;

/// Nios IIの命令メモリーのベースアドレス
static constexpr uint32_t NIOS_INSTRUCTION_RAM_BASE = 0xA5A50000u;

/// FPGAのアプリケーションビットストリームのベースアドレス
static constexpr uint32_t FPGA_APPLICATION_BASE = 0x100000u;

/// FPGAのアプリケーションビットストリームのサイズ
static constexpr uint32_t FPGA_APPLICATION_SIZE = 0x100000u;

/// FPGA_MODEピン
static constexpr auto FPGA_MODE = Gpio::JetsonNanoModulePinGpio12;

/// FPGA_CONFIG_Nピン
static constexpr auto FPGA_CONFIG_N = Gpio::JetsonNanoModulePinGpio11;

// FPGA_APPLICATION_BASEはセクターサイズの倍数である必要がある
static_assert((FPGA_APPLICATION_BASE % Epcq::SECTOR_SIZE) == 0, "FPGA_APPLICATION_BASE must be multiple of Epcq::SECTOR_SIZE");

// FPGA_APPLICATION_SIZEはセクターサイズの倍数である必要がある
static_assert((FPGA_APPLICATION_SIZE % Epcq::SECTOR_SIZE) == 0, "FPGA_APPLICATION_SIZE must be multiple of Epcq::SECTOR_SIZE");

// セクターサイズはページサイズの倍数である必要がある
static_assert((Epcq::SECTOR_SIZE % Epcq::PAGE_SIZE) == 0, "Epcq::SECTOR_SIZE must be multiple of Epcq::PAGE_SIZE");

/// ビット順序を反転するテーブル
static constexpr uint8_t BIT_REVERSAL_TABLE[256] = {
    0, 128, 64, 192, 32, 160, 96,  224, 16, 144, 80, 208, 48, 176, 112, 240, 8,  136, 72, 200, 40, 168, 104, 232, 24, 152, 88, 216, 56, 184, 120, 248,
    4, 132, 68, 196, 36, 164, 100, 228, 20, 148, 84, 212, 52, 180, 116, 244, 12, 140, 76, 204, 44, 172, 108, 236, 28, 156, 92, 220, 60, 188, 124, 252,
    2, 130, 66, 194, 34, 162, 98,  226, 18, 146, 82, 210, 50, 178, 114, 242, 10, 138, 74, 202, 42, 170, 106, 234, 26, 154, 90, 218, 58, 186, 122, 250,
    6, 134, 70, 198, 38, 166, 102, 230, 22, 150, 86, 214, 54, 182, 118, 246, 14, 142, 78, 206, 46, 174, 110, 238, 30, 158, 94, 222, 62, 190, 126, 254,
    1, 129, 65, 193, 33, 161, 97,  225, 17, 145, 81, 209, 49, 177, 113, 241, 9,  137, 73, 201, 41, 169, 105, 233, 25, 153, 89, 217, 57, 185, 121, 249,
    5, 133, 69, 197, 37, 165, 101, 229, 21, 149, 85, 213, 53, 181, 117, 245, 13, 141, 77, 205, 45, 173, 109, 237, 29, 157, 93, 221, 61, 189, 125, 253,
    3, 131, 67, 195, 35, 163, 99,  227, 19, 147, 83, 211, 51, 179, 115, 243, 11, 139, 75, 203, 43, 171, 107, 235, 27, 155, 91, 219, 59, 187, 123, 251,
    7, 135, 71, 199, 39, 167, 103, 231, 23, 151, 87, 215, 55, 183, 119, 247, 15, 143, 79, 207, 47, 175, 111, 239, 31, 159, 95, 223, 63, 191, 127, 255,
};

CommandServerNode::CommandServerNode(const rclcpp::NodeOptions &options) : Node("phoenix_command") {
    using namespace std::placeholders;
    (void)options;

    // 共有メモリーを初期化する
    memset(&_shared_memory, 0, sizeof(_shared_memory));

    // パラメータを宣言し値を取得する
    auto device_path = declare_parameter<std::string>(PARAM_NAME_DEVICE_PATH, DEFAULT_DEVICE_PATH);
    declare_parameter<double>(PARAM_NAME_SPEED_KP, DEFAULT_SPEED_KP);
    declare_parameter<double>(PARAM_NAME_SPEED_KI, DEFAULT_SPEED_KI);
    declare_parameter<double>(PARAM_NAME_COMPENSATION_KP, DEFAULT_COMPENSATION_KP);
    declare_parameter<double>(PARAM_NAME_COMPENSATION_KI, DEFAULT_COMPENSATION_KI);

    // SPIデバイスを開く
    _spi = std::make_shared<Spi>();
    if (!_spi->openDevice(device_path, SPI_FREQUENCY)) {
        RCUTILS_LOG_FATAL("Cannot open '%s'.", device_path.c_str());
        throw;
    }
    _spi->setMode(FPGA_SPI_MODE);
    _avalon_mm = std::make_shared<AvalonMm>(_spi);

    // サービスを登録する
    rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, QOS_DEPTH));
    _clear_error_service = create_service<phoenix_msgs::srv::ClearError>("clear_error", std::bind(&CommandServerNode::clearErrorCallback, this, _1, _2, _3));
    _set_speed_service = create_service<phoenix_msgs::srv::SetSpeed>("set_speed", std::bind(&CommandServerNode::setSpeedCallback, this, _1, _2, _3));
    _program_nios_service =
        create_service<phoenix_msgs::srv::ProgramNios>("program_nios", std::bind(&CommandServerNode::programNiosCallback, this, _1, _2, _3));
    _program_fpga_service =
        create_service<phoenix_msgs::srv::ProgramFpga>("program_fpga", std::bind(&CommandServerNode::programFpgaCallback, this, _1, _2, _3));
}

void CommandServerNode::clearErrorCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                           const std::shared_ptr<phoenix_msgs::srv::ClearError::Request> request,
                                           const std::shared_ptr<phoenix_msgs::srv::ClearError::Response> response) {
    (void)request_header;
    (void)request;

    response->succeeded = false;
    response->any_errors = false;

    // 現在のErrorFlagsを確認する
    uint32_t error_flags;
    if (!_avalon_mm->readData(static_cast<uint32_t>(offsetof(SharedMemory_t, ErrorFlags)), &error_flags)) {
        return;
    }
    if (error_flags != 0) {
        // 何らかのエラーが発生しているので消去を試みる
        // ErrorFlagsに0xFFFFFFFFを書き込むとエラーフラグの消去を要求できる
        if (!_avalon_mm->writeData(static_cast<uint32_t>(offsetof(SharedMemory_t, ErrorFlags)), 0xFFFFFFFFUL)) {
            return;
        }

        // ErrorFlagsをポーリングして変化したか確かめる
        int timeout = 3;
        do {
            std::this_thread::sleep_for(1ms);
            if (!_avalon_mm->readData(static_cast<uint32_t>(offsetof(SharedMemory_t, ErrorFlags)), &error_flags)) {
                return;
            }
            if (error_flags != 0xFFFFFFFFUL) {
                break;
            }
            if (--timeout < 0) {
                return;
            }
        } while (true);
        response->any_errors = (error_flags != 0);
    }
    response->succeeded = true;
}

void CommandServerNode::setSpeedCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                         const std::shared_ptr<phoenix_msgs::srv::SetSpeed::Request> request,
                                         const std::shared_ptr<phoenix_msgs::srv::SetSpeed::Response> response) {
    (void)request_header;

    // パラメータをコピーする
    _shared_memory.Parameters.FrameNumber++;
    _shared_memory.Parameters.speed_x = request->speed_x;
    _shared_memory.Parameters.speed_y = request->speed_y;
    _shared_memory.Parameters.speed_omega = request->speed_omega;
    _shared_memory.Parameters.dribble_power = request->dribble_power;
    _shared_memory.Parameters.speed_gain_p = std::fmaxf(0.0f, getFloatParameter(PARAM_NAME_SPEED_KP));
    _shared_memory.Parameters.speed_gain_i = std::fmaxf(0.0f, getFloatParameter(PARAM_NAME_SPEED_KI));
    _shared_memory.Parameters.compensation_gain_p = std::fmaxf(0.0f, getFloatParameter(PARAM_NAME_COMPENSATION_KP));
    _shared_memory.Parameters.compensation_gain_i = std::fmaxf(0.0f, getFloatParameter(PARAM_NAME_COMPENSATION_KI));

    // チェックサムを計算して格納する
    uint32_t checksum = _shared_memory.Parameters.CalculateChecksum();
    _shared_memory.HeadChecksum = checksum;
    _shared_memory.TailChecksum = checksum;

    // パラメータと前後のチェックサムを書き込む
    response->succeeded = _avalon_mm->writeData(NIOS_SHARED_RAM_BASE + static_cast<uint32_t>(offsetof(SharedMemory_t, HeadChecksum)),
                                                sizeof(SharedMemory_t::Parameters_t) + sizeof(uint32_t) * 2, &_shared_memory.HeadChecksum);
}

void CommandServerNode::programNiosCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                            const std::shared_ptr<phoenix_msgs::srv::ProgramNios::Request> request,
                                            const std::shared_ptr<phoenix_msgs::srv::ProgramNios::Response> response) {
    (void)request_header;

    // GPIOを開く
    Gpio mode_pin(FPGA_MODE);
    Gpio config_pin(FPGA_CONFIG_N);
    if (!mode_pin.isOpened() || !config_pin.isOpened()) {
        response->succeeded = false;
        return;
    }

    // FPGA_MODEからLを出力してNios IIをリセットする
    mode_pin.setOutputEnabled(true);
    mode_pin.setOutputValue(false);

    // 一度の転送で送るデータのバイト数
    static constexpr size_t BYTES_PER_TRANSFER = 32;

    // プログラムを転送する
    bool succeeded = true;
    size_t written_bytes = 0;
    do {
        size_t length = std::min(request->program.size() - written_bytes, BYTES_PER_TRANSFER);
        if (!_avalon_mm->writeData(NIOS_INSTRUCTION_RAM_BASE + written_bytes, length, request->program.data() + written_bytes)) {
            RCUTILS_LOG_ERROR("programNios() was failed at _avalon_mm->writeData(%zu, %zu, [%zu]).", NIOS_INSTRUCTION_RAM_BASE + written_bytes, length,
                              written_bytes);
            succeeded = false;
            break;
        }
        written_bytes += length;
    } while (written_bytes < request->program.size());

    if (succeeded) {
        // ベリファイする
        size_t read_bytes = 0;
        do {
            uint8_t buffer[BYTES_PER_TRANSFER];
            size_t length = std::min(request->program.size() - read_bytes, BYTES_PER_TRANSFER);
            if (!_avalon_mm->readData(NIOS_INSTRUCTION_RAM_BASE + read_bytes, length, buffer)) {
                RCUTILS_LOG_ERROR("programNios() was failed at _avalon_mm->readData(%zu, %zu, [0]).", NIOS_INSTRUCTION_RAM_BASE + read_bytes, length);
                succeeded = false;
                break;
            }
            if (memcmp(buffer, request->program.data() + read_bytes, length) != 0) {
                RCUTILS_LOG_ERROR("programNios() was failed at memcmp([0], [%zu], %zu).", read_bytes, length);
                succeeded = false;
                break;
            }
            read_bytes += length;
        } while (read_bytes < request->program.size());
    }

    if (succeeded) {
        // 共有メモリーを消去する
        memset(&_shared_memory, 0, sizeof(_shared_memory));
        _avalon_mm->writeData(NIOS_SHARED_RAM_BASE, sizeof(SharedMemory_t), &_shared_memory);
    }

    if (succeeded) {
        // FPGA_MODEを解放してNios IIのリセットを解除する
        mode_pin.setOutputEnabled(false);
    }
    else {
        // FPGA_CONFIG_NからLを出力してFPGAをリセットする
        config_pin.setOutputEnabled(true);
        config_pin.setOutputValue(false);

        // FPGA_MODEを解放する
        mode_pin.setOutputEnabled(false);

        // FPGAをリコンフィギュレーションする
        config_pin.setOutputEnabled(false);
    }

    response->succeeded = succeeded;
}

void CommandServerNode::programFpgaCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                            const std::shared_ptr<phoenix_msgs::srv::ProgramFpga::Request> request,
                                            const std::shared_ptr<phoenix_msgs::srv::ProgramFpga::Response> response) {
    (void)request_header;

    // GPIOを開く
    Gpio mode_pin(FPGA_MODE);
    Gpio config_pin(FPGA_CONFIG_N);
    if (!mode_pin.isOpened() || !config_pin.isOpened()) {
        response->succeeded = false;
        return;
    }

    // FPGA_CONFIG_NからLを出力してFPGAをリセットする
    config_pin.setOutputEnabled(true);
    config_pin.setOutputValue(false);

    // FPGA_MODEをLowにしておく
    // FPGAのリコンフィギュレーション後にファクトリーモードのまま維持できる
    mode_pin.setOutputEnabled(true);
    mode_pin.setOutputValue(false);

    // FPGA_CONFIG_Nを解放してFPGAをリコンフィギュレーションする
    config_pin.setOutputEnabled(false);

    // 250ms待つ
    std::this_thread::sleep_for(250ms);

    // 書き込むデータのビット順序を反転する
    for (uint32_t address = FPGA_APPLICATION_BASE; address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE); address++) {
        request->bitstream[address] = BIT_REVERSAL_TABLE[request->bitstream[address]];
    }

    // フラッシュメモリーを操作する
    _spi->setMode(EPCQ_SPI_MODE);
    Epcq epcq(_spi);
    bool succeeded = false;
    do {
        // シリコンIDを読み取る
        uint8_t sillicon_id;
        if (!epcq.readSilliconId(&sillicon_id)) {
            RCUTILS_LOG_ERROR("programFpga() was failed at epcq.readSilliconId().");
            break;
        }
        if ((sillicon_id == 0xFF) || (sillicon_id == 0x4A)) {
            RCUTILS_LOG_ERROR("programFpga() was failed because the sillicon ID is 0x%02X.", sillicon_id);
            break;
        }
        RCUTILS_LOG_INFO("Sillicon ID is 0x%02X.", sillicon_id);

        // セクターを消去する
        uint32_t address;
        for (address = FPGA_APPLICATION_BASE; address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE); address += Epcq::SECTOR_SIZE) {
            if (!epcq.eraseSector(address)) {
                RCUTILS_LOG_ERROR("programFpga() was failed at epcq.eraseSector(0x%06X).", address);
                break;
            }
        }
        if (address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE)) {
            break;
        }

        // ページに書き込む
        for (address = FPGA_APPLICATION_BASE; address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE); address += Epcq::PAGE_SIZE) {
            const uint8_t *data = &request->bitstream[address];
            bool empty = true;
            for (uint32_t index = 0; index < Epcq::PAGE_SIZE; index++) {
                if (data[index] != 0xFF) {
                    empty = false;
                    break;
                }
            }
            if (!empty) {
                if (!epcq.writePage(address, data)) {
                    RCUTILS_LOG_ERROR("programFpga() was failed at epcq.writePage(0x%06X).", address);
                    break;
                }
            }
        }
        if (address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE)) {
            break;
        }

        // ベリファイする
        static constexpr uint32_t VERIFY_SIZE = 1024;
        static_assert((FPGA_APPLICATION_SIZE % VERIFY_SIZE) == 0, "VERIFY_SIZE must be divisor of FPGA_APPLICATION_SIZE");
        for (address = FPGA_APPLICATION_BASE; address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE); address += VERIFY_SIZE) {
            const uint8_t *written_data = &request->bitstream[address];
            bool empty = true;
            for (uint32_t index = 0; index < VERIFY_SIZE; index++) {
                if (written_data[index] != 0xFF) {
                    empty = false;
                    break;
                }
            }
            if (!empty) {
                uint8_t read_data[VERIFY_SIZE];
                if (!epcq.readData(address, VERIFY_SIZE, read_data)) {
                    RCUTILS_LOG_ERROR("programFpga() was failed at epcq.readData(0x%06X, %u).", address, VERIFY_SIZE);
                    break;
                }
                if (memcmp(written_data, read_data, VERIFY_SIZE) != 0) {
                    RCUTILS_LOG_ERROR("programFpga() was failed because verification error at 0x%06X.", address);
                    break;
                }
            }
        }
        if (address < (FPGA_APPLICATION_BASE + FPGA_APPLICATION_SIZE)) {
            break;
        }

        // 操作は正常に終了した
        succeeded = true;
    } while (false);
    _spi->setMode(FPGA_SPI_MODE);

    if (succeeded) {
        // FPGA_MODEを解放してアプリケーションを起動する
        mode_pin.setOutputEnabled(false);
        std::this_thread::sleep_for(250ms);
    }

    response->succeeded = succeeded;
}

float CommandServerNode::getFloatParameter(const std::string &parameter_name, float default_value) {
    auto parameter = get_parameter(parameter_name);
    switch (parameter.get_type()) {
    case rclcpp::PARAMETER_DOUBLE:
        return parameter.as_double();

    case rclcpp::PARAMETER_INTEGER:
        return parameter.as_int();

    default:
        return default_value;
    }
}

} // namespace phoenix

#if PHOENIX_BUILD_LIBRARY
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(phoenix::CommandServerNode)
#elif PHOENIX_BUILD_STANDALONE
int main(int argc, char *argv[]) {
    // ROS2ノードを起動する
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<phoenix::CommandServerNode>());
    rclcpp::shutdown();
    return 0;
}
#endif
