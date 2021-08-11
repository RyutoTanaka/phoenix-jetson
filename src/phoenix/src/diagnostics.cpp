#include "diagnostics.hpp"
#include <status_flags.hpp>
#include <sstream>
#include <cctype>

#ifndef _MSC_VER
extern "C" {
#include <sys/utsname.h>
}
#else
#include <Windows.h>
#endif

namespace phoenix {

static const std::pair<uint32_t, const char *> ERROR_CAUSE_TABLE[] = {
    {ErrorCauseModuleSleep, "Module Sleep"},
    {ErrorCauseFpgaStop, "Fpga Stop"},
    {ErrorCauseDc48vUnderVoltage, "DC48V Under Voltage"},
    {ErrorCauseDc48vOverVoltage, "DC48V Over Voltage"},
    {ErrorCauseMotor1OverCurrent, "Motor 1 Over Current"},
    {ErrorCauseMotor2OverCurrent, "Motor 2 Over Current"},
    {ErrorCauseMotor3OverCurrent, "Motor 3 Over Current"},
    {ErrorCauseMotor4OverCurrent, "Motor 4 Over Current"},
    {ErrorCauseMotor5OverCurrent, "Motor 5 Over Current"},
    {ErrorCauseMotor1HallSensor, "Motor 1 Hall Sensor"},
    {ErrorCauseMotor2HallSensor, "Motor 2 Hall Sensor"},
    {ErrorCauseMotor3HallSensor, "Motor 3 Hall Sensor"},
    {ErrorCauseMotor4HallSensor, "Motor 4 Hall Sensor"},
    {ErrorCauseMotor5HallSensor, "Motor 5 Hall Sensor"},
};

static const std::pair<uint32_t, const char *> FAULT_CAUSE_TABLE[] = {
    {FaultCauseAdc2Timeout, "ADC2 Timeout"},
    {FaultCauseImuTimeout, "IMU Timeout"},
    {FaultCauseMotor1OverTemperature, "Motor Driver 1 Over Temperature"},
    {FaultCauseMotor2OverTemperature, "Motor Driver 2 Over Temperature"},
    {FaultCauseMotor3OverTemperature, "Motor Driver 3 Over Temperature"},
    {FaultCauseMotor4OverTemperature, "Motor Driver 4 Over Temperature"},
    {FaultCauseMotor5OverTemperature, "Motor Driver 5 Over Temperature"},
    {FaultCauseMotor1OverCurrent, "Motor Driver 1 Over Current"},
    {FaultCauseMotor2OverCurrent, "Motor Driver 2 Over Current"},
    {FaultCauseMotor3OverCurrent, "Motor Driver 3 Over Current"},
    {FaultCauseMotor4OverCurrent, "Motor Driver 4 Over Current"},
    {FaultCauseMotor5OverCurrent, "Motor Driver 5 Over Current"},
    {FaultCauseMotor1LoadSwitch, "Load Switch 1 Trip"},
    {FaultCauseMotor2LoadSwitch, "Load Switch 2 Trip"},
    {FaultCauseMotor3LoadSwitch, "Load Switch 3 Trip"},
    {FaultCauseMotor4LoadSwitch, "Load Switch 4 Trip"},
    {FaultCauseMotor5LoadSwitch, "Load Switch 5 Trip"},
};

template<size_t N>
static inline std::string flagsToString(uint32_t flags, const std::pair<uint32_t, const char *> (&table)[N]) {
    std::stringstream ss;
    bool add_separator = false;
    for (auto &item : table) {
        if (flags & item.first) {
            if (add_separator) {
                ss << ", ";
            }
            ss << item.second;
            add_separator = true;
        }
    }
    return ss.str();
}

void createFpgaDiagnostics(const StreamDataStatus_t &status, diagnostic_msgs::msg::DiagnosticStatus &diag) {
    const uint32_t error_flags = status.error_flags;
    const uint32_t fault_flags = status.fault_flags;

    if ((error_flags == 0) && (fault_flags == 0)) {
        // エラーは起きていない
        diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diag.message = "OK";
        return;
    }
    else {
        // 何らかのエラーが起きている
        diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        if ((error_flags == 0xFFFFFFFFUL) || (fault_flags == 0xFFFFFFFFUL)) {
            diag.message = "Invalid Response";
        }
        else if (fault_flags != 0) {
            diag.message = "Fault";
        }
        else {
            diag.message = "Error";
        }

        // エラーの要因を列挙する
        diag.values.reserve(4);
        if (error_flags != 0) {
            diagnostic_msgs::msg::KeyValue item1, item2;
            item1.key = "Error Causes";
            if (error_flags != 0xFFFFFFFFUL) {
                item1.value = flagsToString(error_flags, ERROR_CAUSE_TABLE);
            }
            else {
                item1.value = "Invalid";
            }
            diag.values.push_back(std::move(item1));

            item2.key = "Error Flags";
            item2.value = std::to_string(error_flags);
            diag.values.push_back(std::move(item2));
        }
        if (fault_flags != 0) {
            diagnostic_msgs::msg::KeyValue item1, item2;
            item1.key = "Fault Causes";
            if (fault_flags != 0xFFFFFFFFUL) {
                item1.value = flagsToString(fault_flags, FAULT_CAUSE_TABLE);
            }
            else {
                item1.value = "Invalid";
            }
            diag.values.push_back(std::move(item1));

            item2.key = "Fault Flags";
            item2.value = std::to_string(fault_flags);
            diag.values.push_back(std::move(item2));
        }
    }
}

std::string getHostName(void) {
#ifndef _MSC_VER
    struct utsname buffer;
    if (uname(&buffer) < 0) {
        return {};
    }
    return buffer.nodename;
#else
    char buffer[MAX_COMPUTERNAME_LENGTH + 1];
    DWORD length = sizeof(buffer);
    if (!GetComputerNameA(buffer, &length)) {
        return {};
    }
    return std::string(buffer, length);
#endif
}

std::string getRegularHostName(void) {
    std::string result = getHostName();
    if (!result.empty()) {
        // アルファベットと数字以外の文字をアンダースコアに置換する
        for (char &c : result) {
            if (!std::isalnum(c)) {
                c = '_';
            }
        }

        // 数字で始まる場合は先頭にアンダースコアを追加する
        if (isdigit(result[0])) {
            result = '_' + result;
        }
    }
    return result;
}

} // namespace phoenix
