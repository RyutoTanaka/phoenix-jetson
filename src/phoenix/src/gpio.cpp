#include "gpio.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <thread>

#include <stdio.h>

Gpio::Gpio(int pin_number) : _is_opened(false), _pin_number(pin_number) {
    if ((pin_number < 0) || (255 < pin_number)) {
        return;
    }

    _gpio_value_path = "/sys/class/gpio/gpio" + std::to_string(pin_number) + "/value";
    _gpio_direction_path = "/sys/class/gpio/gpio" + std::to_string(pin_number) + "/direction";

    if (_permission_checked == false) {
        // アクセス権限をチェックする
        printf("Permission Check\n");
        _permission_checked = true;
        if ((access(EXPORT_PATH, W_OK) == 0) && (access(UNEXPORT_PATH, W_OK) == 0)) {
            printf("Permission Ok\n");
            _permission_ok = true;
        }
    }

    if (_permission_ok == true) {
        // GPIOを有効化する
        printf("Enable GPIO %d\n", _pin_number);
        std::ofstream export_stream(EXPORT_PATH);
        export_stream << std::to_string(_pin_number);
        export_stream.close();

        // GPIOにアクセス可能になったか確認する
        printf("Check GPIO %d\n", _pin_number);
        int timeout = 100;
        while (access(_gpio_value_path.c_str(), R_OK | W_OK) != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (--timeout < 0) {
                printf("Timeout '%s'\n", _gpio_value_path.c_str());
                return;
            }
        }
        while (access(_gpio_direction_path.c_str(), W_OK) != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (--timeout < 0) {
                printf("Timeout '%s'\n", _gpio_direction_path.c_str());
                return;
            }
        }

        _is_opened = true;
    }
}

Gpio::~Gpio() {
    if (_is_opened == false) {
        return;
    }

    // GPIOを無効化する
    printf("Disable GPIO %d\n", _pin_number);
    std::ofstream unexport_stream(UNEXPORT_PATH);
    unexport_stream << std::to_string(_pin_number);
    unexport_stream.close();
}

void Gpio::setOutputEnabled(bool enabled) {
    if (_is_opened == false) {
        return;
    }
    std::ofstream direction_stream(_gpio_direction_path);
    direction_stream << (enabled ? "out" : "in");
}

void Gpio::setOutputValue(bool value) {
    if (_is_opened == false) {
        return;
    }
    std::ofstream value_stream(_gpio_value_path);
    value_stream << (value ? "1" : "0");
}

bool Gpio::getInputValue(void) {
    if (_is_opened == false) {
        return false;
    }
    std::ifstream value_stream(_gpio_value_path);
    int result;
    value_stream >> result;
    return result != 0;
}

const char Gpio::EXPORT_PATH[] = "/sys/class/gpio/export";
const char Gpio::UNEXPORT_PATH[] = "/sys/class/gpio/unexport";
bool Gpio::_permission_checked = false;
bool Gpio::_permission_ok = false;
