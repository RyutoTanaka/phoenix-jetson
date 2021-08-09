#include "uart.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

bool Uart::openDevice(const std::string &device_path) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_fd != INVALID_FD) {
        close(_fd);
    }
    _fd = open(device_path.c_str(), O_RDWR);
    if (_fd == INVALID_FD) {
        return false;
    }
    do {
        // 初期設定を読み込んでバイナリ送受信用にそれを編集する
        struct termios setting;
        if (tcgetattr(_fd, &setting) != 0) {
            break;
        }
        cfmakeraw(&setting);
        setting.c_cflag |= CREAD | CLOCAL; // 受信を有効化、制御信号を無効化する
        setting.c_cflag &= ~CSTOPB;        // ストップビットは1ビット
        setting.c_cc[VMIN] = 0;            // 1バイトでも受信したらread()が返る
        setting.c_cc[VTIME] = 1;           // read()のタイムアウトは1/10秒
        if (tcsetattr(_fd, TCSANOW, &setting) != 0) {
            break;
        }
        return true;
    } while (false);
    closeDevice();
    return false;
}

void Uart::closeDevice() {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_fd != -1) {
        close(_fd);
        _fd = -1;
    }
}

bool Uart::setBaudrate(speed_t baudrate) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!isOpened()) {
        return false;
    }
    struct termios setting;
    if (tcgetattr(_fd, &setting) != 0) {
        return false;
    }
    if (cfsetspeed(&setting, baudrate) != 0) {
        return false;
    }
    if (tcsetattr(_fd, TCSANOW, &setting) != 0) {
        return false;
    }
    return true;
}

bool Uart::setParityBitEnabled(bool enable, bool odd) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!isOpened()) {
        return false;
    }
    struct termios setting;
    if (tcgetattr(_fd, &setting) != 0) {
        return false;
    }
    if (enable) {
        setting.c_cflag |= PARENB;
        if (odd) {
            setting.c_cflag |= PARODD;
        }
        else {
            setting.c_cflag &= ~PARODD;
        }
    }
    else {
        setting.c_cflag &= ~PARENB;
    }
    if (tcsetattr(_fd, TCSANOW, &setting) != 0) {
        return false;
    }
    return true;
}

bool Uart::setLongStopBitEnabled(bool enable) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!isOpened()) {
        return false;
    }
    struct termios setting;
    if (tcgetattr(_fd, &setting) != 0) {
        return false;
    }
    if (enable) {
        setting.c_cflag |= CSTOPB;
    }
    else {
        setting.c_cflag &= ~CSTOPB;
    }
    if (tcsetattr(_fd, TCSANOW, &setting) != 0) {
        return false;
    }
    return true;
}

bool Uart::setIgnoreBreakEnabled(bool enable) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!isOpened()) {
        return false;
    }
    struct termios setting;
    if (tcgetattr(_fd, &setting) != 0) {
        return false;
    }
    if (enable) {
        setting.c_cflag |= IGNBRK;
    }
    else {
        setting.c_cflag &= ~IGNBRK;
    }
    if (tcsetattr(_fd, TCSANOW, &setting) != 0) {
        return false;
    }
    return true;
}

bool Uart::writeData(size_t length, const void *data, size_t *written_length) {
    ssize_t result = write(_fd, data, length);
    if (result < 0) {
        return false;
    }
    if (written_length != nullptr) {
        *written_length = result;
    }
    return true;
}

bool Uart::readData(size_t length, void *data, size_t *read_length) {
    ssize_t result = read(_fd, data, length);
    if (result < 0) {
        return false;
    }
    if (read_length != nullptr) {
        *read_length = result;
    }
    return true;
}
