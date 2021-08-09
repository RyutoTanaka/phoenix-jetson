#include "i2c.hpp"
#include <limits>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

bool I2c::openDevice(const std::string &device_path) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_fd != -1) {
        close(_fd);
    }
    _fd = open(device_path.c_str(), O_RDWR);
    return (_fd != -1);
}

void I2c::closeDevice(void) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_fd != -1) {
        close(_fd);
        _fd = -1;
    }
}

bool I2c::readData(uint8_t dev_addr, size_t reg_length, const void *reg_ptr, size_t data_length, void *data_ptr) {
    if ((std::numeric_limits<uint16_t>::max() < reg_length) || (std::numeric_limits<uint16_t>::max() < data_length)){
        return false;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (!isOpened()) {
        return false;
    }
    struct i2c_msg messages[] = {
        {dev_addr, 0, static_cast<uint16_t>(reg_length), const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(reg_ptr))},
        {dev_addr, I2C_M_RD, static_cast<uint16_t>(data_length), reinterpret_cast<uint8_t *>(data_ptr)},
    };
    struct i2c_rdwr_ioctl_data ioctl_data = {messages, 2};
    if (ioctl(_fd, I2C_RDWR, &ioctl_data) != 2) {
        RCUTILS_LOG_ERROR("I2c::Read failed to ioctl: %s\n", strerror(errno));
        return false;
    }
    return true;
}

bool I2c::writeData(uint8_t dev_addr, size_t reg_length, const void *reg_ptr, size_t data_length, const void *data_ptr) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (std::numeric_limits<uint16_t>::max() < (reg_length + data_length)) {
        return false;
    }
    std::vector<uint8_t> buffer(reg_length + data_length);
    memcpy(buffer.data(), reg_ptr, reg_length);
    memcpy(buffer.data() + reg_length, data_ptr, data_length);
    struct i2c_msg message = {dev_addr, 0, static_cast<uint16_t>(buffer.size()), buffer.data()};
    struct i2c_rdwr_ioctl_data ioctl_data = {&message, 1};
    if (ioctl(_fd, I2C_RDWR, &ioctl_data) != 1) {
        RCUTILS_LOG_ERROR("I2c::Write failed to ioctl: %s\n", strerror(errno));
        return false;
    }
    return true;
}
