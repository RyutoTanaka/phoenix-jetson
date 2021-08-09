#include "spi.hpp"
#include <vector>
#include <limits>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

bool Spi::openDevice(const std::string &device_path, unsigned int frequency) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_fd != INVALID_FD) {
        close(_fd);
    }
    _fd = open(device_path.c_str(), O_RDWR);
    if (_fd == INVALID_FD) {
        return false;
    }
    __u8 lsb_first_u8 = 0, bits_per_word_u8 = 8;
    __u32 frequency_u32 = static_cast<__u32>(frequency);
    ioctl(_fd, SPI_IOC_WR_LSB_FIRST, &lsb_first_u8);
    ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_u8);
    ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &frequency_u32);
    _frequency = frequency;
    return (_fd != INVALID_FD);
}

void Spi::closeDevice(void) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_fd != INVALID_FD) {
        close(_fd);
        _fd = INVALID_FD;
    }
}

bool Spi::setMode(int mode) {
    static const int mode_table[4] = {SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3};
    if ((mode < 0) || (3 < mode)) {
        return false;
    }
    __u8 mode_u8 = mode_table[mode];
    return (ioctl(_fd, SPI_IOC_WR_MODE, &mode_u8) == 0);
}

bool Spi::readWrite(size_t length, const void *write_data, void *read_data) {
    if (std::numeric_limits<__u32>::max() < length) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    spi_ioc_transfer transfer;
    memset(&transfer, 0, sizeof(transfer));
    transfer.tx_buf = reinterpret_cast<__u64>(write_data);
    transfer.rx_buf = reinterpret_cast<__u64>(read_data);
    transfer.len = static_cast<__u32>(length);
    if (ioctl(_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
        return false;
    }
    return true;
}

bool Spi::readAfterWrite(size_t write_length, const void *write_data, size_t read_length, void *read_data) {
    if ((std::numeric_limits<__u32>::max() < write_length) || (std::numeric_limits<__u32>::max() < read_length)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    spi_ioc_transfer transfer[2];
    memset(&transfer, 0, sizeof(transfer));
    transfer[0].tx_buf = reinterpret_cast<__u64>(write_data);
    transfer[0].len = static_cast<__u32>(write_length);
    transfer[1].rx_buf = reinterpret_cast<__u64>(read_data);
    transfer[1].len = static_cast<__u32>(read_length);
    if (ioctl(_fd, SPI_IOC_MESSAGE(2), transfer) < 0) {
        return false;
    }
    return true;
}
