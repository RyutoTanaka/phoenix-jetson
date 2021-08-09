#include "epcq.hpp"
#include <cstring>
#include <chrono>
#include <vector>

bool Epcq::readSilliconId(uint8_t *sillicon_id) {
    uint8_t write_data[5] = {0xAB, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t read_data[5];
    if (_spi->readWrite(sizeof(write_data), write_data, read_data)) {
        *sillicon_id = read_data[4];
        return true;
    }
    return false;
}

bool Epcq::readStatus(uint8_t *status) {
    uint8_t write_data[2] = {0x05, 0xFF};
    uint8_t read_data[2];
    if (_spi->readWrite(sizeof(write_data), write_data, read_data)) {
        *status = read_data[1];
        return true;
    }
    return false;
}

bool Epcq::readData(uint32_t address, size_t length, void *data) {
    std::vector<uint8_t> write_data(4 + length);
    std::vector<uint8_t> read_data(4 + length);
    write_data[0] = 0x03;
    write_data[1] = (address >> 16) & 0xFF;
    write_data[2] = (address >> 8) & 0xFF;
    write_data[3] = address & 0xFF;
    if (_spi->readWrite(write_data.size(), write_data.data(), read_data.data())) {
        memcpy(data, read_data.data() + 4, length);
        return true;
    }
    return false;
}

bool Epcq::writePage(uint32_t address, const void *data) {
    uint8_t write_data[4 + PAGE_SIZE] = {0x02};
    write_data[1] = (address >> 16) & 0xFF;
    write_data[2] = (address >> 8) & 0xFF;
    write_data[3] = address & 0xFF;
    memcpy(write_data + 4, data, PAGE_SIZE);
    if (setWriteEnabled(true)) {
        if (_spi->writeData(sizeof(write_data), write_data)) {
            return waitProgressBit(100);
        }
    }
    return false;
}

bool Epcq::eraseSector(uint32_t address) {
    uint8_t write_data[4] = {0xD8};
    write_data[1] = (address >> 16) & 0xFF;
    write_data[2] = (address >> 8) & 0xFF;
    write_data[3] = address & 0xFF;
    if (setWriteEnabled(true)) {
        if (_spi->writeData(sizeof(write_data), write_data)) {
            return waitProgressBit(2500);
        }
    }
    return false;
}

bool Epcq::setWriteEnabled(bool enabled) {
    uint8_t write_data[1];
    write_data[0] = enabled ? 0x06 : 0x04;
    return _spi->writeData(sizeof(write_data), write_data);
}

bool Epcq::waitProgressBit(int timeout_in_ms) {
    auto timeout = std::chrono::milliseconds(timeout_in_ms);
    auto start = std::chrono::system_clock::now();
    uint8_t status = 0xFF;
    while (readStatus(&status)) {
        if (~status & STATUS_WIP) {
            return true;
        }
        if (timeout < (std::chrono::system_clock::now() - start)) {
            break;
        }
    }
    return false;
}
