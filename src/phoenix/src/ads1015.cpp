#include "ads1015.hpp"

bool ADS1015::initialize() {
    uint16_t data;
    return readRegister(REG_CONFIGURATION, &data);
}

bool ADS1015::startConversion(MUX_t mux, FSR_t fsr, DR_t dr) {
    uint16_t config = 0x8103 | (mux << 12) | (fsr << 9) | (dr << 5);
    return writeRegister(REG_CONFIGURATION, config);
}

bool ADS1015::isConversionCompleted(bool *complete) {
    *complete = false;
    uint16_t data;
    if (!readRegister(REG_CONFIGURATION, &data)) {
        return false;
    }
    if (data & 0x8000) {
        *complete = true;
    }
    return true;
}

bool ADS1015::getConversionResult(int16_t *result) {
    *result = 0;
    uint16_t data;
    if (!readRegister(REG_CONVERSION_DATA, &data)) {
        return false;
    }
    *result = data;
    return true;
}

bool ADS1015::readRegister(REG_t reg, uint16_t *data) {
    uint8_t buf[2];
    if (!_i2c->readData(_address, static_cast<uint8_t>(reg), sizeof(buf), buf)) {
        return false;
    }
    *data = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    return true;
}

bool ADS1015::writeRegister(REG_t reg, uint16_t data) {
    uint8_t buf[2];
    buf[0] = data >> 8;
    buf[1] = data;
    return _i2c->writeData(_address, static_cast<uint8_t>(reg), sizeof(buf), buf);
}
