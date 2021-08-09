#include "avalon_mm.hpp"
#include "avalon_st.hpp"
#include <limits>
#include <cstring>

bool AvalonMm::writeData(uint32_t address, size_t length, const void *data) {
    if ((length <= 0) || (std::numeric_limits<uint16_t>::max() < length)) {
        return false;
    }

    // 送信データを作成する
    std::vector<uint8_t> tx_payload_bytes(8 + length), tx_stream_bytes, tx_physical_bytes;
    initializeHeader(0x04, length, address, tx_payload_bytes);
    memcpy(tx_payload_bytes.data() + 8, data, length);
    AvalonStPacketsToBytesConverter::convert(tx_payload_bytes, CHANNEL_NUMBER, tx_stream_bytes);
    streamBytesToPhysicalBytes(tx_stream_bytes, tx_physical_bytes, 12);

    // SPI転送を行う
    std::vector<uint8_t> rx_physical_bytes(tx_physical_bytes.size());
    if (!_spi->readWrite(tx_physical_bytes.size(), tx_physical_bytes.data(), rx_physical_bytes.data())) {
        return false;
    }

    // レスポンスを確認する
    std::vector<uint8_t> rx_stream_bytes, rx_payload_bytes;
    physicalBytesToStreamBytes(rx_physical_bytes, rx_stream_bytes);
    if (!streamBytesToPayloadBytes(rx_stream_bytes, rx_payload_bytes)) {
        return false;
    }
    if ((rx_payload_bytes.size() != 4) || (rx_payload_bytes[0] != 0x84)) {
        return false;
    }
    return true;
}

bool AvalonMm::readData(uint32_t address, size_t length, void *data) {
    if (length <= 0) {
        return true;
    }

    // 送信データを作成する
    std::vector<uint8_t> tx_payload_bytes(8), tx_stream_bytes, tx_physical_bytes;
    initializeHeader(0x14, length, address, tx_payload_bytes);
    AvalonStPacketsToBytesConverter::convert(tx_payload_bytes, CHANNEL_NUMBER, tx_stream_bytes);
    streamBytesToPhysicalBytes(tx_stream_bytes, tx_physical_bytes, 4 + length * 2);

    // SPI転送を行う
    std::vector<uint8_t> rx_physical_bytes(tx_physical_bytes.size());
    if (!_spi->readWrite(tx_physical_bytes.size(), tx_physical_bytes.data(), rx_physical_bytes.data())) {
        return false;
    }

    // レスポンスを確認する
    std::vector<uint8_t> rx_stream_bytes, rx_payload_bytes;
    physicalBytesToStreamBytes(rx_physical_bytes, rx_stream_bytes);
    if (!streamBytesToPayloadBytes(rx_stream_bytes, rx_payload_bytes)) {
        return false;
    }
    if (rx_payload_bytes.size() != length) {
        return false;
    }
    memcpy(data, rx_payload_bytes.data(), length);
    return true;
}

void AvalonMm::initializeHeader(int type, size_t length, uint32_t address, std::vector<uint8_t> &output) {
    output[0] = static_cast<uint8_t>(type);
    output[1] = 0;
    output[2] = static_cast<uint8_t>(length >> 8);
    output[3] = static_cast<uint8_t>(length);
    output[4] = static_cast<uint8_t>(address >> 24);
    output[5] = static_cast<uint8_t>(address >> 16);
    output[6] = static_cast<uint8_t>(address >> 8);
    output[7] = static_cast<uint8_t>(address);
}

void AvalonMm::streamBytesToPhysicalBytes(const std::vector<uint8_t> &stream_bytes, std::vector<uint8_t> &physical_bytes, size_t extend_idle) {
    physical_bytes.reserve(physical_bytes.size() + stream_bytes.size() * 2 + extend_idle);
    for (uint8_t data : stream_bytes) {
        if ((data != 0x4A) && (data != 0x4D)) {
            physical_bytes.push_back(data);
        }
        else {
            physical_bytes.push_back(0x4D);
            physical_bytes.push_back(data ^ 0x20);
        }
    }
    if (0 < extend_idle) {
        size_t length = physical_bytes.size();
        physical_bytes.resize(length + extend_idle);
        memset(physical_bytes.data() + length, 0x4A, extend_idle);
    }
}

void AvalonMm::physicalBytesToStreamBytes(const std::vector<uint8_t> &physical_bytes, std::vector<uint8_t> &stream_bytes) {
    stream_bytes.reserve(stream_bytes.size() + physical_bytes.size());
    bool escape = false;
    for (uint8_t data : physical_bytes) {
        if (escape == true) {
            escape = false;
            stream_bytes.push_back(data ^ 0x20);
        }
        else if (data == 0x4D) {
            escape = true;
        }
        else if (data != 0x4A) {
            stream_bytes.push_back(data);
        }
    }
}

bool AvalonMm::streamBytesToPayloadBytes(const std::vector<uint8_t> &stream_bytes, std::vector<uint8_t> &payload_bytes) {
    bool sof_received = false, eof_received = false;
    auto callback = [&](uint8_t data, int channel, bool sof, bool eof) {
        (void)channel;
        // if (channel != CHANNEL_NUMBER) {
        //    return true;
        //} // チャンネル番号は取りこぼしてしまうので無視する
        if (!sof_received && !sof && eof) {
            return false;
        }
        if ((sof_received || eof_received) && sof) {
            return false;
        }
        sof_received |= sof;
        if (sof_received && !eof_received) {
            payload_bytes.push_back(data);
        }
        eof_received |= eof;
        return true;
    };
    payload_bytes.reserve(payload_bytes.size() + stream_bytes.size());
    AvalonStBytesToPacketsConverter converter;
    if (converter.parse(stream_bytes.data(), stream_bytes.size(), callback) == false) {
        return false;
    }
    return sof_received && eof_received;
}
