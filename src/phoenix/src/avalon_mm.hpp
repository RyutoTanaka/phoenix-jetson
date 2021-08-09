#pragma once

#include "spi.hpp"
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>

/**
 * @brief SPI Slave to Avalon Master Bridgeを使用してメモリー操作する
 */
class AvalonMm {
public:
    /**
     * コンストラクタ
     * @param spi 通信に使用するSpi
     */
    AvalonMm(std::shared_ptr<Spi> &spi) : _spi(spi) {}

    /**
     * @brief メモリーにデータを書き込む
     * @param address 書き込むアドレス
     * @param data 書き込むデータへのポインタ
     * @param length 書き込むデータの長さ
     */
    bool writeData(uint32_t address, size_t length, const void *data);

    /**
     * @brief メモリーに32bitのデータを書き込む
     * @param address 書き込むアドレス
     * @param data 書き込むデータ
     */
    bool writeData(uint32_t address, uint32_t data) {
        return writeData(address, 4, &data);
    }

    /**
     * @brief メモリーからデータを読み込む
     * @param address 読み込むアドレス
     * @param data 読み込んだデータの格納先へのポインタ
     * @param length 読み込むデータの長さ
     */
    bool readData(uint32_t address, size_t length, void *data);

    /**
     * @brief メモリーから32bitのデータを読み込む
     * @param address 読み込むアドレス
     * @param data 読み込んだデータの格納先へのポインタ
     */
    bool readData(uint32_t address, uint32_t *data) {
        return readData(address, 4, data);
    }

private:
    static void initializeHeader(int type, size_t length, uint32_t address, std::vector<uint8_t> &output);

    static void streamBytesToPhysicalBytes(const std::vector<uint8_t> &stream_bytes, std::vector<uint8_t> &physical_bytes, size_t extend_idle = 0);

    static void physicalBytesToStreamBytes(const std::vector<uint8_t> &physical_bytes, std::vector<uint8_t> &stream_bytes);

    static bool streamBytesToPayloadBytes(const std::vector<uint8_t> &stream_bytes, std::vector<uint8_t> &payload_bytes);

    static constexpr int CHANNEL_NUMBER = 0;

    std::shared_ptr<Spi> _spi;
};
