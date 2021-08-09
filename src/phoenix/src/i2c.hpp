#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

class I2c {
public:
    I2c() {}

    ~I2c() {
        closeDevice();
    }

    I2c(const I2c &) = delete;

    /**
     * @brief デバイスを開く
     * @param device_path デバイスパス
     * @return trueならデバイスは開いた。falseなら開けなかった。
     */
    bool openDevice(const std::string &device_path);

    /**
     * @brief デバイスを閉じる
     */
    void closeDevice();

    /**
     * @brief デバイスが開けたか取得する
     * @return trueならデバイスは開いている。falseならデバイスは開いていない。
     */
    bool isOpened(void) const {
        return (_fd != -1);
    }

    /**
     * @brief レジスタからデータを読み込む
     * @param dev_addr I2Cアドレス
     * @param reg_value レジスタアドレス
     * @param data_length 読み込むバイト数
     * @param data_ptr 読み込んだデータを格納するバッファへのポインタ
     * @return trueなら読み込みに成功した。falseなら失敗した。
     */
    bool readData(uint8_t dev_addr, uint8_t reg_value, size_t data_length, void *data_ptr) {
        return readData(dev_addr, 1, &reg_value, data_length, data_ptr);
    }

    /**
     * @brief レジスタからデータを読み込む
     * @param dev_addr I2Cアドレス
     * @param reg_length レジスタアドレスのバイト数
     * @param reg_ptr レジスタアドレスが格納されたバッファへのポインタ
     * @param data_length 読み込むバイト数
     * @param data_ptr 読み込んだデータを格納するバッファへのポインタ
     * @return trueなら読み込みに成功した。falseなら失敗した。
     */
    bool readData(uint8_t dev_addr, size_t reg_length, const void *reg_ptr, size_t data_length, void *data_ptr);

    /**
     * @brief レジスタにデータを書き込む
     * @param dev_addr I2Cアドレス
     * @param reg_value レジスタアドレス
     * @param data_length 書き込むバイト数
     * @param data_ptr 書き込むデータが格納されたバッファへのポインタ
     * @return trueなら書き込みに成功した。falseなら失敗した。
     */
    bool writeData(uint8_t dev_addr, uint8_t reg_value, size_t data_length, const void *data_ptr) {
        return writeData(dev_addr, 1, &reg_value, data_length, data_ptr);
    }

    /**
     * @brief レジスタにデータを書き込む
     * @param dev_addr I2Cアドレス
     * @param reg_length レジスタアドレスのバイト数
     * @param reg_ptr レジスタアドレスが格納されたバッファへのポインタ
     * @param data_length 書き込むバイト数
     * @param data_ptr 書き込むデータが格納されたバッファへのポインタ
     * @return trueなら書き込みに成功した。falseなら失敗した。
     */
    bool writeData(uint8_t dev_addr, size_t reg_length, const void *reg_ptr, size_t data_length, const void *data_ptr);

private:
    std::mutex _mutex;
    int _fd = -1;
};
