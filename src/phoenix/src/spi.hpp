#pragma once

#include <string>
#include <cstddef>
#include <mutex>

/**
 * @brief spidevのラッパークラス
 */
class Spi {
public:
    Spi(void) {}

    ~Spi() {
        closeDevice();
    }

    Spi(const Spi &) = delete;

    /**
     * @brief デバイスを開く
     * @param device_path デバイスパス
     * @param frequency ビットレート[Hz]
     * @return trueならデバイスは開いた。falseなら開けなかった。
     */
    bool openDevice(const std::string &device_path, unsigned int frequency);

    /**
     * @brief デバイスを閉じる
     */
    void closeDevice(void);

    /**
     * @brief デバイスが開けたか取得する
     * @return trueならデバイスは開いている。falseならデバイスは開いていない。
     */
    bool isOpened(void) const {
        return (_fd != -1);
    }

    /**
     * @brief SPIモードを設定する
     * @param mode SPIモード(0~3)
     * @return trueなら設定に成功した。falseなら失敗した。
     */
    bool setMode(int mode);

    /**
     * @brief データの書き込みと読み出しを同時に行う
     * @param length 読み書きするバイト数
     * @param write_data 書き込むデータが格納されたバッファへのポインタ
     * @param read_data 読み込んだデータを格納するバッファへのポインタ
     * @return trueなら読み書きに成功した。falseなら失敗した。
     */
    bool readWrite(size_t length, const void *write_data, void *read_data);

    /**
     * @brief データを書き込む
     * @param length 書き込むバイト数
     * @param write_data 書き込むデータが格納されたバッファへのポインタ
     * @return trueなら書き込みに成功した。falseなら失敗した。
     */
    bool writeData(size_t length, const void *write_data) {
        return readWrite(length, write_data, nullptr);
    }

    /**
     * @brief データの書き込んだ後に読み出しを行う
     * @param write_length 書き込むバイト数
     * @param write_data 書き込むデータが格納されたバッファへのポインタ
     * @param read_length 読み込むバイト数
     * @param read_data 読み込んだデータを格納するバッファへのポインタ
     * @return trueなら読み書きに成功した。falseなら失敗した。
     */
    bool readAfterWrite(size_t write_length, const void *write_data, size_t read_length, void *read_data);

private:
    std::mutex _mutex;
    int _fd = -1;
    unsigned int _frequency = 0;

    static constexpr int INVALID_FD = -1;
};
