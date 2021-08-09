#pragma once

#include <string>
#include <cstdint>
#include <mutex>
#include <termios.h>

class Uart {
public:
    Uart() {}

    ~Uart() {
        closeDevice();
    }

    Uart(const Uart&) = delete;

    /**
     * @brief デバイスを開く
     * 初期設定はパリティビット無し、ストップビット1、ボーレートはデフォルト値である。
     * @param device_path デバイスパス
     * @return trueならデバイスは開いた。falseなら開けなかった。
     */
    bool openDevice(const std::string &device_path);

    /**
     * @brief デバイスが開けたか取得する
     * @return trueならデバイスは開いている。falseなら開いていない。
     */
    bool isOpened(void) const {
        return (_fd != INVALID_FD);
    }

    /**
     * @brief デバイスを閉じる
     */
    void closeDevice();

    /**
     * @brief ボーレートを設定する
     * @param baudrate ボーレート。B9600, B19200, B38400, etc. といったマクロで定義されている値を指定する。
     * @return trueなら設定に成功した。falseなら失敗した。
     */
    bool setBaudrate(speed_t baudrate);

    /**
     * @brief パリティビットを設定する
     * @param enable trueならパリティビットを有効化する。パリティの極性はodd引数で指定する。
     * @param odd trueなら奇数パリティ、falseなら偶数パリティを使用する。
     * @return trueなら設定に成功した。falseなら失敗した。
     */
    bool setParityBitEnabled(bool enable, bool odd);

    /**
     * @brief ストップビットの長さを設定する
     * @param enable trueならストップビットを2ビットにする。falseなら1ビットにする。
     * @return trueなら設定に成功した。falseなら失敗した。
     */
    bool setLongStopBitEnabled(bool enable);
    
    /**
     * @brief ブレーク信号を無視するか設定する
     * @param enable trueならブレーク信号を無視する。falseならヌル文字に変換される。
     * @return trueなら設定に成功した。falseなら失敗した。
     */
    bool setIgnoreBreakEnabled(bool enable);

    /**
     * @brief データを送信する
     * @param length 送信するデータのバイト数
     * @param data 送信するデータが格納されたバッファへのポインタ
     * @param written_length 実際に送信されたバイト数
     * @return trueなら送信に成功した。falseなら失敗した。
     */
    bool writeData(size_t length, const void *data, size_t *written_length = nullptr);

    // データを受信する
    // data        : 受信したデータを格納するバッファへのポインタ
    // length      : 受信するデータのバイト数
    // read_length : 実際に受信したバイト数

    /**
     * @brief データを受信する
     * @param length 受信するデータのバイト数
     * @param data 受信したデータを格納するバッファへのポインタ
     * @param read_length 実際に受信したバイト数
     * @return trueなら受信に成功した。falseなら失敗した。
     */
    bool readData(size_t length, void *data, size_t *read_length = nullptr);

private:
    std::mutex _mutex;
    int _fd = -1;

    static constexpr int INVALID_FD = -1;
};
