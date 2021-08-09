#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

/**
 * @brief Avalon-ST Packets to Bytes Converterで変換されたバイトストリームをパケットに戻す
 */
class AvalonStBytesToPacketsConverter {
public:
    /**
     * @brief パーサーの内部状態を初期化する。
     */
    void reset(void) {
        _channel_number = -1;
        _next_will_be_escaped = 0;
        _next_will_be_channel = false;
        _next_will_be_sof = false;
    }

    /**
     * @brief 受信したデータをパースする。
     * パケット中のペイロードの1バイトごとにコールバック関数が呼ばれる。
     * @param data パースするデータへのポインタ
     * @param length パースするデータの長さ
     * @param callback ペイロードを1バイト処理するコールバック関数
     *                 callback(uint8_t data, int channel, bool sof, bool eof) -> bool
     */
    template<typename CallbackFunc>
    bool parse(const void *data, size_t length, CallbackFunc callback) {
        const uint8_t *data_u8 = reinterpret_cast<const uint8_t *>(data);
        while (0 < length--) {
            uint8_t c = *data_u8++;
            if ((_next_will_be_escaped != 0) || (c < 0x7A) || (0x7D < c)) {
                c ^= _next_will_be_escaped;
                _next_will_be_escaped = 0;
                if (_next_will_be_channel) {
                    _next_will_be_channel = false;
                    _channel_number = c;
                }
                else {
                    if (callback(c, _channel_number, _next_will_be_sof, _next_will_be_eof) == false) {
                        // コールバック関数がfalseを返したらパースを中断する
                        // 残りの入力データは破棄され、AvalonStBytesToPacketsConverterの内部状態は不定となる
                        return false;
                    }
                    _next_will_be_sof = false;
                    _next_will_be_eof = false;
                }
            }
            else if (c == 0x7A) {
                _next_will_be_sof = true;
            }
            else if (c == 0x7B) {
                _next_will_be_eof = true;
            }
            else if (c == 0x7C) {
                _next_will_be_channel = true;
            }
            else if (c == 0x7D) {
                _next_will_be_escaped = 0x20;
            }
        }
        return true;
    }

private:
    /// チャンネル番号
    int _channel_number = -1;

    /// 次のバイトはエスケープ処理されている
    uint8_t _next_will_be_escaped = 0;

    /// 次のバイトはチャンネル番号
    bool _next_will_be_channel = false;

    /// 次のバイトはSOF
    bool _next_will_be_sof = false;

    /// 次のバイトはEOF
    bool _next_will_be_eof = false;
};

/**
 * Avalon-ST Packets to Bytes Converterでパケットをバイトストリームに変換する
 */
class AvalonStPacketsToBytesConverter {
public:
    /**
     * @brief 変換を行う
     * @param payload ペイロード (長さ0のペイロードは長さ0のバイトストリームに変換される)
     * @param channel チャンネル番号 (0～255以外はチャンネル番号なし)
     * @param output 出力されたバイトストリーム。指定されたstd::vectorに追記される。
     */
    static void convert(const std::vector<uint8_t> &payload, int channel, std::vector<uint8_t> &output);
};
