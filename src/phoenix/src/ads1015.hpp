#pragma once

#include <cstdint>
#include <memory>
#include "i2c.hpp"

class ADS1015 {
public:
    /// マルチプレクサの設定値
    enum MUX_t {
        MUX_AIN1_to_AIN0 = 0x0,
        MUX_AIN3_to_AIN0 = 0x1,
        MUX_AIN3_to_AIN1 = 0x2,
        MUX_AIN3_to_AIN2 = 0x3,
        MUX_GND_to_AIN0 = 0x4,
        MUX_GND_to_AIN1 = 0x5,
        MUX_GND_to_AIN2 = 0x6,
        MUX_GND_to_AIN3 = 0x7,
    };

    /// フルスケール電圧の設定値
    enum FSR_t {
        FSR_6144mV = 0x0,
        FSR_4096mV = 0x1,
        FSR_2048mV = 0x2,
        FSR_1024mV = 0x3,
        FSR_512mV = 0x4,
        FSR_256mV = 0x5,
    };

    /// 変換レートの設定値
    enum DR_t {
        DR_128SPS = 0x0,
        DR_250SPS = 0x1,
        DR_490SPS = 0x2,
        DR_920SPS = 0x3,
        DR_1600SPS = 0x4,
        DR_2400SPS = 0x5,
        DR_3300SPS = 0x6,
    };

    ADS1015(std::shared_ptr<I2c> &i2c, uint8_t dev_addr) : _i2c(i2c), _address(dev_addr) {}

    ADS1015(const ADS1015 &) = delete;

    /**
     * @brief デバイスを初期化する
     * @return trueなら初期化に成功した。falseなら失敗した。
     */
    bool initialize(void);

    /**
     * @brief 変換を開始する
     * @param mux マルチプレクサ
     * @param fsr フルスケール電圧
     * @param dr 変換レート
     * @return trueなら変換の開始に成功した。falseなら失敗した。
     */
    bool startConversion(MUX_t mux, FSR_t fsr, DR_t dr);

    /**
     * @brief 変換が完了したか取得する
     * @param complete 変換が完了したかどうかが格納される
     * @return trueなら取得に成功した。falseなら失敗した。
     */
    bool isConversionCompleted(bool *complete);

    /**
     * @brief 変換結果を取得する
     * @param result 変換結果を格納する変数へのポインタ
     * @return trueなら取得に成功した。falseなら失敗した。
     */
    bool getConversionResult(int16_t *result);

private:
    /// レジスタ
    enum REG_t {
        REG_CONVERSION_DATA = 0x00,
        REG_CONFIGURATION = 0x01,
    };

    /**
     * @brief レジスタを読み込む
     * @param reg 読み込むレジスタ
     * @param data 読み込んだデータを格納する変数へのポインタ
     * @return trueなら読み込みに成功した。falseなら失敗した。
     */
    bool readRegister(REG_t reg, uint16_t *data);

    /**
     * @brief レジスタを書き込む
     * @param reg 書き込むレジスタ
     * @param data 書き込むデータ
     * @return trueなら書き込みに成功した。falseなら失敗した。
     */
    bool writeRegister(REG_t reg, uint16_t data);

    std::shared_ptr<I2c> _i2c;
    uint8_t _address = 0;
};
