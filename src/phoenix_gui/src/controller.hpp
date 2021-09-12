#pragma once

#include "gamepad_thread.hpp"
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QGraphicsEllipseItem>
#include <QtWidgets/QGraphicsLineItem>
#include <QtWidgets/QGraphicsScene>
#include <QtCore/QTimer>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

struct ControllerCommand {};

class Ui_Controller;

class ControllerControls : public QGroupBox {
    Q_OBJECT

public:
    ControllerControls(QWidget *parent = nullptr);

    ~ControllerControls();

    /**
     * @brief 絶対位置指令が有効かどうか取得する
     * @return trueなら絶対位置指令が有効である。falseなら無効である。
     */
    bool isAbsolute(void) const;

    /**
     * @brief 指令速度を取得する
     * @return 指令速度
     */
    const geometry_msgs::msg::Twist &targetVelocity(void) const {
        return _target_velocity;
    }

    /**
     * @brief 指令位置を取得する
     * @return 指令位置
     */
    const geometry_msgs::msg::Pose &targetPosition(void) const {
        return _target_position;
    }

    Q_SLOT void setPoseDisplay(const geometry_msgs::msg::Twist &twist, const geometry_msgs::msg::Pose &pose);

    Q_SIGNAL void commandReady(void);

private:
    virtual bool eventFilter(QObject *obj, QEvent *event) override;

    Q_SLOT void connectToGamepad(int index);

    Q_SLOT void transmitCommand(void);

    /// Qt Designerで作成したUI
    Ui_Controller *_ui = nullptr;

    /// ゲームパッド入力を処理するスレッド
    GamepadThread *_gamepad_thread = nullptr;

    /// 指令値の送信タイマー
    QTimer *_timer = nullptr;

    /// 指令速度
    geometry_msgs::msg::Twist _target_velocity;

    /// 指令位置
    geometry_msgs::msg::Pose _target_position;

    struct {
        QGraphicsScene *scene;
        QGraphicsLineItem *cross_h, *cross_v;
        double velocity_scale_x;     // -1.0 ~ +1.0
        double velocity_scale_y;     // -1.0 ~ +1.0
        double velocity_scale_omega; // degree
    } _pad;

    /// 指令値の送信間隔[ms]
    static constexpr int TRANSMIT_PERIOD = 50;
};
