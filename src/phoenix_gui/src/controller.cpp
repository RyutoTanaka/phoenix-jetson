#include "controller.hpp"
#include "ui_controller.h"
#include <QtWidgets/QGestureEvent>
#include "node_thread.hpp"

ControllerControls::ControllerControls(QWidget *parent) : QGroupBox(parent) {
    // UIを生成する
    _ui = new Ui_Controller;
    _ui->setupUi(this);

    _pad.scene = new QGraphicsScene(this);
    _ui->padGraphics->setScene(_pad.scene);
    _ui->padGraphics->viewport()->grabGesture(Qt::PanGesture);
    _ui->padGraphics->viewport()->grabGesture(Qt::PinchGesture);
    _ui->padGraphics->viewport()->installEventFilter(this);
    _pad.cross_h = _pad.scene->addLine(0, 0, 1, 0, QPen(Qt::black));
    _pad.cross_v = _pad.scene->addLine(0, 0, 0, 1, QPen(Qt::black));

    // グループボックスのチェックボックスが変更されたときに送信タイマーの作成・削除を行う
    connect(this, &QGroupBox::clicked, [this](bool checked) {
        if (checked && (_timer == nullptr)) {
            _timer = new QTimer(this);
            connect(_timer, &QTimer::timeout, this, &ControllerControls::transmitCommand);
            _timer->start(TRANSMIT_PERIOD);
        }
        else if (!checked && (_timer != nullptr)) {
            delete _timer;
            _timer = nullptr;
            transmitCommand();
        }
    });

    // ゲームパッドスレッドを作成する
    _ui->gamepadCombobox->addItem("None");
    _gamepad_thread = new GamepadThread();
    connect(_gamepad_thread, &GamepadThread::finished, _gamepad_thread, &QObject::deleteLater);
    connect(
        _gamepad_thread, &GamepadThread::gamepadConnected, this,
        [this](int device_id) {
            // 接続されたゲームパッドをコンボボックスに追加する
            _ui->gamepadCombobox->addItem(QString("XInput %1").arg(device_id), device_id);
        },
        Qt::QueuedConnection);
    connect(
        _gamepad_thread, &GamepadThread::gamepadDisconnected, this,
        [this](int device_id) {
            // 切断されたゲームパッドをコンボボックスから削除する
            int index = _ui->gamepadCombobox->findData(device_id);
            if (index == _ui->gamepadCombobox->currentIndex()) {
                _ui->gamepadCombobox->setCurrentIndex(0);
            }
            if (0 <= index) {
                _ui->gamepadCombobox->removeItem(index);
            }
        },
        Qt::QueuedConnection);
    connect(_ui->gamepadCombobox, qOverload<int>(&QComboBox::currentIndexChanged), this, &ControllerControls::connectToGamepad);
    _gamepad_thread->start();
}

ControllerControls::~ControllerControls() {
    // ゲームパッドスレッドを終了する
    _gamepad_thread->requestInterruption();
    _gamepad_thread->quit();
    _gamepad_thread->wait(NodeThread::QUIT_TIMEOUT);
    _gamepad_thread = nullptr;
}

bool ControllerControls::isAbsolute(void) const {
    return _ui->absoluteCheck->isChecked();
}

void ControllerControls::setPoseDisplay(const geometry_msgs::msg::Twist &twist, const geometry_msgs::msg::Pose &pose) {
    // To do
}

bool ControllerControls::eventFilter(QObject *obj, QEvent *event) {
    const QWidget *viewport = _ui->padGraphics->viewport();
    if (obj == viewport) {
        if (event->type() == QEvent::Gesture) {
            QGestureEvent *gesture_event = static_cast<QGestureEvent *>(event);
            if (QGesture *gesture = gesture_event->gesture(Qt::PanGesture)) {
                QPanGesture *pan = static_cast<QPanGesture *>(gesture);
                QSize size = viewport->size();
                _pad.velocity_scale_x = pan->offset().x() / (size.width() * 0.5);
                _pad.velocity_scale_y = -pan->offset().y() / (size.height() * 0.5);
                if ((pan->state() == Qt::GestureFinished) || (pan->state() == Qt::GestureCanceled)) {
                    _pad.velocity_scale_x = 0.0;
                    _pad.velocity_scale_y = 0.0;
                }
            }
            if (QGesture *gesture = gesture_event->gesture(Qt::PinchGesture)) {
                QPinchGesture *pinch = static_cast<QPinchGesture *>(gesture);
                QPinchGesture::ChangeFlags change_flags = pinch->changeFlags();
                if (change_flags & QPinchGesture::RotationAngleChanged) {
                    double d = pinch->rotationAngle() - pinch->lastRotationAngle();
                    if (d <= -180.0) {
                        d += 360.0;
                    }
                    else if (180.0 <= d) {
                        d -= 360.0;
                    }
                    _pad.velocity_scale_omega -= d;
                }
                if ((pinch->state() == Qt::GestureFinished) || (pinch->state() == Qt::GestureCanceled)) {
                    _pad.velocity_scale_omega = 0.0;
                }
            }
            return true;
        }
        else if (event->type() == QEvent::Resize) {
            int width = viewport->width();
            int height = viewport->height();
            _pad.scene->setSceneRect(0, 0, width, height);
            _pad.cross_h->setLine(0, height / 2, width, height / 2);
            _pad.cross_v->setLine(width / 2, 0, width / 2, height);
        }
    }
    return false;
}

void ControllerControls::transmitCommand(void) {
    bool gamepad_selected;
    int gamepad_device_id = _ui->gamepadCombobox->currentData().toInt(&gamepad_selected);
    if (gamepad_selected) {
        auto input_state = _gamepad_thread->inputState(gamepad_device_id);
        if (input_state) {
            _target_velocity.linear.x = 4.0f * input_state->left_stick_x;
            _target_velocity.linear.y = 4.0f * input_state->left_stick_y;
            _target_velocity.linear.z = -input_state->right_trigger;
            _target_velocity.angular.z = -10.0f * input_state->right_stick_x;
            emit commandReady();
        }
    }
}

void ControllerControls::connectToGamepad(int index) {
    // Do nothing
}
