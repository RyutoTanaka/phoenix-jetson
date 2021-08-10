#pragma once

#include <stream_data.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace phoenix {

/**
 * @brief FPGAの診断ステータスを作成する。
 * 最後に受信したステータスを元に診断ステータスを作成する。
 * @param diag 診断ステータス
 */
void createFpgaDiagnostics(const StreamDataStatus_t &status, diagnostic_msgs::msg::DiagnosticStatus &diag);

}
