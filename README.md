# phoenix-jetson
[phoenix-pcb](https://github.com/Nkyoku/phoenix-pcb)および[phoenix-firmware](https://github.com/Nkyoku/phoenix-firmware)を制御するROS2パッケージ

## 開発環境
- Jetson Nano Developer Kit SD Card Image 32.4.4
- ROS2 Foxy
- Qt 5.15.2
- VSCode

## ファイル
- phoenix-firmware
  対応するバージョンのファームウェアのリポジトリ
- src
  - phoenix  
  基板上に存在するデバイスを制御するノード類を収めたパッケージ
  - phoenix_msgs  
  phoenixパッケージのノードが用いるトピックやサービスのメッセージ
  - phoenix_gui  
  phoenixパッケージのノードから受信したデータを表示したり  
  コントローラでロボットを操縦するパッケージ
