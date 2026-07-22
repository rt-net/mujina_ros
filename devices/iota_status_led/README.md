# IOTA ステータスLEDファーム

**LattePanda IOTA（RP2040）** 用のファームウェアで、Mujina ロボットの
RGB ステータスLEDを点灯させます。マイコンが ROS 2 ホストから「状態名」を USB シリアルで
受け取り、機体の電源ON、ROS 2 プログラムの動作、異常状態を
ひと目で分かるようにします。



受信する状態名は
[`mujina_control/mujina_control/interface/robot_mode_command.py`](../../mujina_control/mujina_control/interface/robot_mode_command.py)
の `RobotModeCommand` に対応しています。ホスト側の小さなノード
（例: `mujina_control/mujina_control/serial_state_bridge.py`）が、現在のモードを
シリアルポートへ周期的に書き込むことを想定しています。
`TRANSITION_TO_STANDBY` は `STANDBY`、`TRANSITION_TO_STANDUP` は `STANDUP` として送信されます。
`DEBUG` はステータスLEDへの送信対象外です。

## ハードウェア

- **ボード:** LattePanda IOTA（RP2040）、36ピンヘッダ
- **LED:** コモンカソードRGB LED（コモンアノードの場合はスケッチ内の `LED_COMMON_ANODE = true` に変更）
- **ピン（すべて PWM）:**

  | 色    | GPIO |
  |-------|------|
  | 赤    | GP2  |
  | 緑    | GP3  |
  | 青    | GP4  |

電流制限抵抗を入れ、コモンピンは GND（コモンカソード）または 3V3（コモンアノード）へ
接続してください。

## 受信状態 / 状態 → LED

| 状態             | LED                          | 意味                                   |
|------------------|------------------------------|----------------------------------------|
| topic未受信      | 黄・フェード明滅（0.5Hz）    | 電源ON（既定 ＝ リセット先）           |
| `STANDBY`        | 緑・フェード明滅（0.5Hz）    | プログラム動作中 / 待機                |
| `STANDUP`        | 緑・フェード明滅（0.5Hz）    | プログラム動作中                       |
| `WALK`           | 緑・点灯                     | WALK中                                 |
| `CALIBRATING`    | 橙・点滅（1Hz）              | 原点キャリブレーション中               |
| `ERROR`          | 赤・点滅（1Hz）              | 異常                                   |
| `EMERGENCY_STOP` | 赤・点灯                     | 非常停止（ブレーキモード）             |

起動直後、および **1秒間** 有効な状態名が来ない場合は topic未受信（黄フェード明滅）表示へ戻ります。

`STANDBY` は ROS 2 プログラムから topic が届いている状態として扱うため、黄色ではなく
緑で表示します。黄色はロボットの待機モードではなく、マイコンに電源が入っていて、
ROS 2 側から有効な状態名を受け取っていない状態を示します。


## ビルドと書き込み

1. Arduino IDE / arduino-cli に **arduino-pico** コア（Earle Philhower）をインストール。
2. ボードは **Raspberry Pi Pico**（RP2040）を選択。
3. `iota_status_led.ino` を開く（`iota_status_led/` スケッチフォルダ内に置いたまま使う）。
4. USB 経由で書き込み（初回でポートが見えない場合は **BOOTSEL** を押しながら接続）。
