# jsk_aerial_robot 運用・段階的変更マニュアル

## 2026-11-19 12:23｜低レベル変更（`servo_bridge` / `spinal`）の段階分離運用
- SessionID: `3d99fae6-96b8-403e-9d80-fa4b37354751`
- 経緯:
  盛岡飛行ベース（`fix/servo-bridge-timestamp` / `eda9ada3`）から低レベル周辺（`servo_bridge`・STM32 `spinal`）を一括変更した実験ブランチ `fix/spinal-gimbal1-wind-safety`（`b1529d48`）を，実機試験での切り分け不能と制御ループ改変リスクを避けるため，単一責務の段階別ブランチへ分離した．

### ブランチ構成（基準：`fix/servo-bridge-timestamp` = `eda9ada3`）

| 段階 | ブランチ名 | 先頭コミット | 変更範囲 | STM32 書換 | 用途・方針 |
|---|---|---|---|---|---|
| Stage 1（推奨既定） | `fix/servo-bridge-gimbal1-only` | `20d90d5c` (+本doc) | `aerial_robot_model/src/servo_bridge/servo_bridge.cpp` のみ（`+21` 行） | 不要（盛岡のまま） | `leus` PC 側で `/gimbalrotor1/servo/states` の重複 `index=0` パディングをスキップし，`joint_states/gimbal1 = -1.57 rad` 上書きバグのみを単独解消する．STM32 への影響ゼロ |
| Stage 2（任意・通信圧縮） | `fix/spinal-servo-len-only` | `cdeefb2d` | Stage 1 + `.../Jsk_Lib/servo/servo.cpp` の `sendData()` / `setGoalAngle` のみ（`+13/-8` 行） | 必要 | Stage 1 確認後，STM32 側 `servo/states` パケット長を実サーボ数 `4`（`68 B → 40 B`）へ短縮したい場合のみ使用．`DirectServo::init` の `MAX_SERVO_NUM` 配列確保と `attitude_control.h/.cpp` は一切無改変 |
| Archive（実験退避・非投入） | `fix/spinal-gimbal1-wind-safety` | `b1529d48` | `servo_bridge.cpp` + `servo.cpp`（`init` 動的確保含む）+ `attitude_control.h/.cpp`（`+86/-32` 行） | 必要 | 400 Hz 姿勢制御ループ改変を含む一括試験アーカイブ．実機にはそのまま書き込まない |

### 使い方（Stage 1：STM32 無改変で `gimbal1` 修正のみ適用）

```bash
# leus / ThinkPad の jsk_aerial_robot で Stage 1 ブランチを反映・ビルド
git -C ~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot checkout fix/servo-bridge-gimbal1-only
cd ~/ros/jsk_aerial_robot_ws
catkin build --no-status aerial_robot_model
```

地上確認ポイント（STM32 書き換え不要）：
1. `/gimbalrotor1/joint_states` の `gimbal1` が `-1.5700 rad` に張り付かず実角度（`0 rad` 近辺）を追従すること．
2. `/gimbalrotor1/torque_allocation_matrix_inv` にスパイクや異常値が出ないこと．

### 使い方（Stage 2：任意で STM32 の `servo/states` 送信長のみ圧縮する場合）

Stage 1 が実機で安定した後に，送信パケット長（`8 → 4`）だけを整えたい場合のみ実施する（`attitude_control.cpp` は盛岡版と同一）：

```bash
git -C ~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot checkout fix/spinal-servo-len-only
export PATH="/opt/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.linux64_1.0.0.202410170706/tools/bin:$PATH"
make -C ~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_nerve/spinal/mcu_project/boards/stm32H7/STM32CubeIDE/Debug -j4 all
/opt/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI \
  -c port=SWD -d ~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_nerve/spinal/mcu_project/boards/stm32H7/STM32CubeIDE/Debug/spinal.elf -v -rst
```
