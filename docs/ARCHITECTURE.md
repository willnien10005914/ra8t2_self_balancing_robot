# Architecture — Dual-Wheel LQR on RA8T2

## Control hierarchy

```
┌─────────────────────────────────────────────────────────────┐
│  Command sources (low rate / event)                         │
│  Console UART │ AI Box UART │ RC (future SCI/CAN/CRSF)      │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  cmd_arbiter  (priority + timeout)                          │
│  ESTOP > Console manual motor > AI follow > RC > Hold/LQR  │
│  Output: motion_cmd_t { vx, wz, brake, lqr_en, ... }        │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  balance_ctrl @ 200–500 Hz  (balmode: LQR | PID | off)      │
│  Inputs: IMU pitch/rate, hall wheel pos/speed, motion_cmd   │
│  LQR: x=[θ,θ̇,φ,φ̇] → common/diff speed                      │
│  PID: vel→lean, pitch PID→v_common, yaw PID→差速            │
│  brake: vx=0, wz=0, keep balancer (station-keeping)         │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  FOC 內環（FSP rm_motor_hall）：速度 PI → 電流 PI → PWM     │
│  Left + Right；Hall 120° / 15 對極（見 motor_params.h）     │
└─────────────────────────────────────────────────────────────┘
```

啟用平衡時，console/AI/RC 只改 **參考軌跡**（`vx_ref`, `wz_ref`），不要旁路外環硬推輪。手動 `mode/speed/posset` 給調機用（平衡 off）。詳見 `docs/TUNING.md`。

## Timing (recommended)

| Loop | Rate | Core / context |
|------|------|----------------|
| FOC current / PWM | 10–20 kHz | Motor IRQ（MCK sample） |
| Hall angle / speed | PWM 同步 | Motor stack |
| IMU sample + attitude | 500–1000 Hz | GPT periodic or IMU DRDY IRQ |
| LQR + differential kinematics | 200–500 Hz | same GPT or RTOS task |
| Command UART parse | event / 50–100 Hz | FreeRTOS task or poll |
| Telemetry / status print | 10–50 Hz | background |

雙核 RA8T2（若選件）：M33 專心雙馬達 FOC，M85 跑 LQR + UART + IMU；單核亦可先全部放在 M85。

## State machine

```
BOOT → IMU_INIT → MOTOR_INIT → IDLE
IDLE -- lqr on / auto --> BALANCING
BALANCING -- motion cmds --> TRACKING   (vx/wz nonzero, still LQR)
BALANCING|TRACKING -- brake --> HOLD    (vx=wz=0, LQR on)
* -- fault / tilt / estop --> FAULT_SAFE (PWM off)
FAULT_SAFE -- clear + reinit --> IDLE
```

開機預設路徑：`IMU_INIT` 成功後自動進入 `BALANCING`（可用 `APP_CFG_LQR_ON_BOOT` 關閉）。

## LQR notes (hoverboard 2WD)

簡化平面模型將車體視為倒立擺 + 差速底盤：

- θ：pitch（IMU），目標 ≈ 直立偏置（含電池/載重校正）
- φ：車輪平均角度 / 位移（Hall 積分）→ 抑制漂移、實現站定
- ψ：yaw（輪速差或 gyro Z）→ 轉彎

離線用 MATLAB/Python 線性化後算 `K`，燒進 `balance_lqr_gain.h` 的常數增益矩陣；現場只調 scale / soft-limit。

前進：提高 `vx_ref`（或暫時 pitch lean ref）  
轉彎：`wz_ref` → `v_L = vx - 0.5*track*wz`, `v_R = vx + 0.5*track*wz`  
Brake：`vx_ref=wz_ref=0`，LQR 繼續把 φ̇→0 並穩 θ。

## Safety interlocks

1. `|θ| > THETA_MAX` → FAULT_SAFE  
2. IMU I2C NACK / 逾時 → FAULT_SAFE  
3. Motor overcurrent / overvoltage（FSP motor error）→ FAULT_SAFE  
4. AI/RC command age > timeout → HOLD（非斷電）  
5. Console `estop` → FAULT_SAFE  

## File map

| Module | Responsibility |
|--------|----------------|
| `app/` | 啟動、週期調度、`app_main` |
| `imu/` | LSM6DSK320X I2C + 姿態估計 |
| `balance/` | LQR、狀態估計包裝 |
| `motor/` | 雙輪 mode/ref、FSP port |
| `cmd/` | console、AI framing、arbiter |
| `util/` | ring buffer、CRC、time |
