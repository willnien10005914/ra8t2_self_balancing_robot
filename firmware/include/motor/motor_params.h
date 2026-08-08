#pragma once

/**
 * 6.5" Hall hub motor (金海芯 / electricassistcart)
 * 規格書：spec/6.5 inch (Hall sensor) 1.pdf
 * 型號：24216D51641050512250336D1-065E
 * 名稱：直流無刷無齒輪轂 — 6寸半 直徑164mm 胎寬44 單邊壓軸（霍爾）
 */

#define MOTOR_SPEC_DIAMETER_M         (0.165f)   /* 含輪胎約 165mm */
#define MOTOR_SPEC_RADIUS_M           (0.0825f)
#define MOTOR_SPEC_RATED_VOLTAGE_V    (36.0f)
#define MOTOR_SPEC_VOLTAGE_MIN_V      (24.0f)
#define MOTOR_SPEC_VOLTAGE_MAX_V      (100.0f)
#define MOTOR_SPEC_RATED_POWER_W      (250.0f)
#define MOTOR_SPEC_RATED_RPM          (500.0f)
#define MOTOR_SPEC_RPM_TOL            (50.0f)
#define MOTOR_SPEC_RATED_CURRENT_A    (5.0f)
#define MOTOR_SPEC_CURRENT_TOL_A      (0.5f)
#define MOTOR_SPEC_PEAK_CURRENT_A     (12.0f)    /* bring-up 保守上限，實測再調 */
#define MOTOR_SPEC_MAX_TORQUE_NM      (8.9f)
#define MOTOR_SPEC_POLE_PAIRS         (15u)      /* 15 對極 / 30PCS 磁鋼 */
#define MOTOR_SPEC_HALL_PHASE_DEG     (120.0f)
#define MOTOR_SPEC_MAX_LOAD_KG        (80.0f)
#define MOTOR_SPEC_IP_RATING          (55u)

/* 線性速度 @ 額定轉速：ωr = 2π·n/60 */
#define MOTOR_SPEC_RATED_MPS \
    (MOTOR_SPEC_RATED_RPM * 0.104719755f * MOTOR_SPEC_RADIUS_M) /* ≈ 4.32 m/s */

/* 相線預設：黃(U) 藍(V) 綠(W)；霍爾：紅VCC 黃HA 藍HB 綠HC 黑GND 白溫控 */
#define MOTOR_SPEC_WIRE_NOTE \
    "U=Yel V=Blu W=Grn; Hall VCC/HA/HB/HC/GND/TEMP"

/**
 * FSP FOC 內環（對應 Renesas AN R01AN6839 速度 PI + 電流 PI）。
 * 真正執行在 rm_motor_*；此處為建議起始增益，於 FSP Configurator / RMW 微調。
 * 本專案有感 Hall，感測層用 rm_motor_hall，控制結構仍同速度→電流級聯。
 */
#define FOC_PWM_CARRIER_HZ            (20000u)
#define FOC_SPEED_LOOP_HZ             (1000u)
#define FOC_CURRENT_LOOP_HZ           (20000u)

/* 速度 PI：rpm error → Iq*（相對增益，上板後以 RMW 微調） */
#define FOC_SPEED_KP                  (0.08f)
#define FOC_SPEED_KI                  (0.40f)
#define FOC_SPEED_OUT_MAX_A           (MOTOR_SPEC_PEAK_CURRENT_A)

/* 電流 PI：Id/Iq error → Vd/Vq */
#define FOC_ID_KP                     (0.50f)
#define FOC_ID_KI                     (800.0f)
#define FOC_IQ_KP                     (0.50f)
#define FOC_IQ_KI                     (800.0f)
#define FOC_CURRENT_OUT_MAX_V         (MOTOR_SPEC_RATED_VOLTAGE_V * 0.9f)
