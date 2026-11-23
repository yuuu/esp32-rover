#pragma once

#include "odometry.h"

void drive_init(void);
// 目標の並進速度 v [m/s] と角速度 w [rad/s] (左回りが正)
void drive_set(float v, float w);
// 目標を 0 にしてブレーキをかけ、速度制御の積分をリセットする
void drive_stop(void);
// 制御周期ごとに呼ぶ: エンコーダを読み、自己位置を更新し、左右の車輪の速度を制御する
void drive_tick(float dt);
pose_t drive_pose(void);
