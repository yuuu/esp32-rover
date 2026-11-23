#pragma once

// 自己位置。原点はスタート地点、x は最初の向き (前)、y は左。
// theta [rad] は左回りが正で、一周しても ±π に折り返さない (連続値)
typedef struct {
    float x;
    float y;
    float theta;
} pose_t;

void odom_reset(void);
// 前回からの左右の車輪の移動距離 [m] で自己位置を更新する
void odom_update(float dl, float dr);
pose_t odom_get(void);
