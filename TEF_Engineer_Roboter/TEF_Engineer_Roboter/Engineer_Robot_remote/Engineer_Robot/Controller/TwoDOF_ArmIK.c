#include "TwoDOF_ArmIK.h"
#include <math.h>
#include "struct_typedef.h"
#include "user_lib.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "3508_angle.h"
#include <stdlib.h>

const RC_ctrl_t *rc_ctrl_Pxy;

// 正解算：根据角1和角2计算点P的坐标
void forward_kinematics(double angle1, double angle2, double *Px, double *Py) {
    double angle1_rad = angle1 * M_PI / 180.0;
    double angle2_rad = angle2 * M_PI / 180.0;

    // 计算点 A 的坐标
    double Ax = OA * cos(angle1_rad);
    double Ay = OA * sin(angle1_rad);

    // 计算点 P 的坐标
    *Px = Ax + AP * cos(angle1_rad + angle2_rad);
    *Py = Ay + AP * sin(angle1_rad + angle2_rad);
}

// 逆解算：根据点P的坐标计算角1和角2
int inverse_kinematics(double Px, double Py, double *angle1, double *angle2) {
    // 计算点 P 到点 O 的距离
    double OP = sqrt(Px * Px + Py * Py);

    // 检查是否可达
    if (OP > OA + AP || OP < fabs(OA - AP)) {
        return -1;  // 点 P 不可达
    }

    // 计算角1
    double alpha = atan2(Py, Px);
    double beta = acos((OA * OA + OP * OP - AP * AP) / (2 * OA * OP));

    // 两组解：肘部向上和肘部向下
    double angle1_1 = (alpha - beta) * 180.0 / M_PI;
    double angle1_2 = (alpha + beta) * 180.0 / M_PI;

    // 计算角2
    double gamma = acos((OA * OA + AP * AP - OP * OP) / (2 * OA * AP));
    double angle2_1 = 180.0 - gamma * 180.0 / M_PI;
    double angle2_2 = gamma * 180.0 / M_PI - 180.0;

    // 选择角3小于180°的解
    if (angle2_1 < 180.0) {
        *angle1 = angle1_1;
        *angle2 = angle2_1;
    } else {
        *angle1 = angle1_2;
        *angle2 = angle2_2;
    }

    return 0;  // 成功
}

// 死区处理函数
void rc_deadband_limit1(int16_t input, int16_t *output, int16_t deadband) {
    if (abs(input) < deadband) {
        *output = 0;  // 在死区内，输出为0
    } else if (input < -500) {
        *output = -500;
    } else if (input > 500) {
        *output = 500;
    } else {
        *output = input;  // 超出死区，直接输出原始值
    }
}

// 更新目标位置并返回角度变化量
int m3508_update_target_position(double *delta_angle1, double *delta_angle2) {
    static int16_t last_x_value = 0;  // 保存上一次的摇杆值
    int16_t x_value = 0;              // 用于存储摇杆通道值

    static int16_t last_y_value = 0;  // 保存上一次的摇杆值
    int16_t y_value = 0;              // 用于存储摇杆通道值

    // 获取当前摇杆值
    rc_ctrl_Pxy = get_remote_control_point();

    // 计算初始点 P 的坐标
    double Xp, Yp;
    forward_kinematics(ctrl_J1.start_angle, ctrl_J2.start_angle, &Xp, &Yp);

    // 对摇杆值进行死区处理
    rc_deadband_limit1(rc_ctrl_Pxy->rc.Ch4, &x_value, 10);  // CH4 控制 X 方向
    rc_deadband_limit1(rc_ctrl_Pxy->rc.Ch2, &y_value, 10);  // CH2 控制 Y 方向

    // 将通道值转换为 Xp 和 Yp 的增量
    double delta_Xp = (double)x_value / 671 * 5;  // 前伸增量（缩放因子为 5）
    double delta_Yp = (double)y_value / 671 * 5;  // 上下增量（缩放因子为 5）

    // 如果摇杆值变化超过一定阈值（避免微小抖动）
    if (fabs(delta_Xp) > 0.05f) {
        Xp += delta_Xp;  // 更新 Xp
    }

    if (fabs(delta_Yp) > 0.05f) {
        Yp += delta_Yp;  // 更新 Yp
    }

    // 保存当前摇杆值
    last_x_value = x_value;
    last_y_value = y_value;

    // 限制点 P 的坐标在机械臂可达范围内
    double OP = sqrt(Xp * Xp + Yp * Yp);
    if (OP > OA + AP || OP < fabs(OA - AP)) {
        return -1;  // 点 P 不可达
    }

    // 计算逆解算得到目标角1 和角2
    double target_angle1, target_angle2;
    if (inverse_kinematics(Xp, Yp, &target_angle1, &target_angle2) != 0) {
        return -1;  // 逆解算失败
    }

    // 计算角度变化量
    *delta_angle1 = target_angle1 - ctrl_J1.start_angle;
    *delta_angle2 = target_angle2 - ctrl_J2.start_angle;

    // 更新起始角度
    ctrl_J1.start_angle = target_angle1;
    ctrl_J2.start_angle = target_angle2;

    return 0;  // 成功
}