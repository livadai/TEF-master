#include "TwoDOF_ArmIK.h"
#include <math.h>
#include "struct_typedef.h"
#include "user_lib.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "3508_angle.h"
#include <stdlib.h>

const RC_ctrl_t *rc_ctrl_Pxy;

// �����㣺���ݽ�1�ͽ�2�����P������
void forward_kinematics(double angle1, double angle2, double *Px, double *Py) {
    double angle1_rad = angle1 * M_PI / 180.0;
    double angle2_rad = angle2 * M_PI / 180.0;

    // ����� A ������
    double Ax = OA * cos(angle1_rad);
    double Ay = OA * sin(angle1_rad);

    // ����� P ������
    *Px = Ax + AP * cos(angle1_rad + angle2_rad);
    *Py = Ay + AP * sin(angle1_rad + angle2_rad);
}

// ����㣺���ݵ�P����������1�ͽ�2
int inverse_kinematics(double Px, double Py, double *angle1, double *angle2) {
    // ����� P ���� O �ľ���
    double OP = sqrt(Px * Px + Py * Py);

    // ����Ƿ�ɴ�
    if (OP > OA + AP || OP < fabs(OA - AP)) {
        return -1;  // �� P ���ɴ�
    }

    // �����1
    double alpha = atan2(Py, Px);
    double beta = acos((OA * OA + OP * OP - AP * AP) / (2 * OA * OP));

    // ����⣺�ⲿ���Ϻ��ⲿ����
    double angle1_1 = (alpha - beta) * 180.0 / M_PI;
    double angle1_2 = (alpha + beta) * 180.0 / M_PI;

    // �����2
    double gamma = acos((OA * OA + AP * AP - OP * OP) / (2 * OA * AP));
    double angle2_1 = 180.0 - gamma * 180.0 / M_PI;
    double angle2_2 = gamma * 180.0 / M_PI - 180.0;

    // ѡ���3С��180��Ľ�
    if (angle2_1 < 180.0) {
        *angle1 = angle1_1;
        *angle2 = angle2_1;
    } else {
        *angle1 = angle1_2;
        *angle2 = angle2_2;
    }

    return 0;  // �ɹ�
}

// ����������
void rc_deadband_limit1(int16_t input, int16_t *output, int16_t deadband) {
    if (abs(input) < deadband) {
        *output = 0;  // �������ڣ����Ϊ0
    } else if (input < -500) {
        *output = -500;
    } else if (input > 500) {
        *output = 500;
    } else {
        *output = input;  // ����������ֱ�����ԭʼֵ
    }
}

// ����Ŀ��λ�ò����ؽǶȱ仯��
int m3508_update_target_position(double *delta_angle1, double *delta_angle2) {
    static int16_t last_x_value = 0;  // ������һ�ε�ҡ��ֵ
    int16_t x_value = 0;              // ���ڴ洢ҡ��ͨ��ֵ

    static int16_t last_y_value = 0;  // ������һ�ε�ҡ��ֵ
    int16_t y_value = 0;              // ���ڴ洢ҡ��ͨ��ֵ

    // ��ȡ��ǰҡ��ֵ
    rc_ctrl_Pxy = get_remote_control_point();

    // �����ʼ�� P ������
    double Xp, Yp;
    forward_kinematics(ctrl_J1.start_angle, ctrl_J2.start_angle, &Xp, &Yp);

    // ��ҡ��ֵ������������
    rc_deadband_limit1(rc_ctrl_Pxy->rc.Ch4, &x_value, 10);  // CH4 ���� X ����
    rc_deadband_limit1(rc_ctrl_Pxy->rc.Ch2, &y_value, 10);  // CH2 ���� Y ����

    // ��ͨ��ֵת��Ϊ Xp �� Yp ������
    double delta_Xp = (double)x_value / 671 * 5;  // ǰ����������������Ϊ 5��
    double delta_Yp = (double)y_value / 671 * 5;  // ������������������Ϊ 5��

    // ���ҡ��ֵ�仯����һ����ֵ������΢С������
    if (fabs(delta_Xp) > 0.05f) {
        Xp += delta_Xp;  // ���� Xp
    }

    if (fabs(delta_Yp) > 0.05f) {
        Yp += delta_Yp;  // ���� Yp
    }

    // ���浱ǰҡ��ֵ
    last_x_value = x_value;
    last_y_value = y_value;

    // ���Ƶ� P �������ڻ�е�ۿɴﷶΧ��
    double OP = sqrt(Xp * Xp + Yp * Yp);
    if (OP > OA + AP || OP < fabs(OA - AP)) {
        return -1;  // �� P ���ɴ�
    }

    // ���������õ�Ŀ���1 �ͽ�2
    double target_angle1, target_angle2;
    if (inverse_kinematics(Xp, Yp, &target_angle1, &target_angle2) != 0) {
        return -1;  // �����ʧ��
    }

    // ����Ƕȱ仯��
    *delta_angle1 = target_angle1 - ctrl_J1.start_angle;
    *delta_angle2 = target_angle2 - ctrl_J2.start_angle;

    // ������ʼ�Ƕ�
    ctrl_J1.start_angle = target_angle1;
    ctrl_J2.start_angle = target_angle2;

    return 0;  // �ɹ�
}