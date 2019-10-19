#ifndef __DRV_LOCATION_H
#define __DRV_LOCATION_H
#include "stm32f4xx.h"
//�Ƕ���ת��Ϊ������ϵ��
#define CHANGE_TO_RADIAN    0.017453f   
//������ת��Ϊ�Ƕ���ϵ��
#define CHANGE_TO_ANGLE     57.2958f
struct posture_data
{
	float pos_x;
	float pos_y;
	float zangle;
	float xangle;
	float yangle;
	float w_z;
	
	float ang_last;
	float ang_tol;
	int cir_num;
};
//��Ľṹ�� ��λmm
typedef struct
{
	float x;
	float y;
}ActPoint;


//��бʽ�ṹ�� ��б���ýǶ��ƵĽǶȴ���
typedef struct
{
	ActPoint point;
	//�Ƕ���
	float    angle;
}ActLine2;


float CcltAngleSub(float minuend, float subtrahend);
float MvByLine(ActLine2 presentLine, ActLine2 targetLine);
//�ⲿ�ӿں���������ٶȿ���

extern struct posture_data s_posture;


void get_loca_sys_data(uint8_t * buffer);
void angle_to_continue(struct posture_data *s_pos);
#endif
