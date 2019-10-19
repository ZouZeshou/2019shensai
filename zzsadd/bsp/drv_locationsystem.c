#include "drv_locationsystem.h"
#include "math.h"
struct posture_data s_posture={0};

/**
 * @brief get the locationsystem data
 * @param None
 * @return None
 * @attention None
 */
void get_loca_sys_data(uint8_t * buffer)
{
	static int init=0;
	static union
	{
		uint8_t data[24];
		float ActVal[6];
	}posture;
	
	if(buffer[0]==0x0D&&buffer[1]==0x0A)
	{
		for(int i=0;i<24;i++)
		{
			posture.data[i]=buffer[i+2];
		}	
//		s_posture.zangle=posture.ActVal[0];
		s_posture.xangle=posture.ActVal[1];
		s_posture.yangle=posture.ActVal[2];
		s_posture.pos_x =-posture.ActVal[3];
		s_posture.pos_y =-posture.ActVal[4];
		s_posture.w_z =posture.ActVal[5];
	}

}
/**
 * @brief deal gyro data to continue
 * @param None
 * @return None
 * @attention None
 */
void angle_to_continue(struct posture_data *s_pos)
{
	if(s_pos->zangle-s_pos->ang_last<-300)
	{
		s_pos->cir_num = s_pos->cir_num + 1;
	}
	else if(s_pos->zangle-s_pos->ang_last>300)
	{
		s_pos->cir_num = s_pos->cir_num - 1;
	}
	s_pos->ang_tol = s_pos->zangle + s_pos->cir_num * 360;
}
/**
* @name 	CcltAngleAdd
* @brief	��-180,180���紦������
* @param	angle1:�Ƕ�1;
			angle2:�Ƕ�2;
* @retval
*/
static float CcltAngleAdd(float angle1, float angle2)
{
	float result = 0.0f;
	result = angle1 + angle2;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
	
}
/**
* @name 	CcltAngleSub
* @brief	��-180,180���紦������
* @param	minuend: ������;
			subtrahend: ���� A - B,AΪ��������BΪ����;
* @retval
*/
float CcltAngleSub(float minuend, float subtrahend)
{
	float result = 0.0f;
	result = minuend - subtrahend;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
}
/**
* @name 	CcltTwoLineIntersection2
* @brief	��������ֱ�ߵĽ���
* @param	line1:ֱ��1;
			line2:ֱ��2;
* @retval
*/
static ActPoint CcltTwoLineIntersection2(ActLine2 line1, ActLine2 line2)
{
	ActPoint intersection;
	//б��
	float k1 = 0.0f;
	float k2 = 0.0f;

	//��Ϊ��������,δ����x�ᴹֱ��ֱ�ߴ���
	k1 = tan(line1.angle * CHANGE_TO_RADIAN);
	k2 = tan(line2.angle * CHANGE_TO_RADIAN);

	intersection.x = (line1.point.x*k1 - line1.point.y - line2.point.x * k2 + line2.point.y)
						 / (k1 - k2);
	intersection.y = k1 * (intersection.x - line1.point.x) + line1.point.y;

	return intersection;
}


/**
* @name 	CcltLineAngle
* @brief	��������ֱ�߷���Ƕ�
* @param	pointStart:��ʼ�㣺
			pointEnd:��ֹ��;
* @retval
*/
static float CcltLineAngle(ActPoint pointStart, ActPoint pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;
	
	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	//atan2f��Χ���԰���-180��180  
	return (atan2f(a, b) * CHANGE_TO_ANGLE); 
}

static float s_angleErrOld = 0.0f; 
void Point2Ponit_Deinit()
{
	s_angleErrOld = 0.0f;
}

/**
  * @name 	MvByLine
  * @brief  	ֱ�߱ջ����ٶ�Ϊ��
  * @param  	presentLine����ǰ��̬��
				targetLine��vĿ���Ŀ�귽�򣬵�ͬ��Ŀ��ֱ�ߣ�
				speed:ֱ���ٶ�;
  * @retval 	����ֵΪ��ǰ�㵽Ŀ�����롣��λmm
  */
float MvByLine(ActLine2 presentLine, ActLine2 targetLine)
{
	//��ǰ�㵽Ŀ��ֱ�ߵĴ���
	ActLine2 verticalLine;											
	//�е�	
	ActPoint  midpoint;											
	//���Ƶ�Ŀ��Ƕ�
	float targetAngle = 0.0f;	
	//��ǰ�㵽Ŀ������	
	float psntToEndDis = 0.0f;																						  
	//Ŀ��ֱ�߼�90�ȣ�������߷���
	verticalLine.angle = CcltAngleAdd(targetLine.angle, 90.0f);
	//�ѵ�ǰ�㸳������
	verticalLine.point = presentLine.point;		
	//���㴹����Ŀ��ֱ	
	midpoint = CcltTwoLineIntersection2(targetLine, verticalLine);
	//�����е������	
	midpoint.x = (midpoint.x + targetLine.point.x) * 0.5f;													  
	//�����е�������
	midpoint.y = (midpoint.y + targetLine.point.y) * 0.5f;													  
	//���㵱ǰ�㵽�е㷽��
	targetAngle = CcltLineAngle(presentLine.point, midpoint);												  
	//��ǰ�㵽Ŀ������
	return targetAngle;
}