#include <main.h>

typedef struct
{
	double   Kp;
	double   Ki;
	double   Kd;
	
	double   TarSpd;																															//Ŀ���ٶ�
	double   CurSpd;																															//��ǰ�ٶ�
	
	double   Err;																																	//��ǰ���
	double   Errl;																																//�ϴ����
	
	double   Integral;
	double   Derivative;
	
	double   In;
	int16_t  Out;
}pid_speed;

typedef struct
{
	double   Kp;
	double   Ki;
	double   Kd;
	
	double   TarPos;																															//Ŀ��λ��
	double   CurPos;																															//��ǰλ��
	
	double   Err;																																	//��ǰ���
	double   Errl;																																//�ϴ����
	
	double   Integral;
	double   Derivative;
	
	double   In;
	int16_t  Out;
}pid_position;

typedef struct
{
	/* λ�û� */
	double   KpPos;
	double   KiPos;
	double   KdPos;
	
	double   TarPos;																															//Ŀ��λ��
	double   CurPos;																															//��ǰλ��
	
	double   ErrPos;																															//��ǰλ�����
	double   ErrlPos;																															//�ϴ�λ�����
	
	double   IntegralPos;
	double   DerivativePos;
	
	double   InPos;
	double   InlPos;
	double   OutPos;
	
	/* �ٶȻ� */
	double   KpSpd;
	double   KiSpd;
	double   KdSpd;
	
	double   TarSpd;																															//Ŀ���ٶ�
	double   CurSpd;																															//��ǰ�ٶ�
	
	double   ErrSpd;																															//��ǰ�ٶ����
	double   ErrlSpd;																															//�ϴ��ٶ����
	
	double   IntegralSpd;
	double   DerivativeSpd;
	
	double   InSpd;
	double   InlSpd;
	int32_t  OutSpd;
	
	/* �������� */
	double   Deviation;																														//��ʼƫ��Ƕ�
	int      RoundCnt;																														//Ȧ��
	double   TotalPos;																														//�ܽǶ�
	
	double   Deadband;
	double   LimitIntegralPos;																										//����λ�û���
	double   LimitIntegralSpd;																										//�����ٶȻ���
	double   LimitSpd;																														//�����ٶ�
	double	 InitPos;																															//��ʼλ��
	double   InitDevPos;																													//��ʼƫ��λ��
}pid_position_speed;


/* �������� */
int32_t PidCount_Spd(pid_speed* Spd, motor_measure_t Class);										//�ٶȻ�
int32_t PidCount_Pos(pid_position* Pos, motor_measure_t Class);									//λ�û�
int32_t PidCount_Pos_Spd(pid_position_speed* Pos_Spd, motor_measure_t Class, int Model, int16_t InitPosit);		//˫��
