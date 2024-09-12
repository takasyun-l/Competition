#include <main.h>

typedef struct
{
	double   Kp;
	double   Ki;
	double   Kd;
	
	double   TarSpd;																															//目标速度
	double   CurSpd;																															//当前速度
	
	double   Err;																																	//当前误差
	double   Errl;																																//上次误差
	
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
	
	double   TarPos;																															//目标位置
	double   CurPos;																															//当前位置
	
	double   Err;																																	//当前误差
	double   Errl;																																//上次误差
	
	double   Integral;
	double   Derivative;
	
	double   In;
	int16_t  Out;
}pid_position;

typedef struct
{
	/* 位置环 */
	double   KpPos;
	double   KiPos;
	double   KdPos;
	
	double   TarPos;																															//目标位置
	double   CurPos;																															//当前位置
	
	double   ErrPos;																															//当前位置误差
	double   ErrlPos;																															//上次位置误差
	
	double   IntegralPos;
	double   DerivativePos;
	
	double   InPos;
	double   InlPos;
	double   OutPos;
	
	/* 速度环 */
	double   KpSpd;
	double   KiSpd;
	double   KdSpd;
	
	double   TarSpd;																															//目标速度
	double   CurSpd;																															//当前速度
	
	double   ErrSpd;																															//当前速度误差
	double   ErrlSpd;																															//上次速度误差
	
	double   IntegralSpd;
	double   DerivativeSpd;
	
	double   InSpd;
	double   InlSpd;
	int32_t  OutSpd;
	
	/* 其他参数 */
	double   Deviation;																														//初始偏差角度
	int      RoundCnt;																														//圈数
	double   TotalPos;																														//总角度
	
	double   Deadband;
	double   LimitIntegralPos;																										//限制位置积分
	double   LimitIntegralSpd;																										//限制速度积分
	double   LimitSpd;																														//限制速度
	double	 InitPos;																															//初始位置
	double   InitDevPos;																													//初始偏差位置
}pid_position_speed;


/* 函数声明 */
int32_t PidCount_Spd(pid_speed* Spd, motor_measure_t Class);										//速度环
int32_t PidCount_Pos(pid_position* Pos, motor_measure_t Class);									//位置环
int32_t PidCount_Pos_Spd(pid_position_speed* Pos_Spd, motor_measure_t Class, int Model, int16_t InitPosit);		//双环
