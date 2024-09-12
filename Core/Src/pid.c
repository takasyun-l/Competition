#include <pid.h>
#include <usart.h>

#define Init 1
#define Use  2
/* 类型定义 */

extern motor_measure_t motor_chassis[7];

pid_speed             Motor3508_PID_SPEED               = {3, 0.1, 0.01, 0, 0, 0, 0, 0, 0, 0, 0};
pid_speed             Motor2006_PID_SPEED               = {3, 0.1, 0.01, 0, 0, 0, 0, 0, 0, 0, 0};

pid_position          Motor3508_PID_POSITION            = {0, 1.0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0};
pid_position_speed    Motor3508_PID_POSITION_SPEED      = {5, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.96, 0.10, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 2000, 8500, 2000, 3626, 0};
pid_position_speed    Motor6020_PID_POSITION_SPEED      = {50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.64, 0.4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3035, 0};
	
pid_speed*            MOTOR3508_PID_SPEED               = &Motor3508_PID_SPEED;
pid_speed*            MOTOR2006_PID_SPEED               = &Motor2006_PID_SPEED;

pid_position*         MOTOR3508_PID_POSITION            = &Motor3508_PID_POSITION;
pid_position_speed*   MOTOR3508_PID_POSITION_SPEED      = &Motor3508_PID_POSITION_SPEED;
pid_position_speed*   MOTOR6020_PID_POSITION_SPEED			= &Motor6020_PID_POSITION_SPEED;
	
double    						final = 0;
double								a = 0.25;

/* 函数定义 */

 
double firstOrderFilter(double in_data)
{
	final = a*in_data + (1-a)*final;	
	return(final);
}
	
int32_t PidCount_Spd(pid_speed* Spd, motor_measure_t Class)
{
	Spd->In = Class.speed_rpm;
	Spd->CurSpd = Spd->In;
	Spd->Err = Spd->TarSpd - Spd->CurSpd;
	
	Spd->Integral += Spd->Err;
	Spd->Derivative = Spd->Err - Spd->Errl;
	
	Spd->Out = Spd->Kp * Spd->Err + Spd->Ki * Spd->Integral + Spd->Kd * Spd->Derivative;
	
	Spd->Errl = Spd->Err;
	
	return Spd->Out;
}

int32_t PidCount_Pos(pid_position* Pos, motor_measure_t Class)
{
	Pos->In = Class.angle;	
	Pos->CurPos = Pos->In;
	
	/* 过零处理 */
	if(Pos->TarPos - Pos->CurPos > 4096)
		Pos->CurPos += 8192;
	else if(Pos->TarPos - Pos->CurPos < -4096)
		Pos->CurPos -= 8192;
	else
		Pos->CurPos += 0;
		
	
	Pos->Err = Pos->TarPos - Pos->CurPos;
	
	Pos->Integral += Pos->Err;
	Pos->Derivative = Pos->Err - Pos->Errl;
	
	Pos->Out = Pos->Kp * Pos->Err + Pos->Ki * Pos->Integral + Pos->Kd * Pos->Derivative;
	
	/* 输出限幅 */
	if(Pos->Out > 8000)
		Pos->Out = 8000;
	else if(Pos->Out < -8000)
		Pos->Out = -8000;
	
	Pos->Errl = Pos->Err;
	
	return Pos->Out;
}

int32_t PidCount_Pos_Spd(pid_position_speed* Pos_Spd, motor_measure_t Class, int Model, int16_t InitPosit)
{
	Pos_Spd->InPos = Class.angle;
	Pos_Spd->InlPos = Class.last_angle;
	Pos_Spd->InSpd = Class.speed_rpm;
	
	Pos_Spd->CurPos = Pos_Spd->InPos;
	
	/* 过圈处理 */
	static _Bool CntFlag = 0;
	if(CntFlag == 0)
	{
		Pos_Spd->Deviation = InitPosit;
	}
	CntFlag = 1;
	
	if(Model == 6020)
	{
		if(Pos_Spd->InitPos - Pos_Spd->Deviation > 4096)
			Pos_Spd->InitDevPos = (Pos_Spd->InitPos - Pos_Spd->Deviation - 8192);
		else if(Pos_Spd->InitPos - Pos_Spd->Deviation < -4096)
			Pos_Spd->InitDevPos = (Pos_Spd->InitPos - Pos_Spd->Deviation + 8192);
		else
			Pos_Spd->InitDevPos = Pos_Spd->InitPos - Pos_Spd->Deviation;
	}
	
	/* 过零处理 */
	if(Pos_Spd->InPos - Pos_Spd->InlPos > 4096)
		Pos_Spd->RoundCnt--;
	else if(Pos_Spd->InPos - Pos_Spd->InlPos < -4096)
		Pos_Spd->RoundCnt++;
	
	/* 电机判断 */
	if(Model == 3508)
		Pos_Spd->TotalPos = (Pos_Spd->RoundCnt * 8192 + Pos_Spd->InPos - Pos_Spd->Deviation - Pos_Spd->InitDevPos) * 187.f / 3591.f;
	else if(Model == 2006)
		Pos_Spd->TotalPos = (Pos_Spd->RoundCnt * 8192 + Pos_Spd->InPos - Pos_Spd->Deviation) / 36.f;
	else if(Model == 6020)
		Pos_Spd->TotalPos = (Pos_Spd->RoundCnt * 8192 + Pos_Spd->InPos - Pos_Spd->Deviation - Pos_Spd->InitDevPos);

	Pos_Spd->ErrPos = Pos_Spd->TarPos - Pos_Spd->TotalPos;
	
	/* 误差死区 */
	if(Pos_Spd->ErrPos <= Pos_Spd->Deadband && Pos_Spd->ErrPos >= -Pos_Spd->Deadband)
	{
		;
	}

	
	Pos_Spd->IntegralPos += Pos_Spd->ErrPos;
	
	/* 积分限幅 */
	if(Pos_Spd->IntegralPos > Pos_Spd->LimitIntegralPos)
		Pos_Spd->IntegralPos = Pos_Spd->LimitIntegralPos;
	else if(Pos_Spd->IntegralPos < -Pos_Spd->LimitIntegralPos)
		Pos_Spd->IntegralPos = -Pos_Spd->LimitIntegralPos;
	
	Pos_Spd->DerivativePos = Pos_Spd->ErrPos - Pos_Spd->ErrlPos;
	
	Pos_Spd->OutPos = Pos_Spd->KpPos * Pos_Spd->ErrPos + Pos_Spd->KiPos * Pos_Spd->IntegralPos + Pos_Spd->KdPos * Pos_Spd->DerivativePos;
	
	Pos_Spd->ErrlPos = Pos_Spd->ErrPos;
		
	Pos_Spd->TarSpd = Pos_Spd->OutPos;
	
	Pos_Spd->CurSpd = Pos_Spd->InSpd;
	
	
	/* 速度限幅 */
	if(Pos_Spd->CurSpd > Pos_Spd->LimitSpd && Pos_Spd->OutPos > 0)
		Pos_Spd->TarSpd = Pos_Spd->LimitSpd;
	else if(Pos_Spd->CurSpd < -Pos_Spd->LimitSpd && Pos_Spd->OutPos < 0)
		Pos_Spd->TarSpd = -Pos_Spd->LimitSpd;
	else
		Pos_Spd->TarSpd = Pos_Spd->OutPos;
	
	Pos_Spd->ErrSpd = Pos_Spd->TarSpd - Pos_Spd->CurSpd;
	
	Pos_Spd->IntegralSpd += Pos_Spd->ErrlSpd;
	
	/* 积分限幅 */
	if(Pos_Spd->IntegralSpd > Pos_Spd->LimitIntegralSpd)
		Pos_Spd->IntegralSpd = Pos_Spd->LimitIntegralSpd;
	else if(Pos_Spd->IntegralSpd < -Pos_Spd->LimitIntegralSpd)
		Pos_Spd->IntegralSpd = -Pos_Spd->LimitIntegralSpd;
		
	
	Pos_Spd->DerivativeSpd = Pos_Spd->ErrSpd - Pos_Spd->ErrlSpd;
	
	if(Model == 6020)
		Pos_Spd->OutSpd = firstOrderFilter(firstOrderFilter(firstOrderFilter(firstOrderFilter(Pos_Spd->KpSpd * Pos_Spd->ErrSpd + Pos_Spd->KiSpd * Pos_Spd->IntegralSpd + Pos_Spd->KdSpd * Pos_Spd->DerivativeSpd))));
	else
		Pos_Spd->OutSpd = Pos_Spd->KpSpd * Pos_Spd->ErrSpd + Pos_Spd->KiSpd * Pos_Spd->IntegralSpd + Pos_Spd->KdSpd * Pos_Spd->DerivativeSpd;
	
	Pos_Spd->ErrlSpd = Pos_Spd->ErrSpd;
	
	/* 输出限幅 */
	if(Model == 3508)
	{
		if(Pos_Spd->OutSpd > 6000)
		Pos_Spd->OutSpd = 6000;
	else if(Pos_Spd->OutSpd < -6000)
		Pos_Spd->OutSpd = -6000;
	}
	else if(Model == 6020)
	{
		if(Pos_Spd->OutSpd > 30000)
			Pos_Spd->OutSpd = 30000;
		else if(Pos_Spd->OutSpd <-30000)
			Pos_Spd->OutSpd = -30000;
	}
	
	return Pos_Spd->OutSpd;
}

