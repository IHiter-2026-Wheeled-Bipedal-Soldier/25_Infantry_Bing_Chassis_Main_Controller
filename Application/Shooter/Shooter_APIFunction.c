/**
  ******************************************************************************
  * @file    Shooter_APIFunction.c
  * @author  26赛季，平衡步兵电控，马帅
  * @date    2026.1.10
  * @brief   发射相关控制函数
  ******************************************************************************
*/
#include "Algorithm.h"
#include "Algorithm_Simple.h"
#include "Shooter_APIFunction.h"
#include "General_AuxiliaryFunc.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Gimbal.h"
#include "GlobalDeclare_Shooter.h"
#include "TIM_Config.h"
#include <arm_math.h>

/**
  * @brief  更新发射相关数据的反馈值
  * @note   发射相关，正式结构体数据的反馈值，在这里进行更新汇总处理
  *         主要的发射相关正式结构体有：
  *             GSTSH_Data：发射主要数据结构体
  * @param  无
  * @retval 无
*/
void Shooter_AllFBDataUpdate(void)
{
	GSTSH_Data.ShooterMode = GEMSH_Mode; //当前发射模式

	// GstSH_Paras.SupplyPellet_PosFB = GstGM_MainCtrl.ST_Rx.SupplyPelletData.SupplyPellet_PosFB; //拨弹电机位置反馈赋值
	// GstSH_Paras.SupplyPellet_VelFB = GstGM_MainCtrl.ST_Rx.SupplyPelletData.SupplyPellet_VelFB; //拨弹电机速度反馈赋值
}

//#region/*************************************摩擦轮电机控制相关函数*************************************************/
//摩擦轮相关控制函数（用于控制摩擦轮电机目标转速设定及滑模控制）
void FrictionWheel_Safe(void)
{
	float FWSpeed_Datum_temp = 0.0f; //摩擦轮目标转速临时变量

	FWSpeed_Datum_temp = 0.0f;

	smcR.fpDes = +FWSpeed_Datum_temp; //符号由旋转方向决定
	smcL.fpDes = -FWSpeed_Datum_temp; //符号由旋转方向决定

	//滑膜控制摩擦轮转速
	SlidingModeCtrler(&smcL);
	SlidingModeCtrler(&smcR);

	//摩擦轮转速过低时停止制动
	if((MyAbsf(smcL.fpFB)+MyAbsf(smcR.fpFB))/2.0f < 60.0f)//转速低于60r/min，即360度/s，
	{
		smcL.fpU = 0.0f;
		smcR.fpU = 0.0f;
	}
}

//摩擦轮相关控制函数（用于控制摩擦轮电机目标转速设定及滑模控制）
void FrictionWheel_Debug(void)
{
	float FWSpeed_Datum_temp = 0.0f; //摩擦轮目标转速临时变量

	if(GstGMSH_Debug_Flags.FrictionWheel_Test_Flag == 1 && ShooterSafetyLocked == false)
	{
		FWSpeed_Datum_temp = GstGMSH_Debug_Paras.ShooterDatum;
	}
	else
	{
		FWSpeed_Datum_temp = 0;
	}

	smcR.fpDes = +FWSpeed_Datum_temp; //符号由旋转方向决定
	smcL.fpDes = -FWSpeed_Datum_temp; //符号由旋转方向决定

	//滑膜控制摩擦轮转速
	SlidingModeCtrler(&smcL);
	SlidingModeCtrler(&smcR);
}

//摩擦轮相关控制函数（用于控制摩擦轮电机目标转速设定及滑模控制）
void FrictionWheel_RCCtrl(void)
{
	float FWSpeed_Datum_temp = 0.0f; //摩擦轮目标转速临时变量

	if(ShooterSafetyLocked == false) //防止误打弹，增加发射安全锁
	{
		//TODO:有根据当前弹速状态，更改相应的摩擦轮转速以及操作手键鼠临时改摩擦轮转速相关函数暂时先不添加

		//如果在打符则保持弹速恒定
		if(Vision_Mode_Now == VisionMode_BuffBig || Vision_Mode_Now == VisionMode_BuffSmall)
		{
			FWSpeed_Datum_temp = FWSpeed_Datum_Buff;//打符参数赋值
		}
		else
		{
			FWSpeed_Datum_temp = FWSpeed_Datum;//非打符参数赋值
		}
	}
	else//安全锁打开，停止摩擦轮
	{
		FWSpeed_Datum_temp = 0.0f;
	}

	smcR.fpDes = +FWSpeed_Datum_temp; //符号由旋转方向决定
	smcL.fpDes = -FWSpeed_Datum_temp; //符号由旋转方向决定

	//滑膜控制摩擦轮转速
	SlidingModeCtrler(&smcL);
	SlidingModeCtrler(&smcR);
}

//摩擦轮相关控制函数（用于控制摩擦轮电机目标转速设定及滑模控制）
void FrictionWheel_KeyMouseCtrl(void)
{
	float FWSpeed_Datum_temp = 0.0f; //摩擦轮目标转速临时变量

	//按键F控制发射安全锁的开关
	if(PRESSED_F == true && PRESSED_F_Pre == false)
	{
		ShooterSafetyLocked = !ShooterSafetyLocked;
	}

	if(ShooterSafetyLocked == false) //防止误打弹，增加发射安全锁
	{
		//TODO:有根据当前弹速状态，更改相应的摩擦轮转速以及操作手键鼠临时改摩擦轮转速相关函数暂时先不添加

		//如果在打符则保持弹速恒定
		if(Vision_Mode_Now == VisionMode_BuffBig || Vision_Mode_Now == VisionMode_BuffSmall)
		{
			FWSpeed_Datum_temp = FWSpeed_Datum_Buff;//打符参数赋值
		}
		else
		{
			FWSpeed_Datum_temp = FWSpeed_Datum;//非打符参数赋值
		}
	}
	else//安全锁打开，停止摩擦轮
	{
		FWSpeed_Datum_temp = 0.0f;
	}

	smcR.fpDes = +FWSpeed_Datum_temp; //符号由旋转方向决定
	smcL.fpDes = -FWSpeed_Datum_temp; //符号由旋转方向决定

	//滑膜控制摩擦轮转速
	SlidingModeCtrler(&smcL);
	SlidingModeCtrler(&smcR);
}
//#endregion

//#region/*************************************拨弹电机控制相关函数*************************************************/
/*----------------------------------------------------------------------------------------
函数名：SupplyPellet_Safe()
功能：拨弹电机安全模式
----------------------------------------------------------------------------------------*/
/**
  * @brief  拨弹电机安全模式控制函数
  * @note   使拨弹电机实现阻尼效果
  * @param  无
  * @retval 无
  */
void SupplyPellet_Safe(void)
{
	/***********************前置处理**************************/
    /*状态切换*/
    if(GSTSH_Data.ShooterMode != GEMSH_Mode) //发射模式切换
    {
		PID_SetDes(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
		PID_SetDes(&GstSH_SupplyPelletVelPID, 0.0f);
		TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosFB);
		SupplyPellet_TD.x1 = GstSH_Paras.SupplyPellet_PosFB;
		SupplyPellet_TD.x2 = 0.0f;

		//TODO：摩擦轮前置处理
    }

	PID_SetKpKiKd(&GstSH_SupplyPelletPosPID, 0.0f, 0.0f, 0.0f);
	PID_SetKpKiKd(&GstSH_SupplyPelletVelPID, GstGMSH_Debug_Paras.SupplyPelletVelKp, GstGMSH_Debug_Paras.SupplyPelletVelKi, GstGMSH_Debug_Paras.SupplyPelletVelKd);
	/*正在发射标志位置0*/
	GstSH_Paras.RC_Shooting_Flag       = false;
	GstSH_Paras.KeyMouse_Shooting_Flag = false;
	GstSH_Paras.Debug_Shooting_Flag    = false;
	/*目标设定*/
	GstSH_Paras.SupplyPellet_PosDes = GstSH_Paras.SupplyPellet_PosFB;	//保持当前位置

	/*拨弹电机PID+TD计算*/
	PID_SetFB(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
	PID_SetFB(&GstSH_SupplyPelletVelPID, GstSH_Paras.SupplyPellet_VelFB);

	TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosDes);
	TD_Cal(&SupplyPellet_TD);
	PID_SetDes(&GstSH_SupplyPelletPosPID, TD_GetOutput(&SupplyPellet_TD));
	PID_Cal(&GstSH_SupplyPelletPosPID);
	PID_SetDes(&GstSH_SupplyPelletVelPID, PID_GetOutput(&GstSH_SupplyPelletPosPID) + SupplyPellet_td_coe_Norm*SupplyPellet_TD.x2); //TD速度前馈
	PID_Cal(&GstSH_SupplyPelletVelPID);
}

/**
  * @brief  拨弹电机调试模式控制函数
  * @note   SupplyPellet_Test_Flag为1时：进入完整拨弹电机调试模式
  * 		SupplyPellet_Test_Flag为0时：仍可测试拨弹电机速度环PID参数
  * @param  无
  * @retval 无
  */
void SupplyPellet_Debug(void)
{
	/***********************前置处理**************************/
    /*状态切换*/
    if(GSTSH_Data.ShooterMode != GEMSH_Mode) //发射模式切换
    {
		PID_SetDes(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
		PID_SetDes(&GstSH_SupplyPelletVelPID, 0.0f);
		TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosFB);
		SupplyPellet_TD.x1 = GstSH_Paras.SupplyPellet_PosFB;
		SupplyPellet_TD.x2 = 0.0f;

		//TODO：摩擦轮前置处理
    }

	if(GstGMSH_Debug_Flags.SupplyPellet_Test_Flag == 1)
	{
		PID_SetKpKiKd(&GstSH_SupplyPelletPosPID, GstGMSH_Debug_Paras.SupplyPelletPosKp, GstGMSH_Debug_Paras.SupplyPelletPosKi, GstGMSH_Debug_Paras.SupplyPelletPosKd);
		PID_SetKpKiKd(&GstSH_SupplyPelletVelPID, GstGMSH_Debug_Paras.SupplyPelletVelKp, GstGMSH_Debug_Paras.SupplyPelletVelKi, GstGMSH_Debug_Paras.SupplyPelletVelKd);
		
		if(GstGMSH_Debug_Paras.SupplyPelletTimeInterval != 0)
		{
			if(GstSH_Paras.Debug_IF_Shoot_Cnt >= GstGMSH_Debug_Paras.SupplyPelletTimeInterval)	//两次弹丸发射间隔为SupplyPelletTimeInterval，单位ms
			{
				/*正在发射标志位置1*/
				GstSH_Paras.Debug_Shooting_Flag = true;
				/*目标设定*/
				GstSH_Paras.SupplyPellet_PosDes += SupplyStep;	//每次发射目标位置增加一个步进角度
				/*发射间隔计数器清零*/
				GstSH_Paras.Debug_IF_Shoot_Cnt = 0;
			}
			else
			{
				/*正在发射标志位置0*/
				GstSH_Paras.Debug_Shooting_Flag = false;
				/*目标设定*/
				GstSH_Paras.SupplyPellet_PosDes = GstSH_Paras.SupplyPellet_PosDes;	//保持当前位置
			}
		}
	}
	else
	{
		PID_SetKpKiKd(&GstSH_SupplyPelletPosPID, 0.0f, 0.0f, 0.0f);
		PID_SetKpKiKd(&GstSH_SupplyPelletVelPID, GstGMSH_Debug_Paras.SupplyPelletVelKp, GstGMSH_Debug_Paras.SupplyPelletVelKi, GstGMSH_Debug_Paras.SupplyPelletVelKd);
		/*正在发射标志位置0*/
		GstSH_Paras.Debug_Shooting_Flag = false;
		/*目标设定*/
		GstSH_Paras.SupplyPellet_PosDes = GstSH_Paras.SupplyPellet_PosFB;	//保持当前位置
	}

	/*拨弹电机PID+TD计算*/
	PID_SetFB(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
	PID_SetFB(&GstSH_SupplyPelletVelPID, GstSH_Paras.SupplyPellet_VelFB);

	TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosDes);
	TD_Cal(&SupplyPellet_TD);
	PID_SetDes(&GstSH_SupplyPelletPosPID, TD_GetOutput(&SupplyPellet_TD));
	PID_Cal(&GstSH_SupplyPelletPosPID);
	PID_SetDes(&GstSH_SupplyPelletVelPID, PID_GetOutput(&GstSH_SupplyPelletPosPID) + SupplyPellet_td_coe_Norm*SupplyPellet_TD.x2); //TD速度前馈
	PID_Cal(&GstSH_SupplyPelletVelPID);

	GstSH_Paras.Debug_IF_Shoot_Cnt++;	// 起到计时器的作用（两次拨弹间隔）
}

/**
  * @brief  拨弹电机遥控器模式控制函数
  * @note   在摩擦轮准备好的情况下，遥控器拨轮下拨回正实现单发，持续下拨实现连发
  * @param  无
  * @retval 无
  */
void SupplyPellet_RCCtrl(void)
{
	//TODO:修改了单发逻辑，因此GstGM_MainCtrl.ST_Tx.SupplyPellet_Flag.Final_ShootOrNot_Flag目前不准（虽然但是原来也不准），后续想一想这个标志位怎么用

	/***********************前置处理**************************/
    /*状态切换*/
    if(GSTSH_Data.ShooterMode != GEMSH_Mode) //发射模式切换
    {
		PID_SetDes(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
		PID_SetDes(&GstSH_SupplyPelletVelPID, 0.0f);
		TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosFB);
		SupplyPellet_TD.x1 = GstSH_Paras.SupplyPellet_PosFB;
		SupplyPellet_TD.x2 = 0.0f;

		//TODO：摩擦轮前置处理
    }

	///*Is_Heat_Safe()*/
	if(__IS_FrictionWheel_Ready())
	{
		PID_SetKpKiKd(&GstSH_SupplyPelletPosPID, PID_SupplyPos_Kp, PID_SupplyPos_Ki, PID_SupplyPos_Kd);
		PID_SetKpKiKd(&GstSH_SupplyPelletVelPID, PID_SupplyVel_Kp, PID_SupplyVel_Ki, PID_SupplyVel_Kd);

		if(__IS_RC_Single_Shoot() || __IS_RC_Continuous_Shoot())//遥控器手动拨弹
		{
			if(GstSH_Paras.RC_IF_Shoot_Cnt >= 60)	//两次弹丸发射间隔为60ms
			{
				/*正在发射标志位置1*/
				GstSH_Paras.RC_Shooting_Flag = true;
				/*发射间隔计数器清零*/
				GstSH_Paras.RC_IF_Shoot_Cnt = 0;

				/*目标设定*/
				GstSH_Paras.SupplyPellet_PosDes += SupplyStep;	//每次发射目标位置增加一个步进角度
			}
			else
			{
				/*正在发射标志位置0*/
				GstSH_Paras.RC_Shooting_Flag = false;

				/*目标设定*/
				GstSH_Paras.SupplyPellet_PosDes = GstSH_Paras.SupplyPellet_PosDes;	//保持当前位置
			}
		}
		else if(RC_AUTO_Control_Continuous_Shoot())//遥控器辅瞄模式，视觉控制打弹
		{
			if(GST_Vision.AimAssistDataReceiveFrame.ShootOrNot == 1 || GST_SystemMonitor.USART6Rx_fps <= 15)
			{
				// if( struct_shoot.RC_IF_Shoot_cnt >= 60.0f)
				// {
				// 	GstGM_MainCtrl.ST_Tx.SupplyPellet_Flag.Final_ShootOrNot_Flag = 1;
				// 	struct_shoot.RC_IF_Shoot_cnt= 0;
				// }
			}
		}
		else//未打弹
		{
			/*正在发射标志位置0*/
			GstSH_Paras.RC_Shooting_Flag = false;

			/*目标设定*/
			GstSH_Paras.SupplyPellet_PosDes = GstSH_Paras.SupplyPellet_PosDes;	//保持当前位置
		}
	}
	else //摩擦轮未准备好，拨弹电机阻尼控制
	{
		PID_SetKpKiKd(&GstSH_SupplyPelletPosPID, 0.0f, 0.0f, 0.0f);
		PID_SetKpKiKd(&GstSH_SupplyPelletVelPID, PID_SupplyVel_Kp, PID_SupplyVel_Ki, PID_SupplyVel_Kd);

		/*正在发射标志位置0*/
		GstSH_Paras.RC_Shooting_Flag = false;

		/*目标设定*/
		GstSH_Paras.SupplyPellet_PosDes = GstSH_Paras.SupplyPellet_PosFB;
	}

	/*拨弹电机PID+TD计算*/
	PID_SetFB(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
	PID_SetFB(&GstSH_SupplyPelletVelPID, GstSH_Paras.SupplyPellet_VelFB);

	TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosDes);
	TD_Cal(&SupplyPellet_TD);
	PID_SetDes(&GstSH_SupplyPelletPosPID, TD_GetOutput(&SupplyPellet_TD));
	PID_Cal(&GstSH_SupplyPelletPosPID);
	PID_SetDes(&GstSH_SupplyPelletVelPID, PID_GetOutput(&GstSH_SupplyPelletPosPID) + SupplyPellet_td_coe_Norm*SupplyPellet_TD.x2); //TD速度前馈
	PID_Cal(&GstSH_SupplyPelletVelPID);

	GstSH_Paras.RC_IF_Shoot_Cnt++;	// 起到计时器的作用（两次拨弹间隔）
}

/**
  * @brief  拨弹电机键鼠模式控制函数
  * @note   在摩擦轮准备好的情况下，鼠标左键或右键按下回正实现单发，持续按下实现连发
  * @param  无
  * @retval 无
  */
void SupplyPellet_KeyMouseCtrl(void)
{
	//TODO:修改了单发逻辑，因此GstGM_MainCtrl.ST_Tx.SupplyPellet_Flag.Final_ShootOrNot_Flag目前不准（虽然但是原来也不准），后续想一想这个标志位怎么用

	/***********************前置处理**************************/
    /*状态切换*/
    if(GSTSH_Data.ShooterMode != GEMSH_Mode) //发射模式切换
    {
		PID_SetDes(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
		PID_SetDes(&GstSH_SupplyPelletVelPID, 0.0f);
		TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosFB);
		SupplyPellet_TD.x1 = GstSH_Paras.SupplyPellet_PosFB;
		SupplyPellet_TD.x2 = 0.0f;

		//TODO：摩擦轮前置处理
    }

	///*Is_Heat_Safe()*/
	if(__IS_FrictionWheel_Ready())
	{
		PID_SetKpKiKd(&GstSH_SupplyPelletPosPID, PID_SupplyPos_Kp, PID_SupplyPos_Ki, PID_SupplyPos_Kd);
		PID_SetKpKiKd(&GstSH_SupplyPelletVelPID, PID_SupplyVel_Kp, PID_SupplyVel_Ki, PID_SupplyVel_Kd);

		if(__IS_KeyMouse_Single_Shoot() || __IS_KeyMouse_Continuous_Shoot())//键鼠手动拨弹
		{
			if(GstSH_Paras.KeyMouse_IF_Shoot_Cnt >= 60)	//两次弹丸发射间隔为60ms
			{
				/*正在发射标志位置1*/
				GstSH_Paras.KeyMouse_Shooting_Flag = true;
				/*发射间隔计数器清零*/
				GstSH_Paras.KeyMouse_IF_Shoot_Cnt = 0;

				/*目标设定*/
				GstSH_Paras.SupplyPellet_PosDes += SupplyStep;	//每次发射目标位置增加一个步进角度
			}
			else
			{
				/*正在发射标志位置0*/
				GstSH_Paras.KeyMouse_Shooting_Flag = false;

				/*目标设定*/
				GstSH_Paras.SupplyPellet_PosDes = GstSH_Paras.SupplyPellet_PosDes;	//保持当前位置
			}
		}
		// else if(RC_AUTO_Control_Continuous_Shoot())//遥控器辅瞄模式，视觉控制打弹
		// {
		// 	if(GST_Vision.AimAssistDataReceiveFrame.ShootOrNot == 1 || GST_SystemMonitor.USART6Rx_fps <= 15)
		// 	{
		// 		// if( struct_shoot.RC_IF_Shoot_cnt >= 60.0f)
		// 		// {
		// 		// 	GstGM_MainCtrl.ST_Tx.SupplyPellet_Flag.Final_ShootOrNot_Flag = 1;
		// 		// 	struct_shoot.RC_IF_Shoot_cnt= 0;
		// 		// }
		// 	}
		// }
		else//未打弹
		{
			/*正在发射标志位置0*/
			GstSH_Paras.KeyMouse_Shooting_Flag = false;

			/*目标设定*/
			GstSH_Paras.SupplyPellet_PosDes = GstSH_Paras.SupplyPellet_PosDes;	//保持当前位置
		}
	}
	else //摩擦轮未准备好，拨弹电机阻尼控制
	{
		PID_SetKpKiKd(&GstSH_SupplyPelletPosPID, 0.0f, 0.0f, 0.0f);
		PID_SetKpKiKd(&GstSH_SupplyPelletVelPID, PID_SupplyVel_Kp, PID_SupplyVel_Ki, PID_SupplyVel_Kd);

		/*正在发射标志位置0*/
		GstSH_Paras.KeyMouse_Shooting_Flag = false;

		/*目标设定*/
		GstSH_Paras.SupplyPellet_PosDes = GstSH_Paras.SupplyPellet_PosFB;
	}

	/*拨弹电机PID+TD计算*/
	PID_SetFB(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
	PID_SetFB(&GstSH_SupplyPelletVelPID, GstSH_Paras.SupplyPellet_VelFB);

	TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosDes);
	TD_Cal(&SupplyPellet_TD);
	PID_SetDes(&GstSH_SupplyPelletPosPID, TD_GetOutput(&SupplyPellet_TD));
	PID_Cal(&GstSH_SupplyPelletPosPID);
	PID_SetDes(&GstSH_SupplyPelletVelPID, PID_GetOutput(&GstSH_SupplyPelletPosPID) + SupplyPellet_td_coe_Norm*SupplyPellet_TD.x2); //TD速度前馈
	PID_Cal(&GstSH_SupplyPelletVelPID);

	GstSH_Paras.KeyMouse_IF_Shoot_Cnt++;	// 起到计时器的作用（两次拨弹间隔）
}
//#endregion

//#region/*************************************摩擦轮/拨弹辅助函数*************************************************/
/**
  * @brief  摩擦轮是否准备好的判断函数
  * @note   摩擦轮转速与目标转速的误差在200rpm以内：返回True
  * 	    摩擦轮转速与目标转速的误差在200rpm以上：返回False
  * @param  无
  * @retval 无
  */
bool __IS_FrictionWheel_Ready(void)
{
	if(MyAbsf(smcR.fpDes-smcR.fpFB)<=200 && MyAbsf(smcL.fpDes-smcL.fpFB)<=200)
	{
		GstSH_Paras.FrictionWheel_ReadyOrNot_Flag = TRUE;
	}
	else 
	{
		GstSH_Paras.FrictionWheel_ReadyOrNot_Flag = FALSE;
	}
	
	if(GstSH_Paras.FrictionWheel_ReadyOrNot_Flag == FALSE)
		return FALSE; // 摩擦轮转速未达标，不可打弹
	else
		return TRUE; // 摩擦轮转速已达标，可打弹
}

/**
  * @brief  卡弹检测与保护函数
  * @note   目前暂时不用，因为目前的拨盘在卡弹情况下也无法向回转多个SupplyStep进行回退
  * 		后续如果换车或者拨盘设计有改动需要增加卡弹保护时再完善这个函数
  * @param  无
  * @retval 无
*/
void Bullet_Blocked_Protection(void)
{
    //卡弹保护
    //卡弹条件检测（位置+速度双阈值判断）
	if((MyAbsf(GstSH_SupplyPelletPosPID.Des - GstSH_SupplyPelletPosPID.FB) > 4.0f*MyAbsf(SupplyStep))
		&& (MyAbsf(GstSH_SupplyPelletVelPID.FB) < 20.0f))
	{
		//卡弹状态确认与保护触发
		GstSH_Paras.Bullet_Blocked_Cnt ++;
		if(GstSH_Paras.Bullet_Blocked_Cnt >= 300)  //进入堵转判断条件300ms
		{
			GstSH_Paras.IS_Bullet_Blocked_Flag = true; //堵转标志位置1
		}
		else
		{
			GstSH_Paras.IS_Bullet_Blocked_Flag = false;
		}
	}
	else
	{
		GstSH_Paras.Bullet_Blocked_Cnt = 0;
		GstSH_Paras.IS_Bullet_Blocked_Flag = false;
	}

	//若确认卡弹，进入保护状态
	if(GstSH_Paras.IS_Bullet_Blocked_Flag == true)
	{
		GstSH_Paras.Bullet_Blocked_Protection_Flag = true;
	}

    //保护状态下的恢复策略
	if(GstSH_Paras.Bullet_Blocked_Protection_Flag == true)
	{
		// GstSH_SupplyPelletVelPID.U = 0.0f; //拨弹电机输出设为0，停止拨弹
		GstSH_SupplyPelletPosPID.Des -= (int)(MyAbsf(GstSH_SupplyPelletPosPID.FB - GstSH_SupplyPelletPosPID.Des)/SupplyStep)*SupplyStep; //拨弹电机回退
		if (Locked_Rotor_Protect_Cnt >= 1000)
		{
			Locked_Rotor_Protect_Cnt = 0;
			Locked_Rotor_Protect_Flag = false;
		}
		Locked_Rotor_Protect_Cnt++;
	}
}

/*----------------------------------------------------------------------------------------
函数名：bool Is_Heat_Safe(void)
功能： 判断枪口热量是否安全
备注： 1-可以拨弹 0-不能拨弹
----------------------------------------------------------------------------------------*/
bool Is_Heat_Safe(void)
{
	// int32_t Heatlimit =  (ShooterHeat_Limit-20); //枪口热量上限
	// static float Pre_ShooterHeat = 0.0f;    //记录上一次枪口热量
	// //默认裁判系统通信正常
    // if(PelletSpeed_mes != Pre_ShootSpeed)
    // {
    //     Allowed_PelletNum--;    //发射速度更新，说明打出弹丸，计数
    //     PelletNum++;            //已经发射子弹颗数
    // }
    // else if(Is_Float_Equal(ShooterHeat_mes, Pre_ShooterHeat)==FALSE ||
    //         Is_Float_Equal(ShooterHeat_mes, 0.0f)==TRUE)              //枪口热量更新周期为20ms
    // {
    //     Allowed_PelletNum =(int16_t)((Heatlimit - ShooterHeat_mes) / 10); //一个周期更新会计算该周期可发射弹丸数量
    // }

    // Pre_ShooterHeat = ShooterHeat_mes;  //记录枪口热量
    // Pre_ShootSpeed = PelletSpeed_mes;   //记录弹丸速度
    // Heat_Left = (float)Heatlimit - (float)ShooterHeat_mes;
	// ///由于裁判系统帧率过低，采取不用裁判系统的热量判断的双重热量判断
    // Heat_Calculate();
    // Allowed_PelletNum_Friction =(int16_t)((Heatlimit - now_heat) / 10); //一个周期更新会计算该周期可发射弹丸数量
    // if(Allowed_PelletNum_Friction<=1||Allowed_PelletNum<=1 )  
    // {
    //     return FALSE;  //不允许发送
    // }
    // else                        
    // {
        return TRUE;   //允许发射
    // }
}
/*----------------------------------------------------------------------------------------
函数名：void Heat_Calculate(void)
功能： 不用裁判系统的热量判断
备注：
----------------------------------------------------------------------------------------*/
void Heat_Calculate(void) //由于裁判系统帧率过低，采取不用裁判系统的热量判断的双重热量判断
{
        // //摩擦轮速度差值
		// FMR_speed_diff = MyAbsf(smcR.fpDes-smcR.fpFB);
		// FML_speed_diff = MyAbsf(smcL.fpDes-smcL.fpFB);

		// //用摩擦轮转速差作简单判断是否打弹
	    // if(FMR_speed_diff >= 200 && FML_speed_diff >=200 && downflag == 0)
		// {
		// 	now_heat+=10;
		// 	downflag = 1;
        //     supply_cnt++;
		// }
		// else if(FMR_speed_diff < 60 && FML_speed_diff < 60 && downflag == 1)
		// {
		// 	downflag = 0;
		// }
		// now_heat -= (float) ShooterHeat_Rate * 0.001f;
		// if(now_heat<=0)
		// {
		// 	now_heat = 0;
		// }
}

/**
  * @brief  遥控器模式单发控制信号的判断函数
  * @note   触发方式：遥控器拨轮向下拨并回正（触发时间 < 500ms）
  * @param  无
  * @retval 无
  */
bool __IS_RC_Single_Shoot(void)
{
    // 获取当前滚轮状态（下拨=1，回正=0）
    GstSH_Paras.RC_Single_Shoot_Now_status = (GST_Receiver.ST_RC.Roller >= RCRoller_DownTH) ? 1 : 0;

    // 边沿检测：从下拨变为回正（下降沿）
    if(GstSH_Paras.RC_Single_Shoot_Now_status == 0 && GstSH_Paras.RC_Single_Shoot_Pre_status == 1)
    {
		GstSH_Paras.RC_Single_Shoot_Flag = 1;
    }
    else
    {
        GstSH_Paras.RC_Single_Shoot_Flag = 0;// 非边沿情况，清除单发标志
    }

    // 更新上一状态
    GstSH_Paras.RC_Single_Shoot_Pre_status = GstSH_Paras.RC_Single_Shoot_Now_status;

    if(GstSH_Paras.RC_Single_Shoot_Flag == 0)
        return FALSE; // 不允许发射
    else
        return TRUE;  // 允许发射
}

/**
  * @brief  遥控器模式连发控制信号的判断函数
  * @note   触发方式：遥控器拨轮向下拨并保持（触发时间 > 500ms）
  * @param  无
  * @retval 无
  */
bool __IS_RC_Continuous_Shoot(void)
{
    if(GST_Receiver.ST_RC.Roller >= RCRoller_DownTH) //遥控器滚轮下拨
	{
		GstSH_Paras.RC_IF_Continuous_Shoot_Cnt++;
	}
	else
	{
		GstSH_Paras.RC_IF_Continuous_Shoot_Cnt = 0;
	}

	if(GstSH_Paras.RC_IF_Continuous_Shoot_Cnt > 500)//连续拨动时间超过500ms，进入连发模式
	{
		GstSH_Paras.RC_Continuous_Shoot_Flag = 1;
	}
	else											//连续拨动时间未超过500ms，连发模式判断计数器清零
	{
		GstSH_Paras.RC_Continuous_Shoot_Flag = 0;
	}

	if (GstSH_Paras.RC_Continuous_Shoot_Flag == 0)
		return FALSE; //不允许发射
	else
		return TRUE;  //允许发射
}

/*----------------------------------------------------------------------------------------
函数名：bool RC_AUTO_Control_Continuous_Shoot(void)
功能：遥控器拨弹辅瞄控制连发模式判断
----------------------------------------------------------------------------------------*/
bool RC_AUTO_Control_Continuous_Shoot(void)
{
    // if(IsRollerUp()) //遥控器滚轮上拨
	// {
	// 	struct_shoot.RC_AUTO_IF_Shooting_cnt++;
	// }
	// else
	// {
	// 	struct_shoot.RC_AUTO_IF_Shooting_cnt = 0;
	// }

	// if (struct_shoot.RC_AUTO_IF_Shooting_cnt > 500)
	// {
	// 	struct_shoot.RC_AUTO_Shooting_flag = 1;
	// }
	// else
	// {
	// 	struct_shoot.RC_AUTO_Shooting_flag = 0;
	// }

	// if (struct_shoot.RC_AUTO_Shooting_flag == 0)
	// 	return FALSE; // 不允许发射
	// else
	// 	return TRUE; // 允许发射

	return FALSE; //TODO:目前先关闭这个功能，后续再完善
}

/**
  * @brief  键鼠模式单发控制信号的判断函数
  * @note   触发方式：键鼠左键按下并回正（触发时间 < 500ms）
  * @param  无
  * @retval 无
  */
bool __IS_KeyMouse_Single_Shoot(void)
{
    // 获取当前鼠标左键状态（按下=1，未按下=0）
    GstSH_Paras.KeyMouse_Single_Shoot_Now_status = (GST_Receiver.ST_Mouse.Left || GST_Receiver.ST_Mouse.Right);

    // 边沿检测：从按下变为回正松开（下降沿）
    if(GstSH_Paras.KeyMouse_Single_Shoot_Now_status == 0 && GstSH_Paras.KeyMouse_Single_Shoot_Pre_status == 1)
    {
		GstSH_Paras.KeyMouse_Single_Shoot_Flag = 1;
    }
    else
    {
        GstSH_Paras.KeyMouse_Single_Shoot_Flag = 0;// 非边沿情况，清除单发标志
    }

    // 更新上一状态
    GstSH_Paras.KeyMouse_Single_Shoot_Pre_status = GstSH_Paras.KeyMouse_Single_Shoot_Now_status;

    if(GstSH_Paras.KeyMouse_Single_Shoot_Flag == 0)
        return FALSE; // 不允许发射
    else
        return TRUE;  // 允许发射
}

/**
  * @brief  键鼠模式连发控制信号的判断函数
  * @note   触发方式：键鼠左键按下并保持（触发时间 > 500ms）
  * @param  无
  * @retval 无
  */
bool __IS_KeyMouse_Continuous_Shoot(void)
{
    if(GST_Receiver.ST_Mouse.Left || GST_Receiver.ST_Mouse.Right) //键鼠左键或右键按下
	{
		GstSH_Paras.KeyMouse_IF_Continuous_Shoot_Cnt++;
	}
	else
	{
		GstSH_Paras.KeyMouse_IF_Continuous_Shoot_Cnt = 0;
	}

	if(GstSH_Paras.KeyMouse_IF_Continuous_Shoot_Cnt > 500)//连续按下时间超过500ms，进入连发模式
	{
		GstSH_Paras.KeyMouse_Continuous_Shoot_Flag = 1;
	}
	else											//连续按下时间未超过500ms，连发模式判断计数器清零
	{
		GstSH_Paras.KeyMouse_Continuous_Shoot_Flag = 0;
	}

	if (GstSH_Paras.KeyMouse_Continuous_Shoot_Flag == 0)
		return FALSE; //不允许发射
	else
		return TRUE;  //允许发射
}
//#endregion
