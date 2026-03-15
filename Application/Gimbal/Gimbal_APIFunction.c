/**
  ******************************************************************************
  * @file    Gimbal_APIFunction.c
  * @author  26赛季，平衡步兵电控，苏文远
  * @date    2026.3.8
  * @brief   云台相关控制函数
  ******************************************************************************
*/
#include "Algorithm.h"
#include "Algorithm_Simple.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Chassis.h"
#include "GlobalDeclare_Gimbal.h"
#include "General_AuxiliaryFunc.h"
#include "Gimbal_APIFunction.h"
#include "TIM_Config.h"
#include <arm_math.h>


/**
  * @brief  更新云台相关数据的反馈值
  * @note   云台相关，正式结构体数据的反馈值，在这里进行更新汇总处理
  *         主要的云台相关正式结构体有：
  *             GSTGM_Data：云台主要数据结构体
  *             Pitch_Motor_Paras：Pitch轴电机参数结构体
  *             Yaw_Motor_Paras：Yaw轴电机参数结构体
  * @param  无
  * @retval 无
*/
void Gimbal_AllFBDataUpdate(void)
{
    /********************* 运动模式更新 **********************/
    GSTGM_Data.GimbalMode  = GEMGM_Mode;

    /*************** IMU1-云台运动姿态数据更新 ***************/
    GSTGM_Data.PitchPosFB   = 	GstGM_IMU1.ST_Rx.PitchAngle;
    GSTGM_Data.PitchVelFB   = 	GstGM_IMU1.ST_Rx.PitchAngleVel;
    GSTGM_Data.YawPosFB     = 	GstGM_IMU1.ST_Rx.YawAngle;
    GSTGM_Data.YawVelFB     = 	GstGM_IMU1.ST_Rx.YawSpeed;
    GSTGM_Data.RollPosFB    = 	GstGM_IMU1.ST_Rx.RollAngle;
    // GSTGM_Data.RollVelFB   = 	GstGB_IMU1.ST_Rx.RollAngleVel;//无此变量

    /*************** 云台目标值更新 ***************/
    GSTGM_Data.PitchPosDes = Pitch_Motor_Paras.PosDes;
    GSTGM_Data.PitchVelDes = Pitch_Motor_Paras.VelDes;
    GSTGM_Data.YawPosDes   = Yaw_Motor_Paras.PosDes;
    GSTGM_Data.YawVelDes   = Yaw_Motor_Paras.VelDes;

    /*************** 云台反馈值更新 ***************/
    Pitch_Motor_Paras.PosFB = GstGM_IMU1.ST_Rx.PitchAngle;    //电机反馈的当前角度值，单位度
    Pitch_Motor_Paras.VelFB = GstGM_IMU1.ST_Rx.PitchAngleVel; //电机反馈的当前角速度值，单位度/s
    Yaw_Motor_Paras.PosFB = GstGM_IMU1.ST_Rx.YawAngle;        //假设IMU1的YawAngle为当前yaw角度反馈值(实际可能有点区别)
    Yaw_Motor_Paras.VelFB = GstGM_IMU1.ST_Rx.YawSpeed;        //假设IMU1的YawSpeed为当前yaw角速度反馈值(实际可能有点区别）

}

/**
  * @brief  云台运动控制处理函数
  * @note   云台运动控制的主要处理函数
  *         包括两轴电机的PID+TD计算等
  *         在允许运动的云台策略Stratgy中调用
  * @param  无
  * @retval 无
*/
void GM_MotorProcess(void)
{
    /*云台TD计算*/
    TD_SetInput(&PitchTD, Pitch_Motor_Paras.PosDes);
    TD_Cal(&PitchTD);

    TD_SetInput(&YawTD, Yaw_Motor_Paras.PosDes);
    TD_Cal(&YawTD);


    /*云台PID计算*/
    PID_SetFB(&GstGM_PitchPosPID, Pitch_Motor_Paras.PosFB);
    PID_SetFB(&GstGM_PitchVelPID, Pitch_Motor_Paras.VelFB);
    PID_SetDes(&GstGM_PitchPosPID, TD_GetOutput(&PitchTD));
  	PID_Cal(&GstGM_PitchPosPID);
    PID_SetDes(&GstGM_PitchVelPID, (GstGM_PitchPosPID.U + pitch_td_coe*PitchTD.x2)); //TD速度前馈 正负号由云控方向决定，换车需修改
  	PID_Cal(&GstGM_PitchVelPID);

    PID_SetFB(&GstGM_YawPosPID, Yaw_Motor_Paras.PosFB);
    PID_SetFB(&GstGM_YawVelPID, Yaw_Motor_Paras.VelFB);
    PID_SetDes(&GstGM_YawPosPID, TD_GetOutput(&YawTD));
  	PID_Cal(&GstGM_YawPosPID);
    PID_SetDes(&GstGM_YawVelPID, (GstGM_YawPosPID.U + yaw_td_coe*YawTD.x2)); //TD速度前馈 正负号由云控方向决定，换车需修改
  	PID_Cal(&GstGM_YawVelPID);
}

//#region /*******************************视觉模式更新与处理相关函数********************************************/
/**
  * @brief  视觉状态与模式更新函数
  * @note   视觉使能状态与辅瞄模式选择（辅瞄，打符模式等）
  * @param  无
  * @retval 无
*/
void Vision_StateMode_Update(void)
{
    /*根据模式设定视觉状态和视觉当前模式*/
    if(GEMGM_Mode == GMMode_AutoAim)
    {
        Vision_State_Enable_Now = Vision_Enable; //使能视觉状态
        Vision_Mode_Now = VisionMode_Assist_Aim_Normal; //设定当前视觉模式
    }
    else if(GEMGM_Mode == GMMode_Buff_Small)
    {
        Vision_State_Enable_Now = Vision_Enable;
        Vision_Mode_Now = VisionMode_BuffSmall;
    }
    else if(GEMGM_Mode == GMMode_Buff_Big)
    {
        Vision_State_Enable_Now = Vision_Enable;
        Vision_Mode_Now = VisionMode_BuffBig;
    }
    else if(GEMGM_Mode == GMMode_Buff_Interfere_Small)
    {
        Vision_State_Enable_Now = Vision_Enable;
        Vision_Mode_Now = VisionMode_Interfere_BuffSmall;
    }
    else if(GEMGM_Mode == GMMode_Buff_Interfere_Big)
    {
        Vision_State_Enable_Now = Vision_Enable;
        Vision_Mode_Now = VisionMode_Interfere_BuffBig;
    }
    else
    {
        Vision_State_Enable_Now = Vision_Disable;//当前视觉模式不使能
    }
}

/**
  * @brief  视觉模式切换时处理函数
  * @note   视觉切换过程中，云台目标值时刻等于反馈值。
  * @note   不切换时，云台目标值等于视觉反馈值
  * @param  无
  * @retval 无
*/
void Vision_Switch_Process(void)
{
    //判断辅瞄模式是否处于正在切换状态
    static uint8_t cnt = 0;
    if(Vision_Mode_Now != Vision_Mode_Pre)
    {
        Vision_Mode_Changing = TRUE; //视觉模式正在切换，切换标志位置1
    }

    if(Vision_Mode_Changing == TRUE)
    {
        cnt++;
        if(cnt >= 50) //视觉切换标志位持续50ms
        {
            Vision_Mode_Changing = FALSE;
            cnt = 0;
        }
    }

    if(Vision_Mode_Changing == TRUE) //如果处于视觉切换状态，云台目标值等于反馈值
    {
        Pitch_Motor_Paras.PosDes = Pitch_Motor_Paras.PosFB;
        Yaw_Motor_Paras.PosDes   = Yaw_Motor_Paras.PosFB;
    }
    else //如果不处于切换状态，则将视觉目标值赋值给云台目标值
    {
        if(Vision_State_Enable_Now == Vision_Enable) //&& GST_SystemMonitor.USART6Rx_fps >= 10 //&& GST_Vision.AimAssistDataReceiveFrame.FindTargetOrNot == TRUE
        {
            Pitch_Motor_Paras.PosDes = GST_Vision.AimAssistDataReceiveFrame.Pitch;
            Yaw_Motor_Paras.PosDes   = GST_Vision.AimAssistDataReceiveFrame.Yaw;
        }
    }
}
//#endregion
