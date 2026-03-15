/**
  ******************************************************************************
  * @file    Gimbal_Stratgy.c
  * @author  26赛季，平衡步兵电控，苏文远
  * @date    2026.3.8
  * @brief   云台模式选择及控制主逻辑
  ******************************************************************************
*/

#include "General_AuxiliaryFunc.h"
#include "Algorithm_Simple.h"
#include "Gimbal_Stratgy.h"
#include "GlobalDeclare_Gimbal.h"
#include "GlobalDeclare_Chassis.h"
#include "TIM_Config.h"
#include <arm_math.h>


//#region /*******************************云台模式选择与更新相关函数********************************************/
/********************* 模式切换 **********************/
// 任意状态 -> Debug: GstGMSH_Debug_Flags的云台测试标志位置1
bool _Is_Gimbal_Debug(void)
{
    if(GstGMSH_Debug_Flags.Gimbal_Test_Flag == 1)
    {return true;}
    else
    {return false;}
}

// 任意状态 -> Disabled: 底盘进入安全模式(手动安全或自动安全)
bool _Is_Any_to_Disabled(void)
{
    if(GEMCH_Mode == CHMode_ManualSafe || GEMCH_Mode == CHMode_AutoSafe)
    {return true;}
    else
    {return false;}
}

// Disabled -> RC_Free: 底盘进入遥控器待机Standby模式
bool _Is_Disabled_to_RCFree(void)
{
    if(GEMCH_Mode == CHMode_RC_Standby)
    {return true;}
    else
    {return false;}
}

// Disabled -> RC_Follow: 底盘进入Follow模式
bool _Is_Disabled_to_RCFollow(void)
{
    if(GEMCH_Mode == CHMode_RC_Follow)
    {return true;}
    else
    {return false;}
}

// Disabled -> KeyMouse: 底盘进入键鼠待机Standby模式
bool _Is_Disabled_to_KeyMouse(void)
{
    if(GEMCH_Mode == CHMode_KeyMouse_Standby)
    {return true;}
    else
    {return false;}
}

// RC_Free -> RC_Follow: 底盘模式切换
bool _Is_RCFree_to_Follow(void)
{
    if(GEMCH_Mode == CHMode_RC_Follow)
    {return true;}
    else
    {return false;}
}

// RC_Follow -> RC_Free: 底盘模式切换
bool _Is_RCFollow_to_Free(void)
{
    if(GEMCH_Mode == CHMode_RC_Free)
    {return true;}
    else
    {return false;}
}

// RC_Free -> AutoAim: 视觉找到目标，并且视觉小电脑接收帧率正常
bool _Is_RCFree_to_AutoAim(void)
{
    if(GST_Vision.AimAssistDataReceiveFrame.FindTargetOrNot == true && GST_SystemMonitor.USART6Rx_fps >= 10)
    {return true;}
    else
    {return false;}
}

// KeyMouse -> AutoAim: 鼠标右键按下，视觉找到目标并且小电脑接收帧率正常
bool _Is_KeyMouse_to_AutoAim(void)
{
    if(GST_Receiver.ST_Mouse.Right == 1 && GST_Vision.AimAssistDataReceiveFrame.FindTargetOrNot == true && GST_SystemMonitor.USART6Rx_fps >= 10)
    {return true;}
    else
    {return false;}
}

// KeyMouse -> Buff_Small: F键激活小符
bool _Is_KeyMouse_to_Buff_Small(void)
{
    // if(PRESSED_F == TRUE && PRESSED_F_Pre == FALSE)
    // {return true;}
    return false;
}

// KeyMouse -> Buff_Big: G键激活大符
bool _Is_KeyMouse_to_Buff_Big(void)
{
    // if(PRESSED_G == TRUE && PRESSED_G_Pre == FALSE)
    // {return true;}
    return false;
}

// KeyMouse -> Buff_Interfere_Small: Z键激活反小符
bool _Is_KeyMouse_to_Buff_Interfere_Small(void)
{
    // if(PRESSED_Z == TRUE && PRESSED_Z_Pre == FALSE)
    // {return true;}
    return false;
}

// KeyMouse -> Buff_Interfere_Big: X键激活反大符
bool _Is_KeyMouse_to_Buff_Interfere_Big(void)
{
    // if(PRESSED_X == TRUE && PRESSED_X_Pre == FALSE)
    // {return true;}
    return false;
}

// AutoAim -> KeyMouse: 鼠标右键松开，并且底盘为键鼠跟随模式
bool _Is_AutoAim_to_KeyMouse(void)
{
    if(GEMCH_Mode == CHMode_KeyMouse_Follow && GST_Receiver.ST_Mouse.Right == 0)
    {return true;}
    else
    {return false;}
}

// AutoAim -> RC_Free: 视觉小电脑未找到目标，或视觉小电脑接收帧率异常
bool _Is_AutoAim_to_RCFree(void)
{
    if(GST_Vision.AimAssistDataReceiveFrame.FindTargetOrNot == false || GST_SystemMonitor.USART6Rx_fps < 10)//GEMCH_Mode == CHMode_RC_Free && 
    {return true;}
    else
    {return false;}
}

// AutoAim -> RC_Follow: 
bool _Is_AutoAim_to_RCFollow(void)
{
    if(GEMCH_Mode == CHMode_RC_Follow && GST_Vision.AimAssistDataReceiveFrame.FindTargetOrNot == false)
    {return true;}
    else
    {return false;}
}

// Buff_Small -> KeyMouse: 再次按F键取消
bool _Is_Buff_Small_to_KeyMouse(void)
{
    // if(PRESSED_F == TRUE && PRESSED_F_Pre == FALSE)
    // {return true;}
    return false;
}

// Buff_Small -> RC_Free: 底盘进入安全模式
bool _Is_Buff_Small_to_RCFree(void)
{
    // if(GEMCH_Mode == CHMode_RC_ManualSafe || GEMCH_Mode == CHMode_RC_AutoSafe)
    // {return true;}
    return false;
}

// Buff_Small -> RC_Follow: 底盘进入安全模式
bool _Is_Buff_Small_to_RCFollow(void)
{
    // if(GEMCH_Mode == CHMode_RC_ManualSafe || GEMCH_Mode == CHMode_RC_AutoSafe)
    // {return true;}
    return false;
}

// Buff_Big -> KeyMouse: 再次按G键取消
bool _Is_Buff_Big_to_KeyMouse(void)
{
    // if(PRESSED_G == TRUE && PRESSED_G_Pre == FALSE)
    // {return true;}
    return false;
}

// Buff_Big -> RC_Free: 底盘进入安全模式
bool _Is_Buff_Big_to_RCFree(void)
{
    // if(GEMCH_Mode == CHMode_RC_ManualSafe || GEMCH_Mode == CHMode_RC_AutoSafe)
    // {return true;}
    return false;
}

// Buff_Big -> RC_Follow: 底盘进入安全模式
bool _Is_Buff_Big_to_RCFollow(void)
{
    // if(GEMCH_Mode == CHMode_RC_ManualSafe || GEMCH_Mode == CHMode_RC_AutoSafe)
    // {return true;}
    return false;
}

// Buff_Interfere_Small -> KeyMouse: 再次按Z键取消
bool _Is_Buff_Interfere_Small_to_KeyMouse(void)
{
    // if(PRESSED_Z == TRUE && PRESSED_Z_Pre == FALSE)
    // {return true;}
    return false;
}

// Buff_Interfere_Small -> RC_Free: 底盘进入安全模式
bool _Is_Buff_Interfere_Small_to_RCFree(void)
{
    // if(GEMCH_Mode == CHMode_RC_ManualSafe || GEMCH_Mode == CHMode_RC_AutoSafe)
    // {return true;}
    return false;
}

// Buff_Interfere_Small -> RC_Follow: 底盘进入安全模式
bool _Is_Buff_Interfere_Small_to_RCFollow(void)
{
    // if(GEMCH_Mode == CHMode_RC_ManualSafe || GEMCH_Mode == CHMode_RC_AutoSafe)
    // {return true;}
    return false;
}

// Buff_Interfere_Big -> KeyMouse: 再次按X键取消
bool _Is_Buff_Interfere_Big_to_KeyMouse(void)
{
    // if(PRESSED_X == TRUE && PRESSED_X_Pre == FALSE)
    // {return true;}
    return false;
}

// Buff_Interfere_Big -> RC_Free: 底盘进入安全模式
bool _Is_Buff_Interfere_Big_to_RCFree(void)
{
    // if(GEMCH_Mode == CHMode_RC_ManualSafe || GEMCH_Mode == CHMode_RC_AutoSafe)
    // {return true;}
    return false;
}

// Buff_Interfere_Big -> RC_Follow: 底盘进入安全模式
bool _Is_Buff_Interfere_Big_to_RCFollow(void)
{
    // if(GEMCH_Mode == CHMode_RC_ManualSafe || GEMCH_Mode == CHMode_RC_AutoSafe)
    // {return true;}
    return false;
}

/**
  * @brief  云台模式切换函数（状态机）
  * @note   根据不同的条件切换云台的模式，要注意优先级的问题，最高优先级是Debug和Disabled（全局中断）
  *         次高优先级是RC_Free/RC_Follow/KeyMouse基础控制模式
  *         最低优先级是AutoAim/Buff视觉辅助模式
  * @param  无
  * @retval GMMode_EnumTypeDef的枚举类型，云台的工作状态
*/
GMMode_EnumTypeDef GimbalModeChoose(void)
{
    // 状态变量准备
    GMMode_EnumTypeDef CurrentMode = GSTGM_Data.GimbalMode;
    GMMode_EnumTypeDef NextMode = CurrentMode;

    /***** Layer 1: 全局中断 (Global Interrupts) - 最高优先级 *****/
    if(_Is_Gimbal_Debug())
    {
        return GMMode_Debug;
    }
    else if(_Is_Any_to_Disabled())
    {
        return GMMode_Disabled;
    }

    /***** Layer 2: 状态机流转 (Switch-Case) *****/
    switch (CurrentMode)
    {
        /* --- 失能模式 --- */
        case GMMode_Disabled:
            if(_Is_Disabled_to_RCFree())                     NextMode = GMMode_RC_Free;
            else if(_Is_Disabled_to_KeyMouse())              NextMode = GMMode_KeyMouse;
            else                                             NextMode = GMMode_Disabled;
            break;

        /* --- 基本模式 --- */
        //TODO：目前遥控器控制模式只接了AutoAim一个视觉辅助模式，后面可以再加入其他视觉辅助模式
        case GMMode_RC_Free:
            if(_Is_RCFree_to_Follow())                       NextMode = GMMode_RC_Follow;
            else if(_Is_RCFree_to_AutoAim())                 NextMode = GMMode_AutoAim;
            else                                             NextMode = GMMode_RC_Free;
            break;

        case GMMode_RC_Follow:
            if(_Is_RCFollow_to_Free())                       NextMode = GMMode_RC_Free;
            else                                             NextMode = GMMode_RC_Follow;
            break;

        case GMMode_KeyMouse:
            if(_Is_KeyMouse_to_AutoAim())                    NextMode = GMMode_AutoAim;
            else if(_Is_KeyMouse_to_Buff_Small())            NextMode = GMMode_Buff_Small;
            else if(_Is_KeyMouse_to_Buff_Big())              NextMode = GMMode_Buff_Big;
            else if(_Is_KeyMouse_to_Buff_Interfere_Small())  NextMode = GMMode_Buff_Interfere_Small;
            else if(_Is_KeyMouse_to_Buff_Interfere_Big())    NextMode = GMMode_Buff_Interfere_Big;
            else                                             NextMode = GMMode_KeyMouse;
            break;

        /* --- 视觉辅助模式 --- */
        case GMMode_AutoAim:
            if(_Is_AutoAim_to_KeyMouse())                    NextMode = GMMode_KeyMouse;
            else if(_Is_AutoAim_to_RCFree())                 NextMode = GMMode_RC_Free;
            else if(_Is_AutoAim_to_RCFollow())               NextMode = GMMode_RC_Follow;
            else                                             NextMode = GMMode_AutoAim;
            break;

        case GMMode_Buff_Small:
            if(_Is_Buff_Small_to_KeyMouse())                 NextMode = GMMode_KeyMouse;
            else if(_Is_Buff_Small_to_RCFree())              NextMode = GMMode_RC_Free;
            else if(_Is_Buff_Small_to_RCFollow())            NextMode = GMMode_RC_Follow;
            else                                             NextMode = GMMode_Buff_Small;
            break;

        case GMMode_Buff_Big:
            if(_Is_Buff_Big_to_KeyMouse())                   NextMode = GMMode_KeyMouse;
            else if(_Is_Buff_Big_to_RCFree())                NextMode = GMMode_RC_Free;
            else if(_Is_Buff_Big_to_RCFollow())              NextMode = GMMode_RC_Follow;
            else                                             NextMode = GMMode_Buff_Big;
            break;

        case GMMode_Buff_Interfere_Small:
            if(_Is_Buff_Interfere_Small_to_KeyMouse())       NextMode = GMMode_KeyMouse;
            else if(_Is_Buff_Interfere_Small_to_RCFree())    NextMode = GMMode_RC_Free;
            else if(_Is_Buff_Interfere_Small_to_RCFollow())  NextMode = GMMode_RC_Follow;
            else                                             NextMode = GMMode_Buff_Interfere_Small;
            break;

        case GMMode_Buff_Interfere_Big:
            if(_Is_Buff_Interfere_Big_to_KeyMouse())         NextMode = GMMode_KeyMouse;
            else if(_Is_Buff_Interfere_Big_to_RCFree())      NextMode = GMMode_RC_Free;
            else if(_Is_Buff_Interfere_Big_to_RCFollow())    NextMode = GMMode_RC_Follow;
            else                                             NextMode = GMMode_Buff_Interfere_Big;
            break;

        default:
            NextMode = GMMode_Disabled;
            break;
    }

    return NextMode;
}
//#endregion


//#region /*******************************根据云台模式进行控制相关函数********************************************/
/**
  * @brief  遥控器模式或键鼠模式下，云台失能控制函数
  * @note   自由模式下的控制策略
  * @param  无
  * @retval 无
*/
void GM_Ctrl_Disabled(void)
{
    /***********************前置处理**************************/
    /*状态切换*/
    if(GEMGM_Mode != GSTGM_Data.GimbalMode) //云台模式切换
    {
        PID_SetDes(&GstGM_PitchPosPID, Pitch_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_PitchVelPID, Pitch_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Pitch_Motor_Paras.PosDes = Pitch_Motor_Paras.PosFB;
        Pitch_Motor_Paras.VelDes = Pitch_Motor_Paras.VelFB;
        TD_SetInput(&PitchTD, Pitch_Motor_Paras.PosFB);
        PitchTD.x1 = Pitch_Motor_Paras.PosFB;
        PitchTD.x2 = 0.0f;

        PID_SetDes(&GstGM_YawPosPID, Yaw_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_YawVelPID, Yaw_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Yaw_Motor_Paras.PosDes = Yaw_Motor_Paras.PosFB;
        Yaw_Motor_Paras.VelDes = Yaw_Motor_Paras.VelFB;
        TD_SetInput(&YawTD, Yaw_Motor_Paras.PosFB);
        YawTD.x1 = Yaw_Motor_Paras.PosFB;
        YawTD.x2 = 0.0f;
    }

    //设置PID参数为0
    PID_SetKpKiKd(&GstGM_PitchPosPID, 0.0f, 0.0f, 0.0f);
    PID_SetKpKiKd(&GstGM_PitchVelPID, 0.0f, 0.0f, 0.0f);
    PID_SetKpKiKd(&GstGM_YawPosPID,   0.0f, 0.0f, 0.0f);
    PID_SetKpKiKd(&GstGM_YawVelPID,   0.0f, 0.0f, 0.0f);

    //限幅
    Pitch_Motor_Paras.PosDes = Limit(Pitch_Motor_Paras.PosDes, PitchMin, PitchMax);

    GM_MotorProcess();
}

/**
  * @brief  遥控器模式下，自由模式云台控制函数
  * @note   自由模式下的控制策略
  * @param  无
  * @retval 无
*/
void GM_RCCtrl_Free(void)
{
    /***********************前置处理**************************/
    /*状态切换*/
    if(GEMGM_Mode != GSTGM_Data.GimbalMode) //云台模式切换
    {
        PID_SetDes(&GstGM_PitchPosPID, Pitch_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_PitchVelPID, Pitch_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Pitch_Motor_Paras.PosDes = Pitch_Motor_Paras.PosFB;
        Pitch_Motor_Paras.VelDes = Pitch_Motor_Paras.VelFB;
        TD_SetInput(&PitchTD, Pitch_Motor_Paras.PosFB);
        PitchTD.x1 = Pitch_Motor_Paras.PosFB;
        PitchTD.x2 = 0.0f;

        PID_SetDes(&GstGM_YawPosPID, Yaw_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_YawVelPID, Yaw_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Yaw_Motor_Paras.PosDes = Yaw_Motor_Paras.PosFB;
        Yaw_Motor_Paras.VelDes = Yaw_Motor_Paras.VelFB;
        TD_SetInput(&YawTD, Yaw_Motor_Paras.PosFB);
        YawTD.x1 = Yaw_Motor_Paras.PosFB;
        YawTD.x2 = 0.0f;
    }

    //设置PID参数为正常模式
    PID_SetKpKiKd(&GstGM_PitchPosPID, PID_PitchPos_Kp_Norm, PID_PitchPos_Ki_Norm, PID_PitchPos_Kd_Norm);
    PID_SetKpKiKd(&GstGM_PitchVelPID, PID_PitchVel_Kp_Norm, PID_PitchVel_Ki_Norm, PID_PitchVel_Kd_Norm);
    PID_SetKpKiKd(&GstGM_YawPosPID,   PID_YawPos_Kp_Norm,   PID_YawPos_Ki_Norm,   PID_YawPos_Kd_Norm);
    PID_SetKpKiKd(&GstGM_YawVelPID,   PID_YawVel_Kp_Norm,   PID_YawVel_Ki_Norm,   PID_YawVel_Kd_Norm);

    //计算pitch和yaw轴增量
    float PitchIncrement = GST_Receiver.ST_RC.JoyStickR_Y - RCChannelValue_Mid; // Pitch轴增量
    float YawIncrement   = GST_Receiver.ST_RC.JoyStickR_X - RCChannelValue_Mid; // Yaw轴增量

    //Pitch目标值赋值
    if(MyAbsf(PitchIncrement)> RCChannel_DeadZone)
    {
        Pitch_Motor_Paras.PosDes += RC_SST_Pitch * PitchIncrement;
    }
    //Yaw目标值赋值
    if(MyAbsf(YawIncrement) > RCChannel_DeadZone)
    {
        Yaw_Motor_Paras.PosDes -= RC_SST_Yaw * YawIncrement;
    }

    //限幅
    Pitch_Motor_Paras.PosDes = Limit(Pitch_Motor_Paras.PosDes, PitchMin, PitchMax);

    GM_MotorProcess();
}

/**
  * @brief  遥控器模式下，跟随模式云台控制函数
  * @note   跟随模式下的控制策略
  * @param  无
  * @retval 无
*/
void GM_RCCtrl_Follow(void)
{
    /***********************前置处理**************************/
    /*状态切换*/
    if(GEMGM_Mode != GSTGM_Data.GimbalMode) //云台模式切换
    {
        PID_SetDes(&GstGM_PitchPosPID, Pitch_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_PitchVelPID, Pitch_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Pitch_Motor_Paras.PosDes = Pitch_Motor_Paras.PosFB;
        Pitch_Motor_Paras.VelDes = Pitch_Motor_Paras.VelFB;
        TD_SetInput(&PitchTD, Pitch_Motor_Paras.PosFB);
        PitchTD.x1 = Pitch_Motor_Paras.PosFB;
        PitchTD.x2 = 0.0f;

        PID_SetDes(&GstGM_YawPosPID, Yaw_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_YawVelPID, Yaw_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Yaw_Motor_Paras.PosDes = Yaw_Motor_Paras.PosFB;
        Yaw_Motor_Paras.VelDes = Yaw_Motor_Paras.VelFB;
        TD_SetInput(&YawTD, Yaw_Motor_Paras.PosFB);
        YawTD.x1 = Yaw_Motor_Paras.PosFB;
        YawTD.x2 = 0.0f;
    }

    //设置PID参数为正常模式
    PID_SetKpKiKd(&GstGM_PitchPosPID, PID_PitchPos_Kp_Norm, PID_PitchPos_Ki_Norm, PID_PitchPos_Kd_Norm);
    PID_SetKpKiKd(&GstGM_PitchVelPID, PID_PitchVel_Kp_Norm, PID_PitchVel_Ki_Norm, PID_PitchVel_Kd_Norm);
    PID_SetKpKiKd(&GstGM_YawPosPID,   PID_YawPos_Kp_Norm,   PID_YawPos_Ki_Norm,   PID_YawPos_Kd_Norm);
    PID_SetKpKiKd(&GstGM_YawVelPID,   PID_YawVel_Kp_Norm,   PID_YawVel_Ki_Norm,   PID_YawVel_Kd_Norm);

    //计算pitch和yaw轴增量
    float PitchIncrement = GST_Receiver.ST_RC.JoyStickR_Y - RCChannelValue_Mid; // Pitch轴增量
    float YawIncrement   = GST_Receiver.ST_RC.JoyStickR_X - RCChannelValue_Mid; // Yaw轴增量

    //Pitch目标值赋值
    if(MyAbsf(PitchIncrement)> RCChannel_DeadZone)
    {
        Pitch_Motor_Paras.PosDes += RC_SST_Pitch * PitchIncrement;
    }
    //Yaw目标值赋值
    if(MyAbsf(YawIncrement) > RCChannel_DeadZone)
    {
        Yaw_Motor_Paras.PosDes -= RC_SST_Yaw * YawIncrement;
    }

    //限幅
    Pitch_Motor_Paras.PosDes = Limit(Pitch_Motor_Paras.PosDes, PitchMin, PitchMax);

    GM_MotorProcess();
}

/**
  * @brief  键鼠模式下云台控制函数
  * @note   键鼠模式下的控制策略
  * @param  无
  * @retval 无
*/
void GM_Ctrl_KeyMouse(void)
{
    /***********************前置处理**************************/
    /*状态切换*/
    if(GEMGM_Mode != GSTGM_Data.GimbalMode) //云台模式切换
    {
        PID_SetDes(&GstGM_PitchPosPID, Pitch_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_PitchVelPID, Pitch_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Pitch_Motor_Paras.PosDes = Pitch_Motor_Paras.PosFB;
        Pitch_Motor_Paras.VelDes = Pitch_Motor_Paras.VelFB;
        TD_SetInput(&PitchTD, Pitch_Motor_Paras.PosFB);
        PitchTD.x1 = Pitch_Motor_Paras.PosFB;
        PitchTD.x2 = 0.0f;

        PID_SetDes(&GstGM_YawPosPID, Yaw_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_YawVelPID, Yaw_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Yaw_Motor_Paras.PosDes = Yaw_Motor_Paras.PosFB;
        Yaw_Motor_Paras.VelDes = Yaw_Motor_Paras.VelFB;
        TD_SetInput(&YawTD, Yaw_Motor_Paras.PosFB);
        YawTD.x1 = Yaw_Motor_Paras.PosFB;
        YawTD.x2 = 0.0f;
    }

    //设置PID参数为正常模式
    PID_SetKpKiKd(&GstGM_PitchPosPID, PID_PitchPos_Kp_Norm, PID_PitchPos_Ki_Norm, PID_PitchPos_Kd_Norm);
    PID_SetKpKiKd(&GstGM_PitchVelPID, PID_PitchVel_Kp_Norm, PID_PitchVel_Ki_Norm, PID_PitchVel_Kd_Norm);
    PID_SetKpKiKd(&GstGM_YawPosPID,   PID_YawPos_Kp_Norm,   PID_YawPos_Ki_Norm,   PID_YawPos_Kd_Norm);
    PID_SetKpKiKd(&GstGM_YawVelPID,   PID_YawVel_Kp_Norm,   PID_YawVel_Ki_Norm,   PID_YawVel_Kd_Norm);

    //E键控制Yaw轴旋转180度
    if(IS_Key_ShortClick(&Key_E))
    {
        Yaw_Motor_Paras.PosDes += 180.0f;
        GSTCH_Data.F_DirectionInvert = !GSTCH_Data.F_DirectionInvert; //底盘方向取反
        GstCH_FollowMode_Paras.YawZeroPoint -= 180.0f;                //跟随模式零位+180度
    }

    //TODO：目前使用位控，以后可以试试速控
    //计算pitch和yaw轴增量（键鼠模式使用鼠标移动）
    float PitchIncrement = (float)GST_Receiver.ST_Mouse.Y / 32768.0f;   //鼠标Y控制Pitch，范围为-1到1
    float YawIncrement   = (float)GST_Receiver.ST_Mouse.X / 32768.0f;   //鼠标X控制Yaw，  范围为-1到1

    //Pitch目标值赋值
    if(MyAbsf(PitchIncrement) > 0)
    {
        PitchIncrement = Limit(PitchIncrement, -0.012f, 0.012f); //对Pitch增量进行限幅，防止输入错误导致云台旋转过快
        Pitch_Motor_Paras.PosDes -= KeyMouse_SST_Pitch * PitchIncrement;
    }
    //Yaw目标值赋值
    if(MyAbsf(YawIncrement) > 0)
    {
        YawIncrement = Limit(YawIncrement, -0.005f, 0.005f); //对Yaw增量进行限幅，防止输入错误导致云台旋转过快
        Yaw_Motor_Paras.PosDes -= KeyMouse_SST_Yaw * YawIncrement;
    }

    //限幅
    Pitch_Motor_Paras.PosDes = Limit(Pitch_Motor_Paras.PosDes, PitchMin, PitchMax);

    GM_MotorProcess();
}



/**
  * @brief  视觉辅瞄模式云台控制函数
  * @note   AutoAim模式下的控制策略
  * @param  无
  * @retval 无
*/
void GM_Ctrl_AutoAim(void)
{
    /***********************前置处理**************************/
    /*状态切换*/
    if(GEMGM_Mode != GSTGM_Data.GimbalMode) //云台模式切换
    {
        PID_SetDes(&GstGM_PitchPosPID, Pitch_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_PitchVelPID, Pitch_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Pitch_Motor_Paras.PosDes = Pitch_Motor_Paras.PosFB;
        Pitch_Motor_Paras.VelDes = Pitch_Motor_Paras.VelFB;
        TD_SetInput(&PitchTD, Pitch_Motor_Paras.PosFB);
        PitchTD.x1 = Pitch_Motor_Paras.PosFB;
        PitchTD.x2 = 0.0f;

        PID_SetDes(&GstGM_YawPosPID, Yaw_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_YawVelPID, Yaw_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Yaw_Motor_Paras.PosDes = Yaw_Motor_Paras.PosFB;
        Yaw_Motor_Paras.VelDes = Yaw_Motor_Paras.VelFB;
        TD_SetInput(&YawTD, Yaw_Motor_Paras.PosFB);
        YawTD.x1 = Yaw_Motor_Paras.PosFB;
        YawTD.x2 = 0.0f;
    }

    //设置PID参数为正常模式
    PID_SetKpKiKd(&GstGM_PitchPosPID, PID_PitchPos_Kp_Norm, PID_PitchPos_Ki_Norm, PID_PitchPos_Kd_Norm);
    PID_SetKpKiKd(&GstGM_PitchVelPID, PID_PitchVel_Kp_Norm, PID_PitchVel_Ki_Norm, PID_PitchVel_Kd_Norm);
    PID_SetKpKiKd(&GstGM_YawPosPID,   PID_YawPos_Kp_Norm,   PID_YawPos_Ki_Norm,   PID_YawPos_Kd_Norm);
    PID_SetKpKiKd(&GstGM_YawVelPID,   PID_YawVel_Kp_Norm,   PID_YawVel_Ki_Norm,   PID_YawVel_Kd_Norm);

    //云台目标值赋值
    Pitch_Motor_Paras.PosDes = GST_Vision.AimAssistDataReceiveFrame.Pitch;
    Yaw_Motor_Paras.PosDes   = GST_Vision.AimAssistDataReceiveFrame.Yaw;

    //限幅
    Pitch_Motor_Paras.PosDes = Limit(Pitch_Motor_Paras.PosDes, PitchMin, PitchMax);

    GM_MotorProcess();
}

/**
  * @brief  小符模式云台控制函数
  * @note   打小符模式下的控制策略
  * @param  无
  * @retval 无
*/
void GM_Ctrl_BuffSmall(void)
{
    GM_MotorProcess();
}

/**
  * @brief  大符模式云台控制函数
  * @note   打大符模式下的控制策略
  * @param  无
  * @retval 无
*/
void GM_Ctrl_BuffBig(void)
{
    GM_MotorProcess();
}

/**
  * @brief  反小符模式云台控制函数
  * @note   反打小符模式下的控制策略
  * @param  无
  * @retval 无
*/
void GM_Ctrl_BuffInterfereSmall(void)
{
    GM_MotorProcess();
}

/**
  * @brief  反大符模式云台控制函数
  * @note   反打大符模式下的控制策略
  * @param  无
  * @retval 无
*/
void GM_Ctrl_BuffInterfereBig(void)
{
    GM_MotorProcess();
}

/**
  * @brief  云台调试模式控制函数
  * @note   用于云台PID、TD调参或测试功能
  * @param  无
  * @retval 无
*/
void GM_Ctrl_Debug(void)
{
    /***********************前置处理**************************/
    /*状态切换*/
    if(GEMGM_Mode != GSTGM_Data.GimbalMode) //云台模式切换
    {
        PID_SetDes(&GstGM_PitchPosPID, Pitch_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_PitchVelPID, Pitch_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Pitch_Motor_Paras.PosDes = Pitch_Motor_Paras.PosFB;
        Pitch_Motor_Paras.VelDes = Pitch_Motor_Paras.VelFB;
        TD_SetInput(&PitchTD, Pitch_Motor_Paras.PosFB);
        PitchTD.x1 = Pitch_Motor_Paras.PosFB;
        PitchTD.x2 = 0.0f;

        PID_SetDes(&GstGM_YawPosPID, Yaw_Motor_Paras.PosFB);  //目标值设为当前位置，防止切换时偏差导致阶跃输入进而发散
        PID_SetDes(&GstGM_YawVelPID, Yaw_Motor_Paras.VelFB);  //目标值设为当前速度，防止切换时偏差导致阶跃输入进而发散
        Yaw_Motor_Paras.PosDes = Yaw_Motor_Paras.PosFB;
        Yaw_Motor_Paras.VelDes = Yaw_Motor_Paras.VelFB;
        TD_SetInput(&YawTD, Yaw_Motor_Paras.PosFB);
        YawTD.x1 = Yaw_Motor_Paras.PosFB;
        YawTD.x2 = 0.0f;
    }

    /*Pitch电机调试*/
    if(GstGMSH_Debug_Flags.Pitch_Test_Flag == 1 && GstPitch_DebugDes_AutoAlter.StartFlag == 1)
    {
        //调整Pitch轴PID参数
        PID_SetKpKiKd(&GstGM_PitchPosPID, GstGMSH_Debug_Paras.PitchPosKp, GstGMSH_Debug_Paras.PitchPosKi, GstGMSH_Debug_Paras.PitchPosKd);
        PID_SetKpKiKd(&GstGM_PitchVelPID, GstGMSH_Debug_Paras.PitchVelKp, GstGMSH_Debug_Paras.PitchVelKi, GstGMSH_Debug_Paras.PitchVelKd);
        
        //测试目标值自动设定
        Test_TargetAutoAlter(&GstPitch_DebugDes_AutoAlter, &Pitch_Motor_Paras.PosDes);
    }
    else
    {
        PID_SetKpKiKd(&GstGM_PitchPosPID, 0.0f, 0.0f, 0.0f);
        PID_SetKpKiKd(&GstGM_PitchVelPID, 0.0f, 0.0f, 0.0f);

        //目标值设为当前角度
        Pitch_Motor_Paras.PosDes = Pitch_Motor_Paras.PosFB;
        TD_SetInput(&PitchTD, Pitch_Motor_Paras.PosFB);
        PitchTD.x1 = Pitch_Motor_Paras.PosFB;
        PitchTD.x2 = 0.0f;
    }

    /*Yaw电机调试*/
    if(GstGMSH_Debug_Flags.Yaw_Test_Flag == 1 && GstYaw_DebugDes_AutoAlter.StartFlag == 1)
    {
        //调整Yaw轴PID参数
        PID_SetKpKiKd(&GstGM_YawPosPID, GstGMSH_Debug_Paras.YawPosKp, GstGMSH_Debug_Paras.YawPosKi, GstGMSH_Debug_Paras.YawPosKd);
        PID_SetKpKiKd(&GstGM_YawVelPID, GstGMSH_Debug_Paras.YawVelKp, GstGMSH_Debug_Paras.YawVelKi, GstGMSH_Debug_Paras.YawVelKd);
    
        //测试目标值自动设定
        Test_TargetAutoAlter(&GstYaw_DebugDes_AutoAlter, &Yaw_Motor_Paras.PosDes);
    }
    else
    {
        PID_SetKpKiKd(&GstGM_YawPosPID, 0.0f, 0.0f, 0.0f);
        PID_SetKpKiKd(&GstGM_YawVelPID, 0.0f, 0.0f, 0.0f);

        //目标值设为当前角度
        Yaw_Motor_Paras.PosDes = Yaw_Motor_Paras.PosFB;
        TD_SetInput(&YawTD, Yaw_Motor_Paras.PosFB);
        YawTD.x1 = Yaw_Motor_Paras.PosFB;
        YawTD.x2 = 0.0f;
    }

    //限幅
    Pitch_Motor_Paras.PosDes = Limit(Pitch_Motor_Paras.PosDes, PitchMin, PitchMax);

    GM_MotorProcess();
}

/**
  * @brief  根据云台模式控制云台
  * @note   此过程无优先级
  * @param  ModeNow：当前模式
  * @retval void
*/
void GimbalModeControl(GMMode_EnumTypeDef ModeNow)
{
    switch(ModeNow)
    {
        case GMMode_Disabled:
            GM_Ctrl_Disabled();          break;

        case GMMode_RC_Free:
            GM_RCCtrl_Free();              break;

        case GMMode_RC_Follow:
            GM_RCCtrl_Follow();            break;

        case GMMode_KeyMouse:
            GM_Ctrl_KeyMouse();            break;

        case GMMode_AutoAim:
            GM_Ctrl_AutoAim();             break;

        case GMMode_Buff_Small:
            GM_Ctrl_BuffSmall();           break;

        case GMMode_Buff_Big:
            GM_Ctrl_BuffBig();             break;

        case GMMode_Buff_Interfere_Small:
            GM_Ctrl_BuffInterfereSmall();  break;

        case GMMode_Buff_Interfere_Big:
            GM_Ctrl_BuffInterfereBig();    break;

        case GMMode_Debug:
            GM_Ctrl_Debug();             break;

      default:
          GM_Ctrl_Disabled();            break;
    }
}
//#endregion


//#region /*******************************视觉控制相关函数********************************************/
/**
  * @brief  视觉模式预处理函数
  * @note   包括视觉模式更新 与 视觉模式切换处理等
  * @param  无
  * @retval 无
*/
void Vision_Process(void)
{
    Vision_StateMode_Update(); //视觉状态与模式更新
    Vision_Switch_Process();   //视觉模式切换处理
    
    //使能状态及辅瞄模式记录更新
    Vision_State_Enable_Pre = Vision_State_Enable_Now;
    Vision_Mode_Pre = Vision_Mode_Now;
}
//#endregion
