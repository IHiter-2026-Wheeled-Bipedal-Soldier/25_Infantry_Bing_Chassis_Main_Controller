/**
  ******************************************************************************
  * @file    Shooter_Stratgy.c
  * @author  26赛季，平衡步兵电控，苏文远
  * @date    2026.3.10
  * @brief   发射模式选择及控制主逻辑
  ******************************************************************************
*/

#include "TIM_Config.h"
#include "Algorithm_Simple.h"
#include "GlobalDeclare_Gimbal.h"
#include "Shooter_Stratgy.h"
#include <arm_math.h>

//#region /*******************************发射模式选择与更新相关函数********************************************/
/********************* 模式切换 **********************/
//TODO：发射状态机未完善
// 任意状态 -> Debug: GstGMSH_Debug_Flags的发射测试标志位或拨弹测试标志位置1
bool _Is_Shooter_Debug(void)
{
    if(GstGMSH_Debug_Flags.FrictionWheel_Test_Flag == 1 || GstGMSH_Debug_Flags.SupplyPellet_Test_Flag == 1)
    {return true;}
    else
    {return false;}
}

// 任意状态 -> Safe: 云台模式为失能模式或调试模式
bool _Is_Any_to_Safe(void)
{
    if(GSTGM_Data.GimbalMode == GMMode_Disabled || GSTGM_Data.GimbalMode == GMMode_Debug)
    {return true;}
    else
    {return false;}
}

// Safe -> RC: 云台进入遥控器Free模式
bool _Is_Safe_to_RC(void)
{
    if(GEMGM_Mode == GMMode_RC_Free)
    {return true;}
    else
    {return false;}
}

// Safe -> KeyMouse: 云台进入键鼠模式
bool _Is_Safe_to_KeyMouse(void)
{
    if(GEMGM_Mode == GMMode_KeyMouse)
    {return true;}
    else
    {return false;}
}

/**
  * @brief  发射控制模式选择
  * @note   遥控器，键鼠，安全模式选择
  * @param  无
  * @retval 
*/
SHMode_EnumTypeDef Shooter_ModeChoose(void)
{
    // 状态变量准备
    SHMode_EnumTypeDef CurrentMode = GEMSH_Mode;
    SHMode_EnumTypeDef NextMode = CurrentMode;

    /***** Layer 1: 全局中断 (Global Interrupts) - 最高优先级 *****/
    if(_Is_Shooter_Debug())
    {
        return SHMode_Debug;
    }
    else if(_Is_Any_to_Safe())
    {
        return SHMode_Safe;
    }

    /***** Layer 2: 状态机流转 (Switch-Case) *****/
    switch (CurrentMode)
    {
        /* --- 安全模式 --- */
        case SHMode_Safe:
            if(_Is_Safe_to_RC())                            NextMode = SHMode_RC;
            else if(_Is_Safe_to_KeyMouse())                 NextMode = SHMode_KeyMouse;
            else                                            NextMode = SHMode_Safe;
            break;

        /* --- 遥控器模式 --- */
        case SHMode_RC:
                                                            NextMode = SHMode_RC;
            break;

        /* --- 键鼠模式 --- */
        case SHMode_KeyMouse:
                                                            NextMode = SHMode_KeyMouse;
            break;

        default:
                                                            NextMode = SHMode_Safe;
            break;
    }

    return NextMode;
}


/********************* 根据模式控制 **********************/
/**
  * @brief  发射安全模式控制函数
  * @note   安全模式下的控制策略
  * @param  无
  * @retval 无
*/
void SHCtrl_Safe(void)
{
    /*安全锁打开，防止误打弹*/
    ShooterSafetyLocked = true;
    SupplyStep = (1.0f)*360.0f/SupplyPellet_Num; //根据拨盘齿数计算步进角度(正负号由拨弹正方向决定，换车需修改)
    /***********************前置处理**************************/
    /*状态切换*/
    if(GSTSH_Data.ShooterMode != GEMSH_Mode) //发射模式切换
    {
        PID_SetDes(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
		PID_SetDes(&GstSH_SupplyPelletVelPID, 0.0f);
		TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosFB);
		SupplyPellet_TD.x1 = GstSH_Paras.SupplyPellet_PosFB;
		SupplyPellet_TD.x2 = 0.0f;
    }

    FrictionWheel_Safe();

    SupplyPellet_Safe(); //拨弹电机闭环控制
	// Bullet_Blocked_Protection(); //卡弹保护
}

/**
  * @brief  发射调试模式控制函数
  * @note   调试模式下的控制策略
  * @param  无
  * @retval 无
*/
void SHCtrl_Debug(void)
{
    SupplyStep = (1.0f)*360.0f/SupplyPellet_Num; //根据拨盘齿数计算步进角度(正负号由拨弹正方向决定，换车需修改)
    /***********************前置处理**************************/
    /*状态切换*/
    if(GSTSH_Data.ShooterMode != GEMSH_Mode) //发射模式切换
    {
		PID_SetDes(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
		PID_SetDes(&GstSH_SupplyPelletVelPID, 0.0f);
		TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosFB);
		SupplyPellet_TD.x1 = GstSH_Paras.SupplyPellet_PosFB;
		SupplyPellet_TD.x2 = 0.0f;
    }

    FrictionWheel_Debug();

	SupplyPellet_Debug(); //拨弹电机闭环控制
	// Bullet_Blocked_Protection(); //卡弹保护
}

/**
  * @brief  发射遥控器模式控制函数
  * @note   遥控器模式下的控制策略
  * @param  无
  * @retval 无
*/
void SHCtrl_RC(void)
{
    SupplyStep = (1.0f)*360.0f/SupplyPellet_Num; //根据拨盘齿数计算步进角度(正负号由拨弹正方向决定，换车需修改)
    /***********************前置处理**************************/
    /*状态切换*/
    if(GSTSH_Data.ShooterMode != GEMSH_Mode) //发射模式切换
    {
		PID_SetDes(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
		PID_SetDes(&GstSH_SupplyPelletVelPID, 0.0f);
		TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosFB);
		SupplyPellet_TD.x1 = GstSH_Paras.SupplyPellet_PosFB;
		SupplyPellet_TD.x2 = 0.0f;
    }
    
    FrictionWheel_RCCtrl();

    SupplyPellet_RCCtrl(); //拨弹电机闭环控制
	Bullet_Blocked_Protection(); //卡弹保护
}

/**
  * @brief  发射键鼠模式控制函数
  * @note   键鼠模式下的控制策略
  * @param  无
  * @retval 无
*/
void SHCtrl_KeyMouse(void)
{
    SupplyStep = (1.0f)*360.0f/SupplyPellet_Num; //根据拨盘齿数计算步进角度(正负号由拨弹正方向决定，换车需修改)
    /***********************前置处理**************************/
    /*状态切换*/
    if(GSTSH_Data.ShooterMode != GEMSH_Mode) //发射模式切换
    {
		PID_SetDes(&GstSH_SupplyPelletPosPID, GstSH_Paras.SupplyPellet_PosFB);
		PID_SetDes(&GstSH_SupplyPelletVelPID, 0.0f);
		TD_SetInput(&SupplyPellet_TD, GstSH_Paras.SupplyPellet_PosFB);
		SupplyPellet_TD.x1 = GstSH_Paras.SupplyPellet_PosFB;
		SupplyPellet_TD.x2 = 0.0f;
    }
    
    FrictionWheel_KeyMouseCtrl();

    SupplyPellet_KeyMouseCtrl(); //拨弹电机闭环控制
	Bullet_Blocked_Protection(); //卡弹保护
}

/**
  * @brief  根据发射模式控制发射和拨弹机构
  * @note   此过程无优先级
  * @param  ModeNow：当前模式
  * @retval void
*/
void Shooter_ModeControl(SHMode_EnumTypeDef ModeNow)
{
    switch(ModeNow)
    {
        case SHMode_Safe:
            SHCtrl_Safe();            break;

        case SHMode_RC:
            SHCtrl_RC();              break;

        case SHMode_KeyMouse:
            SHCtrl_KeyMouse();        break;

        case SHMode_Debug:
            SHCtrl_Debug();           break;

        default:
            SHCtrl_Safe();            break;
    }
}




