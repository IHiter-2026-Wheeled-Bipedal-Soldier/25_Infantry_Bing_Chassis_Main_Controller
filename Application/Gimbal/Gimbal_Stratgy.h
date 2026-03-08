/**
  ******************************************************************************
  * @file    Gimbal_Stratgy.h
  * @author  26赛季，平衡步兵电控，马帅
  * @date    2026.1.10
  * @brief   云台模式选择及控制主逻辑相关函数头文件
  ******************************************************************************
*/

#ifndef __GIMBAL_STRATGY_H
#define __GIMBAL_STRATGY_H

#include "GlobalDeclare_Gimbal.h"
#include "Gimbal_APIFunction.h"

/*云台模式切换函数*/
GMMode_EnumTypeDef GimbalModeChoose(void); // 云台模式切换（状态机）
void GimbalModeControl(GMMode_EnumTypeDef ModeNow);

/*云台各模式控制函数*/
void GM_RCCtrl_Disabled(void);         // 失能模式
void GM_RCCtrl_Free(void);             // 自由模式（遥控器）
void GM_RCCtrl_Follow(void);           // 跟随模式（遥控器）
void GM_Ctrl_KeyMouse(void);           // 键鼠模式
void GM_Ctrl_AutoAim(void);            // 视觉辅瞄模式
void GM_Ctrl_BuffSmall(void);          // 小符模式
void GM_Ctrl_BuffBig(void);            // 大符模式
void GM_Ctrl_BuffInterfereSmall(void); // 反小符模式
void GM_Ctrl_BuffInterfereBig(void);   // 反大符模式

/*视觉模式预处理函数*/
void Vision_Process(void);

#endif
