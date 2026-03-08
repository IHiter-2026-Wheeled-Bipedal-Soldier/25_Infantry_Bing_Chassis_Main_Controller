/**
  ******************************************************************************
  * @file    Shooter_APIFunction.h
  * @author  26赛季，平衡步兵电控，马帅
  * @date    2026.1.21
  * @brief   发射功能函数头文件
  ******************************************************************************
*/
#ifndef __SHOOTER_APIFUNCTION_H
#define __SHOOTER_APIFUNCTION_H
#include "GlobalDeclare_Shooter.h"



void Shooter_AllFBDataUpdate(void);

/*摩擦轮电机控制相关函数*/
void FrictionWheel_Safe(void);
void FrictionWheel_Debug(void);
void FrictionWheel_RCCtrl(void);
void FrictionWheel_KeyMouseCtrl(void);

/*拨弹电机控制相关函数*/
void SupplyPellet_Safe(void);
void SupplyPellet_Debug(void);
void SupplyPellet_RCCtrl(void);
void SupplyPellet_KeyMouseCtrl(void);

/*摩擦轮/拨弹辅助函数*/
bool __IS_FrictionWheel_Ready(void);
void Bullet_Blocked_Protection(void);
void Heat_Calculate(void);
bool Is_Heat_Safe(void);
bool __IS_RC_Single_Shoot(void);
bool __IS_RC_Continuous_Shoot(void);
bool RC_AUTO_Control_Continuous_Shoot(void);
bool __IS_KeyMouse_Single_Shoot(void);
bool __IS_KeyMouse_Continuous_Shoot(void);

#endif
