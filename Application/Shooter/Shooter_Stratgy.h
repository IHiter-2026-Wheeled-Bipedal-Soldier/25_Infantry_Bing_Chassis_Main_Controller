/**
  ******************************************************************************
  * @file    Shooter_Stratgy.h
  * @author  26赛季，平衡步兵电控，马帅
  * @date    2026.1.10
  * @brief   发射模式选择及控制主逻辑相关函数头文件
  ******************************************************************************
*/

#ifndef __SHOOTER_STRATGY_H
#define __SHOOTER_STRATGY_H

#include "GlobalDeclare_Shooter.h"
#include "Shooter_APIFunction.h"

SHMode_EnumTypeDef Shooter_ModeChoose(void);
void Shooter_ModeControl(SHMode_EnumTypeDef ModeNow);

#endif
