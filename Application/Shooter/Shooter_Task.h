/**
 ******************************************************************************
 * @file    Shooter_Task.h
 * @author  26赛季，平衡步兵电控，马帅
 * @date    2025.9.19
 * @brief   ShooterTask任务头文件
 ******************************************************************************
 */
#ifndef __SHOOTER_TASK_H
#define __SHOOTER_TASK_H

#include <stdbool.h>
// #include "Shooter_Stratgy.h"
// #include "Shooter_APIFunction.h"
/****************************变量定义****************************/

/****************************函数声明****************************/
void ShooterTask(void* arg);
void ShooterControl(void);

#endif
