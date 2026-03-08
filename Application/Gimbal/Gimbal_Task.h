/**
 ******************************************************************************
 * @file    Gimbal_Task.h
 * @author  26赛季，平衡步兵电控，苏文远
 * @date    2026.3.8
 * @brief   GimbalTask任务头文件
 ******************************************************************************
 */
#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include <stdbool.h>
#include "Gimbal_Stratgy.h"
#include "Gimbal_APIFunction.h"
/****************************变量定义****************************/

/****************************函数声明****************************/
void GimbalTask(void* arg);
void GimbalControl(void);

#endif
