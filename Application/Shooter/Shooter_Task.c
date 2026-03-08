/**
  ******************************************************************************
  * @file    Shooter_Task.c
  * @author  26赛季，平衡步兵电控，马帅
  * @date    2025.9.19
  * @brief   发射控制任务
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include <FreeRTOS.h>
#include <task.h>
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Shooter.h"
#include "Shooter_Task.h"
#include "Shooter_Stratgy.h"


static TickType_t S_lastWakeTimeShooterTask = 0; //上次唤醒的时间，DelayUntil绝对延时函数的参数
/**
  * @brief  Shooter发射控制任务
  * @note   发射的控制任务函数
  *         while(1)是因为FreeRTOS的任务函数必须是一个死循环
  * @param  无
  * @retval 无
  */
void ShooterTask(void* arg)
{
    Shooter_Task_Init();        //发射任务相关初始化

    while(1)
    {
        // ShooterControl();       //发射的总控制   //待完成

        GST_SystemMonitor.ShooterTask_cnt++;		//发射帧率统计：cnt计数
        vTaskDelayUntil(&S_lastWakeTimeShooterTask, GSH_TaskPeriod); //绝对延时
    }
}

/**
  * @brief  发射控制函数
  * @param  无
  * @retval 无
*/
void ShooterControl(void)
{
    /*数据反馈更新*/
    Shooter_AllFBDataUpdate();       //更新发射相关数据的反馈值
    
    /*发射控制*/
    GEMSH_Mode = Shooter_ModeChoose(); // 发射模式选择
    Shooter_ModeControl(GEMSH_Mode);   // 根据不同模式控制发射
}
