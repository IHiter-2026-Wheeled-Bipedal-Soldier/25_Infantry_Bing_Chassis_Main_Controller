/**
  ******************************************************************************
  * @file    Gimbal_Task.c
  * @author  26赛季，平衡步兵电控，苏文远
  * @date    2026.3.8
  * @brief   云台控制任务
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include <FreeRTOS.h>
#include <task.h>
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Gimbal.h"
#include "Gimbal_Task.h"
#include "General_AuxiliaryFunc.h"

static TickType_t S_lastWakeTimeGimbalTask = 0; //上次唤醒的时间，DelayUntil绝对延时函数的参数

/**
  * @brief  Gimbal云台控制任务
  * @note   云台的控制任务函数
  *         while(1)是因为FreeRTOS的任务函数必须是一个死循环
  * @param  无
  * @retval 无
  */
void GimbalTask(void* arg)
{
    Gimbal_Task_Init();        //云台任务相关初始化

    while(1)
    {
        GimbalControl();       //云台的总控制   //待完成

        GST_SystemMonitor.GimbalTask_cnt++;		//云台帧率统计：cnt计数
        vTaskDelayUntil(&S_lastWakeTimeGimbalTask, GGM_TaskPeriod); //绝对延时
    }
}


//待修改
/*测试时的分割线---------------------------------------------------------------------------------------------*/
//待修改

/**
  * @brief  云台控制函数
  * @param  无
  * @retval 无
*/
void GimbalControl(void)
{
    /*数据反馈更新*/
    Gimbal_AllFBDataUpdate();       //更新云台相关数据的反馈值

    //static FunctionalState PitchDropCmd = DISABLE; // 云台失能标志位(未开遥控器以及未调试时，云台失能，且在关遥控器时保护云台不撞到机械限位)ENABLE时在慢慢将云台放下
    //static uint16_t PitchDrop_cnt = 0;				   // 云台失能计数变量对放下云台进行计时，在两秒之内将云台放下

    /*视觉处理*/
    Vision_Process(); //视觉模式预处理函数，视觉相关状态更新、视觉切换处理等

    /*云台控制*/
    GEMGM_Mode = GimbalModeChoose(); // 云台控制模式选择
    GimbalModeControl(GEMGM_Mode);   // 根据不同模式控制云台

}
