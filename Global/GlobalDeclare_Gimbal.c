/**
  ******************************************************************************
  * @file    GlobalDeclare_Gimbal.c
  * @author  26赛季，平衡步兵电控，苏文远
  * @date    2026.2.27
  * @brief   用来存放与云台有关的全局变量
  ******************************************************************************
*/

/*************************************************头文件引用*************************************************/
#include "stm32f4xx.h"
#include "GlobalDeclare_Gimbal.h"
#include "Algorithm.h"
#include "FreeRTOS.h"

/****************************************宏定义、常量定义（不需要修改）****************************************/
/*FreeRTOS任务相关*/
const TickType_t GGM_TaskPeriod = 1;                                               //Gimbal的任务周期，单位为FreeRTOS的系统节拍。默认是ms（取决于configTICK_RATE_HZ）
const float      GGM_TaskTime   = (float)GGM_TaskPeriod/(float)configTICK_RATE_HZ; //任务运行周期，单位为秒

/*一些默认定义*/
#define SampleTime_Default  GGM_TaskTime    //默认采样时间，单位秒
#define PitchMaxCurrent     29000.0f        //Pitch电机最大电流
#define YawMaxCurrent       29000.0f        //Yaw电机最大电流

/****************************************宏定义、常量定义（可能需要修改）****************************************/
//#region /****TD相关系数****************************************/
#define TD_SampleTime    SampleTime_Default      //TD采样时间，单位秒
// Patience调的结果，后续可以继续优化
#define TD_Pitch_r      2000.0f               //Pitch轴TD：速度因子，越大跟踪越快，但微分信号的噪声也会越大
#define TD_Pitch_h0     16*TD_SampleTime      //Pitch轴TD：滤波因子，越大滤波效果越好，通常取采样时间的整数倍
#define TD_Yaw_r        5000.0f               //Yaw轴TD：速度因子，越大跟踪越快，但微分信号的噪声也会越大
#define TD_Yaw_h0       15*TD_SampleTime      //Yaw轴TD：滤波因子，越大滤波效果越好，通常取采样时间的整数倍
//#endregion

//#region /****PID相关参数***************************************/
/*云台Pitch轴*/
#define PID_PitchPos_Kp          0.0f             //Pitch位置环PID：比例系数Kp
#define PID_PitchPos_Ki          0.0f             //Pitch位置环PID：积分系数Ki
#define PID_PitchPos_Kd          0.0f             //Pitch位置环PID：微分系数Kd
#define PID_PitchVel_Kp          0.0f             //Pitch速度环PID：比例系数Kp
#define PID_PitchVel_Ki          0.0f             //Pitch速度环PID：积分系数Ki
#define PID_PitchVel_Kd          0.0f             //Pitch速度环PID：微分系数Kd
#define PID_PitchPos_UMax        29000.0f         //Pitch位置环PID：总输出最大值
#define PID_PitchPos_UpMax       29000.0f         //Pitch位置环PID：Kp项输出最大值
#define PID_PitchPos_UiMax       29000.0f         //Pitch位置环PID：Ki项输出最大值
#define PID_PitchPos_UdMax       29000.0f         //Pitch位置环PID：Kd项输出最大值
#define PID_PitchPos_AddMax      0.8f             //Pitch位置环PID：误差单次累加最大值
#define PID_PitchVel_UMax        29000.0f         //Pitch速度环PID：总输出最大值
#define PID_PitchVel_UpMax       29000.0f         //Pitch速度环PID：Kp项输出最大值
#define PID_PitchVel_UiMax       29000.0f         //Pitch速度环PID：Ki项输出最大值
#define PID_PitchVel_UdMax       29000.0f         //Pitch速度环PID：Kd项输出最大值
#define PID_PitchVel_AddMax      2.0f             //Pitch速度环PID：误差单次累加最大值

/*云台Yaw轴*/
#define PID_YawPos_Kp            0.0f             //Yaw位置环PID：比例系数Kp
#define PID_YawPos_Ki            0.0f             //Yaw位置环PID：积分系数Ki
#define PID_YawPos_Kd            0.0f             //Yaw位置环PID：微分系数Kd
#define PID_YawVel_Kp            0.0f             //Yaw速度环PID：比例系数Kp
#define PID_YawVel_Ki            0.0f             //Yaw速度环PID：积分系数Ki
#define PID_YawVel_Kd            0.0f             //Yaw速度环PID：微分系数Kd
#define PID_YawPos_UMax          29000.0f         //Yaw位置环PID：总输出最大值
#define PID_YawPos_UpMax         29000.0f         //Yaw位置环PID：Kp项输出最大值
#define PID_YawPos_UiMax         29000.0f         //Yaw位置环PID：Ki项输出最大值
#define PID_YawPos_UdMax         29000.0f         //Yaw位置环PID：Kd项输出最大值
#define PID_YawPos_AddMax        2.0f             //Yaw位置环PID：误差单次累加最大值
#define PID_YawVel_UMax          29000.0f         //Yaw速度环PID：总输出最大值
#define PID_YawVel_UpMax         29000.0f         //Yaw速度环PID：Kp项输出最大值
#define PID_YawVel_UiMax         29000.0f         //Yaw速度环PID：Ki项输出最大值
#define PID_YawVel_UdMax         29000.0f         //Yaw速度环PID：Kd项输出最大值
#define PID_YawVel_AddMax        3.0f             //Yaw速度环PID：误差单次累加最大值
//#endregion

//#region /****云台角度限制相关****************************************/
float Gravity_FeedForward = 6000.0f;    //云台Pitch重力前馈
//#endregion

//#region /****云台角度限制相关****************************************/
float PitchMax = +25.0f;    //云台Pitch最大值（IMU角度），单位度
float PitchMin = -27.0f;    //云台Pitch最小值（IMU角度），单位度
//#endregion

//#region /****灵敏度相关***********************************************/
float RC_SST_Yaw         =  0.00015f; //Yaw轴灵敏度（遥控器模式）0.0003f
float RC_SST_Pitch       =  0.0001f; //Pitch轴灵敏度（遥控器模式）
float KeyMouse_SST_Yaw   =  80.0f;   //Yaw轴灵敏度（键鼠模式）
float KeyMouse_SST_Pitch =  25.0f;   //Pitch轴灵敏度（键鼠模式）
//#endregion

//#region /****Pitch TD & PID 参数（辅瞄/遥控器）**********************/
float TD_pitch_Norm     = 7000.0f;  //Pitch TD：正常模式下的速度因子
float pitch_td_coe      = 0.0f;     //Pitch TD：正常模式下的速度前馈系数
float pitch_td_coe_Norm = 0.55f;    //Pitch TD：正常模式下的滤波系数
//#endregion

//#region /****Yaw TD & PID 参数（辅瞄/遥控器）************************/
float TD_yaw_Norm     = 1800.0f;  //Yaw TD：正常模式下的速度因子
float yaw_td_coe      = 0.0f;     //Yaw TD：正常模式下的速度前馈系数
float yaw_td_coe_Norm = 0.55f;    //Yaw TD：正常模式下的滤波系数
//#endregion

/********************************************变量定义（不需要修改）********************************************/
//#region /****通讯相关************************************************/
IMU1Data_StructTypeDef GstGM_IMU1;                        //云台云控IMU1的通讯数据结构体，包括接收和发送
uint8_t                GFGM_IMU1Restart = IMU1RestartNO;  //云台云控IMU1重启标志位，默认不重启
//#endregion

//#region /****编码器相关***********************************************/
volatile ST_ENCODER g_stPitchEncoder   = {0, 0, 0, 0, 36, 8192, 0}; //Pitch电机编码器结构体
volatile ST_ENCODER g_stYawEncoder     = {0, 0, 0, 0, 36, 8192, 0}; //Yaw电机编码器结构体
volatile ST_ENCODER g_stCMREncoder     = {0, 0, 0, 0, 36, 8192, 0}; //右摩擦轮电机编码器结构体
volatile ST_ENCODER g_stCMLEncoder     = {0, 0, 0, 0, 36, 8192, 0}; //左摩擦轮电机编码器结构体
volatile ST_ENCODER g_stShooterEncoder = {0, 0, 0, 0, 36, 8192, 0}; //拨弹电机编码器结构体

// float PitchEncoderZero_Norm = 5387.0f; //Pitch编码器零点（正常模式），单位：编码器值
//#endregion

//#region /****TD算法相关*****************************/
/*云台TD跟踪结构体，初始化顺序：r, h0, SampleTime（速度因子、滤波因子、采样时间）*/
TD_StructTypeDef PitchTD = {TD_Pitch_r, TD_Pitch_h0, TD_SampleTime}; //Pitch TD跟踪微分结构体
TD_StructTypeDef YawTD   = {TD_Yaw_r,   TD_Yaw_h0,   TD_SampleTime}; //Yaw   TD跟踪微分结构体
//#endregion

//#region /****PID控制相关*****************************/
/*云台两轴PID控制结构体，初始化顺序：Kp, Ki, Kd, UMax, UpMax, UiMax, UdMax, AddMax：比例系数、积分系数、微分系数、总输出最大值、Kp项输出最大值、Ki项输出最大值、Kd项输出最大值、SumE单次累加的最大值*/
PID_StructTypeDef GstGM_PitchPosPID = PID_INIT(PID_PitchPos_Kp, PID_PitchPos_Ki, PID_PitchPos_Kd, PID_PitchPos_UMax, PID_PitchPos_UpMax, PID_PitchPos_UiMax, PID_PitchPos_UdMax, PID_PitchPos_AddMax); //云台Pitch位置环PID结构体
PID_StructTypeDef GstGM_PitchVelPID = PID_INIT(PID_PitchVel_Kp, PID_PitchVel_Ki, PID_PitchVel_Kd, PID_PitchVel_UMax, PID_PitchVel_UpMax, PID_PitchVel_UiMax, PID_PitchVel_UdMax, PID_PitchVel_AddMax); //云台Pitch速度环PID结构体
PID_StructTypeDef GstGM_YawPosPID   = PID_INIT(PID_YawPos_Kp, PID_YawPos_Ki, PID_YawPos_Kd, PID_YawPos_UMax, PID_YawPos_UpMax, PID_YawPos_UiMax, PID_YawPos_UdMax, PID_YawPos_AddMax); //云台Yaw位置环PID结构体
PID_StructTypeDef GstGM_YawVelPID   = PID_INIT(PID_YawVel_Kp, PID_YawVel_Ki, PID_YawVel_Kd, PID_YawVel_UMax, PID_YawVel_UpMax, PID_YawVel_UiMax, PID_YawVel_UdMax, PID_YawVel_AddMax); //云台Yaw速度环PID结构体
//#endregion

float GGB_RollAngle = 0.0f; //云台Roll轴角度（待优化：只在发给视觉时用到）

//#region /****云台数据结构体*****************************************/
GMData_StructTypeDef GSTGM_Data;     //云台正式数据结构体，存放和云台相关的几乎所有数据
//#endregion

//#region /****云台操作模式相关*****************************************/
GMMode_EnumTypeDef GEMGM_Mode = GMMode_Disabled;  //云台操作模式，默认为失能（安全）模式
//#endregion

//#region /****调试模式相关*****************************************/
GimbalShooter_DebugFlags_StructTypeDef GstGMSH_Debug_Flags = {0};     //云台发射调试相关标志位结构体
GimbalShooter_DebugParas_StructTypeDef GstGMSH_Debug_Paras = {0.0f};  //云台发射调试相关参数结构体
//#endregion

//#region /****视觉状态相关*********************************************/
uint8_t Vision_State_Enable_Now = Vision_Disable;               //当前视觉模式使能状态
uint8_t Vision_State_Enable_Pre = Vision_Disable;               //上次视觉模式使能状态
uint8_t Vision_Mode_Now         = VisionMode_Assist_Aim_Normal; //当前视觉辅瞄模式（默认辅瞄装甲板模式）
uint8_t Vision_Mode_Pre         = VisionMode_Assist_Aim_Normal; //上次视觉辅瞄模式（默认辅瞄装甲板模式）
bool    Vision_Mode_Changing    = FALSE;                        //视觉模式是否正在切换标志位
//#endregion

//#region /****电机结构体相关*********************************************/
Pitch_Motor_StructTypeDef Pitch_Motor_Paras = {0.0f};   //Pitch电机各种参数结构体（MIT协议）
Yaw_Motor_StructTypeDef   Yaw_Motor_Paras   = {0.0f};   //Yaw电机各种参数结构体
//#endregion

/**
  * @brief  云台任务初始化函数
  * @note   在Gimbal控制任务循环开始之前调用（GimbalTask的while(1)之前调用）
  *         对云台相关的所有参数进行初始化，并使能达妙电机
  * @param  无
  * @retval 无
*/
void Gimbal_Task_Init(void)
{
    /**********************************电机相关**************************************/

    /*电机PID相关*/
    PID_StructInit(&GstGM_PitchPosPID, 0.0f, 0.0f, 0.0f, PID_PitchPos_UMax, PID_PitchPos_UpMax, PID_PitchPos_UiMax, PID_PitchPos_UdMax, PID_PitchPos_AddMax);
    PID_StructInit(&GstGM_PitchVelPID, 0.0f, 0.0f, 0.0f, PID_PitchVel_UMax, PID_PitchVel_UpMax, PID_PitchVel_UiMax, PID_PitchVel_UdMax, PID_PitchVel_AddMax);
    PID_StructInit(&GstGM_YawPosPID,   0.0f, 0.0f, 0.0f, PID_YawPos_UMax,   PID_YawPos_UpMax,   PID_YawPos_UiMax,   PID_YawPos_UdMax,   PID_YawPos_AddMax);
    PID_StructInit(&GstGM_YawVelPID,   0.0f, 0.0f, 0.0f, PID_YawVel_UMax,   PID_YawVel_UpMax,   PID_YawVel_UiMax,   PID_YawVel_UdMax,   PID_YawVel_AddMax);

    /*电机TD相关*/
    TD_StructInit(&PitchTD, TD_Pitch_r, TD_Pitch_h0, TD_SampleTime);
    TD_StructInit(&YawTD,   TD_Yaw_r,   TD_Yaw_h0,   TD_SampleTime);
}
