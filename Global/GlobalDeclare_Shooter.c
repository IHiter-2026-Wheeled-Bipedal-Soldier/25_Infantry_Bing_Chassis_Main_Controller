/**
  ******************************************************************************
  * @file    GlobalDeclare_Shooter.c
  * @author  26赛季，平衡步兵电控，马帅
  * @date    2026.1.21
  * @brief   用来存放与发射有关的全局变量
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include "stm32f4xx.h"
#include "GlobalDeclare_Shooter.h"
#include "Algorithm.h"
#include "FreeRTOS.h"

/****************************************宏定义、常量定义（不需要修改）****************************************/
/*FreeRTOS任务相关*/
const TickType_t GSH_TaskPeriod = 1;      //Shooter的任务周期，单位为FreeRTOS的系统节拍。默认是ms（取决于configTICK_RATE_HZ）
const float GSH_TaskTime = (float)GSH_TaskPeriod/(float)configTICK_RATE_HZ; //任务运行周期，单位为秒
/*一些默认定义*/
#define SampleTime_Default GSH_TaskTime   //默认采样时间，单位秒

/****************************************宏定义、常量定义（可能需要修改）****************************************/
//#region /****TD相关系数****************************************/
#define TD_SampleTime    SampleTime_Default      //TD采样时间，单位秒

#define TD_SupplyPellet_r      40000.0f               //Pitch轴TD：速度因子，越大跟踪越快，但微分信号的噪声也会越大
#define TD_SupplyPellet_h0     1*TD_SampleTime      //Pitch轴TD：滤波因子，越大滤波效果越好，通常取采样时间的整数倍
//#endregion

//#region /****发射数据结构体*****************************************/
SHData_StructTypeDef GSTSH_Data;     //发射正式数据结构体，存放和发射相关的主要数据
//#endregion

//发射相关变量定义
bool ShooterSafetyLocked = false;	     //发射安全锁，防止误打弹
bool FrictionWheel_Ready_flag = false; //摩擦轮准备完毕标志位
uint8_t FWSpeed_Des  = 25;             //目标弹速（比赛规则要求，测速模块进行测量）
float FWSpeed_Datum = 7000;	           //摩擦轮目标转速（需要进行实测，每场比赛前标弹速）
float FWSpeed_Datum_Buff = 6300;	     //打符摩擦轮目标转速（打符时默认不对摩擦轮转速进行适应性更改）

#define ST_SMC_INIT(fpUMax,b,eps,gain,dead,TD_r,TD_h,TD_T) \
        {0,0,0,0,fpUMax,b,eps,gain,dead,0,0,0,TD_r,TD_h,TD_T,0}	

ST_SMC smcR = ST_SMC_INIT(15000,4.9,8000,54,5,7000,0.001,0.001);  //滑模控制
ST_SMC smcL = ST_SMC_INIT(15000,4.9,8000,54,5,7000,0.001,0.001);  //滑模控制

/************************位姿、电机PID相关************************/
PID_StructTypeDef GstSH_SupplyPelletPosPID = PID_INIT(PID_SupplyPos_Kp, PID_SupplyPos_Ki, PID_SupplyPos_Kd, PID_SupplyPos_UMax, PID_SupplyPos_UpMax, PID_SupplyPos_UiMax, PID_SupplyPos_UdMax, PID_SupplyPos_AddMax);  //拨弹电机位置环PID结构体
PID_StructTypeDef GstSH_SupplyPelletVelPID = PID_INIT(PID_SupplyVel_Kp, PID_SupplyVel_Ki, PID_SupplyVel_Kd, PID_SupplyVel_UMax, PID_SupplyVel_UpMax, PID_SupplyVel_UiMax, PID_SupplyVel_UdMax, PID_SupplyVel_AddMax);  //拨弹电机速度环PID结构体
float ShooterPosDes = 0; //拨弹电机目标位置
float ShooterPosDes_Pre = 0; //拨弹电机过去目标位置

/*拨弹电机TD相关*/
float TD_SupplyPellet_Norm = 1800;
float SupplyPellet_td_coe_Norm = 0.0f;
TD_StructTypeDef SupplyPellet_TD = {TD_SupplyPellet_r, TD_SupplyPellet_h0, TD_SampleTime};


int8_t IfFlip = 1;//拨弹电机正反转标志
float SupplyPellet_Num = 8;//拨盘齿数
float SupplyStep = 0; //拨弹电机步进角度

//卡弹保护相关变量定义
bool Is_Locked_Rotor_Flag = false;		  // 判断是否堵转的标志位
bool Locked_Rotor_Protect_Flag = false; // 堵转保护标志位
int Is_Locked_Rotor_Cnt = 0;				    // 判断是否堵转的计时器
int Locked_Rotor_Protect_Cnt = 0;			  // 堵转保护的计时器

SHMode_EnumTypeDef GEMSH_Mode = SHMode_RC;   //发射模式，默认为安全模式

Shooter_StructTypeDef GstSH_Paras = {0};  //发射操作相关的结构体变量，包含发射状态、发射计数等

float FMR_speed_diff;//右摩擦轮转速差
float FML_speed_diff;//左摩擦轮转速差
float now_heat = 0;
int downflag = 1;
float supply_cnt=0;//打出子弹个数
float Pre_ShootSpeed  = 0.0f;    //记录上一次弹丸射速
float Heat_Left= 0.0f;//剩余热量
uint8_t PelletNum = 0;           //已经发射多少颗子弹，每次重载子弹自动清零
int16_t Allowed_PelletNum  = 0;  //一个周期内（100ms）实时可发射弹丸数量,受热量限制
int16_t Allowed_PelletNum_Friction=0;  //一个周期内（100ms）实时可发射弹丸数量,受摩擦轮转速限制

/**
  * @brief  发射任务初始化函数
  * @note   在Shooter控制任务循环开始之前调用（ShooterTask的while(1)之前调用）
  *         对发射相关的所有参数进行初始化
  * @param  无
  * @retval 无
*/
void Shooter_Task_Init(void)
{
    /**********************************电机相关**************************************/
    /*电机PID相关*/
    PID_StructInit(&GstSH_SupplyPelletPosPID, 0.0f, 0.0f, 0.0f, PID_SupplyPos_UMax, PID_SupplyPos_UpMax, PID_SupplyPos_UiMax, PID_SupplyPos_UdMax, PID_SupplyPos_AddMax);
    PID_StructInit(&GstSH_SupplyPelletVelPID, 0.0f, 0.0f, 0.0f, PID_SupplyVel_UMax, PID_SupplyVel_UpMax, PID_SupplyVel_UiMax, PID_SupplyVel_UdMax, PID_SupplyVel_AddMax);

    /*电机TD相关*/
    TD_StructInit(&SupplyPellet_TD, TD_SupplyPellet_r, TD_SupplyPellet_h0, TD_SampleTime);
}
