/**
  ******************************************************************************
  * @file    GlobalDeclare_Shooter.h
  * @author  26赛季，平衡步兵电控，马帅
  * @date    2026.1.21
  * @brief   用来声明、引出与发射有关的全局变量
  ******************************************************************************
*/
#ifndef __GLOBALDECLARE_SHOOTER_H
#define __GLOBALDECLARE_SHOOTER_H

#include "stdint.h"
#include "Algorithm.h"
#include "FreeRTOS.h"


//#region /**** 变量引出extern声明****************************************************************/
/*FreeRTOS任务相关*/
extern const TickType_t GSH_TaskPeriod;
extern const float GSH_TaskTime;

//发射相关变量定义
extern bool ShooterSafetyLocked;	   //发射安全锁，防止误打弹
extern bool FrictionWheel_Ready_flag ; //摩擦轮准备完毕标志位
extern uint8_t FWSpeed_Des;            //目标弹速（比赛规则要求，测速模块进行测量）
extern float FWSpeed_Datum;	           //摩擦轮目标转速（需要进行实测，每场比赛前标弹速）
extern float FWSpeed_Datum_Buff;	   //打符摩擦轮目标转速（打符时默认不对摩擦轮转速进行适应性更改）

//pid赋值宏
#define PID_INIT(Kp,Ki,Kd,UMax,UpMax,UiMax,UdMax,AddMax) \
        {Kp,Ki,Kd,UMax,UpMax,UiMax,UdMax,AddMax,0,0,0,0,0,0,0,0,0}

#define SupplyPelletMaxCurrent      15000.0f     //拨弹电机电流最大值
#define ShooterMaxCurrent           15000.0f     //摩擦轮电机电流最大值

//#region /****PID相关参数***************************************/
/*拨弹电机*/
#define PID_SupplyPos_Kp          30.0f           //SupplyMotor位置环PID：比例系数Kp
#define PID_SupplyPos_Ki          0.0f           //SupplyMotor位置环PID：积分系数Ki
#define PID_SupplyPos_Kd          0.0f           //SupplyMotor位置环PID：微分系数Kd
#define PID_SupplyVel_Kp          20.0f           //SupplyMotor速度环PID：比例系数Kp
#define PID_SupplyVel_Ki          0.0f           //SupplyMotor速度环PID：积分系数Ki
#define PID_SupplyVel_Kd          100.0f           //SupplyMotor速度环PID：微分系数Kd
#define PID_SupplyPos_UMax        10000.0f       //SupplyMotor位置环PID：总输出最大值
#define PID_SupplyPos_UpMax       8000.0f        //SupplyMotor位置环PID：Kp项输出最大值
#define PID_SupplyPos_UiMax       3000.0f        //SupplyMotor位置环PID：Ki项输出最大值
#define PID_SupplyPos_UdMax       8000.0f        //SupplyMotor位置环PID：Kd项输出最大值
#define PID_SupplyPos_AddMax      1.0f           //SupplyMotor位置环PID：误差单次累加最大值
#define PID_SupplyVel_UMax        10000.0f       //SupplyMotor速度环PID：总输出最大值
#define PID_SupplyVel_UpMax       8000.0f        //SupplyMotor速度环PID：Kp项输出最大值
#define PID_SupplyVel_UiMax       3000.0f        //SupplyMotor速度环PID：Ki项输出最大值
#define PID_SupplyVel_UdMax       8000.0f        //SupplyMotor速度环PID：Kd项输出最大值
#define PID_SupplyVel_AddMax      5.0f           //SupplyMotor速度环PID：误差单次累加最大值
//#endregion


extern float TD_SupplyPellet_Norm ;
extern float SupplyPellet_td_coe_Norm;
extern TD_StructTypeDef SupplyPellet_TD;

extern ST_SMC smcR;
extern ST_SMC smcL;

extern PID_StructTypeDef GstSH_SupplyPelletPosPID;    //拨弹电机位置环PID结构体
extern PID_StructTypeDef GstSH_SupplyPelletVelPID;  //拨弹电机速度环PID结构体
extern float ShooterPosDes;
extern float ShooterPosDes_Pre; //拨弹电机过去目标位置
extern int8_t IfFlip ;//拨弹电机正反转标志
extern float SupplyPellet_Num ;//拨盘齿数
extern float SupplyStep; //拨弹电机步进角度

//卡弹保护相关变量定义
extern bool Is_Locked_Rotor_Flag;		// 判断是否堵转的标志位
extern bool Locked_Rotor_Protect_Flag; // 堵转保护标志位
extern int Is_Locked_Rotor_Cnt;				// 判断是否堵转的计时器
extern int Locked_Rotor_Protect_Cnt;			// 堵转保护的计时器

/*发射模式主要控制结构体*/
typedef struct
{
    /*电机相关*/
    float SupplyPellet_PosDes;  //拨弹电机角度目标值，单位度（注意是减速箱输出端角度）
    float SupplyPellet_VelDes;  //拨弹电机速度目标值，单位度/s（注意是减速箱输出端角速度）
    float SupplyPellet_PosFB;   //拨弹电机角度反馈值，单位度（注意是减速箱输出端角度）
    float SupplyPellet_VelFB;   //拨弹电机速度反馈值，单位度/s（注意是减速箱输出端角速度）
    float SupplyPellet_Current; //拨弹电机CAN发送电流，注意不是实际电流

    /*遥控器操作相关*/
    //单发模式相关
	bool RC_Single_Shoot_Flag;                    //单发模式标志位
	bool RC_Single_Shoot_Now_status;              //单发模式当前判断标志位（用于边沿检测）
	bool RC_Single_Shoot_Pre_status;              //单发模式上一状态标志位（用于边沿检测）
    //连发模式相关
	bool RC_Continuous_Shoot_Flag;                //连发模式标志位
    uint32_t RC_IF_Continuous_Shoot_Cnt;          //连发模式判断计数器

    /*键鼠操作相关*/
    //单发模式相关
	bool KeyMouse_Single_Shoot_Flag;              //单发模式标志位
	bool KeyMouse_Single_Shoot_Now_status;        //单发模式当前判断标志位（用于边沿检测）
	bool KeyMouse_Single_Shoot_Pre_status;        //单发模式上一状态标志位（用于边沿检测）
    //连发模式相关
	bool KeyMouse_Continuous_Shoot_Flag;          //连发模式标志位
    uint32_t KeyMouse_IF_Continuous_Shoot_Cnt;    //连发模式判断计数器

    /*发射间隔相关*/
    bool RC_Shooting_Flag;                  //正在发射标志位
    uint32_t RC_IF_Shoot_Cnt;               //用于间隔时间发射子弹
    bool KeyMouse_Shooting_Flag;            //正在发射标志位
    uint32_t KeyMouse_IF_Shoot_Cnt;         //用于间隔时间发射子弹
    bool Debug_Shooting_Flag;            //正在发射标志位
    uint32_t Debug_IF_Shoot_Cnt;         //用于间隔时间发射子弹


    uint32_t RC_AUTO_IF_Shooting_cnt;
    bool RC_AUTO_Shooting_flag;
	
    bool FrictionWheel_ReadyOrNot_Flag;      //摩擦轮准备完毕标志位
    bool SupplyPellet_ReadyOrNot_Flag;       //拨弹电机准备完毕标志位

    /*卡弹（堵转）相关*/
    bool IS_Bullet_Blocked_Flag;		     //卡弹判断标志位
    bool Bullet_Blocked_Protection_Flag;     //卡弹保护标志位
    int Bullet_Blocked_Cnt;			         //卡弹判断计数器
    int Locked_Rotor_Protect_Cnt;	         //卡弹保护的计时器

}Shooter_StructTypeDef;

/*发射操作模式枚举*/
typedef enum
{
    SHMode_Safe = 0,      //安全模式
    SHMode_RC,            //遥控器模式
    SHMode_KeyMouse,      //键鼠模式

    SHMode_Debug,         //发射调试模式
}SHMode_EnumTypeDef;

/*发射数据结构体*/
typedef struct
{
    /*模式相关*/
    SHMode_EnumTypeDef ShooterMode;  //当前发射模式

    /*云台姿态相关*/
    // float PitchPosDes;       //Pitch角度目标值，单位度，抬头/低头为正
    // float PitchVelDes;       //Pitch角速度目标值，单位度/s，抬头/低头为正
    // float PitchPosFB;        //Pitch角度反馈，单位度，抬头/低头为正
    //待补充
}SHData_StructTypeDef;

extern SHData_StructTypeDef GSTSH_Data;
extern SHMode_EnumTypeDef GEMSH_Mode;


#define ShooterHeat_mes     ((int32_t)GstGM_MainCtrl.ST_Rx.RefereeData.shooter_id1_17mm_cooling_heat)    //枪口热量
#define ShooterHeat_Rate    ((float)GstGM_MainCtrl.ST_Rx.RefereeData.shooter_id1_17mm_cooling_rate)//枪口热量冷却
#define ShooterHeat_Limit   ((int32_t)GstGM_MainCtrl.ST_Rx.RefereeData.shooter_id1_17mm_cooling_limit) //枪口热量上限
#define PelletSpeed_mes     ((float)GstGM_MainCtrl.ST_Rx.RefereeData.bullet_speed) //弹速


extern Shooter_StructTypeDef GstSH_Paras;   //发射操作相关的结构体变量，包含发射状态、发射计数等

extern float FMR_speed_diff;//右摩擦轮转速差
extern float FML_speed_diff;//左摩擦轮转速差
extern float now_heat ;
extern int downflag ;
extern float supply_cnt;//打出子弹个数
extern float Pre_ShootSpeed ;    //记录上一次弹丸射速
extern uint8_t PelletNum ;            //已经发射多少颗子弹，每次重载子弹自动清零
extern int16_t Allowed_PelletNum ;  //一个周期内（100ms）实时可发射弹丸数量,受热量限制
extern int16_t Allowed_PelletNum_Friction;  //一个周期内（100ms）实时可发射弹丸数量,受摩擦轮转速限制
extern float Heat_Left;//剩余热量

void Shooter_Task_Init(void);

#endif
