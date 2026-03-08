/**
  ******************************************************************************
  * @file    GlobalDeclare_Gimbal.h
  * @author  26赛季，平衡步兵电控，苏文远
  * @date    2026.2.27
  * @brief   用来声明、引出与云台有关的全局变量
  ******************************************************************************
*/
#ifndef __GLOBALDECLARE_GIMBAL_H
#define __GLOBALDECLARE_GIMBAL_H

#include "stdint.h"
#include "Algorithm.h"
#include "FreeRTOS.h"

//#region /**** 枚举声明************************************************************************/
/*云台模式相关枚举*/
typedef enum
{
    GMMode_Disabled = 0,          // 失能模式（安全）
    GMMode_RC_Free,               // 遥控器控制(Free模式)
    GMMode_RC_Follow,             // 遥控器控制(Follow模式)
    GMMode_KeyMouse,              // 键鼠控制
    GMMode_AutoAim,               // 视觉辅瞄（正常）
    GMMode_Buff_Small,            // 小符
    GMMode_Buff_Big,              // 大符
    GMMode_Buff_Interfere_Small,  // 反小符
    GMMode_Buff_Interfere_Big,    // 反大符

    GMMode_Debug,                 //云台调试模式
    //待补充
}GMMode_EnumTypeDef;

/*************************************结构体声明**************************************/
/*IMU1云台云控数据处理结构体，包括发送和接收(注意4字节对齐)(32位单片机默认)*/
//待优化：云控现在不需要Reload了，可以后面看看删掉（注意云控那边也要修改）
typedef struct
{
    struct
    {
        uint8_t head[2];        //帧头
        float PitchAngle;       //Pitch角度，单位度
		float PitchAngleBuff;   
        float PitchAngleVel;    //Pitch角速度，单位度/s
        float YawAngle;         //Yaw角度，单位度
        float YawSpeed;         //Yaw角速度，单位度/s
		float RollAngle;        //Roll角度，单位
		float RollAngleBuff;    //待优化：这个变量没有用到
        float Distance_Z;       
        uint8_t tail[2];             
    } ST_Rx;

    struct
    {
        uint8_t head[2];
        uint8_t ReloadStatus;   //待优化：这是之前赛季的补弹标志位，已经用不到了
        uint8_t ReStart;        
        uint8_t tail[2];
    } ST_Tx;                         
}IMU1Data_StructTypeDef;

/*双主控通讯结构体，包括发送和接收*/
 typedef struct
{
    #pragma pack(1)     //作用：取消字节对齐
    struct
    {
        uint8_t head[2];        //帧头                               2
        /*辅助结构体1：云台YAW相关数据*/
    
        struct YawData_RX_StructTypeDef
        {
            float Yaw_encoder_Angle;//Yaw轴编码器角度                 4

        }YawData;

        /*辅助结构体2：发射拨弹相关数据*/
    
        struct SupplyPelletData_RX_StructTypeDef
        {
            float SupplyPellet_PosFB; //拨弹电机角度反馈，单位度（注意是减速箱输出端角度）        4
            float SupplyPellet_VelFB; //拨弹电机速度反馈，单位度/s（注意是减速箱输出端角速度）    4

        }SupplyPelletData;
        
        /*辅助结构体3：遥控器数据结构体*/
        struct
        {
            uint16_t JoyStickR_X; //右摇杆X轴，通道0                    2
            uint16_t JoyStickR_Y; //右摇杆Y轴，通道1                    2
            uint16_t JoyStickL_X; //左摇杆X轴，通道2                    2
            uint16_t JoyStickL_Y; //左摇杆Y轴，通道3                    2
            uint16_t Roller;      //拨轮，通道4                         2
            uint8_t  Level_L;     //左拨杆                             1
            uint8_t  Level_R;     //右拨杆                             1
        }ST_RC;

        /*辅助结构体4：键鼠数据结构体*/
        struct
        {
            int16_t X; //鼠标X方向速度，左负右正                             2
            int16_t Y; //鼠标Y方向速度，上负下正                             2
            int16_t Z;                                                  // 2
            uint8_t Left;   //左键是否按下                                  1
            uint8_t Right;  //右键是否按下                                  1
            uint16_t usKeyboard;//键盘数据//TODO:这个数据在云台主控中直接放到结构体里，在底盘主控中需要单独处理             2
        }ST_Mouse;

        /*辅助结构体5：底盘发给云台标志位结构体*/
        struct Chassiss_To_Gimbal_Flag_StructTypeDef
        {
            int8_t supply_stop_flag;                                //1
            int8_t flag1;                                           //1
            int8_t flag2;                                           //1
            int8_t flag3;                                           //1
            int8_t flag4;                                           //1
            CHMode_EnumTypeDef CHMode;                              //底盘当前模式 1
        }ST_Chassis_To_Gimbal_Flag;

        /*辅助结构体5：裁判系统相关数据结构体*/
        struct RefereeData_StructTypeDef
        {
            uint8_t robot_id;//本机器人 ID                          1
            uint8_t robot_level;//机器人等级                        1
            uint8_t bullet_freq;//弹丸频率（单位：Hz）                1
            uint16_t shooter_id1_17mm_cooling_rate;//机器人射击热量每秒冷却值                 2
            uint16_t shooter_id1_17mm_cooling_limit;//机器人射击热量上限                     2
            uint16_t shooter_id1_17mm_cooling_heat;//第 1 个 17mm 发射机构的射击热量          2
            uint16_t bullet_remaining_num_17mm;//机器人自身拥有的 17mm 弹丸允许发弹量          2
            float bullet_speed;//弹丸初速度（单位：m/s）                                      4   
        }ST_RefereeData;

        uint8_t tail[2];             //2
    } ST_Rx; //共57字节

    struct
    {
        uint8_t head[2]; //帧头                             2

        /*辅助结构体1：云台相关数据*/
        struct
        {
            float Yaw_Current ;     //Yaw轴电机计算后得出的电流值    4
            float Yaw_IMU_Angle;    //Yaw轴IMU角度               4
            float Pitch_IMU_Angle;  //Pitch轴编码器角度         4
        }GimbalData;

        /*辅助结构体2：拨弹相关结构体*/
        struct
        {
            int16_t SupplyPellet_Current; //拨弹电机计算后得出的电流值  2
            int8_t Final_ShootOrNot_Flag;   //电控打弹标志位，为1打弹，为0不打弹（和视觉打弹标志位区分）     1
            int16_t SupplyPellet_Frequency; //弹频，每秒打弹数量                                     2
        }SupplyPelletData;
        
        uint8_t tail[2]; //帧尾                             2
    } ST_Tx; //共21字节
    #pragma pack()
}ChassisGimbal_Data_StructTypeDef;   //底盘云台通讯协议结构体

/*云台数据结构体*/
typedef struct
{
    /*模式相关*/
    GMMode_EnumTypeDef GimbalMode;   //当前云台模式

    /*云台姿态相关*/
    float PitchPosDes;       //Pitch角度目标值，单位度，抬头/低头为正
    float PitchVelDes;       //Pitch角速度目标值，单位度/s，抬头/低头为正
    float PitchPosFB;        //Pitch角度反馈，单位度，抬头/低头为正
    float PitchVelFB;        //Pitch角速度反馈，单位度/s，抬头/低头为正
    float YawPosDes;         //Yaw角度目标值，单位度，从上往下看顺时针/逆时针为正
    float YawVelDes;         //Yaw角速度目标值，单位度/s，从上往下看顺时针/逆时针为正
    float YawPosFB;          //Yaw角度反馈，单位度，从上往下看顺时针/逆时针为正
    float YawVelFB;          //Yaw角速度反馈，单位度/s，从上往下看顺时针/逆时针为正
    float RollPosDes;        //Roll角度目标值，单位度
    float RollVelDes;        //Roll角速度目标值，单位度/s
    float RollPosFB;         //Roll角度反馈，单位度
    float RollVelFB;         //Roll角速度反馈，单位度/s
    //待补充
}GMData_StructTypeDef;

/*大疆Yaw电机控制参数结构体*/
typedef struct
{
    /*电机目标*/
    float PosDes;         //Yaw角度目标值，单位度
    float VelDes;         //Yaw角速度目标值，单位度/s

    /*电机反馈*/
    float PosFB;          //Yaw角度反馈，单位度
    float VelFB;          //Yaw角速度反馈，单位度/s

    float Yaw_Current;    //Yaw电流输出（直接发给电机的）

}Yaw_Motor_StructTypeDef;

/*达妙Pitch电机控制参数结构体（MIT协议）*/
typedef struct
{
    /*电机目标*/
    float PosDes;         //Pitch角度目标值，单位度
    float VelDes;         //Pitch角速度目标值，单位度/s

    /*电机反馈*/
    float PosFB;          //Pitch角度反馈，单位度
    float VelFB;          //Pitch角速度反馈，单位度/s

    /*PID参数*/
    float Kp;             //比例系数
    float Kd;             //微分系数
    float Ki_Torque;      //积分系数（用于位置积分前馈力矩）

    /*前馈力矩*/
    float Torque;         //前馈力矩，单位N

}Pitch_Motor_MIT_StructTypeDef;

/*云台发射相关调试标志位结构体*/
typedef struct
{
    /*Gimbal测试相关*/
    bool Gimbal_Test_Flag;        //云台测试标志位，True为测试，False为不测试
    bool Pitch_Test_Flag;         //Pitch电机测试标志位，True为测试，False为不测试
    bool Yaw_Test_Flag;           //Yaw电机测试标志位，True为测试，False为不测试

    /*Shooter测试相关*/
    bool FrictionWheel_Test_Flag; //摩擦轮电机测试标志位，True为测试，False为不测试
    bool SupplyPellet_Test_Flag;  //拨弹电机测试标志位，True为测试，False为不测试

}GimbalShooter_DebugFlags_StructTypeDef;

/*云台发射相关调试参数结构体*/
typedef struct
{
    /*Gimbal测试相关*/
    float PitchPosKp;                   //Pitch位置环PID：比例系数Kp
    float PitchPosKi;                   //Pitch位置环PID：积分系数Ki
    float PitchPosKd;                   //Pitch位置环PID：微分系数Kd
    float PitchVelKp;                   //Pitch速度环PID：比例系数Kp
    float PitchVelKi;                   //Pitch速度环PID：积分系数Ki
    float PitchVelKd;                   //Pitch速度环PID：微分系数Kd

    float YawPosKp;                     //Yaw位置环PID：比例系数Kp
    float YawPosKi;                     //Yaw位置环PID：积分系数Ki
    float YawPosKd;                     //Yaw位置环PID：微分系数Kd
    float YawVelKp;                     //Yaw速度环PID：比例系数Kp
    float YawVelKi;                     //Yaw速度环PID：积分系数Ki
    float YawVelKd;                     //Yaw速度环PID：微分系数Kd

    /*Shooter测试相关*/
    float ShooterDatum;

    float SupplyPelletPosKp;            //拨弹位置环PID：比例系数Kp
    float SupplyPelletPosKi;            //拨弹位置环PID：积分系数Ki
    float SupplyPelletPosKd;            //拨弹位置环PID：微分系数Kd
    float SupplyPelletVelKp;            //拨弹速度环PID：比例系数Kp
    float SupplyPelletVelKi;            //拨弹速度环PID：积分系数Ki
    float SupplyPelletVelKd;            //拨弹速度环PID：微分系数Kd
    uint16_t SupplyPelletTimeInterval;  //拨弹发射间隔，单位ms

}GimbalShooter_DebugParas_StructTypeDef;


//#region /**** 变量引出extern声明****************************************************************/
/*FreeRTOS任务相关*/
extern const TickType_t GGM_TaskPeriod;
extern const float GGM_TaskTime;

/****************************通讯相关****************************/
extern IMU1Data_StructTypeDef GstGM_IMU1; 
extern uint8_t GFGM_IMU1Restart;
extern ChassisGimbal_Data_StructTypeDef GstGM_MainCtrl;//双主控通信结构体

/************************PID、位姿相关************************/
extern PID_StructTypeDef GstGM_PitchPosPID;
extern PID_StructTypeDef GstGM_PitchVelPID;
extern PID_StructTypeDef GstGM_YawPosPID;
extern PID_StructTypeDef GstGM_YawVelPID;
extern float GGM_RollAngle;
extern float GGM_RollAngleBuff;

extern GMData_StructTypeDef GSTGM_Data;     //云台正式数据结构体，存放和云台相关的几乎所有数据

/************************调试模式相关************************/
extern GimbalShooter_DebugFlags_StructTypeDef GstGMSH_Debug_Flags;  //云台发射调试相关标志位结构体
extern GimbalShooter_DebugParas_StructTypeDef GstGMSH_Debug_Paras;  //云台发射调试相关参数结构体
//#endregion

/************************************宏定义引出声明************************************/
/********************通讯相关********************/
/*GFGB_IMU1Restart的取值*/
#define IMU1RestartYES  0xF  //IMU1确定重启
#define IMU1RestartNO   0x0  //IMU1不重启

//pid赋值宏
#define PID_INIT(Kp,Ki,Kd,UMax,UpMax,UiMax,UdMax,AddMax) \
        {Kp,Ki,Kd,UMax,UpMax,UiMax,UdMax,AddMax,0,0,0,0,0,0,0,0,0}

#define PitchMaxCurrent             29000.0f
#define YawMaxCurrent               29000.0f

/****** 不同模式PID参数宏定义 ******/
//正常模式（Free/Follow）
#define PID_PitchPos_Kp_Norm       0.0f        //Pitch位置环PID：比例系数Kp
#define PID_PitchPos_Ki_Norm       0.0f        //Pitch位置环PID：积分系数Ki
#define PID_PitchPos_Kd_Norm       0.0f        //Pitch位置环PID：微分系数Kd
#define PID_PitchVel_Kp_Norm       0.0f        //Pitch速度环PID：比例系数Kp
#define PID_PitchVel_Ki_Norm       0.0f        //Pitch速度环PID：积分系数Ki
#define PID_PitchVel_Kd_Norm       0.0f        //Pitch速度环PID：微分系数Kd

#define PID_YawPos_Kp_Norm         0.0f        //Yaw位置环PID：比例系数Kp
#define PID_YawPos_Ki_Norm         0.0f        //Yaw位置环PID：积分系数Ki
#define PID_YawPos_Kd_Norm         0.0f        //Yaw位置环PID：微分系数Kd
#define PID_YawVel_Kp_Norm         0.0f        //Yaw速度环PID：比例系数Kp
#define PID_YawVel_Ki_Norm         0.0f        //Yaw速度环PID：积分系数Ki
#define PID_YawVel_Kd_Norm         0.0f        //Yaw速度环PID：微分系数Kd

//辅瞄模式（AutoAim）
#define PID_PitchPos_Kp_AutoAim    0.0f        //Pitch位置环PID：比例系数Kp
#define PID_PitchPos_Ki_AutoAim    0.0f        //Pitch位置环PID：积分系数Ki
#define PID_PitchPos_Kd_AutoAim    0.0f        //Pitch位置环PID：微分系数Kd
#define PID_PitchVel_Kp_AutoAim    0.0f        //Pitch速度环PID：比例系数Kp
#define PID_PitchVel_Ki_AutoAim    0.0f        //Pitch速度环PID：积分系数Ki
#define PID_PitchVel_Kd_AutoAim    0.0f        //Pitch速度环PID：微分系数Kd

#define PID_YawPos_Kp_AutoAim      0.0f        //Yaw位置环PID：比例系数Kp
#define PID_YawPos_Ki_AutoAim      0.0f        //Yaw位置环PID：积分系数Ki
#define PID_YawPos_Kd_AutoAim      0.0f        //Yaw位置环PID：微分系数Kd
#define PID_YawVel_Kp_AutoAim      0.0f        //Yaw速度环PID：比例系数Kp
#define PID_YawVel_Ki_AutoAim      0.0f        //Yaw速度环PID：积分系数Ki
#define PID_YawVel_Kd_AutoAim      0.0f        //Yaw速度环PID：微分系数Kd

//打符模式（Buff）
#define PID_PitchPos_Kp_Buff       0.0f        //Pitch位置环PID：比例系数Kp
#define PID_PitchPos_Ki_Buff       0.0f        //Pitch位置环PID：积分系数Ki
#define PID_PitchPos_Kd_Buff       0.0f        //Pitch位置环PID：微分系数Kd
#define PID_PitchVel_Kp_Buff       0.0f        //Pitch速度环PID：比例系数Kp
#define PID_PitchVel_Ki_Buff       0.0f        //Pitch速度环PID：积分系数Ki
#define PID_PitchVel_Kd_Buff       0.0f        //Pitch速度环PID：微分系数Kd

#define PID_YawPos_Kp_Buff         0.0f        //Yaw位置环PID：比例系数Kp
#define PID_YawPos_Ki_Buff         0.0f        //Yaw位置环PID：积分系数Ki
#define PID_YawPos_Kd_Buff         0.0f        //Yaw位置环PID：微分系数Kd
#define PID_YawVel_Kp_Buff         0.0f        //Yaw速度环PID：比例系数Kp
#define PID_YawVel_Ki_Buff         0.0f        //Yaw速度环PID：积分系数Ki
#define PID_YawVel_Kd_Buff         0.0f        //Yaw速度环PID：微分系数Kd

/*视觉协议掩码*/
typedef enum
{
    //对应掩码Vision_Type_Mask = 0x0f
    VisionMode_Assist_Aim_Normal      = 0x00, //辅瞄-敌方正常模式
    VisionMode_Assist_Aim_TOP         = 0x01, //辅瞄-敌方小陀螺模式
    VisionMode_Interfere_BuffBig      = 0x02, //反大符
    VisionMode_Interfere_BuffSmall    = 0x05, //反小符
    VisionMode_BuffSmall              = 0x03, //小符
    VisionMode_BuffBig                = 0x04, //大符

    //对应掩码Target_Type_Mask = 0x10
    Vision_Target_Red                 = 0x00, //目标为红
    Vision_Target_Blue                = 0x10, //目标为蓝

    //对应掩码Vision_Cmd_Mask  = 0x80
    Vision_Disable                    = 0x00, //视觉失能
    Vision_Enable                     = 0x80, //视觉使能
} USART_Protocol_ID;

typedef enum
{
    //不同敌人对应的掩码
    Base       			 = 0x01, //基地
    Hero        		 = 0x02, //英雄
    Engineer        	 = 0x04, //工程
    Soldier_3      		 = 0x08, //3号步兵
    Soldier_4            = 0x10, //4号步兵
    Soldier_5            = 0x20, //5号步兵
    OutPost              = 0x40, //前哨
    Sentry               = 0x80, //哨兵
} En_Enemy_Status_Judge;

/*电机码盘结构体*/
typedef struct
{
	int32_t siRawValue;	   // 本次编码器的原始值
	int32_t siPreRawValue; // 上一次编码器的原始值
	int32_t siDiff;		   // 编码器两次原始值的差值
	int32_t siSumValue;	   // 编码器累加值
	float siGearRatio;     // 电机减速器减速比
	int32_t siNumber;	   // 编码器线数
	float fpSpeed;	       // 电机减速器输出轴转速，单位：r/min
} ST_ENCODER;

/*全局变量*/
extern GMMode_EnumTypeDef GEMGM_Mode;

extern TD_StructTypeDef PitchTD;
extern TD_StructTypeDef YawTD;
extern float pitch_td_coe_Norm;
extern float yaw_td_coe_Norm;
extern float TD_yaw_Norm ;
extern float TD_pitch_Norm;
extern float yaw_td_coe;
extern float pitch_td_coe;
extern float GstGM_PitchPos_I_SumErr;  //Pitch位置积分累计误差
extern float PitchMax;
extern float PitchMin;   
extern float RC_SST_Yaw;	      // 遥控模式 YAW轴灵敏度
extern float RC_SST_Pitch;        // 遥控模式 PITCH轴敏度度
extern float KeyMouse_SST_Yaw;	  // 键鼠模式 YAW轴灵敏度
extern float KeyMouse_SST_Pitch;  // 键鼠模式 PITCH轴灵敏度

extern uint8_t Vision_State_Enable_Now; //当前视觉模式使能状态
extern uint8_t Vision_State_Enable_Pre; //过去视觉模式使能状态
extern uint8_t Vision_Mode_Now;         //当前视觉模式
extern uint8_t Vision_Mode_Pre;         //过去视觉模式
extern bool    Vision_Mode_Changing;    //视觉模式是否正在切换标志位

//云台电机参数结构体
extern Pitch_Motor_MIT_StructTypeDef Pitch_Motor_Paras;
extern Yaw_Motor_StructTypeDef Yaw_Motor_Paras;

//云台电机编码器反馈
//TODO:这一部分需要挪到shoot中，之后再进行封装（MS）
extern volatile ST_ENCODER g_stCMREncoder;
extern volatile ST_ENCODER g_stCMLEncoder;

extern void Gimbal_Task_Init(void);

#endif
