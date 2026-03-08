/*******************************************************************************
 * @file    GlobalDeclare_General.h
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.10.11
 * @brief   extern需要给外部调用的全局变量
 ******************************************************************************
 */
#ifndef __GLOBALDECLARE_GENERAL_H
#define __GLOBALDECLARE_GENERAL_H

#include <stdbool.h>
#include <stdint.h>
#include "Algorithm.h"

/****************************************结构体声明****************************************/
/*辅助结构体：机器人底盘默认可配置的控制变量结构体：无需解除标志位就允许配置*/
typedef struct {
    /*增加一个成员就需要在Chassis_RobotCtrlDefaultConfigDataReset里增加对应的清零代码*/
    float LegLen1Des;  // 左腿腿长目标值（未经过TD处理），单位mm
    float LegLen2Des;  // 右腿腿长目标值（未经过TD处理），单位mm
    float LegLen1ManualDes; // 手动控制的目标腿长（三档：Min, Mid, High）
    float LegLen2ManualDes; // 手动控制的目标腿长（三档：Min, Mid, High）

    float DisDes;          // 底盘位移目标值，向前为正，单位m
    float VelDes;          // 底盘速度目标值，向前为正，单位m/s
    float YawDeltaDes;     // 底盘Yaw轴偏转角度目标值，逆时针为正，单位度/s
    float YawAngleVelDes;  // 偏转角速度目标值，逆时针为正，单位度/s

    float Leg1FFForce;  // 左腿前馈力，单位N
    float Leg2FFForce;  // 右腿前馈力，单位N
} _ChassisCtrlData_DefaultConfig_StructTypeDef;

/*辅助结构体：机器人底盘强制可配置的控制变量结构体：必须解除标志位才可以配置*/
typedef struct
{
    /*增加一个成员就需要在Chassis_RobotCtrlForceConfigDataReset里增加对应的清零代码*/
    /*LQR用户自定义配置*/
    bool  F_LQR_UserSetEnable;  //LQR自定义配置标志位。true：所有目标值均可由用户自定义。false：除了前四项外，其他目标值保持默认0值

    float Theta1Des;          //左腿摆角目标值，后摆为正，单位度
    float Theta1AngleVelDes;  //左腿摆角速度目标值，后摆为正，单位度/s
    float Theta2Des;          //右腿摆角目标值，后摆为正，单位度
    float Theta2AngleVelDes;  //右腿摆角速度目标值，后摆为正，单位度/s
    float PitchAngleDes;      //俯仰角目标值，抬头为正，单位度
    float PitchAngleVelDes;   //俯仰角速度目标值，抬头为正，单位度/s

    /*VMC用户自定义配置*/
    bool  F_VMC_UserSetEnable;  //VMC自定义配置标志位。true：VMC的力、力矩由用户自定义，Leg1F、Leg2F、Leg1T、Leg2T自定义变量生效。false：VMC自动计算力和力矩

    float Leg1FDes;      //左腿沿杆力的目标值，单位N
    float Leg2FDes;      //右腿沿杆力的目标值，单位N
    float Leg1TDes;      //左腿力矩目标值，后摆为正，单位N·m
    float Leg2TDes;      //右腿力矩目标值，后摆为正，单位N·m

    /*轮毂电机扭矩用户自定义配置*/
    bool F_HMTorque_UserSetEnable;  //轮毂电机扭矩自定义配置标志位。true：轮毂电机扭矩由用户自定义，HM1T、HM2T生效。false：轮毂电机扭矩由底盘控制策略计算得出
    
    float HM1TDes;  //左轮毂电机扭矩目标值，正值为前进，单位N·m
    float HM2TDes;  //右轮毂电机扭矩目标值，正值为前进，单位N·m

    /*关节电机扭矩用户自定义配置*/
    bool F_JMTorque_UserSetEnable;  //关节电机扭矩自定义配置标志位。true：关节电机扭矩由用户自定义，JM1T~JM4T生效。false：关节电机扭矩由底盘控制策略计算得出

    float JM1TDes;      //关节电机1力矩目标值，单位N·m
    float JM2TDes;      //关节电机2力矩目标值，单位N·m
    float JM3TDes;      //关节电机3力矩目标值，单位N·m
    float JM4TDes;      //关节电机4力矩目标值，单位N·m
}_ChassisCtrlData_ForceConfig_StructTypeDef;

/*机器人整体控制结构体*/
typedef struct
{
    /*底盘相关*/
    _ChassisCtrlData_DefaultConfig_StructTypeDef STCH_Default;   //底盘默认可配置控制变量结构体
    _ChassisCtrlData_ForceConfig_StructTypeDef   STCH_Force;     //底盘强制可配置控制变量结构体

    ////待补充 /*云台相关数据结构体*/
}RobotControl_StructTypeDef;

/*系统监控结构体，用来查看各个任务的帧率*/
typedef struct
{
    /************************计数器************************/
    uint16_t CAN1Rx_cnt;        //CAN1Rx计数器
    uint16_t CAN2Rx_cnt;        //CAN2Rx计数器
    uint16_t HubMotor1Rx_cnt;   //左轮毂电机接收计数器
    uint16_t HubMotor2Rx_cnt;   //右轮毂电机接收计数器

    uint16_t USART1Rx_cnt;      //USART1Rx计数器
    uint16_t USART2Rx_cnt;      //USART2Rx计数器
    uint16_t USART3Rx_cnt;      //USART3Rx计数器
    uint16_t UART4Rx_cnt;       //UART4Rx计数器
    uint16_t UART5Rx_cnt;       //UART5Rx计数器
    uint16_t USART6Rx_cnt;      //USART6Rx计数器

    uint16_t SendDataTask_cnt;  //SendDataTask计数器
    uint16_t ChassisTask_cnt;   //ChassisTask计数器
    uint16_t GimbalTask_cnt;    //GimbalTask计数器
    uint16_t ShooterTask_cnt;     //ShooterTask计数器
    uint16_t DebugTask_cnt;     //DebugTask计数器

    /************************帧率************************/
    uint16_t CAN1Rx_fps;        //CAN1Rx帧率
    uint16_t CAN2Rx_fps;        //CAN2Rx帧率
    uint16_t HubMotor1Rx_fps;   //左轮毂电机帧率
    uint16_t HubMotor2Rx_fps;   //右轮毂电机帧率

    uint16_t USART1Rx_fps;      //USART1Rx帧率
	uint16_t USART2Rx_fps;      //USART2Rx帧率
	uint16_t USART3Rx_fps;      //USART3Rx帧率
	uint16_t UART4Rx_fps;       //UART4Rx帧率
	uint16_t UART5Rx_fps;       //UART5Rx帧率
	uint16_t USART6Rx_fps;      //USART6Rx帧率

    uint16_t SendDataTask_fps;  //SendDataTask帧率
    uint16_t ChassisTask_fps;   //ChassisTask帧率
    uint16_t GimbalTask_fps;    //GimbalTask帧率
    uint16_t ShooterTask_fps;   //ShooterTask帧率
    uint16_t DebugTask_fps;     //DebugTask帧率
}SystemMonitor_StructTypeDef;

/*遥控器接收机 接收DR16 18字节数据的结构体*/
typedef struct
{
    /*辅助结构体1：遥控器数据结构体*/
    struct _RCControlData_StructTypeDef
    {
        uint16_t JoyStickR_X; //右摇杆X轴，通道0，左小右大
        uint16_t JoyStickR_Y; //右摇杆Y轴，通道1，上小下大
        uint16_t JoyStickL_X; //左摇杆X轴，通道2，左小右大
        uint16_t JoyStickL_Y; //左摇杆Y轴，通道3，上小下大
        uint16_t Roller;      //拨轮，通道4，下大上小
        uint8_t  Level_L;     //左拨杆
        uint8_t  Level_R;     //右拨杆
    }ST_RC;

    /*辅助结构体2：鼠标数据结构体*/
    //待优化：这里的变量命名不是很清楚（比如这个Z是干嘛的，Left表示左键有点奇怪）
    struct _KeyMouseControlData_StructTypeDef
    {
        int16_t X;
        int16_t Y;
        int16_t Z;
        uint8_t Left;   //左键是否按下
        uint8_t Right;  //右键是否按下
    }ST_Mouse;
    
    uint16_t usKeyboard;
}ReceiverData_StructTypeDef;

/*C620反馈数据结构体*/
typedef struct
{
    int16_t EncoderValue; //编码器反馈值0-8191（0x1FFF），对应电机的0-360度
    int16_t AngleVelFB;      //速度反馈，单位：r/min（注意是电机不是减速箱输出轴）
    int16_t CurrentFB;    //电流反馈，单位：mA
    int8_t  TempFB;       //温度反馈，单位：°C
}C620FeedBackData_StructTypeDef;

/*编码器相关数据结构体*/
//待考虑：要不要加入差值、零点之类的变量
typedef struct
{
    /*需要初始化赋值的成员*/
    const int32_t PPR;   //转一圈产生的编码器脉冲数

    /*不需要初始化赋值的成员*/
    int32_t Value;       //编码器值
    int32_t ValuePre;    //编码器上次值
    int32_t ValueSum;    //编码器值累计和
}EncoderData_StructTypeDef;

// #pragma region
// /****枚举声明*********************************************************************************/
/*机器人左右侧枚举类型*/
typedef enum {
    LeftSide,  // 左侧
    RightSide  // 右侧
} RobotSide_EnumTypeDef;
// #pragma endregion

/********************************不需要修改的变量引出extern声明********************************/
/*************机器人整体控制结构体*************/

extern RobotControl_StructTypeDef GST_RMCtrl;

/***************test标志位、变量***************/
extern uint8_t  G_TestFlag;
extern uint8_t  G_u8Test;
extern uint16_t G_u16Test;
extern uint32_t G_u32Test;
extern int16_t  G_s16Test;
extern int32_t  G_s32Test;
extern float    G_fTest;
extern float    G_fTest1;
extern float    G_fTest2;
extern float    G_fTest3;

/********Debug使用，系统监控结构体********/
extern SystemMonitor_StructTypeDef GST_SystemMonitor;
extern Debug_TargetAutoAlter_StructTypeDef GstPitch_DebugDes_AutoAlter; //Pitch轴调试目标自动设定结构体
extern Debug_TargetAutoAlter_StructTypeDef GstYaw_DebugDes_AutoAlter;   //Yaw轴调试目标自动设定结构体

/*****************通讯相关****************/
extern bool GF_USART1_RxDone;
extern ReceiverData_StructTypeDef GST_Receiver;


/****************************************宏定义引出声明（一般不需要修改）****************************************/
/************************遥控器相关************************/
/*拨杆相关*/
#define RCLevel_Up       ((uint8_t)1)   //遥控器拨杆在上
#define RCLevel_Mid      ((uint8_t)3)   //遥控器拨杆在中
#define RCLevel_Down     ((uint8_t)2)   //遥控器拨杆在下
/*通道（拨轮、摇杆）相关*/
#define RCChannelValue_Min          ((uint16_t)364)  //遥控器通道最小值
#define RCChannelValue_Mid          ((uint16_t)1024) //遥控器通道中间值
#define RCChannelValue_Max          ((uint16_t)1684) //遥控器通道最大值
#define RCJoyStick_UpTH             ((uint16_t)RCChannelValue_Max - 100)   //遥控器摇杆上阈值，Y轴超过该值认为摇杆在上
#define RCJoyStick_DownTH           ((uint16_t)RCChannelValue_Min + 100)   //遥控器摇杆下阈值，Y轴低于该值认为摇杆在下
#define RCJotStick_LeftTH           ((uint16_t)RCChannelValue_Min + 100)   //遥控器摇杆左阈值，X轴低于该值认为摇杆在左
#define RCJoyStick_RightTH          ((uint16_t)RCChannelValue_Max - 100)   //遥控器摇杆右阈值，X轴超过该值认为摇杆在右
#define RCRoller_UpTH               ((uint16_t)RCChannelValue_Min + 100)   //遥控器拨轮上阈值，拨轮低于该值认为拨轮在上
#define RCRoller_DownTH             ((uint16_t)RCChannelValue_Max - 100)   //遥控器拨轮下阈值，拨轮超过该值认为拨轮在下
#define RCChannel_DeadZone          ((uint16_t)340)    //遥控器通道死区范围，摇杆在中间位置时，偏差小于该值则认为是0输入
// TODO 下面的宏定义暂时没什么用，可以考虑删掉。如果有用到的话改成一个更合适的名字
// #define RC_CH_VALUE_GIMBAL_DEAD     ((u16)20)
// #define RC_CH_VALUE_RANGE           ((u16)660)


/********SystemMonitor相关，最低帧率阈值取正常值的70%左右********/
#define USART1Rx_fpsMinTH       50      //串口1接收帧率最小阈值，低于该值可能是遥控器断开
#define UART4Rx_fpsMinTH        800     //串口4接收帧率最小阈值，低于该值可能是IMU2断开
#define HubMotorRx_fpsMinTH     700     //轮毂电机接收帧率最小阈值，低于该值可能是电机断开

/********电机、电调相关********/
#define ESC_C620Period              0.001f                   //C620电调反馈周期，单位：秒
#define ESC_C620PPR                 8192                     //C620电调的编码器每圈脉冲数
#define ESC_C620MaxCurrent          16384                    //C620电调的最大电流值，对应20A
#define ESC_C620MinCurrent          -16384                   //C620电调的最小电流值，对应-20A
#define ESC_C620AmpereToCurrent     (1/20.0f*16384.0f)       //C620电调电流转换系数，单位：映射电流值/A

// #define Motor_3508ReductionRatio        3591.0f/187.0f          //3508电机减速比(本体的减速箱)
#define Motor_3508GearboxReductionRatio 268.0f/17.0f            //3508电机外接齿轮箱减速比
#define Motor_3508MaxCurrent            ESC_C620MaxCurrent      //3508电机（连在C620电调上）的最大电流值
#define Motor_3508MinCurrent            ESC_C620MinCurrent      //3508电机（连在C620电调上）的最小电流值
#define Motor_3508AmpereToCurrent       ESC_C620AmpereToCurrent //3508电机（连在C620电调上）电流转换系数，单位：映射电流值/A
#define Motor_3508Kt                    0.3f                    //3508电机转矩常数，单位：Nm/A

#define Motor_MG8016Ei6MaxTorque        37.0f	                //瓴控的MG8016E-i6电机最大转矩，单位：Nm


/********计算相关********/
/*角弧度制互换*/
#define A2R     PI/180.0f    //角度制转弧度制，AngleToRadian
#define R2A     180.0f/PI    //弧度制转角度制，RadianToAngle
/*其他单位互换*/
#define MM2M    0.001f       //毫米转米，MillimeterToMeter
#define M2MM    1000.0f      //米转毫米，MeterToMillimeter
/*其他常数*/
#define GravityAcc  9.80f    //标准重力加速度，单位：m/s²
#define GravityAcc_Harbin 9.806639f //哈尔滨地区重力加速度，单位：m/s²
#define GravityAcc_ChangSha 9.7915f //长沙地区重力加速度，单位：m/s²
#define GravityAcc_ShenZhen 9.7803f //深圳地区重力加速度，单位：m/s²

//TODO:键鼠模式增加部分（MS）
#define true	1
#define false	0
#define FALSE false
#define TRUE true

//键盘键位定义
extern bool PRESSED_W;
extern bool PRESSED_S;
extern bool PRESSED_A;
extern bool PRESSED_D;
extern bool PRESSED_SHIFT;
extern bool PRESSED_CTRL;
extern bool PRESSED_Q;
extern bool PRESSED_E;
extern bool PRESSED_R;
extern bool PRESSED_F;
extern bool PRESSED_G;
extern bool PRESSED_Z;
extern bool PRESSED_X;
extern bool PRESSED_C;
extern bool PRESSED_V;
extern bool PRESSED_B;

extern bool PRESSED_W_Pre;
extern bool PRESSED_S_Pre;
extern bool PRESSED_A_Pre;
extern bool PRESSED_D_Pre;
extern bool PRESSED_SHIFT_Pre;
extern bool PRESSED_CTRL_Pre;
extern bool PRESSED_Q_Pre;
extern bool PRESSED_E_Pre;
extern bool PRESSED_R_Pre;
extern bool PRESSED_F_Pre;
extern bool PRESSED_G_Pre;
extern bool PRESSED_Z_Pre;
extern bool PRESSED_X_Pre;
extern bool PRESSED_C_Pre;
extern bool PRESSED_V_Pre;
extern bool PRESSED_B_Pre;

#define KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define KEY_PRESSED_OFFSET_R		((uint16_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F		((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G		((uint16_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z	    ((uint16_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X	    ((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C		((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V		((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B		((uint16_t)0x01<<15)

//TODO:裁判系统发送数据相关结构体（MS）
//裁判系统数据ID
typedef enum
{
    GameStatus_ID               = 0x0001,  //比赛状态，1Hz周期发送
    GameResult_ID               = 0x0002,  //比赛结果，比赛结束后发送
    GameRobotHP_ID              = 0x0003,  //机器人血量数据，3Hz周期发送
    EventData_ID                = 0x0101,  //场地事件数据，1Hz周期发送
    RefereeWarning_ID           = 0x0104,  //裁判警告数据，警告发生后发送
    DartRemainingTime_ID        = 0x0105,  //飞镖发射口倒计时，1Hz周期发送
    GameRobotStatus_ID          = 0x0201,  //机器人状态数据，10Hz周期发送
    PowerHeatData_ID            = 0x0202,  //实时底盘缓冲能量和射击热量数据，10Hz周期发送
    GameRobotPos_ID             = 0x0203,  //机器人位置数据，1Hz发送
    Buff_ID                     = 0x0204,  //机器人增益和底盘能量数据，3Hz周期发送
    RobotHurt_ID                = 0x0206,  //伤害状态数据，伤害发生后发送
    ShootData_ID                = 0x0207,  //实时射击数据，子弹发射后发送
    BulletRemaining_ID          = 0x0208,  //弹丸剩余发射数，仅空中机器人，哨兵机器人以及ICRA机器人发送，1Hz周期发送
    RFIDStatus_ID               = 0x0209,  //机器人RFID状态，3Hz周期发送
    DART_ID                     = 0x020A,  //飞镖状态，飞镖发射后发送
    SelfRobotPosition_ID        = 0x020B,  //己方位置信息
    Radarmark_ID                = 0x020C,  //雷达标记数据
    Robot_Interactive_ID        = 0x0301,  //机器人间交互数据，发送方触发发送，上限10Hz
} EN_RSYS_ID;

//机器人ID
typedef enum
{
    Red_1_Hero       = 1,
    Red_2_Engineer   = 2,
    Red_3_Standard   = 3,
    Red_4_Standard   = 4,
    Red_5_Standard   = 5,
    Red_6_Aerial     = 6,
    Red_7_Sentry     = 7,
    Red_9_Radar      = 9,

    Blue_1_Hero     = 101,
    Blue_2_Engineer = 102,
    Blue_3_Standard = 103,
    Blue_4_Standard = 104,
    Blue_5_Standard = 105,
    Blue_6_Aerial   = 106,
    Blue_7_Sentry   = 107,
    Blue_9_Radar    = 109,
} EN_Robot_ID;

//客户端ID
typedef enum
{
    Red_1_Hero_Client       = 0x101,
    Red_2_Engineer_Client   = 0x102,
    Red_3_Standard_Client   = 0x103,
    Red_4_Standard_Client   = 0x104,
    Red_5_Standard_Client   = 0x105,
    Red_6_Aerial_Client     = 0x106,

    Blue_1_Hero_Client      = 0x0165,
    Blue_2_Engineer_Client  = 0x0166,
    Blue_3_Standard_Client  = 0x0167,
    Blue_4_Standard_Client  = 0x0168,
    Blue_5_Standard_Client  = 0x0169,
    Blue_6_Aerial_Client    = 0x016A,
} EN_Client_ID;

//1.比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。
typedef __packed struct
{
	uint8_t game_type: 4;//比赛类型
	uint8_t game_progress: 4;//当前比赛阶段
	uint16_t stage_remain_time;//当前阶段剩余时间，单位：秒
	uint64_t SyncTimeStamp;//UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} ext_game_status_t;

//2.比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。
typedef __packed struct
{
	uint8_t winner;//比赛胜利者（平局or红方胜or蓝方胜）
} ext_game_result_t;

//3.机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。
//新赛季后只会收到己方机器人血量TODO:暂时未更改变量名，但注释已改（MS）
typedef __packed struct
{
	uint16_t red_1_robot_HP;//己方 1 号英雄机器人血量
	uint16_t red_2_robot_HP;//己方 2 号工程机器人血量
	uint16_t red_3_robot_HP;//己方 3 号步兵机器人血量
	uint16_t red_4_robot_HP;//己方 4 号步兵机器人血量
	uint16_t red_5_robot_HP;//保留位 
	uint16_t red_7_robot_HP;//己方 7 号哨兵机器人血量
	uint16_t red_outpost_HP;//己方前哨站血量 
	uint16_t red_base_HP;   //己方基地血量
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

//6.场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人。
typedef __packed struct
{
    uint32_t event_type;//为一组32bit数据码，主要包括区域占领情况，飞镖命中时间等场上信息
} ext_event_data_t;

//8.裁判警告信息：cmd_id(0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。
typedef __packed struct
{
    uint8_t level;//己方最后一次受到判罚的等级
    uint8_t foul_robot_id;//己方最后一次受到判罚的违规机器人ID
    uint8_t count;//己方最后一次受到判罚的违规机器人对应判罚等级的违规次数
} ext_referee_warning_t;

//9.飞镖发射口倒计时：cmd_id(0x0105)。发送频率：1Hz周期发送，发送范围：己方机器人。
typedef __packed struct
{
    uint8_t dart_remaining_time;//己方飞镖发射剩余时间，单位：秒
    uint16_t dart_info;//飞镖目标状态及击中次数，飞镖此时选定的击打目标等
} ext_dart_remaining_time_t;

//10.比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人。
typedef __packed struct
{
    uint8_t robot_id;//本机器人 ID
    uint8_t robot_level;//机器人等级
    uint16_t remain_HP;//机器人当前血量
    uint16_t max_HP;//机器人血量上限

    uint16_t shooter_id1_17mm_cooling_rate;//机器人射击热量每秒冷却值
    uint16_t shooter_id1_17mm_cooling_limit;//机器人射击热量上限
    uint16_t chassis_power_limit;//机器人底盘功率上限

	uint8_t mains_power_gimbal_output: 1;//电源管理模块的输出情况：gimbal 口输出，0 为无输出，1 为 24V 输出
	uint8_t mains_power_chassis_output: 1;//电源管理模块的输出情况：chassis 口输出，0 为无输出，1 为 24V 输出
	uint8_t mains_power_shooter_output: 1;//电源管理模块的输出情况：shooter 口输出，0 为无输出，1 为 24V 输出
} ext_game_robot_status_t;

//新赛季有相应通信协议更改
//11.实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。
typedef __packed struct
{
	uint16_t reserved;
	uint16_t reserved1;
	float reserved2;
	uint16_t chassis_power_buffer;//缓冲能量（单位：J）
	uint16_t shooter_id1_17mm_cooling_heat;//第 1 个 17mm 发射机构的射击热量
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

//12.机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人。
typedef __packed struct
{
    float x;	//位置 x 坐标，单位 m
    float y;	//位置 y 坐标，单位 m
    float yaw;  //本机器人测速模块的朝向，单位：度。正北为 0 度
} ext_game_robot_pos_t;

//13.机器人增益： 0x0204。发送频率： 1Hz 周期发送，发送范围：单一机器人。
typedef __packed struct
{
    uint8_t recovery_buff; //机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint16_t cooling_buff; //机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s）
    uint8_t defence_buff; //机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff; //机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint16_t attack_buff; //机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
    uint8_t remaining_energy;//机器人剩余能量值反馈
} ext_buff_t;


//15.伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人。
typedef __packed struct
{
    uint8_t armor_id : 4;//当扣血原因为装甲模块被弹丸攻击、受撞击或离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号，其他原因值为0
    uint8_t hurt_type : 4;//血量变化类型（被弹丸攻击or模块离线or受到撞击）
} ext_robot_hurt_t;

//16.实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人。
typedef __packed  struct
{
	uint8_t bullet_type;//弹丸类型
	uint8_t shooter_id;//发射机构 ID
	uint8_t bullet_freq;//弹丸频率（单位：Hz）
	float bullet_speed;//弹丸初速度（单位：m/s）
} ext_shoot_data_t;

//17.子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人，哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人。
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;//机器人自身拥有的 17mm 弹丸允许发弹量
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;//剩余金币数量
    uint16_t projectile_allowance_fortress; //剩余堡垒弹丸数量
} ext_bullet_remaining_t;

//18.机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。
typedef __packed struct
{
    uint32_t rfid_status;//是否已检测到该增益点 RFID 卡
    uint8_t rfid_status_2; //是否已检测到该增益点 RFID 卡
} ext_rfid_status_t;

//19.飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
typedef __packed struct
{
    uint8_t dart_launch_opening_status; //当前飞镖发射站的状态
    uint8_t reserved; //保留位
    uint16_t target_change_time; //切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
    uint16_t latest_launch_cmd_time;//最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0。
} ext_dart_client_cmd_t;

//20.己方机器人位置坐标：0x020B
typedef __packed struct
{
 float hero_x;
 float hero_y;
 float engineer_x;
 float engineer_y;
 float standard_3_x;
 float standard_3_y;
 float standard_4_x;
 float standard_4_y;
 float standard_5_x;//保留位
 float standard_5_y;//保留位
}ground_robot_position_t;

//21.对方机器人被标记进度：0x020C
typedef __packed struct
{
 uint16_t mark_progress; //16bit数字码，展示敌方易伤情况与己方特殊标识情况
}radar_mark_data_t;

extern ext_game_status_t			  G_ST_Game_Status;//1.比赛状态数据
extern ext_game_result_t              G_ST_Game_Result;//2.比赛结果数据
extern ext_game_robot_HP_t            G_ST_Game_Robot_HP;//3.机器人血量数据
extern ext_event_data_t               G_ST_Event_Data;//6.场地事件数据
extern ext_referee_warning_t          G_ST_Referee_Warning;//8.裁判警告信息
extern ext_dart_remaining_time_t	  G_ST_Dart_Remaining_Time;//9.飞镖发射口倒计时
extern ext_game_robot_status_t        G_ST_Game_Robot_Status;//10.比赛机器人状态
extern ext_power_heat_data_t          G_ST_Power_Heat_Data;//11.实时功率热量数据
extern ext_game_robot_pos_t           G_ST_Game_Robot_Pos;//12.机器人位置
extern ext_buff_t					  G_ST_Buff;//13.机器人增益
extern ext_robot_hurt_t               G_ST_Robot_Hurt;//15.伤害状态
extern ext_shoot_data_t               G_ST_Shoot_Data;//16.实时射击信息
extern ext_bullet_remaining_t         G_ST_Bullet_Remaining;//17.子弹剩余发射数
extern ext_rfid_status_t			  G_ST_RFID_Status;//18.机器人 RFID 状态
extern ext_dart_client_cmd_t          G_ST_Dart_Client_Cmd;//19.飞镖机器人客户端指令数据
extern ground_robot_position_t        G_ST_Ground_Robot_Position;//20.己方机器人位置坐标
extern radar_mark_data_t              G_ST_Radar_Mark_Data;//21.对方机器人被易伤标记进度

//辅瞄收发数据相关结构体
typedef struct
{
	#pragma pack(1)
    struct{
        unsigned char m_head[2];
        unsigned char m_id;
        unsigned char m_length;
        float Pitch;                // 4
        float Yaw;                  // 4
        float Rol;  
		float PelletSpeed;
		float _positionX;
		float _positionY;
		float _positionZ;
		float targetHP;
		float targetNum;
		float z_coordinate;
		float x_coordinate;
		float angle_toward_world_coordinate;
		float pmflag;//辅瞄模式选择
		unsigned char enemy_attack_flag;
		unsigned char enemy_vul_rig;
		unsigned char enemy_rec_rig;
		unsigned char reserved;
		unsigned short blue_1_robot_HP;
		unsigned short blue_3_robot_HP;
		unsigned short blue_4_robot_HP;
		unsigned short blue_5_robot_HP;
		float computer_shutdown;
        float bullet_cnt;
        unsigned char m_tail[2];
    }AimAssistDataSendFrame;
    struct{
        unsigned char m_head[2];
        unsigned char FindTargetOrNot;
        unsigned char m_length;
        float Pitch;                
        float Yaw;    
		float ShootOrNot;
		float Target;
		float _positionX;
		float _positionY;
		float _positionZ;
		float _attackValue;
		float _omega;
        unsigned char m_tail[2];
    }AimAssistDataReceiveFrame;
#pragma pack()
} ST_VISION;

extern ST_VISION GST_Vision; //辅瞄收发数据相关结构体

#endif
