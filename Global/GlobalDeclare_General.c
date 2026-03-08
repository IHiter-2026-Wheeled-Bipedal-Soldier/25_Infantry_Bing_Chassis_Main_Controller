/**
  ******************************************************************************
  * @file    GlobalDeclare_General.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.9.19
  * @brief   用来存放通用的全局变量，如果是与底盘或者云台有关的全局变量，
             请前往GlobalDeclare_Chassis或GlobalDeclare_Gimbal
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include <stdbool.h>
#include "GlobalDeclare_General.h"
#include <arm_math.h>

/******************************************不需要修改的变量定义******************************************/
/********test标志位、变量********/

uint8_t  G_TestFlag = 0;     //test用标志位，可以随便放在一些你想放的位置方便Debug，但是注意Debug后要删掉
uint8_t  G_u8Test;           //test用的，uint8_t变量
uint16_t G_u16Test;          //test用的，uint16_t变量
uint32_t G_u32Test;          //test用的，uint32_t变量
int16_t  G_s16Test;          //test用的，int16_t变量
int32_t  G_s32Test;          //test用的，int32_t变量
float    G_fTest;            //test用的，float变量
float    G_fTest1;           //test用的，float变量1
float    G_fTest2;           //test用的，float变量2
float    G_fTest3;           //test用的，float变量3

/********Debug使用，系统监控结构体********/
SystemMonitor_StructTypeDef GST_SystemMonitor;  //系统监控结构体，帮助统计各个任务的帧率
Debug_TargetAutoAlter_StructTypeDef GstPitch_DebugDes_AutoAlter = {0}; //Pitch轴调试目标自动设定结构体
Debug_TargetAutoAlter_StructTypeDef GstYaw_DebugDes_AutoAlter = {0};   //Yaw轴调试目标自动设定结构体

/********通讯相关********/
bool GF_USART1_RxDone = false;           //串口1接收完成标志位
ReceiverData_StructTypeDef GST_Receiver;    //遥控器接收机接收数据结构体

/*机器人整体控制结构体*/
RobotControl_StructTypeDef GST_RMCtrl; //机器人整体控制结构体，这个结构体只输入，不输出。如果要获取对应变量的输出，请前往对应的数据结构体，比如GSTCH_Data等。

/********键鼠相关********/
//基本按键
bool PRESSED_W      = FALSE;
bool PRESSED_S      = FALSE;
bool PRESSED_A      = FALSE;
bool PRESSED_D      = FALSE;
bool PRESSED_SHIFT  = FALSE;
bool PRESSED_CTRL   = FALSE;
bool PRESSED_Q      = FALSE;
bool PRESSED_E      = FALSE;
//图传接收器增加的按键
bool PRESSED_R      = FALSE;
bool PRESSED_F      = FALSE;
bool PRESSED_G      = FALSE;
bool PRESSED_Z      = FALSE;
bool PRESSED_X      = FALSE;
bool PRESSED_C      = FALSE;
bool PRESSED_V      = FALSE;
bool PRESSED_B      = FALSE;

//基本按键
bool PRESSED_W_Pre      = FALSE;
bool PRESSED_S_Pre      = FALSE;
bool PRESSED_A_Pre      = FALSE;
bool PRESSED_D_Pre      = FALSE;
bool PRESSED_SHIFT_Pre  = FALSE;
bool PRESSED_CTRL_Pre   = FALSE;
bool PRESSED_Q_Pre      = FALSE;
bool PRESSED_E_Pre      = FALSE;
//图传接收器增加的按键
bool PRESSED_R_Pre      = FALSE;
bool PRESSED_F_Pre      = FALSE;
bool PRESSED_G_Pre      = FALSE;
bool PRESSED_Z_Pre      = FALSE;
bool PRESSED_X_Pre      = FALSE;
bool PRESSED_C_Pre      = FALSE;
bool PRESSED_V_Pre      = FALSE;
bool PRESSED_B_Pre      = FALSE;



//裁判系统接收数据结构体
ext_game_status_t			         G_ST_Game_Status;//1.比赛状态数据
ext_game_result_t              G_ST_Game_Result;//2.比赛结果数据
ext_game_robot_HP_t            G_ST_Game_Robot_HP;//3.机器人血量数据
ext_event_data_t               G_ST_Event_Data;//6.场地事件数据
ext_referee_warning_t          G_ST_Referee_Warning;//8.裁判警告信息
ext_dart_remaining_time_t	     G_ST_Dart_Remaining_Time;//9.飞镖发射口倒计时
ext_game_robot_status_t        G_ST_Game_Robot_Status;//10.比赛机器人状态
ext_power_heat_data_t          G_ST_Power_Heat_Data;//11.实时功率热量数据
ext_game_robot_pos_t           G_ST_Game_Robot_Pos;//12.机器人位置
ext_buff_t					           G_ST_Buff;//13.机器人增益
ext_robot_hurt_t               G_ST_Robot_Hurt;//15.伤害状态
ext_shoot_data_t               G_ST_Shoot_Data;//16.实时射击信息
ext_bullet_remaining_t         G_ST_Bullet_Remaining;//17.子弹剩余发射数
ext_rfid_status_t			         G_ST_RFID_Status;//18.机器人 RFID 状态
ext_dart_client_cmd_t          G_ST_Dart_Client_Cmd;//19.飞镖机器人客户端指令数据
ground_robot_position_t        G_ST_Ground_Robot_Position;//20.己方机器人位置坐标
radar_mark_data_t              G_ST_Radar_Mark_Data;//21.对方机器人被易伤标记进度

//辅瞄接收数据相关结构体
ST_VISION GST_Vision = {0};
