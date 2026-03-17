/**
 ******************************************************************************
 * @file    Chassis_Stratgy.c
 * @author  26赛季，平衡步兵电控，林宸曳
 * @date    2025.11.2
 * @brief   底盘控制策略相关函数
 *          把底盘的控制策略函数放在这里，方便外部调用
 ******************************************************************************
 */

#include "Algorithm_Simple.h"
#include "Chassis_APIFunction.h"
#include "General_AuxiliaryFunc.h"
#include "GlobalDeclare_Chassis.h"
#include "TIM_Config.h"
#include "Chassis_Stratgy.h"
#include <arm_math.h>


/**
  * @brief  底盘模式选择参数结构体更新函数
  * @note   把底盘模式选择的相关参数，更新到底盘模式选择参数结构体中，以方便后面的模式选择处理
  *         在模式处理之前，应该先调用此函数，更新参数，然后再进入模式选择函数
  * @param  pModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型，底盘模式选择参数结构体指针
  * @retval 无
  */
//* 模式选择参数结构体更新函数
void ChassisStrategy_ModeChooseParaStructUpdate(Chassis_ModeChooseParameter_StructTypeDef* pModeChoosePara) 
{
    /*更新上次模式*/
    pModeChoosePara->MC_ModePre = GEMCH_ModePre;

    /*更新各个模式开始时间、当前时间*/
    pModeChoosePara->MC_ST_ModeStartTime = GSTCH_Data.ST_ModeStartTime; //各个模式开始时间
    pModeChoosePara->MC_TimeNow = RunTimeGet();                         //获取当前时间

    /*运动姿态变量*/
    pModeChoosePara->MC_LegLenAvgFB = (GSTCH_Data.LegLen1FB + GSTCH_Data.LegLen2FB) / 2.0f; //两腿腿长反馈值平均值
    pModeChoosePara->MC_VelFB = GSTCH_Data.VelFB;                                           //底盘速度反馈值

    /*离地检测标志*/
    if(GSTCH_Data.F_OffGround1 == true || GSTCH_Data.F_OffGround2 == true)
    {pModeChoosePara->MC_F_OffGround = true;}//离地状态标志，只要有任何一侧离地都算
    else
    {pModeChoosePara->MC_F_OffGround = false;}

    /*AutoSafe模式专用变量*/
    pModeChoosePara->MC_HubMotor1Rx_fps = GST_SystemMonitor.HubMotor1Rx_fps;    //轮毂电机1接收帧率
    pModeChoosePara->MC_HubMotor2Rx_fps = GST_SystemMonitor.HubMotor2Rx_fps;    //轮毂电机2接收帧率
    pModeChoosePara->MC_UART4Rx_fps     = GST_SystemMonitor.UART4Rx_fps;        //串口4，即IMU2接收帧率
}

// #pragma region 检测当前条件是否满足进入某模式的函数

// ==========================================
// 全局中断 (Global Interrupts)
// ==========================================

// 正常状态 -> ManualSafe: 左拨杆在下且右拨杆在上
bool _Is_Normal_To_ManualSafe(void)
{
    /* 如果遥控器拨杆为左下右上，进入ManualSafeMode */
    if(IsLeftLevelDown() && IsRightLevelUp())
    {return true;}
    else
    {return false;} // 否则不进入
}

// 正常状态 -> AutoSafe: 通讯异常
bool _Is_Normal_To_AutoSafe(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    /*用一些临时变量来存储相关变量*/
    float HM1_Rx_fps = ST_ModeChoosePara.MC_HubMotor1Rx_fps; //轮毂电机1通讯帧率
    float HM2_Rx_fps = ST_ModeChoosePara.MC_HubMotor2Rx_fps; //轮毂电机2通讯帧率
    float UART4_Rx_fps = ST_ModeChoosePara.MC_UART4Rx_fps;   //串口4，即IMU2通讯帧率

    /* 如果遥控器断开连接，进入AutoSafeMode */
    if(IsRCConnected() == false)
    {return true;}

    /* 如果轮毂电机通讯异常，进入AutoSafeMode */
    if(HM1_Rx_fps < HubMotorRx_fpsMinTH || HM2_Rx_fps < HubMotorRx_fpsMinTH)
    {return true;}

    /* 如果IMU2通讯异常，进入AutoSafeMode */
    if(UART4_Rx_fps < UART4Rx_fpsMinTH)
    {return true;}
    /* 否则不进入 */
    return false;
}

// ==========================================
// 状态恢复 (Recovery)
// ==========================================

// ManualSafe -> RC_Standby: 左拨杆在中
bool _Is_ManualSafe_To_RCStandby(void)
{
    /*如果左拨杆在中位，进入RC_StandbyMode*/
    if(IsLeftLevelMid() == true)
    {return true;}
    else
    {return false;} // 否则不进入
    
}

// ManualSafe -> KeyMouse_Standby: 右拨杆在中
bool _Is_ManualSafe_To_KeyMouseStandby(void)
{
    /*如果右拨杆在中位，进入KeyMouse_StandbyMode*/
    if(IsRightLevelMid() == true)
    {return true;}
    else
    {return false;} // 否则不进入
    
}

// AutoSafe -> ManualSafe: 所有通讯正常
bool _Is_AutoSafe_To_ManualSafe(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    // 复用上面的检测，取反
    if(_Is_Normal_To_AutoSafe(ST_ModeChoosePara) == false)
    {return true;} // 通讯正常，进入
    else
    {return false;}  // 通讯异常，不进入
}

// ==========================================
// 核心流程 (Core Flow)
// ==========================================

// RCStandby -> RCStandUp: 拨轮向上超过0.8s后回正（小于0.8s是切换摩擦轮状态）
bool _Is_RCStandby_To_RCStandUp(void)
{
    if(Gst_CHSH_RollerMode_Paras.ChassisMode_Flag == 1)
    {return true;}
    else
    {return false;}
}

// KeyMouseStandby -> KeyMouseStandUp: 短按 B 键
bool _Is_KeyMouseStandby_To_KeyMouseStandUp(void)
{
    if(IS_Key_ShortClick(&Key_B)) // 检测到短按 B 键
    {return true;}
    else
    {return false;}
}

// RCStandUp -> RCFree: 时间到了
bool _Is_RCStandUp_To_RCFree(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    float TimeNow = ST_ModeChoosePara.MC_TimeNow;      //当前时间
    float ThisModeStartTime = ST_ModeChoosePara.MC_ST_ModeStartTime.RC_StandUp; //起立模式开始时间
    float ThisModeTotalTime = TimeNow - ThisModeStartTime;  //起立模式总时长 = 当前时间 - 起立模式开始时间
    // 如果StandUp模式总时间没有到，继续保持StandUp模式
    if(ThisModeTotalTime >= CHMode_RC_StandUp_TotalTime)
    {return true;}
    else
    {return false;}
}
// KeyMouseStandUp -> KeyMouseFollow: 时间到了
bool _Is_KeyMouseStandUp_To_KeyMouseFollow(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    float TimeNow = ST_ModeChoosePara.MC_TimeNow;      //当前时间
    float ThisModeStartTime = ST_ModeChoosePara.MC_ST_ModeStartTime.KeyMouse_StandUp; //起立模式开始时间
    float ThisModeTotalTime = TimeNow - ThisModeStartTime;  //起立模式总时长 = 当前时间 - 起立模式开始时间
    // 如果StandUp模式总时间没有到，继续保持StandUp模式
    if(ThisModeTotalTime >= CHMode_KeyMouse_StandUp_TotalTime)
    {return true;}
    else
    {return false;}
}

// KeyMouseFollow -> KeyMouseSitDown: 按下 Ctrl+B 键
bool _Is_KeyMouseFollow_To_KeySitDown(void)
{
    if(Key_B.Key_Now && Key_CTRL.Key_Now) // 检测到 Ctrl+B 键
    {return true;}
    else
    {return false;}
}

// RCFree -> RCSitDown: 拨轮向上超过0.8s后回正（小于0.8s是切换摩擦轮状态）
bool _Is_RCFree_To_RCSitDown(void)
{
    if(Gst_CHSH_RollerMode_Paras.ChassisMode_Flag == 1)
    {return true;}
    else
    {return false;}
}

// RCSitDown -> RCStandby: 腿长达到最小
bool _Is_RCSitDown_To_RCStandby(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    float LegLenAvgFB = ST_ModeChoosePara.MC_LegLenAvgFB;  //腿长平均值
    if(LegLenAvgFB <= LegLenMin + LegLenMinTH)
    {return true;}
    else
    {return false;}
}
// KeyMouseSitDown -> KeyMouseStandby: 腿长达到最小
bool _Is_KeyMouseSitDown_To_KeyMouseStandby(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    float LegLenAvgFB = ST_ModeChoosePara.MC_LegLenAvgFB;  //腿长平均值
    if(LegLenAvgFB <= LegLenMin + LegLenMinTH)
    {return true;}
    else
    {return false;}
}

// ==========================================
// 模式切换与动作 (Mode Switch & Actions)
// ==========================================

// RCFree -> RCFollow: 左拨杆在上
bool _Is_RCFree_To_RCFollow(void)
{
    return IsLeftLevelUp();
}

// RCFollow -> RCFree: 左拨杆在中
bool _Is_RCFollow_To_RCFree(void)
{
    return IsLeftLevelMid();
}

// RCFree/RCFollow -> Jump: 遥控器模式暂不开启（原来操作：拨轮向下并回正）
bool _Is_RC_Jump(void)
{
    if(GSTCH_Data.F_RollerDownLatched == true){
        GSTCH_Data.F_RollerDownLatched = false;
        return true;
    }
    else
    {return false;}
}

// KeyMouseFollow -> Jump: Ctrl+V 键
bool _Is_KeyMouse_Jump(void)
{
    if(Key_CTRL.Key_Now && !Key_SHIFT.Key_Now && IS_Key_ShortClick(&Key_V)) // 检测到 Ctrl+V 键
    {return true;}
    else
    {return false;}
}

// KeyMouseFollow -> Stair: Ctrl+Q 键
bool _Is_KeyMouse_Stair(void)
{
    if(Key_CTRL.Key_Now && !Key_SHIFT.Key_Now && IS_Key_ShortClick(&Key_Q)) // 检测到 Ctrl+Q 键
    {return true;}
    else
    {return false;}
}

// RCFree/RCFollow/KeyMouseFollow -> OffGround: 离地标志位成立
bool _Is_To_OffGround(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    return ST_ModeChoosePara.MC_F_OffGround;
}

// OffGround/Jump -> Landed: 检测到落地 (离地标志位清零)
bool _Is_To_Landed(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara)
{
    return !ST_ModeChoosePara.MC_F_OffGround;
}


// #pragma endregion

//* 底盘模式更新函数。当检测模式条件函数返回真值时将当前模式变量变到对应变量值
/**
  * @brief  遥控器模式下，底盘模式切换函数
  * @note   根据不同的条件切换底盘的模式，要注意优先级的问题，最高优先级和次高优先级分别是手动安全模式和自动安全模式
  *         后面要添加的其他状态切换，比如跳跃、离地等等，添加时要注意优先级的顺序
  * @param  ST_ModeChoosePara：Chassis_ModeChooseParameter_StructTypeDef类型的结构体，底盘模式选择相关参数，包含的内容由用户自定义
  * @retval ChassisMode_EnumTypeDef的枚举类型，底盘的工作状态
*/
ChassisMode_EnumTypeDef ChassisStrategy_ModeChoose_RCControl(Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara) 
{
    // 状态变量准备
    ChassisMode_EnumTypeDef CurrentMode = ST_ModeChoosePara.MC_ModePre;
    // 默认返回当前状态
    ChassisMode_EnumTypeDef NextMode = CurrentMode;

    // 用于记忆跳跃或者离地前是 Free 还是 Follow（默认是free）
    static ChassisMode_EnumTypeDef LastActiveMode = CHMode_RC_Free;

    /* ========================================================== */
    /*   Layer 1: 全局中断 (Global Interrupts)                    */
    /* ========================================================== */
    
    if(_Is_Normal_To_ManualSafe())
    {return CHMode_ManualSafe;}

    if(_Is_Normal_To_AutoSafe(ST_ModeChoosePara))
    {return CHMode_AutoSafe;}

    /* ========================================================== */
    /*   Layer 2: 状态机流转 (Switch-Case)                        */
    /* ========================================================== */

    switch (CurrentMode)
    {
        /* --- 故障与待机 --- */
        case CHMode_ManualSafe:
            if(_Is_ManualSafe_To_RCStandby())                               NextMode = CHMode_RC_Standby;
            else if(_Is_ManualSafe_To_KeyMouseStandby())                    NextMode = CHMode_KeyMouse_Standby;
            break;
        case CHMode_AutoSafe:
            if(_Is_AutoSafe_To_ManualSafe(ST_ModeChoosePara))               NextMode = CHMode_ManualSafe;
            break;
        case CHMode_RC_Standby:
            if(_Is_RCStandby_To_RCStandUp())                                NextMode = CHMode_RC_StandUp;
            break;
        case CHMode_KeyMouse_Standby:
            if(_Is_KeyMouseStandby_To_KeyMouseStandUp())                    NextMode = CHMode_KeyMouse_StandUp;
            break;


        /* --- 启动与关闭 --- */
        case CHMode_RC_StandUp:
            if(_Is_RCStandUp_To_RCFree(ST_ModeChoosePara))                  NextMode = CHMode_RC_Free;
            break;
        case CHMode_RC_SitDown:
            if(_Is_RCSitDown_To_RCStandby(ST_ModeChoosePara))               NextMode = CHMode_RC_Standby;
            break;
        case CHMode_KeyMouse_StandUp:
            if(_Is_KeyMouseStandUp_To_KeyMouseFollow(ST_ModeChoosePara))    NextMode = CHMode_KeyMouse_Follow;
            break;
        case CHMode_KeyMouse_SitDown:
            if(_Is_KeyMouseSitDown_To_KeyMouseStandby(ST_ModeChoosePara))   NextMode = CHMode_KeyMouse_Standby;
            break;


        /* --- 核心运动模式 --- */
        case CHMode_RC_Free:
            LastActiveMode = CHMode_RC_Free;         // 持续更新记忆：我在RCFree
            if(_Is_RCFree_To_RCSitDown())                                   NextMode = CHMode_RC_SitDown;
            else if(_Is_RCFree_To_RCFollow())                               NextMode = CHMode_RC_Follow;
            else if(_Is_RC_Jump())                                          NextMode = CHMode_RC_Jump;
            else if(_Is_To_OffGround(ST_ModeChoosePara))                    NextMode = CHMode_OffGround;
            break;
        case CHMode_RC_Follow:
            LastActiveMode = CHMode_RC_Follow;       // 持续更新记忆：我在RCFollow
            if(_Is_RCFollow_To_RCFree())                                    NextMode = CHMode_RC_Free;
            else if(_Is_RC_Jump())                                          NextMode = CHMode_RC_Jump;
            else if(_Is_To_OffGround(ST_ModeChoosePara))                    NextMode = CHMode_OffGround;
            break;
        case CHMode_KeyMouse_Follow:
            LastActiveMode = CHMode_KeyMouse_Follow; // 持续更新记忆：我在KeyMouseFollow
            if(_Is_KeyMouseFollow_To_KeySitDown())                          NextMode = CHMode_KeyMouse_SitDown;
            else if(_Is_KeyMouse_Jump())                                    NextMode = CHMode_KeyMouse_Jump;
            else if(_Is_KeyMouse_Stair())                                   NextMode = CHMode_KeyMouse_Stair;
            else if(_Is_To_OffGround(ST_ModeChoosePara))                    NextMode = CHMode_OffGround;
            break;


        /* --- 特殊状态  --- */
        case CHMode_RC_Jump:
            // 检测到落地 -> 回到 LastActiveMode (RCFree / RCFollow / KeyMouseFollow)
            if(GSTCH_Data.F_JumpLanding && _Is_To_Landed(ST_ModeChoosePara))   NextMode = LastActiveMode; 
            break;
        case CHMode_KeyMouse_Jump:
            // 检测到落地 -> 回到 LastActiveMode (RCFree / RCFollow / KeyMouseFollow)
            if(GSTCH_Data.F_JumpLanding && _Is_To_Landed(ST_ModeChoosePara))   NextMode = LastActiveMode; 
            break;
        case CHMode_KeyMouse_Stair:
            // 检测到磕台阶完成 -> 回到 LastActiveMode (RCFree / RCFollow / KeyMouseFollow)
            if(GSTCH_Data.F_StairFinished && _Is_To_Landed(ST_ModeChoosePara)) NextMode = LastActiveMode; 
            break;
        case CHMode_OffGround:
            // 检测到落地 -> 回到 LastActiveMode (RCFree / RCFollow / KeyMouseFollow)
            if(_Is_To_Landed(ST_ModeChoosePara))                               NextMode = LastActiveMode;
            break;

        default:
            NextMode = CHMode_ManualSafe;
            break;
    }
    return NextMode;
}

//* 获取底盘各模式开始时间函数。当除安全模式以外有多个模式时需要加入（只有起立模式如果没有计时会无法设置目标值）
/**
 * @brief  获取底盘各模式开始时间的函数
 * @note   在任务切换时，获取当前任务的开始时间，方便在模式控制函数中使用
 *         后续如果新写了模式，也可以在这里添加对应的获取开始时间表达式
 * @param  pCHData：CHData_StructTypeDef类型的指针，底盘数据结构体指针
 * @param  Mode：ChassisMode_EnumTypeDef类型的枚举值，底盘当前的工作状态
 * @param  ModePre：ChassisMode_EnumTypeDef类型的枚举值，底盘上次的工作状态
 * @retval 无
 */
void ChassisStrategy_ModeStartTimeUpdate(CHData_StructTypeDef* pCHData,
                                        ChassisMode_EnumTypeDef Mode,
                                        ChassisMode_EnumTypeDef ModePre) {
    /*如果模式没有切换，直接返回*/
    if(Mode == ModePre) {
        return;
    }

    /*如果模式切换，记录当前时间为该模式的开始时间*/
    uint32_t TimeNow = RunTimeGet();
    switch (Mode)
    {
        case CHMode_ManualSafe:         pCHData->ST_ModeStartTime.ManualSafe         = TimeNow; break;
        case CHMode_AutoSafe:           pCHData->ST_ModeStartTime.AutoSafe           = TimeNow; break;
        case CHMode_OffGround:          pCHData->ST_ModeStartTime.OffGround          = TimeNow; break;
        case CHMode_RC_StandUp:         pCHData->ST_ModeStartTime.RC_StandUp         = TimeNow; break;
        case CHMode_RC_Standby:         pCHData->ST_ModeStartTime.RC_Standby         = TimeNow; break;
        case CHMode_RC_Free:            pCHData->ST_ModeStartTime.RC_Free            = TimeNow; break;
        case CHMode_RC_SitDown:         pCHData->ST_ModeStartTime.RC_SitDown         = TimeNow; break;
        case CHMode_RC_Follow:          pCHData->ST_ModeStartTime.RC_Follow          = TimeNow; break;
        case CHMode_RC_Jump:            pCHData->ST_ModeStartTime.RC_Jump            = TimeNow; break;
        case CHMode_KeyMouse_Standby:   pCHData->ST_ModeStartTime.KeyMouse_Standby   = TimeNow; break;
        case CHMode_KeyMouse_StandUp:   pCHData->ST_ModeStartTime.KeyMouse_StandUp   = TimeNow; break;
        case CHMode_KeyMouse_Follow:    pCHData->ST_ModeStartTime.KeyMouse_Follow    = TimeNow; break;
        case CHMode_KeyMouse_SitDown:   pCHData->ST_ModeStartTime.KeyMouse_SitDown   = TimeNow; break;
        case CHMode_KeyMouse_Jump:      pCHData->ST_ModeStartTime.KeyMouse_Jump      = TimeNow; break;
        case CHMode_KeyMouse_Stair:     pCHData->ST_ModeStartTime.KeyMouse_Stair     = TimeNow; break;
        default: break;
    }
}

/**
 * @brief  底盘模式选择更新函数
 * @note   用于更新底盘模式选择相关参数
 * @param  无
 * @retval 无
 */
void CH_ChassisModeUpdate(void)
{
    Chassis_ModeChooseParameter_StructTypeDef ST_ModeChoosePara_tmp;
    //* 实现的是局部变量：ModeTemp、MC_ModePre 向全局变量
    // GEMCH_ModePre、GEMCH_Mode 赋值
    GEMCH_ModePre = GEMCH_Mode;
    ChassisStrategy_ModeChooseParaStructUpdate(&ST_ModeChoosePara_tmp);  // 更新底盘模式选择参数结构体
    //! 这个函数会在这里检测当前是否有遥控器指令指示需要切换到什么状态
    GEMCH_Mode = ChassisStrategy_ModeChoose_RCControl(ST_ModeChoosePara_tmp);  // 调用底盘模式选择函数
    ChassisStrategy_ModeStartTimeUpdate(&GSTCH_Data, GEMCH_Mode, GEMCH_ModePre);
}

// #pragma region 模式具体功能实现函数

/**
 * @brief  遥控器指令预处理函数
 * @note   实现对遥控器拨轮和摇杆这种不能自动锁定的输入的预处理
 */
void CH_RCInputPre_Process(void) {
    // === 静态变量：用于记忆历史状态 (上膛标志) ===
    static bool Latch_Roller_Up = false;
    static bool Latch_Roller_Down = false;
    static bool Latch_RJoy_Up = false;
    static bool Latch_RJoy_Down = false;

    // ============================================================
    // 拨轮处理 (Roller Logic) - 用于状态机切换
    // ============================================================
    
    // --- 上拨处理 ---
    if(IsRollerUp()) {
        Latch_Roller_Up = true;         // 上膛
        Latch_Roller_Down = false;      // 互斥
    } 
    else if(!IsRollerUp() && !IsRollerDown()) {
        if(Latch_Roller_Up) {
            GSTCH_Data.F_RollerUpLatched = true; // 击发
            Latch_Roller_Up = false;    // 复位
        } else {
            GSTCH_Data.F_RollerUpLatched = false;
        }
    } else { // 异常/下拨
        GSTCH_Data.F_RollerUpLatched = false;
        if(IsRollerDown()) 
        {Latch_Roller_Up = false;}
    }

    // --- 下拨处理 ---
    if(IsRollerDown()) {
        Latch_Roller_Down = true;
        Latch_Roller_Up = false;
    } else if(!IsRollerUp() && !IsRollerDown()) {
        if(Latch_Roller_Down) {
            GSTCH_Data.F_RollerDownLatched = true;
            Latch_Roller_Down = false;
        } else {
            GSTCH_Data.F_RollerDownLatched = false;
        }
    } else {
        GSTCH_Data.F_RollerDownLatched = false;
        if(IsRollerUp()) 
        {Latch_Roller_Down = false;}
    }

    // ============================================================
    // 右摇杆处理 (Right Joystick Logic) - 用于腿长控制
    // ============================================================
    
    // --- 上推处理 ---
    if(IsRightJoyStickUp()) {
        Latch_RJoy_Up = true;
        Latch_RJoy_Down = false;
    } else if(!IsRightJoyStickUp() && !IsRightJoyStickDown()) { // 摇杆回中
        if(Latch_RJoy_Up) {
            GSTCH_Data.F_RJoyUpLatched = true;
            Latch_RJoy_Up = false;
        } else {
            GSTCH_Data.F_RJoyUpLatched = false;
        }
    } else {
        GSTCH_Data.F_RJoyUpLatched = false;
        if(IsRightJoyStickDown()) 
        {Latch_RJoy_Up = false;}
    }

    // --- 下推处理 ---
    if(IsRightJoyStickDown()) {
        Latch_RJoy_Down = true;
        Latch_RJoy_Up = false;
    } else if(!IsRightJoyStickUp() && !IsRightJoyStickDown()) { // 摇杆回中
        if(Latch_RJoy_Down) {
            GSTCH_Data.F_RJoyDownLatched = true;
            Latch_RJoy_Down = false;
        } else {
            GSTCH_Data.F_RJoyDownLatched = false;
        }
    } else {
        GSTCH_Data.F_RJoyDownLatched = false;
        if(IsRightJoyStickUp()) 
        {Latch_RJoy_Down = false;}
    }
}

// /**
//  * @brief  RCFree/Follow模式下的腿长档位更新
//  * @note   仅在KeyMouseFollow中调用，使用右摇杆和拨轮控制腿长档位，并进行平滑过渡
//  * @param  无
//  * @retval 无
//  */
// static void _CH_LegLen_RCManualUpdate(void)
// {
//     typedef enum {
//         CH_LegLenLevel_Low = 0,
//         CH_LegLenLevel_Mid,
//         CH_LegLenLevel_High,
//     } ChassisLegLenLevel_EnumTypeDef;

//     static ChassisLegLenLevel_EnumTypeDef S_LegLenLevel = CH_LegLenLevel_Mid;
//     static float S_LegLen1Cmd = 0.0f;
//     static float S_LegLen2Cmd = 0.0f;
//     const float LegLenManualStep = 0.001f; // 单次变化步长，单位m

//     if(GEMCH_ModePre == CHMode_KeyMouse_SitDown || GEMCH_ModePre == CHMode_KeyMouse_Standby)
//     {
//         S_LegLenLevel = CH_LegLenLevel_Mid;
//         S_LegLen1Cmd = LegLenMid;
//         S_LegLen2Cmd = LegLenMid;
//     }

//     // 短按Ctrl + X：逐级升高
//     if(Key_CTRL.Key_Now == true && IS_Key_ShortClick(&Key_X))
//     {
//         if(S_LegLenLevel == CH_LegLenLevel_Low)
//         {S_LegLenLevel = CH_LegLenLevel_Mid;}
//         else if(S_LegLenLevel == CH_LegLenLevel_Mid)
//         {S_LegLenLevel = CH_LegLenLevel_High;}
//         else
//         {S_LegLenLevel = CH_LegLenLevel_High;}
//     }
//     // 短按Ctrl + Z：逐级降低
//     else if(Key_CTRL.Key_Now == true && IS_Key_ShortClick(&Key_Z))
//     {
//         if(S_LegLenLevel == CH_LegLenLevel_High)
//         {S_LegLenLevel = CH_LegLenLevel_Mid;}
//         else if(S_LegLenLevel == CH_LegLenLevel_Mid)
//         {S_LegLenLevel = CH_LegLenLevel_Low;}
//         else
//         {S_LegLenLevel = CH_LegLenLevel_Low;}
//     }

//     // 手动腿长目标输出（左右腿一致）
//     switch (S_LegLenLevel)
//     {
//         case CH_LegLenLevel_Low:
//             GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenLow;
//             GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenLow;
//             break;
//         case CH_LegLenLevel_High:
//             GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenHigh;
//             GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenHigh;
//             break;
//         case CH_LegLenLevel_Mid:
//             GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
//             GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
//             break;
//         default:
//             GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
//             GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
//             break;
//     }

//     // 平滑更新默认目标腿长，避免切换瞬间冲击
//     S_LegLen1Cmd = StepChangeValue(S_LegLen1Cmd, GST_RMCtrl.STCH_Default.LegLen1ManualDes, LegLenManualStep);
//     S_LegLen2Cmd = StepChangeValue(S_LegLen2Cmd, GST_RMCtrl.STCH_Default.LegLen2ManualDes, LegLenManualStep);

//     GST_RMCtrl.STCH_Default.LegLen1Des = S_LegLen1Cmd;
//     GST_RMCtrl.STCH_Default.LegLen2Des = S_LegLen2Cmd;
// }

/**
 * @brief  RCFree/Follow模式下的腿长档位更新
 * @note   仅在RCFree/Follow中调用，使用右摇杆控制腿长档位，并进行平滑过渡
 *          注意：由于和云台控制冲突，一般不使用
 * @param  无
 * @retval 无
 */
static void _CH_LegLen_RCManualUpdate(void)
{
    typedef enum {
        CH_LegLenLevel_Low = 0,
        CH_LegLenLevel_Mid,
        CH_LegLenLevel_High,
    } ChassisLegLenLevel_EnumTypeDef;

    static ChassisLegLenLevel_EnumTypeDef S_LegLenLevel = CH_LegLenLevel_Mid;
    static float S_LegLen1Cmd = 0.0f;
    static float S_LegLen2Cmd = 0.0f;
    const float LegLenManualStep = 0.001f; // 单次变化步长，单位m

    if(GEMCH_ModePre == CHMode_RC_Standby || GEMCH_ModePre == CHMode_RC_Free)
    {
        S_LegLenLevel = CH_LegLenLevel_Mid;
        S_LegLen1Cmd = LegLenMid;
        S_LegLen2Cmd = LegLenMid;
    }

    // 右摇杆上拨：逐级升高
    else if(GSTCH_Data.F_RJoyUpLatched == true) {
        if(S_LegLenLevel == CH_LegLenLevel_Low) {
            S_LegLenLevel = CH_LegLenLevel_Mid;
        } else if(S_LegLenLevel == CH_LegLenLevel_Mid) {
            S_LegLenLevel = CH_LegLenLevel_High;
        } else {
            S_LegLenLevel = CH_LegLenLevel_High;
        }
        GSTCH_Data.F_RJoyUpLatched = false;
    }
    // 右摇杆下拨：逐级降低
    else if(GSTCH_Data.F_RJoyDownLatched == true) {
        if(S_LegLenLevel == CH_LegLenLevel_High) {
            S_LegLenLevel = CH_LegLenLevel_Mid;
        } else if(S_LegLenLevel == CH_LegLenLevel_Mid) {
            S_LegLenLevel = CH_LegLenLevel_Low;
        } else {
            S_LegLenLevel = CH_LegLenLevel_Low;
        }
        GSTCH_Data.F_RJoyDownLatched = false;
    }

    // 手动腿长目标输出（左右腿一致）
    switch (S_LegLenLevel) {
        case CH_LegLenLevel_Low:
            GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenLow;
            GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenLow;
            break;
        case CH_LegLenLevel_High:
            GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenHigh;
            GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenHigh;
            break;
        case CH_LegLenLevel_Mid:
        default:
            GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
            GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
            break;
    }

    // 平滑更新默认目标腿长，避免切换瞬间冲击
    S_LegLen1Cmd = StepChangeValue(S_LegLen1Cmd, GST_RMCtrl.STCH_Default.LegLen1ManualDes, LegLenManualStep);
    S_LegLen2Cmd = StepChangeValue(S_LegLen2Cmd, GST_RMCtrl.STCH_Default.LegLen2ManualDes, LegLenManualStep);

    GST_RMCtrl.STCH_Default.LegLen1Des = S_LegLen1Cmd;
    GST_RMCtrl.STCH_Default.LegLen2Des = S_LegLen2Cmd;
}

/**
 * @brief  KeyMouseFollow模式下的腿长档位更新
 * @note   仅在KeyMouseFollow中调用，使用右摇杆和拨轮控制腿长档位，并进行平滑过渡
 * @param  无
 * @retval 无
 */
static void _CH_LegLen_KeyMouseManualUpdate(void)
{
    typedef enum {
        CH_LegLenLevel_Low = 0,
        CH_LegLenLevel_Mid,
        CH_LegLenLevel_High,
    } ChassisLegLenLevel_EnumTypeDef;

    static ChassisLegLenLevel_EnumTypeDef S_LegLenLevel = CH_LegLenLevel_Mid;
    static float S_LegLen1Cmd = 0.0f;
    static float S_LegLen2Cmd = 0.0f;
    const float LegLenManualStep = 0.001f; // 单次变化步长，单位m

    if(GEMCH_ModePre == CHMode_KeyMouse_SitDown || GEMCH_ModePre == CHMode_KeyMouse_Standby)
    {
        S_LegLenLevel = CH_LegLenLevel_Mid;
        S_LegLen1Cmd = LegLenMid;
        S_LegLen2Cmd = LegLenMid;
    }

    // 短按Ctrl + X：逐级升高
    if(Key_CTRL.Key_Now == true && IS_Key_ShortClick(&Key_X))
    {
        if(S_LegLenLevel == CH_LegLenLevel_Low)
        {S_LegLenLevel = CH_LegLenLevel_Mid;}
        else if(S_LegLenLevel == CH_LegLenLevel_Mid)
        {S_LegLenLevel = CH_LegLenLevel_High;}
        else
        {S_LegLenLevel = CH_LegLenLevel_High;}
    }
    // 短按Ctrl + Z：逐级降低
    else if(Key_CTRL.Key_Now == true && IS_Key_ShortClick(&Key_Z))
    {
        if(S_LegLenLevel == CH_LegLenLevel_High)
        {S_LegLenLevel = CH_LegLenLevel_Mid;}
        else if(S_LegLenLevel == CH_LegLenLevel_Mid)
        {S_LegLenLevel = CH_LegLenLevel_Low;}
        else
        {S_LegLenLevel = CH_LegLenLevel_Low;}
    }

    // 手动腿长目标输出（左右腿一致）
    switch (S_LegLenLevel)
    {
        case CH_LegLenLevel_Low:
            GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenLow;
            GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenLow;
            break;
        case CH_LegLenLevel_High:
            GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenHigh;
            GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenHigh;
            break;
        case CH_LegLenLevel_Mid:
            GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
            GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
            break;
        default:
            GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
            GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
            break;
    }

    // 平滑更新默认目标腿长，避免切换瞬间冲击
    S_LegLen1Cmd = StepChangeValue(S_LegLen1Cmd, GST_RMCtrl.STCH_Default.LegLen1ManualDes, LegLenManualStep);
    S_LegLen2Cmd = StepChangeValue(S_LegLen2Cmd, GST_RMCtrl.STCH_Default.LegLen2ManualDes, LegLenManualStep);

    GST_RMCtrl.STCH_Default.LegLen1Des = S_LegLen1Cmd;
    GST_RMCtrl.STCH_Default.LegLen2Des = S_LegLen2Cmd;
}

//#region*******************************通用模式相关控制函数***********************************/
/**
 * @brief  遥控器模式下，手动安全模式控制函数
 * @note   手动安全模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_ManualSafeMode_Ctrl(void) {
    /*清空底盘相关的Des数据、位移、控制数据*/
    Chassis_AllDesDataReset();     // 清空底盘相关的Des数据
    Chassis_DisFBClear();          // 底盘位移反馈值清零
    Chassis_RobotCtrlDataReset();  // 底盘控制数据清零

    /*判断是否要手动标定关节电机零点位置*/
    GFCH_LegCalibration = IsEnterManualCalibration();
}

/**
 * @brief  遥控器模式下，自动安全模式控制函数
 * @note   自动安全模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_AutoSafeMode_Ctrl(void) {
    /*清空底盘相关的Des数据、位移、控制数据*/
    Chassis_AllDesDataReset();     // 清空底盘相关的Des数据
    Chassis_DisFBClear();          // 底盘位移反馈值清零
    Chassis_RobotCtrlDataReset();  // 底盘控制数据清零
}
//#endregion

//#region*******************************遥控器模式相关控制函数***********************************/
/**
 * @brief  遥控器模式下，待机模式控制函数
 * @note   待机模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_RCStandbyMode_Ctrl(void)
{
    /*清空底盘相关的Des数据、位移、控制数据*/
    Chassis_AllDesDataReset();      //清空底盘相关的Des数据
    Chassis_DisFBClear();           //底盘位移反馈值清零
    Chassis_RobotCtrlDataReset();   //底盘控制数据清零
}

/**
 * @brief  遥控器模式下，起立模式控制函数
 * @note   起立模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_RCStandUpMode_Ctrl(void) {

    //! 数据预清零（如果只想在模式进入时执行一次的变量那么写在这里面）
    //* 下面的函数只会在模式切换的前四毫秒执行，会进行数据清除、参数设置等操作
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_StandUp < CHMode_AllMode_PreProcessTime) {
        //* 腿长PID目标值和系数设定
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpStandUp, 0.0f, PID_LegLen_KdStandUp);  // 左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpStandUp, 0.0f, PID_LegLen_KdStandUp);  // 右腿长PID系数Kp、Ki、Kd赋值

        //* 起立模式下腿长TD系数r设定
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rStandUp);  // 左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rStandUp);  // 右腿长TD系数r赋值

        /*清空底盘相关的一些数据*/
        Chassis_AllDesDataReset();     // 清空底盘相关的Des数据
        Chassis_DisFBClear();          // 底盘位移反馈值清零
        Chassis_RobotCtrlDataReset();  // 底盘控制数据清零
        return;
    }

    /********************任务开始一段时间后的调整，时间的单位是ms********************/
    /*配置默认可配置的控制量*/
    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMid;             //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMid;             //右腿目标腿长
    GST_RMCtrl.STCH_Default.DisDes          = 0.0f;                  //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;                  //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;                  //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;                  //目标偏航角速度
    GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静态前馈力）
    GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静态前馈力）

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
  * @brief  遥控器模式下，缓慢坐下模式控制函数
  * @note   缓慢坐下模式下的控制策略
  * @param  无
  * @retval 无
*/
void ChModeControl_RCSitDownMode_Ctrl(void) {
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_SitDown < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rSlowSitDown);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rSlowSitDown);   //右腿长TD系数r赋值
    }

    Chassis_DisFBClear();           //底盘位移反馈值清零

    /*配置默认可配置的控制量*/
    float YawAngleVelDes_Pre = GSTCH_Data.YawAngleVelDes;   //目标偏航角速度上次值
    float Leg1FFForce_Pre = GSTCH_Data.Leg1ForceDes;        //左腿前馈力上次值
    float Leg2FFForce_Pre = GSTCH_Data.Leg2ForceDes;        //右腿前馈力上次值

    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMin;        //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMin;        //右腿目标腿长
    GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;             //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = StepChangeValue(YawAngleVelDes_Pre, 0.0f, SlowSitDown_YawAngleVelBrakeStep);  //目标偏航角速度
    GST_RMCtrl.STCH_Default.Leg1FFForce     = StepChangeValue(Leg1FFForce_Pre, LegFFForce_SlowSitDown, SlowSitDown_LegFFForceDecStep);  //左腿前馈力
    GST_RMCtrl.STCH_Default.Leg2FFForce     = StepChangeValue(Leg2FFForce_Pre, LegFFForce_SlowSitDown, SlowSitDown_LegFFForceDecStep);  //右腿前馈力

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
    //! 在模式切换函数中有个腿长判断函数位置，在那里实现的腿长小于一个值就切换到坐下模式 
}

/**
 * @brief  遥控器模式下，自由模式控制函数
 * @note   自由模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_RCFreeMode_Ctrl(void) {
    /****************************该模式的前置处理****************************/
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_Free < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);   //右腿长TD系数r赋值
        
        Chassis_DisFBClear();                         //底盘位移反馈值清零

        /*配置默认可配置的控制量*/
        GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
        GST_RMCtrl.STCH_Default.VelDes          = GSTCH_Data.VelFB; //目标速度
        GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
        GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静止时的默认值）
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静止时的默认值）

        /*test：腿长设为固定的Mid*/
        GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
        GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
    }
    // Free模式下腿长档位更新
    // _CH_LegLenManualUpdate();
    
    /*************************任务开始一段时间后*************************/
    /*检测是否进入小陀螺模式*/
    static bool F_TopMode = false;
    F_TopMode = ChModeControl_FreeMode_RCControl_IsEnterTopMode(GSTCH_Data);

    /*如果不是小陀螺模式，正常进行速度、转向的调控*/
    if(F_TopMode == false)
    {ChModeControl_FreeMode_RCControl_MoveHandler(&GSTCH_Data, &GST_RMCtrl);} //移动处理函数，包括平移、转弯的速度获取

    /*如果是小陀螺模式，进行小陀螺的相关处理*/
    else if(F_TopMode == true)
    {ChModeControl_FreeMode_RCControl_TopHandler(&GSTCH_Data, &GST_RMCtrl);} //小陀螺处理函数

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
 * @brief  遥控器模式下，跟随模式控制函数
 * @note   跟随模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_RCFollowMode_Ctrl(void) {
    /****************************该模式的前置处理****************************/
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.RC_Free < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);   //右腿长TD系数r赋值
        
        Chassis_DisFBClear();                         //底盘位移反馈值清零

        /*配置默认可配置的控制量*/
        GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
        GST_RMCtrl.STCH_Default.VelDes          = GSTCH_Data.VelFB; //目标速度
        GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
        GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静止时的默认值）
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静止时的默认值）

        /*test：腿长设为固定的Mid*/
        GST_RMCtrl.STCH_Default.LegLen1ManualDes = LegLenMid;
        GST_RMCtrl.STCH_Default.LegLen2ManualDes = LegLenMid;
    }
    // Follow模式下腿长档位更新
    _CH_LegLen_RCManualUpdate();
    
    /*************************任务开始一段时间后*************************/
    /*无小陀螺模式，正常进行速度、转向的调控*/
    ChModeControl_FollowMode_RCControl_MoveHandler(&GSTCH_Data, &GST_RMCtrl); //移动处理函数，包括平移、转弯的速度获取

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}
//#endregion

//#region*******************************键鼠模式相关控制函数***********************************/
/**
 * @brief  键鼠模式下，待机模式控制函数
 * @note   待机模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_KeyMouseStandbyMode_Ctrl(void)
{
    /*清空底盘相关的Des数据、位移、控制数据*/
    Chassis_AllDesDataReset();      //清空底盘相关的Des数据
    Chassis_DisFBClear();           //底盘位移反馈值清零
    Chassis_RobotCtrlDataReset();   //底盘控制数据清零

    // 腿长目标值更新
    _CH_LegLen_KeyMouseManualUpdate();
}

/**
 * @brief  键鼠模式下，起立模式控制函数
 * @note   起立模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_KeyMouseStandUpMode_Ctrl(void) {

    //! 数据预清零（如果只想在模式进入时执行一次的变量那么写在这里面）
    //* 下面的函数只会在模式切换的前四毫秒执行，会进行数据清除、参数设置等操作
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.KeyMouse_StandUp < CHMode_AllMode_PreProcessTime) {
        //* 腿长PID目标值和系数设定
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpStandUp, 0.0f, PID_LegLen_KdStandUp);  // 左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpStandUp, 0.0f, PID_LegLen_KdStandUp);  // 右腿长PID系数Kp、Ki、Kd赋值

        //* 起立模式下腿长TD系数r设定
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rStandUp);  // 左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rStandUp);  // 右腿长TD系数r赋值

        // 腿长目标值更新
        _CH_LegLen_KeyMouseManualUpdate();

        /*清空底盘相关的一些数据*/
        Chassis_AllDesDataReset();     // 清空底盘相关的Des数据
        Chassis_DisFBClear();          // 底盘位移反馈值清零
        Chassis_RobotCtrlDataReset();  // 底盘控制数据清零
        return;
    }

    /********************任务开始一段时间后的调整，时间的单位是ms********************/
    /*配置默认可配置的控制量*/
    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMid;             //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMid;             //右腿目标腿长
    GST_RMCtrl.STCH_Default.DisDes          = 0.0f;                  //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;                  //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;                  //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;                  //目标偏航角速度
    GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静态前馈力）
    GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静态前馈力）

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
  * @brief  键鼠模式下，缓慢坐下模式控制函数
  * @note   缓慢坐下模式下的控制策略
  * @param  无
  * @retval 无
*/
void ChModeControl_KeyMouseSitDownMode_Ctrl(void) {
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.KeyMouse_SitDown < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rSlowSitDown);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rSlowSitDown);   //右腿长TD系数r赋值

        // 腿长目标值更新
        _CH_LegLen_KeyMouseManualUpdate();
    }

    Chassis_DisFBClear();           //底盘位移反馈值清零

    /*配置默认可配置的控制量*/
    float YawAngleVelDes_Pre = GSTCH_Data.YawAngleVelDes;   //目标偏航角速度上次值
    float Leg1FFForce_Pre = GSTCH_Data.Leg1ForceDes;        //左腿前馈力上次值
    float Leg2FFForce_Pre = GSTCH_Data.Leg2ForceDes;        //右腿前馈力上次值

    GST_RMCtrl.STCH_Default.LegLen1Des      = LegLenMin;        //左腿目标腿长
    GST_RMCtrl.STCH_Default.LegLen2Des      = LegLenMin;        //右腿目标腿长
    GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = 0.0f;             //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = StepChangeValue(YawAngleVelDes_Pre, 0.0f, SlowSitDown_YawAngleVelBrakeStep);  //目标偏航角速度
    GST_RMCtrl.STCH_Default.Leg1FFForce     = StepChangeValue(Leg1FFForce_Pre, LegFFForce_SlowSitDown, SlowSitDown_LegFFForceDecStep);  //左腿前馈力
    GST_RMCtrl.STCH_Default.Leg2FFForce     = StepChangeValue(Leg2FFForce_Pre, LegFFForce_SlowSitDown, SlowSitDown_LegFFForceDecStep);  //右腿前馈力

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
    //! 在模式切换函数中有个腿长判断函数位置，在那里实现的腿长小于一个值就切换到坐下模式 
}

/**
 * @brief  键鼠模式下，跟随模式控制函数
 * @note   跟随模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_KeyMouseFollowMode_Ctrl(void) {
    /****************************该模式的前置处理****************************/
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.KeyMouse_Follow < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);   //右腿长TD系数r赋值

        Chassis_DisFBClear();                         //底盘位移反馈值清零

        /*配置默认可配置的控制量*/
        GST_RMCtrl.STCH_Default.DisDes          = 0.0f;             //目标位移
        GST_RMCtrl.STCH_Default.VelDes          = GSTCH_Data.VelFB;             //目标速度
        GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
        GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力（静止时的默认值）
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力（静止时的默认值）

    }
    // Follow模式下腿长档位更新
    _CH_LegLen_KeyMouseManualUpdate();
    
    /*************************任务开始一段时间后*************************/
    /*检测是否进入小陀螺模式*/
    static bool F_TopMode = false;
    F_TopMode = ChModeControl_FollowMode_KeyMouseControl_IsEnterTopMode(GSTCH_Data);

    /*如果不是小陀螺模式，正常进行速度、转向的调控*/
    if(F_TopMode == false)
    {ChModeControl_FollowMode_KeyMouseControl_MoveHandler(&GSTCH_Data, &GST_RMCtrl);} //移动处理函数，包括平移、转弯的速度获取

    /*如果是小陀螺模式，进行小陀螺的相关处理*/
    else if(F_TopMode == true)
    {ChModeControl_FollowMode_KeyMouseControl_TopHandler(&GSTCH_Data, &GST_RMCtrl);} //小陀螺处理函数

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}


/**
 * @brief  遥控器模式或键鼠模式下，离地模式控制函数
 * @note   离地模式下的控制策略
 * @param  无
 * @retval 无
 */
void ChModeControl_OffGroundMode_Ctrl(void)
{
    // TODO 腿部前馈力要不要变？PID、TD参数要不要变？腿长是否需要考虑一下变化？
    
    /****************************该模式的前置处理****************************/
    if(RunTimeGet() - GSTCH_Data.ST_ModeStartTime.OffGround < CHMode_AllMode_PreProcessTime)
    {
        /*腿长PID系数赋值*/
        PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //左腿长PID系数Kp、Ki、Kd赋值
        PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm); //右腿长PID系数Kp、Ki、Kd赋值

        /*腿长TD系数赋值*/
        TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);   //左腿长TD系数r赋值
        TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);   //右腿长TD系数r赋值
        
        /*配置默认可配置的控制量*/
        GST_RMCtrl.STCH_Default.Leg1FFForce     = LegFFForce_Gravity_1;  //左腿前馈力
        GST_RMCtrl.STCH_Default.Leg2FFForce     = LegFFForce_Gravity_2;  //右腿前馈力
    };

    /*************************任务开始一段时间后*************************/
    GST_RMCtrl.STCH_Default.DisDes          = GSTCH_Data.DisFB; //目标位移
    GST_RMCtrl.STCH_Default.VelDes          = GSTCH_Data.VelFB; //目标速度
    GST_RMCtrl.STCH_Default.YawDeltaDes     = 0.0f;             //目标偏航角度
    GST_RMCtrl.STCH_Default.YawAngleVelDes  = 0.0f;             //目标偏航角速度

    /*左腿目标腿长*/
    if(GSTCH_Data.F_OffGround1 == true)
    {GST_RMCtrl.STCH_Default.LegLen1Des  = LegLenOffGround;}
    else
    {GST_RMCtrl.STCH_Default.LegLen1Des  = LegLenMid;}

    /*右腿目标腿长*/
    if(GSTCH_Data.F_OffGround2 == true)
    {GST_RMCtrl.STCH_Default.LegLen2Des  = LegLenOffGround;}
    else
    {GST_RMCtrl.STCH_Default.LegLen2Des  = LegLenMid;}

    /*********************从控制结构体中获取数据，进行相关解算*************************/
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
 * @brief  键鼠模式下，跳跃模式控制函数
 * @note   跳跃模式状态机实现，包含6个阶段：
 *         1. Wait:      等待阶段，进入跳跃后的短暂准备
 *         2. Compress:  压缩蓄力，下蹲储能
 *         3. Takeoff:   起跳伸展，快速伸腿获得初速度
 *         4. Retract:   收腿阶段，离地后快速收缩腿部
 *         5. AirFree:   空中自由，保持姿态等待落地
 *         6. Landing:   着陆缓冲，触地瞬间缓冲冲击
 * @param  无
 * @retval 无
 */
void ChModeControl_KeyMouseJumpMode_Ctrl(void)
{
    // static ChassisJumpPhase_EnumTypeDef JumpPhase = CH_JumpPhase_Wait;

    // 获取当前时间
    uint32_t TimeNow = RunTimeGet();
    uint32_t ModeStartTime = GSTCH_Data.ST_ModeStartTime.KeyMouse_Jump;

    // 获取腿长平均值
    float LegLenAvg = (GSTCH_Data.LegLen1FB + GSTCH_Data.LegLen2FB) / 2.0f;
    float LegLen1Err = fabs(GSTCH_Data.LegLen1FB - LegLenJumpCompressTarget);
    float LegLen2Err = fabs(GSTCH_Data.LegLen2FB - LegLenJumpCompressTarget);

    // 离地标志（任一侧离地即为离地状态）
    bool IsOffGround = GSTCH_Data.F_OffGround1 || GSTCH_Data.F_OffGround2;

    // ========== 前置处理：进入跳跃后的短暂等待期 ==========
    if(TimeNow - ModeStartTime < CHMode_AllMode_PreProcessTime)
    {JumpPhase = CH_JumpPhase_Wait;}

    // ========== 状态机主逻辑 ==========
    switch (JumpPhase)
    {
        // -------------------- Wait: 等待阶段 --------------------
        case CH_JumpPhase_Wait:
            // 等待期结束，进入压缩蓄力阶段
            if(TimeNow - ModeStartTime >= CHMode_AllMode_PreProcessTime)
            {JumpPhase = CH_JumpPhase_Compress;}

            // 保持低腿长准备起跳，前馈力为重力补偿
            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenLow;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenLow;
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2;

            Chassis_DisFBClear();
            break;

        // -------------------- Compress: 压缩蓄力阶段 --------------------
        case CH_JumpPhase_Compress:
        {
            // 切换条件：双腿都接近压缩目标（误差<10mm 或 实际<目标）
            bool Leg1Compressed = (LegLen1Err < 0.025f) || (GSTCH_Data.LegLen1FB < LegLenJumpCompressTarget);
            bool Leg2Compressed = (LegLen2Err < 0.025f) || (GSTCH_Data.LegLen2FB < LegLenJumpCompressTarget);

            if(Leg1Compressed && Leg2Compressed)
            {JumpPhase = CH_JumpPhase_Takeoff;}

            // 压缩阶段：保持蓄力腿长，较大PID快速响应
            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Compress, 0.0f, PID_LegLen_KdJump_Compress);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Compress, 0.0f, PID_LegLen_KdJump_Compress);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpCompressTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpCompressTarget;
            // 压缩阶段前馈力：重力+额外下压力（可选）
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2;
            break;
        }

        // -------------------- Takeoff: 起跳伸展阶段 --------------------
        case CH_JumpPhase_Takeoff:
        {
            // 切换条件：腿长达到阈值 或 检测到离地
            if((LegLenAvg >= LegLenJumpRetractThreshold) || IsOffGround)
            {JumpPhase = CH_JumpPhase_Retract;}

            // 起跳阶段：分段PID控制
            float TakeoffErr = LegLenJumpTarget - LegLenAvg;
            if(TakeoffErr > 0.030f) // 远离目标：较大PID + 大前馈（快速伸展）
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Takeoff, 0.0f, PID_LegLen_KdJump_Takeoff);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Takeoff, 0.0f, PID_LegLen_KdJump_Takeoff);
            }
            else // 接近目标：减小前馈（缓冲）
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Takeoff * 0.8f, 0.0f, PID_LegLen_KdJump_Takeoff * 0.7f);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Takeoff * 0.8f, 0.0f, PID_LegLen_KdJump_Takeoff * 0.7f);
            }

            // 起跳阶段不加TD跟踪，追求快速响应

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpTarget;
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Jump;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Jump;
            break;
        }

        // -------------------- Retract: 收腿阶段 --------------------
        case CH_JumpPhase_Retract:
        {
            // 收腿完成判断
            float RetractErr = fabs(LegLenAvg - LegLenJumpRetractTarget);
            bool RetractDone = (RetractErr < 0.020f);

            // 切换条件：收腿完成且在空中 -> 空中自由
            //          收腿完成且已触地 -> 着陆缓冲
            if(RetractDone)
            {
                if(IsOffGround)
                {JumpPhase = CH_JumpPhase_AirFree;}
                else
                {JumpPhase = CH_JumpPhase_Landing;}
            }

            // 收腿阶段：较大PID快速收缩
            float ErrThreshold = 0.030f;  // 分段阈值
            if(RetractErr > ErrThreshold) // 远离目标：大PID + 大前馈（快速收缩）
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Retract, 0.0f, PID_LegLen_KdJump_Retract);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Retract, 0.0f, PID_LegLen_KdJump_Retract);
            }
            else // 接近目标：减小参数
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Retract * 0.8f, 0.0f, PID_LegLen_KdJump_Retract * 0.8f);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Retract * 0.8f, 0.0f, PID_LegLen_KdJump_Retract * 0.8f);
            }

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.Leg1FFForce = - LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = - LegFFForce_Gravity_2;
            break;
        }

        // -------------------- AirFree: 空中自由阶段 --------------------
        case CH_JumpPhase_AirFree:
        {
            // 切换条件：检测到触地（支持力恢复）
            if(!IsOffGround)
            {JumpPhase = CH_JumpPhase_Landing;}

            // 空中阶段：保持低腿长，较小PID保持柔顺
            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_AirFree, 0.0f, PID_LegLen_KdJump_AirFree);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_AirFree, 0.0f, PID_LegLen_KdJump_AirFree);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpRetractTarget;
            // 空中前馈力：较小，仅维持姿态
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1 * 0.5f;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2 * 0.5f;
            break;
        }

        // -------------------- Landing: 着陆缓冲阶段 --------------------
        case CH_JumpPhase_Landing:
        {
            // 着陆阶段持续一定时间后自动退出（由模式切换逻辑处理）
            // 这里只负责缓冲控制

            // 着陆阶段：较大PID抵抗冲击
            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Landing, 0.0f, PID_LegLen_KdJump_Landing);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Landing, 0.0f, PID_LegLen_KdJump_Landing);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2;
            break;
        }

        default:
            JumpPhase = CH_JumpPhase_Wait;
            break;
    }

    // ========== 更新全局标志位 ==========
    GSTCH_Data.F_JumpTakeoff = (JumpPhase == CH_JumpPhase_Takeoff);
    GSTCH_Data.F_JumpRetract = (JumpPhase == CH_JumpPhase_Retract);
    GSTCH_Data.F_JumpLanding = (JumpPhase == CH_JumpPhase_Landing);

    // ========== 通用控制量设置 ==========
    GST_RMCtrl.STCH_Default.DisDes = GSTCH_Data.DisFB;
    GST_RMCtrl.STCH_Default.VelDes = GSTCH_Data.VelFB;
    GST_RMCtrl.STCH_Default.YawDeltaDes = 0.0f;
    GST_RMCtrl.STCH_Default.YawAngleVelDes = 0.0f;

    // 调用运动处理函数
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
 * @brief  遥控器模式下，跳跃模式控制函数
 * @note   跳跃模式状态机实现，包含6个阶段：
 *         1. Wait:      等待阶段，进入跳跃后的短暂准备
 *         2. Compress:  压缩蓄力，下蹲储能
 *         3. Takeoff:   起跳伸展，快速伸腿获得初速度
 *         4. Retract:   收腿阶段，离地后快速收缩腿部
 *         5. AirFree:   空中自由，保持姿态等待落地
 *         6. Landing:   着陆缓冲，触地瞬间缓冲冲击
 * @param  无
 * @retval 无
 */
void ChModeControl_RCJumpMode_Ctrl(void)
{
    // static ChassisJumpPhase_EnumTypeDef JumpPhase = CH_JumpPhase_Wait;

    // 获取当前时间
    uint32_t TimeNow = RunTimeGet();
    uint32_t ModeStartTime = GSTCH_Data.ST_ModeStartTime.RC_Jump;

    // 获取腿长平均值
    float LegLenAvg = (GSTCH_Data.LegLen1FB + GSTCH_Data.LegLen2FB) / 2.0f;
    float LegLen1Err = fabs(GSTCH_Data.LegLen1FB - LegLenJumpCompressTarget);
    float LegLen2Err = fabs(GSTCH_Data.LegLen2FB - LegLenJumpCompressTarget);

    // 离地标志（任一侧离地即为离地状态）
    bool IsOffGround = GSTCH_Data.F_OffGround1 || GSTCH_Data.F_OffGround2;

    // ========== 前置处理：进入跳跃后的短暂等待期 ==========
    if(TimeNow - ModeStartTime < CHMode_AllMode_PreProcessTime)
    {JumpPhase = CH_JumpPhase_Wait;}

    // ========== 状态机主逻辑 ==========
    switch (JumpPhase)
    {
        // -------------------- Wait: 等待阶段 --------------------
        case CH_JumpPhase_Wait:
            // 等待期结束，进入压缩蓄力阶段
            if(TimeNow - ModeStartTime >= CHMode_AllMode_PreProcessTime)
            {JumpPhase = CH_JumpPhase_Compress;}

            // 保持低腿长准备起跳，前馈力为重力补偿
            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenLow;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenLow;
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2;

            Chassis_DisFBClear();
            break;

        // -------------------- Compress: 压缩蓄力阶段 --------------------
        case CH_JumpPhase_Compress:
        {
            // 切换条件：双腿都接近压缩目标（误差<10mm 或 实际<目标）
            bool Leg1Compressed = (LegLen1Err < 0.025f) || (GSTCH_Data.LegLen1FB < LegLenJumpCompressTarget);
            bool Leg2Compressed = (LegLen2Err < 0.025f) || (GSTCH_Data.LegLen2FB < LegLenJumpCompressTarget);

            if(Leg1Compressed && Leg2Compressed)
            {JumpPhase = CH_JumpPhase_Takeoff;}

            // 压缩阶段：保持蓄力腿长，较大PID快速响应
            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Compress, 0.0f, PID_LegLen_KdJump_Compress);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Compress, 0.0f, PID_LegLen_KdJump_Compress);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpCompressTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpCompressTarget;
            // 压缩阶段前馈力：重力+额外下压力（可选）
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2;
            break;
        }

        // -------------------- Takeoff: 起跳伸展阶段 --------------------
        case CH_JumpPhase_Takeoff:
        {
            // 切换条件：腿长达到阈值 或 检测到离地
            if((LegLenAvg >= LegLenJumpRetractThreshold) || IsOffGround)
            {JumpPhase = CH_JumpPhase_Retract;}

            // 起跳阶段：分段PID控制
            float TakeoffErr = LegLenJumpTarget - LegLenAvg;
            if(TakeoffErr > 0.030f) // 远离目标：较大PID + 大前馈（快速伸展）
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Takeoff, 0.0f, PID_LegLen_KdJump_Takeoff);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Takeoff, 0.0f, PID_LegLen_KdJump_Takeoff);
            }
            else // 接近目标：减小前馈（缓冲）
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Takeoff * 0.8f, 0.0f, PID_LegLen_KdJump_Takeoff * 0.7f);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Takeoff * 0.8f, 0.0f, PID_LegLen_KdJump_Takeoff * 0.7f);
            }

            // 起跳阶段不加TD跟踪，追求快速响应

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpTarget;
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Jump;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Jump;
            break;
        }

        // -------------------- Retract: 收腿阶段 --------------------
        case CH_JumpPhase_Retract:
        {
            // 收腿完成判断
            float RetractErr = fabs(LegLenAvg - LegLenJumpRetractTarget);
            bool RetractDone = (RetractErr < 0.020f);

            // 切换条件：收腿完成且在空中 -> 空中自由
            //          收腿完成且已触地 -> 着陆缓冲
            if(RetractDone)
            {
                if(IsOffGround)
                {JumpPhase = CH_JumpPhase_AirFree;}
                else
                {JumpPhase = CH_JumpPhase_Landing;}
            }

            // 收腿阶段：较大PID快速收缩
            float ErrThreshold = 0.030f;  // 分段阈值
            if(RetractErr > ErrThreshold) // 远离目标：大PID + 大前馈（快速收缩）
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Retract, 0.0f, PID_LegLen_KdJump_Retract);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Retract, 0.0f, PID_LegLen_KdJump_Retract);
            }
            else // 接近目标：减小参数
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Retract * 0.8f, 0.0f, PID_LegLen_KdJump_Retract * 0.8f);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Retract * 0.8f, 0.0f, PID_LegLen_KdJump_Retract * 0.8f);
            }

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.Leg1FFForce = - LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = - LegFFForce_Gravity_2;
            break;
        }

        // -------------------- AirFree: 空中自由阶段 --------------------
        case CH_JumpPhase_AirFree:
        {
            // 切换条件：检测到触地（支持力恢复）
            if(!IsOffGround)
            {JumpPhase = CH_JumpPhase_Landing;}

            // 空中阶段：保持低腿长，较小PID保持柔顺
            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_AirFree, 0.0f, PID_LegLen_KdJump_AirFree);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_AirFree, 0.0f, PID_LegLen_KdJump_AirFree);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpRetractTarget;
            // 空中前馈力：较小，仅维持姿态
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1 * 0.5f;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2 * 0.5f;
            break;
        }

        // -------------------- Landing: 着陆缓冲阶段 --------------------
        case CH_JumpPhase_Landing:
        {
            // 着陆阶段持续一定时间后自动退出（由模式切换逻辑处理）
            // 这里只负责缓冲控制

            // 着陆阶段：较大PID抵抗冲击
            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Landing, 0.0f, PID_LegLen_KdJump_Landing);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Landing, 0.0f, PID_LegLen_KdJump_Landing);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenJumpRetractTarget;
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2;
            break;
        }
        default:
            JumpPhase = CH_JumpPhase_Wait;
            break;
    }

    // ========== 更新全局标志位 ==========
    GSTCH_Data.F_JumpTakeoff = (JumpPhase == CH_JumpPhase_Takeoff);
    GSTCH_Data.F_JumpRetract = (JumpPhase == CH_JumpPhase_Retract);
    GSTCH_Data.F_JumpLanding = (JumpPhase == CH_JumpPhase_Landing);

    // ========== 通用控制量设置 ==========
    GST_RMCtrl.STCH_Default.DisDes = GSTCH_Data.DisFB;
    GST_RMCtrl.STCH_Default.VelDes = GSTCH_Data.VelFB;
    GST_RMCtrl.STCH_Default.YawDeltaDes = 0.0f;
    GST_RMCtrl.STCH_Default.YawAngleVelDes = 0.0f;

    // 调用运动处理函数
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

/**
 * @brief  键鼠模式下，磕台阶模式控制函数
 * @note   跳跃模式状态机实现，包含6个阶段：
 *         1. Wait:      等待阶段，进入跳跃后的短暂准备
 *         2. Compress:  压缩蓄力，下蹲储能
 *         3. Takeoff:   起跳伸展，快速伸腿获得初速度
 *         4. Retract:   收腿阶段，离地后快速收缩腿部
 *         5. AirFree:   空中自由，保持姿态等待落地
 *         6. Landing:   着陆缓冲，触地瞬间缓冲冲击
 * @param  无
 * @retval 无
 */
void ChModeControl_KeyMouseStairMode_Ctrl(void)
{
	static uint16_t Landing_Time_cnt = 0;
    float Retract_ErrThreshold = 0.150f;         // 分段阈值，单位m
    float AccXFB = GstCH_IMU2.ST_Rx.AccX;        //机体前进方向加速度，单位m/s^2
    float PitchFB = GstCH_IMU2.ST_Rx.PitchAngle; //机体俯仰角度反馈，单位度
    float LegLenFB = (GSTCH_Data.LegLen1FB+GSTCH_Data.LegLen2FB)/2.0f; //腿长平均值反馈，单位m

    //磕台阶完成标志位置0
    GSTCH_Data.F_StairFinished = 0;

    switch(StairPhase)
    {
        /*1. Boost: 机体加速阶段*/
        case CH_StairPhase_Boost:
            // 加速阶段：若向前加速并且Pitch角度正常，则进入准备阶段
            if(MyAbsf(AccXFB) > 35.0f && MyAbsf(PitchFB - ChassisPitchAngleZP) < 6.0f)
            {StairPhase = CH_StairPhase_Prepare;}

            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);
            
            // 目标腿长
            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenStairHigh;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenStairHigh;
            // 前馈力
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2;
            break;
        
        /*2. Prepare: 磕台阶准备阶段*/
        case CH_StairPhase_Prepare:
            // 准备阶段：若Pitch角度差值过大，则认为磕到台阶，进入迅速收腿阶段
            // TODO：老代码加入了时间判断，若准备阶段时间大于2s则自动收腿，感觉不是很合适，目前暂时不用
            if(MyAbsf(PitchFB - ChassisPitchAngleZP) >= 8.0f) //Crashing_Time_cnt >= 2000 || fabs(G_ST_IMU2.Receive.pitch_angle - Med_Angle_Norm)>=8.0f
            {StairPhase = CH_StairPhase_Retract;}             //Crashing_Time_cnt = 0;// Crashing_Time_cnt++;

            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpNorm, 0.0f, PID_LegLen_KdNorm);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);
            
            // 目标腿长
            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenStairHigh;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenStairHigh;
            // 前馈力
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2;
            break;

        /*3. Retract: 迅速收腿阶段*/
        case CH_StairPhase_Retract:
            // 切换条件：腿长接近目标，认为已经着地，进入着陆模式
            if(MyAbsf(LegLenFB - GST_RMCtrl.STCH_Default.LegLen1Des) < 0.08f)
            {StairPhase = CH_StairPhase_Landing;}
            // TODO：老代码另一个状态切换逻辑，看不懂
            // if(fabs(LengthLeftReal-LengthAvgDes)<80.0f && fabs(LengthRightReal-LengthAvgDes)<80.0f && !OffGround_Flag_Pre && OffGround_Flag)
            //     g_chassis_stair_mode = Stair_Landing_Mode;
        
        // 收腿阶段：较大PID快速收缩
            if(MyAbsf(LegLenFB - GST_RMCtrl.STCH_Default.LegLen1Des*MM2M) > Retract_ErrThreshold) // 远离目标：大PID + 大前馈（快速收缩）
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Retract, 0.0f, PID_LegLen_KdJump_Retract);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Retract, 0.0f, PID_LegLen_KdJump_Retract);
            }
            else // 接近目标：减小参数
            {
                PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Retract * 0.8f, 0.0f, PID_LegLen_KdJump_Retract * 0.8f);
                PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Retract * 0.8f, 0.0f, PID_LegLen_KdJump_Retract * 0.8f);
            }

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenStairRetract;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenStairRetract;
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Stair_Retract;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Stair_Retract;
            break;

        /*4. Landing: 着陆阶段*/
        case CH_StairPhase_Landing:
            // 切换条件：持续一定时间后退出磕台阶模式
            Landing_Time_cnt++;
            if(Landing_Time_cnt > 500)
            {
                Landing_Time_cnt = 0;
                GSTCH_Data.F_StairFinished = 1;
            }

            PID_SetKpKiKd(&GstCH_LegLen1PID, PID_LegLen_KpJump_Landing, 0.0f, PID_LegLen_KdJump_Landing);
            PID_SetKpKiKd(&GstCH_LegLen2PID, PID_LegLen_KpJump_Landing, 0.0f, PID_LegLen_KdJump_Landing);

            TD_Setr(&GstCH_LegLen1TD, TD_LegLen_rNorm);
            TD_Setr(&GstCH_LegLen2TD, TD_LegLen_rNorm);

            GST_RMCtrl.STCH_Default.LegLen1Des = LegLenMid;
            GST_RMCtrl.STCH_Default.LegLen2Des = LegLenMid;
            GST_RMCtrl.STCH_Default.Leg1FFForce = LegFFForce_Gravity_1;
            GST_RMCtrl.STCH_Default.Leg2FFForce = LegFFForce_Gravity_2;

            break;
        default:
            StairPhase = CH_StairPhase_Boost;
            break;
    }
    
    // ========== 更新全局标志位 ==========
    // GSTCH_Data.F_StairFinished = (StairPhase == CH_StairPhase_Landing);

    // ========== 通用控制量设置 ==========
    ChModeControl_FollowMode_KeyMouseControl_MoveHandler(&GSTCH_Data, &GST_RMCtrl);

    // 调用运动处理函数
    CH_MotionUpdateAndProcess(GST_RMCtrl);
}

// #pragma endregion

//* 模式控制最终执行函数。根据当前模式变量的变量值执行对应的模式具体功能实现函数
/**
 * @brief  遥控器模式下，底盘模式控制函数
 * @note   根据当前底盘的工作状态，调用相应的控制策略函数
 * @param  ModeNow：ChassisMode_EnumTypeDef类型的枚举值，当前底盘的工作状态
 * @retval 无
 */
void ChassisModeControl_Ctrl(ChassisMode_EnumTypeDef ModeNow)
{
    switch (ModeNow)
    {
        /*通用手动安全模式*/
        case CHMode_ManualSafe:ChModeControl_ManualSafeMode_Ctrl();               break;
        /*通用自动安全模式*/
        case CHMode_AutoSafe:ChModeControl_AutoSafeMode_Ctrl();                   break;
        /*通用离地模式*/
        case CHMode_OffGround:ChModeControl_OffGroundMode_Ctrl();                 break;

        /*RC待机模式*/
        case CHMode_RC_Standby:ChModeControl_RCStandbyMode_Ctrl();                break;
        /*RC起立模式*/
        case CHMode_RC_StandUp:ChModeControl_RCStandUpMode_Ctrl();                break;
        /*RC底盘自由模式*/
        case CHMode_RC_Free:ChModeControl_RCFreeMode_Ctrl();                      break;
        /*RC底盘跟随模式*/
        case CHMode_RC_Follow:ChModeControl_RCFollowMode_Ctrl();                  break;
        /*RC底盘缓慢坐下模式*/
        case CHMode_RC_SitDown:ChModeControl_RCSitDownMode_Ctrl();                break;

        /*键鼠待机模式*/
        case CHMode_KeyMouse_Standby:ChModeControl_KeyMouseStandbyMode_Ctrl();    break;
        /*键鼠起立模式*/
        case CHMode_KeyMouse_StandUp:ChModeControl_KeyMouseStandUpMode_Ctrl();    break;
        /*键鼠跟随模式*/
        case CHMode_KeyMouse_Follow:ChModeControl_KeyMouseFollowMode_Ctrl();      break;
        /*键鼠缓慢坐下模式*/
        case CHMode_KeyMouse_SitDown:ChModeControl_KeyMouseSitDownMode_Ctrl();    break;

        /*RC跳跃模式*/
        case CHMode_RC_Jump:ChModeControl_RCJumpMode_Ctrl();                      break;
        /*键鼠跳跃模式*/
        case CHMode_KeyMouse_Jump:ChModeControl_KeyMouseJumpMode_Ctrl();          break;
        /*键鼠磕台阶模式*/
        case CHMode_KeyMouse_Stair:ChModeControl_KeyMouseStairMode_Ctrl();        break;

        default:
            ChModeControl_ManualSafeMode_Ctrl();                                  break;
    }
}
