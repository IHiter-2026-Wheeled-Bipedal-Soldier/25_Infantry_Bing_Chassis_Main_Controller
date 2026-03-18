/**
  ******************************************************************************
  * @file    IRQ_Handler_Config.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.10
  * @brief   各个STM32中断服务函数的配置，主要是CAN、USART、WWDG的中断服务函数
  ******************************************************************************
*/

/****************************头文件引用****************************/
#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdio.h>
#include "USART_Communication.h"
#include "CAN_Communication.h"
#include "WWDG_Config.h"
#include "GlobalDeclare_General.h"
#include "GlobalDeclare_Chassis.h"
#include "GlobalDeclare_Shooter.h"
#include "SuperCapacitor.h"
#include "Gimbal_Task.h"

/**
  * @brief  CAN1发送中断服务函数
  * @param  无
  * @retval 无
  */
void CAN1_TX_IRQHandler(void)
{
    /****************检测发送邮箱空****************/
    if(CAN_GetITStatus(CAN1,CAN_IT_TME) == SET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
    }
}

/**
  * @brief  CAN1接收中断服务函数
  * @note   利用不同ID号来处理数据
  * @param  无
  * @retval 无
  */
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg CAN_RxMsg;
    
    float Yaw_AngleFB_temp = 0.0f; //临时变量，存储Yaw角度反馈

    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &CAN_RxMsg);

        /****************根据不同ID号来处理数据****************/
        //Lucky:下面这个switch可以试着封装一下
        switch(CAN_RxMsg.StdId)
        {
            /* ---- 云台电机反馈 ---- */
            case 0x205: //Pitch ID:1
				// Abs_Encoder_Process(&g_stPitchEncoder, Get_Encoder_Number(&CAN_RxMsg));
				// GSTGM_Data.PitchPosFB = ( g_stPitchEncoder.siSumValue - PitchEncoderZero_Norm ) / 8192.0f * 360.0f;
				// Pitch_Angle = Angle_Inf_To_180( Pitch_Angle );
				GST_SystemMonitor.PitchMotorRx_cnt++;
				break;
            case 0x20A:  //Yaw电机 ID:6
                //TODO：老代码有Benjamin_Position_Revise，即手动标定偏航零点
				Abs_Encoder_Process(&g_stYawEncoder, (CAN_RxMsg.Data[0]<<8 | CAN_RxMsg.Data[1]));
				Yaw_AngleFB_temp = (g_stYawEncoder.siSumValue)/8192.0f*360.0f; //Yaw反馈角度，单位度
                GstCH_FollowMode_Paras.RelativeYawAngle = Angle_Inf_To_180(Yaw_AngleFB_temp - GstCH_FollowMode_Paras.YawZeroPoint);
				// if( PRESSED_SHIFT && PRESSED_F && ( Vision_State_Now & Vision_Cmd_Mask ) == Vision_Disable )
				// Benjamin_Position_Revise = Angle_Inf_To_180( Yaw_Angle - YawEncoderZero_Norm);
				GST_SystemMonitor.YawMotorRx_cnt++;
				break;
            
            /* ---- 发射电机反馈 ---- */
			case 0x206:	//右摩擦轮 ID:6
				Abs_Encoder_Process(&g_stCMREncoder, (CAN_RxMsg.Data[0]<<8 | CAN_RxMsg.Data[1]));
				smcR.fpFB=Get_Speed(&CAN_RxMsg);
				// CM1_TEMP=Get_Temperature(&CAN_RxMsg);
				GST_SystemMonitor.FrictionMotor2Rx_cnt ++;
				break;
			case 0x207:	//左摩擦轮 ID:7
				Abs_Encoder_Process(&g_stCMLEncoder, (CAN_RxMsg.Data[0]<<8 | CAN_RxMsg.Data[1]));
				smcL.fpFB=Get_Speed(&CAN_RxMsg);
				// CM2_TEMP=Get_Temperature(&CAN_RxMsg);
				GST_SystemMonitor.FrictionMotor1Rx_cnt++;
				break;
			case 0x208:	//拨弹电机2006 ID:8
				Abs_Encoder_Process(&g_stShooterEncoder, (CAN_RxMsg.Data[0]<<8 | CAN_RxMsg.Data[1]));
				GstSH_Paras.SupplyPellet_PosFB = g_stShooterEncoder.siSumValue*360.0f/8192.0f/36.0f; //拨弹电机位置反馈，单位度（注意是减速箱输出端的角度）
				GstSH_Paras.SupplyPellet_VelFB = ((int16_t)(CAN_RxMsg.Data[2]<<8 | CAN_RxMsg.Data[3]))*6.0f/36.0f; //拨弹电机速度反馈，单位度/s（注意是减速箱输出端的角速度）
				GstSH_Paras.SupplyPellet_TorqueFB = CAN_RxMsg.Data[4]<<8 | CAN_RxMsg.Data[5]; //拨弹电机力矩反馈值，单位Nm（注意不是减速箱输出端力矩）
				GST_SystemMonitor.SupplyPelletRx_cnt++;
				break;
			
            /* ---- 超级电容反馈 ---- */
            case 0x400:
                // 电容电压: Data[0-1], uint16, /100 -> V
                GSTCH_Capacitor.CAP_Vol  = ((CAN_RxMsg.Data[0] << 8) | CAN_RxMsg.Data[1]) / 100.0f;
                // 电池输出功率: Data[2-3], 需要根据实际协议确认
                GSTCH_Capacitor.Pow_Out = ((CAN_RxMsg.Data[2] << 8) | CAN_RxMsg.Data[3]) / 100.0f;
                // 电容输出电压: Data[2-3] 或 Data[4-5], 需确认协议
                GSTCH_Capacitor.Volt_Out = ((CAN_RxMsg.Data[2] << 8) | CAN_RxMsg.Data[3]) / 100.0f;
                // 电池输入功率: Data[4-5], int16, /100 -> W (可正可负)
                GSTCH_Capacitor.Pow_In = (int16_t)((CAN_RxMsg.Data[4] << 8) | CAN_RxMsg.Data[5]) / 100.0f;
                GST_SystemMonitor.CapatitorRx_cnt++;
                break;

			case 0x401:
//                CAP_current[0] = (s16)( ( CAN_RxMsg.Data[0] << 8 ) | (CAN_RxMsg.Data[1] ) ) / 100.0f;
//                CAP_current[1] = (s16)( ( CAN_RxMsg.Data[2] << 8 ) | (CAN_RxMsg.Data[3] ) ) / 100.0f;
//                CAP_current[2] = (s16)( ( CAN_RxMsg.Data[4] << 8 ) | (CAN_RxMsg.Data[5] ) ) / 100.0f;
//                CAP_current[3] = (s16)( ( CAN_RxMsg.Data[6] << 8 ) | (CAN_RxMsg.Data[7] ) ) / 100.0f;
//                GST_SystemMonitor.CapacitorRx_cnt++;
                break;
			default:
				break;
        }
        GST_SystemMonitor.CAN1Rx_cnt++;
    }
}

/**
  * @brief  CAN2发送中断服务函数
  * @param  无
  * @retval 无
  */
void CAN2_TX_IRQHandler(void)
{
    /****************检测发送邮箱空****************/
    if(CAN_GetITStatus(CAN2, CAN_IT_TME) == SET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
    }
}

/**
  * @brief  CAN2接收中断服务函数
  * @note   处理轮毂电机的CAN数据
  * @param  无
  * @retval 无
  */
void CAN2_RX0_IRQHandler(void)
{
    /****************如果不是CAN2接收中断，直接返回****************/
    if(CAN_GetITStatus(CAN2, CAN_IT_FMP0) != SET)
    {return;}

    CanRxMsg CAN_RxMsg;

    /****************清除中断标志位、接收数据****************/
    CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
    CAN_Receive(CAN2, CAN_FIFO0, &CAN_RxMsg);

    /****************根据不同ID号来处理数据****************/
    switch(CAN_RxMsg.StdId)
    {
        /****************右轮毂电机****************/
        case CANID_HM2:
            CANRx_C620DataParse(&GstCH_HM2RxC620Data, &CAN_RxMsg); //解析CAN数据到电调反馈结构体中
            GST_SystemMonitor.HubMotor2Rx_cnt++;
            break;
        
        /****************左轮毂电机****************/
        case CANID_HM1:
            CANRx_C620DataParse(&GstCH_HM1RxC620Data, &CAN_RxMsg); //解析CAN数据到电调反馈结构体中
            GST_SystemMonitor.HubMotor1Rx_cnt++;
            break;
    }
    GST_SystemMonitor.CAN2Rx_cnt++;
}

/**
  * @brief  判断串口1单次接收是否完成的函数
  * @note   判断原理就是DMA每传输一个数据，它的数据传输计数器就会减1
  *         USART1Rx使用的DMA配置为循环模式，减到0后会自动变回设定的重装值UA1RxDMAbuf_LEN，此时一次接收结束
  *         所以只需要看DMA的DataCounter就知道单次接收是否结束了
  * @param  无
  * @retval true：单次接收结束  false：单次接收未结束
*/
bool __IsUSART1SingleRecOK(void)
{
    if(DMA_GetCurrDataCounter(USART1_RX_STREAM) == UA1RxDMAbuf_LEN)
    {return true;}

    else
    {return false;}
}

/**
  * @brief  USART1接收中断服务函数，对遥控器传输的数据进行解析，从而控制机器人
  * @note   串口1，连接遥控接收机DR16。
  *         中断进入条件：空闲中断（IDLE中断）
  *         中断产生机制：DR16每隔7ms通过DBus发送一帧数据（18字节），一帧数据发送完成后，IDLE中断就会被触发
  * @param  无
  * @retval 无
*/
void USART1_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != SET)
    {return;}
    
    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    USART1->SR;
    USART1->DR;	

    /****************************串口1接收数据****************************/
    if(__IsUSART1SingleRecOK())
    {
        GF_USART1_RxDone = true;        //串口1接收完成标志位，数据在ReceiverTask里面被处理
        GST_SystemMonitor.USART1Rx_cnt++;
    }
    else
    {
        DMA_Cmd(USART1_RX_STREAM, DISABLE);                         //设置当前计数值前先禁用DMA
        DMA_SetCurrDataCounter(USART1_RX_STREAM, UA1RxDMAbuf_LEN);  //设置当前待发的数据的数量
        DMA_Cmd(USART1_RX_STREAM, ENABLE);                          //启用串口DMA接收
    }
}

/**
  * @brief  判断串口2单次接收是否完成的函数
  * @note   判断原理就是DMA每传输一个数据，它的数据传输计数器就会减1
  *         USART2Rx使用的DMA配置为循环模式，减到0后会自动变回设定的重装值UA2RxDMAbuf_LEN，此时一次接收结束
  *         所以只需要看DMA的DataCounter就知道单次接收是否结束了
  * @param  无
  * @retval true：单次接收结束  false：单次接收未结束
*/
bool __IsUSART2SingleRecOK(void)
{
    if(DMA_GetCurrDataCounter(USART2_RX_STREAM) == UA2RxDMAbuf_LEN)
    {return true;}

    else
    {return false;}
}

/**
  * @brief  串口2的中断服务函数，与云台云控通讯
  * @note   
  *         中断进入条件：空闲中断（IDLE中断）
  * @param  无
  * @retval 无
*/
void USART2_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != SET)
    {return;}

    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    USART2->SR;
    USART2->DR;		
        
    /****************************串口2接收数据以及数据处理****************************/
    if(__IsUSART2SingleRecOK())
    {
        UA2Rx_IMU1DataProcess();//IMU1，云台云控数据的接收、处理
        GST_SystemMonitor.USART2Rx_cnt++;
    }
    else
    {
        DMA_Cmd(USART2_RX_STREAM, DISABLE);                         //设置当前计数值前先禁用DMA
        DMA_SetCurrDataCounter(USART2_RX_STREAM, UA2RxDMAbuf_LEN);  //设置当前待发的数据的数量
        DMA_Cmd(USART2_RX_STREAM, ENABLE);                          //启用串口DMA接收
    }
}

/**
  * @brief  串口3的中断服务函数，作为备用（串口发送时会进入，但是没有对数据进行处理）
  * @note   中断进入条件：空闲中断（IDLE中断），用电脑给主控发消息的时候进入。
  *         这个函数暂时没有什么作用，如果后续有用可以自行修改内部内容
  * @param  无
  * @retval 无
*/
void USART3_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != SET)
    {return;}

    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    USART3->SR;
    USART3->DR;

    /****************************串口3数据接收****************************/
    GST_SystemMonitor.USART3Rx_cnt++;

    DMA_Cmd(USART3_RX_STREAM, DISABLE);         //设置当前计数值前先禁用DMA
    USART3_RX_STREAM->NDTR = UA3RxDMAbuf_LEN;   //设置当前待发的数据的数量:Number of Data units to be TRansferred
    DMA_Cmd(USART3_RX_STREAM, ENABLE);          //启用串口DMA接收
}

/**
  * @brief  判断串口4单次接收是否完成的函数
  * @note   判断原理就是DMA每传输一个数据，它的数据传输计数器就会减1
  *         UART4Rx使用的DMA配置为循环模式，减到0后会自动变回设定的重装值UA4RxDMAbuf_LEN，此时一次接收结束
  *         所以只需要看DMA的DataCounter就知道单次接收是否结束了
  * @param  无
  * @retval true：单次接收结束  false：单次接收未结束
*/
bool __IsUART4SingleRecOK(void)
{
    
    if(DMA_GetCurrDataCounter(UART4_RX_STREAM) == UA4RxDMAbuf_LEN)
    {return true;}

    else
    {return false;}
}

/**
  * @brief  串口4的中断服务函数，作为和IMU2（底盘云控）的通讯
  * @note   主要是关节电机的反馈数据、底盘的IMU数据
  * @param  无
  * @retval 无
*/
void UART4_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != SET)
    {return;}

    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    UART4->SR;
    UART4->DR;

    /****************************串口4数据接收、解析****************************/
    if(__IsUART4SingleRecOK())
    {
        UA4Rx_IMU2DataProcess();        //IMU2底盘云控数据的接收、处理
        GST_SystemMonitor.UART4Rx_cnt++;
    }
    else
    {
        DMA_Cmd(UART4_RX_STREAM, DISABLE);         //设置当前计数值前先禁用DMA
        UART4_RX_STREAM->NDTR = UA4RxDMAbuf_LEN;   //设置当前待发的数据的数量:Number of Data units to be TRansferred
        DMA_Cmd(UART4_RX_STREAM, ENABLE);          //启用串口DMA接收
    }
}

/**
  * @brief  判断串口5单次接收是否完成的函数
  * @note   判断原理就是DMA每传输一个数据，它的数据传输计数器就会减1
  *         UART5Rx使用的DMA配置为循环模式，减到0后会自动变回设定的重装值UA5RxDMAbuf_LEN，此时一次接收结束
  *         所以只需要看DMA的DataCounter就知道单次接收是否结束了
  * @param  无
  * @retval true：单次接收结束  false：单次接收未结束
*/
bool __IsUART5SingleRecOK(void)
{

    if(DMA_GetCurrDataCounter(UART5_RX_STREAM) == UA5RxDMAbuf_LEN)
    {return true;}

    else
    {return false;}
}
/**
  * @brief  串口5的中断服务函数，作为和裁判系统的通讯
  * @note   主要是裁判系统的反馈数据、比赛状态数据、机器人相关官方数据信息
  * @param  无
  * @retval 无
*/
void UART5_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(UART5, USART_IT_IDLE) != SET)
    {return;}

    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    UART5->SR;
    UART5->DR;

    /****************************串口5数据接收、解析****************************/

        UA5Rx_RefereeDataProcess();        //裁判系统数据的接收、处理
        GST_SystemMonitor.UART5Rx_cnt++;

}

/**
  * @brief  判断串口6单次接收是否完成的函数
  * @note   判断原理就是DMA每传输一个数据，它的数据传输计数器就会减1
  *         USART6Rx使用的DMA配置为循环模式，减到0后会自动变回设定的重装值UA6RxDMAbuf_LEN，此时一次接收结束
  *         所以只需要看DMA的DataCounter就知道单次接收是否结束了
  * @param  无
  * @retval true：单次接收结束  false：单次接收未结束
*/
bool __IsUSART6SingleRecOK(void)
{

    if(DMA_GetCurrDataCounter(USART6_RX_STREAM) == UA6RxDMAbuf_LEN)
    {return true;}

    else
    {return false;}
}
/**
  * @brief  串口6的中断服务函数，作为和辅瞄小电脑的通讯
  * @note   主要是辅瞄小电脑发送的视觉数据接收
  * @param  无
  * @retval 无
*/
void USART6_IRQHandler(void)
{
    /****************************如果不是IDLE中断直接返回****************************/
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != SET)
    {return;}

    /****************************先读SR后读DR，清除IDLE中断标志位****************************/
    USART6->SR;
    USART6->DR;

    /****************************串口6数据接收、解析****************************/
    if(__IsUSART6SingleRecOK())
    {
        UA6Rx_VisionDataProcess();
    }
    else
    {
        DMA_Cmd(USART6_RX_STREAM, DISABLE);         //设置当前计数值前先禁用DMA
        USART6_RX_STREAM->NDTR = UA6RxDMAbuf_LEN;   //设置当前待发的数据的数量:Number of Data units to be TRansferred
        DMA_Cmd(USART6_RX_STREAM, ENABLE);          //启用串口DMA接收
    }
}

/**
  * @brief  窗口看门狗中断服务函数
  * @note   该函数用于Debug的时候打印相关数据
  * @param  无
  * @retval 无
*/
void WWDG_IRQHandler()
{
    WWDG_SetCounter(WWDG_CounterValue);  //喂狗，值在0x40-0x7F之间
    WWDG_ClearFlag();
}

/**
  * @brief  硬件错误中断服务函数
  * @note   当出现堆栈错误、数据类型错误、Flash被覆盖等等问题的时候会跳转到这里
  * @param  无
  * @retval 无
*/
// void HardFault_Handler(void)
// {
//     while(1)
//     {
//         printf("HardFault!");
//     }
// }
