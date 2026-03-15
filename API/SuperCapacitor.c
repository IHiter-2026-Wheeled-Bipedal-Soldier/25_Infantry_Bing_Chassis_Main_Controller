/**
  ******************************************************************************
  * @file    SuperCapacitor.c
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2026.3.8
  * @brief   超级电容相关
  ******************************************************************************
*/

#include "SuperCapacitor.h"
#include "stm32f4xx_can.h"

/**
  * @brief  使能超级电容输出函数
  * @note   发送特定的CANID号的CAN报文可以使能超级电容输出
  * @param  ST_CAP：CapacitorMessage_StructTypeDef类型的结构体，包含了超级电容相关的参数
  * @retval 无
  */
void SuperCap_Enable(CapacitorMessage_StructTypeDef ST_CAP)
{
    CanTxMsg CanTxMessage = {0};
    CanTxMessage.StdId = ST_CAP.CANTxID_Enable;
    CanTxMessage.IDE = CAN_Id_Standard;
    CanTxMessage.RTR = CAN_RTR_Data;
    CanTxMessage.DLC = 0x08;
    CAN_Transmit(CAN1, &CanTxMessage);
}

/**
  * @brief  禁用超级电容输出函数
  * @note   发送特定的CANID号的CAN报文可以关闭超级电容输出
  * @param  ST_CAP：CapacitorMessage_StructTypeDef类型的结构体，包含了超级电容相关的参数
  * @retval 无
  */
void SuperCap_Disable(CapacitorMessage_StructTypeDef ST_CAP)
{
    CanTxMsg CanTxMessage = {0};
    CanTxMessage.StdId = ST_CAP.CANTxID_Disable;
    CanTxMessage.IDE = CAN_Id_Standard;
    CanTxMessage.RTR = CAN_RTR_Data;
    CanTxMessage.DLC = 0x08;
    CAN_Transmit(CAN1, &CanTxMessage);
}

/**
  * @brief  向超级电容发送充电功率的函数
  * @note   发送特定的CANID号的CAN报文，并且在数据部分携带充电功率信息，可以控制超级电容的充电功率
  * @param  CANx：CAN_TypeDef类型的指针，指定使用哪个CAN外设发送数据
  * @param  ST_CAP：CapacitorMessage_StructTypeDef类型的结构体，包含了超级电容相关的参数，其中TxPower成员是要发送的充电功率值，单位为瓦特
  * @retval 无
  */
void SuperCap_SendPower(CAN_TypeDef *CANx, CapacitorMessage_StructTypeDef ST_CAP)
{
    CanTxMsg tx_message;
    tx_message.StdId = ST_CAP.CANTxID;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    s16 Power = (s16)(ST_CAP.TxPower) * 100;

    tx_message.Data[0] = 0;
    tx_message.Data[1] = 0;
    tx_message.Data[2] = (u8)(Power >> 8);
    tx_message.Data[3] = (u8)Power;
    tx_message.Data[4] = (u8)0;
    tx_message.Data[5] = (u8)0;
    tx_message.Data[6] = (u8)0;
    tx_message.Data[7] = (u8)0;
    
    CAN_Transmit(CANx, &tx_message);
}
