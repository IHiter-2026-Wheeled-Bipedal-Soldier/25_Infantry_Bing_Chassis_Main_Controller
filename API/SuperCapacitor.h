/**
  ******************************************************************************
  * @file    SuperCapacitor.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2026.3.8
  * @brief   超级电容相关
  ******************************************************************************
*/

#ifndef __SUPERCAPACITOR_H
#define __SUPERCAPACITOR_H

#include "stm32f4xx_can.h"

/*超电数据结构体*/
typedef struct
{
    /*需要初始化赋值的成员*/
    uint16_t CANTxID;   //超电CAN发送ID
    uint16_t CANRxID;   //超电CAN接收ID
    uint16_t CANTxID_Enable;    //超电CAN发送ID-使能
    uint16_t CANTxID_Disable;   //超电CAN发送ID-失能

    /*不需要初始化赋值的成员*/
    float CAP_Vol;      //电容电压
    float Pow_Out;      //电池输出
    float Volt_Out;     //电容输出
    float Pow_In;       //电池输出

    float TxPower;  //超电充电功率
} CapacitorMessage_StructTypeDef;

void SuperCap_Enable(CapacitorMessage_StructTypeDef ST_CAP);
void SuperCap_Disable(CapacitorMessage_StructTypeDef ST_CAP);
void SuperCap_SendPower(CAN_TypeDef *CANx, CapacitorMessage_StructTypeDef ST_CAP);

#endif
