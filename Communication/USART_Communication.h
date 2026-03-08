/**
  ******************************************************************************
  * @file    USART_Communication.h
  * @author  26赛季，平衡步兵电控，林宸曳
  * @date    2025.10.13
  * @brief   串口的通讯相关函数、缓冲区等等
  ******************************************************************************
*/
#ifndef __USART_COMMUNICATION_H
#define __USART_COMMUNICATION_H

#include "stdint.h"
#include "stm32f4xx.h"

/****************************变量引出extern声明****************************/
/********串口发送/接收缓冲区长度********/
extern const uint8_t UA1RxDMAbuf_LEN;
extern const uint8_t UA2TxDMAbuf_LEN;
extern const uint8_t UA2RxDMAbuf_LEN;
extern const uint8_t UA3TxDMAbuf_LEN;
extern const uint8_t UA3RxDMAbuf_LEN;
extern const uint8_t UA4TxDMAbuf_LEN;
extern const uint8_t UA4RxDMAbuf_LEN;
extern const uint8_t UA5TxDMAbuf_LEN;
extern const uint16_t UA5RxDMAbuf_LEN;
extern const uint8_t UA6TxDMAbuf_LEN;
extern const uint8_t UA6RxDMAbuf_LEN;
/********串口缓冲区数组********/
extern uint8_t UA1RxDMAbuf[];
extern uint8_t UA2RxDMAbuf[];
extern uint8_t UA3RxDMAbuf[];
extern uint8_t UA4RxDMAbuf[];
extern uint8_t UA5TxDMAbuf[];
extern uint8_t UA5RxDMAbuf[];
extern uint8_t UA6RxDMAbuf[];

/****************************宏定义引出声明****************************/
/******** DMA数据流的宏定义 ********/
#define USART1_RX_STREAM        DMA2_Stream2 //DMA2_Channel4
#define USART2_TX_STREAM        DMA1_Stream6 //DMA1_Channel4
#define USART2_RX_STREAM        DMA1_Stream5 //DMA1_Channel4
#define USART3_TX_STREAM        DMA1_Stream3 //DMA1_Channel4
#define USART3_RX_STREAM        DMA1_Stream1 //DMA1_Channel4
#define UART4_TX_STREAM         DMA1_Stream4 //DMA1_Channel4
#define UART4_RX_STREAM         DMA1_Stream2 //DMA1_Channel4
#define UART5_TX_STREAM         DMA1_Stream7 //DMA1_Channel4
#define UART5_RX_STREAM         DMA1_Stream0 //DMA1_Channel4
#define USART6_RX_STREAM        DMA2_Stream1 //DMA2_Channel5
#define USART6_TX_STREAM        DMA2_Stream6 //DMA2_Channel5

typedef struct
{
    USART_TypeDef* USARTx;              //串口号
    DMA_Stream_TypeDef* DMAy_Streamx;   //DMA数据流
    uint8_t* pMailbox;                       //接收邮箱地址
    __IO uint8_t* pDMAbuf;                   //DMA内存基地址
    uint16_t MbLen;                          //邮箱大小
    uint16_t DMALen;                         //DMA缓存区大小
    uint16_t rxConter;                       //当前接收帧结束地址+1
    uint16_t rxBufferPtr;                    //当前帧起始地址
    uint16_t rxSize;                         //当前帧大小
} USART_RX_TypeDef;

typedef struct
{
    USART_TypeDef* USARTx;
    DMA_Stream_TypeDef* DMAy_Streamx;
    uint8_t* pMailbox;
    __IO uint8_t* pDMAbuf;
    uint16_t MbLen;
    uint16_t DMALen;
} USART_TX_TypeDef;

extern uint8_t UA5RxMailbox[];
extern USART_RX_TypeDef UART5_Rcr;
extern USART_TX_TypeDef UART5_Tcr;

#define RSYS_RX_FREE             0
#define RSYS_RX_Length           1
#define RSYS_RX_Num              2
#define RSYS_RX_CRC8             3
#define RSYS_RX_CmdID            4
#define RSYS_RX_Data             5
#define RSYS_RX_CRC16            6

/*串口5通信缓冲长度*/
#define UART5_RXMB_LEN            250   
#define UART5_TXMB_LEN            20


/****************************函数声明****************************/
/********串口3、6的DMA打印函数********/
void USART3_DMA_printf(const char* fmt, ...);
void USART6_DMA_printf(const char* fmt, ...);

/********串口收发、数据解析函数********/
void UA1Rx_ReceiverDataProcess(void);
void UA2Rx_IMU1DataProcess(void);
void UA2Tx_SendDataToIMU1(void);
void UA4Rx_IMU2DataProcess(void);
void UA4Tx_SendDataToIMU2(void);
void UA5Rx_RefereeDataProcess(void);
void UA5Tx_SendDataToReferee(void);
void MonitorDataDeal(uint16_t usCmdID);
void Rc_RsysProtocol(void); //裁判系统协议解析
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t USART_Receive(USART_RX_TypeDef* USARTx);
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint8_t ucCRC8);

#endif
