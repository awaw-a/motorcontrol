#pragma once
#include <cstdint>
#include <cstddef>
#define HAL_UART_MODULE_ENABLED 1
#define HAL_DMA_MODULE_ENABLED 1
#define HAL_CAN_MODULE_ENABLED 1
#define HAL_GPIO_MODULE_ENABLED 1
#define USE_HAL_UART_REGISTER_CALLBACKS 1
#define USE_HAL_CAN_REGISTER_CALLBACKS 1
#define CAN_NUM 2
#define DMA_CIRCULAR 1
#define HAL_UART_ERROR_PE 1
#define HAL_UART_ERROR_FE 2
#define HAL_UART_ERROR_NE 4
#define HAL_UART_ERROR_ORE 8
#define HAL_UART_ERROR_DMA 16
#define HAL_DMA_ERROR_NONE 0
#define HAL_UART_STATE_READY 0
#define HAL_UART_STATE_BUSY_TX 1
#define HAL_UART_RX_COMPLETE_CB_ID 0
#define HAL_UART_ERROR_CB_ID 1
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
struct DMA_HandleTypeDef { uint32_t counter=0; struct {uint32_t Mode=DMA_CIRCULAR;} Init; uint32_t ErrorCode=0; };
struct UART_HandleTypeDef { DMA_HandleTypeDef* hdmarx{}; DMA_HandleTypeDef* hdmatx{}; uint32_t gState=0, ErrorCode=0, RxState=0; };
using UartCB=void(*)(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_StatusTypeDef HAL_UART_RegisterCallback(UART_HandleTypeDef*,uint32_t,UartCB);
#define __HAL_UART_CLEAR_PEFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_FEFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_NEFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)0)
struct CAN_HandleTypeDef {int value{};};
struct CAN_RxHeaderTypeDef {uint32_t StdId{},ExtId{},IDE{},RTR{},DLC{},Timestamp{},FilterMatchIndex{};};
struct CAN_TxHeaderTypeDef {uint32_t StdId,ExtId,IDE,RTR,DLC,TransmitGlobalTime;};
struct CAN_FilterTypeDef {uint32_t FilterIdHigh,FilterIdLow,FilterMaskIdHigh,FilterMaskIdLow,FilterFIFOAssignment,FilterBank,FilterMode,FilterScale,FilterActivation,SlaveStartFilterBank;};
constexpr uint32_t CAN_ID_STD=0,CAN_ID_EXT=4,CAN_RTR_DATA=0,CAN_RTR_REMOTE=2,CAN_FILTER_FIFO0=0,CAN_FILTERMODE_IDMASK=0,CAN_FILTERSCALE_32BIT=1,ENABLE=1,CAN_IT_RX_FIFO0_MSG_PENDING=2;
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,const CAN_FilterTypeDef*);
struct GPIO_TypeDef{};
extern GPIO_TypeDef stub_gpio;
#define LED_0_Pin 128
#define LED_0_GPIO_Port (&stub_gpio)
enum GPIO_PinState{GPIO_PIN_RESET,GPIO_PIN_SET};
void HAL_GPIO_WritePin(GPIO_TypeDef*,uint16_t,GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*,uint16_t);
void HAL_GPIO_TogglePin(GPIO_TypeDef*,uint16_t);
void Error_Handler();

#define HAL_UART_STATE_BUSY_RX 2
#define __HAL_DMA_GET_COUNTER(d) ((d)->counter)
#define __DMB() ((void)0)
constexpr uint32_t CAN_TX_MAILBOX0=1,CAN_TX_MAILBOX1=2,CAN_TX_MAILBOX2=4;
constexpr uint32_t CAN_IT_TX_MAILBOX_EMPTY=1,CAN_RX_FIFO0=0,CAN_RX_FIFO1=1;
constexpr uint32_t HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID=0,HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID=1;
constexpr uint32_t HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID=2,HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID=3,HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID=4;
constexpr uint32_t HAL_CAN_TX_MAILBOX0_ABORT_CB_ID=5,HAL_CAN_TX_MAILBOX1_ABORT_CB_ID=6,HAL_CAN_TX_MAILBOX2_ABORT_CB_ID=7;
uint32_t HAL_GetTick();
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef*,const CAN_TxHeaderTypeDef*,const uint8_t*,uint32_t*);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,uint32_t,CAN_RxHeaderTypeDef*,uint8_t*);
HAL_StatusTypeDef HAL_CAN_RegisterCallback(CAN_HandleTypeDef*,uint32_t,void(*)(CAN_HandleTypeDef*));
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t);
