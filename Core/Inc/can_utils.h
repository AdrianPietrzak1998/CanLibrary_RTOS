/*
 * can_utils.h
 *
 *  Created on: Jan 5, 2024
 *      Author: Adrian
 */

#ifndef INC_CAN_UTILS_H_
#define INC_CAN_UTILS_H_

#include "main.h"

/*
 * Can Interface settings.
 * This Settings must be the same as your stm config
 */
#define STM32_CAN1_ENABLE          1
#define STM32_CAN2_ENABLE          1
#define STM32_CAN3_ENABLE          1
#define STM32_HAL_CAN_RX_INT_FIFO0 1
#define STM32_HAL_CAN_RX_INT_FIFO1 1

/*
 * Library config
 * DYNAMIC_ALLOCATION - If enabled you can dynamic init and deinit message. If is disable you must set BUFFER_SIZE. Max length of message
 * 						is equal this value.
 * DIAG_TIME_ROUTINE  - Error handling after the time has elapsed
 * 						Each receiver must be configured (Can20ReceiverOvrTimSet()) if to be error handling
 * DIAG_TIME_FUN_CONT - If is disable, error handling function is call not more often that MaxLastReceiveTime.
 * 						If is enabled, handler is calling by each Can20ReceiverOvrTimRoutine call, until the message comes.
 * USE_FREERTOS       - Enable when use freeRTOS. If this is set, you must call Can20aInit() before receiver initialization.
 * TRY_SEND_MAX_FAIL  - If would you like send long message, your Can controller may be too slow. Low value will cause sending errors.
 * 						The faster the bus, the lower the value you can set
 * 						0 - 0xffffffff
 * MAX_MSG_CAN        - Max Can rececivers which you can init.
 */
#define DYNAMIC_ALLOCATION_BUF         0

#if !DYNAMIC_ALLOCATION_BUF
#define BUFFER_SIZE                    512
#endif
#define DIAG_TIME_ROUTINE              1

#if DIAG_TIME_ROUTINE
#define DIAG_TIME_FUN_CONT             0
#endif

#define USE_FREERTOS                   1
#define TRY_SEND_MAX_FAIL              70000
#define MAX_MSG_CAN                    300

#define __RETURN_STATUS_NOK(fun) if(fun != HAL_OK) return fun

#define __GET_MS HAL_GetTick()


typedef struct{
	union{
		uint32_t *buf_32_ptr;
		uint8_t  *buf_8_ptr;
	};
#if !DYNAMIC_ALLOCATION_BUF
	union{
		uint32_t buf_32[BUFFER_SIZE / 4];
		uint8_t buf_8[BUFFER_SIZE];
	};
#endif


	uint32_t firstAddr;
	size_t MsgSize;
	uint32_t crc;
	CAN_HandleTypeDef *hcan;
	uint32_t CpltReceiveTimeStamp;
	uint8_t CrcEnabled : 1;
	uint8_t DataIsCorrect : 1;
	uint8_t IsCheckFrame :  1;
#if DIAG_TIME_ROUTINE
	uint32_t MaxLastReceiveTime;
	void (*OverTimeFunPtr)();
#endif
	void (*ActionFunctionPtr)();
}CanReceiver_t;

typedef enum{
	CANLIB_OK         = 0x00U,
	CANLIB_NMATCH     = 0x01U,
	CANLIB_CRC_ERR    = 0x02U,
	CANLIB_SIZE_ERR   = 0x03U,
	CANLIB_MEMORY_ERR = 0x04U,
	CANLIB_ERR        = 0x05U
}CANlib_StatusTypedef;

HAL_StatusTypeDef Can20aInit(void);
HAL_StatusTypeDef Can20SendSimple(CAN_HandleTypeDef *hcan, uint32_t addr, uint8_t *PtrData, uint8_t LengthData);
HAL_StatusTypeDef Can20SendCheck(CAN_HandleTypeDef *hcan, uint32_t addr);
HAL_StatusTypeDef Can20SendLongMsgCrc(CAN_HandleTypeDef *hcan, uint32_t firstAddr, uint8_t *PtrData, uint32_t LengthData);
HAL_StatusTypeDef Can20ReceiverCheckFrameInit(CanReceiver_t *CanReceiver, CAN_HandleTypeDef *hcan,
		uint32_t Addr, void(*ActionFunPtr)(CanReceiver_t *));
HAL_StatusTypeDef Can20ReceiverInit(CanReceiver_t *CanReceiver, size_t MsgDataLength, CAN_HandleTypeDef *hcan,
		uint32_t FirstAddr, uint8_t CrcEnable, void(*ActionFunPtr)(CanReceiver_t *) );
CANlib_StatusTypedef Can20ReceiverRoutine(CAN_HandleTypeDef *hcan, uint32_t rxfifo);
#if DIAG_TIME_ROUTINE
CANlib_StatusTypedef Can20ReceiverOvrTimSet(CanReceiver_t *CanReceiver, uint32_t MaxLastReceiveTime, void (*OverTimeFunPtr)());
#if !USE_FREERTOS
void Can20ReceiverOvrTimRoutine(void);
#endif
#endif


#if DYNAMIC_ALLOCATION_BUF
CANlib_StatusTypedef Can20Receiverdeinit(CanReceiver_t *CanReceiver);
#endif

#endif /* INC_CAN_UTILS_H_ */
