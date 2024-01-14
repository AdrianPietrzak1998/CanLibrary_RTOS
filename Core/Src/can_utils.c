/*
 * can_utils.c
 *
 *  Created on: Jan 5, 2024
 *      Author: Adrian
 */

#include "can_utils.h"
#include "crc.h"
#include "can.h"
#include <stdlib.h>
#include "string.h"

//#include "tim.h"
#if USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

TaskHandle_t xCan20CheckHandle;
SemaphoreHandle_t xMapperModSemaphore;
SemaphoreHandle_t xSendSemaphore;

#endif

CanReceiver_t *CanReceiverMapper[MAX_MSG_CAN];
uint16_t CanReceiverCounter;

HAL_StatusTypeDef Can20SendSimple(CAN_HandleTypeDef *hcan, uint32_t addr, uint8_t *PtrData, uint8_t LengthData)
{
	HAL_StatusTypeDef status;
	uint32_t TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t SendFault = 0;

	if(LengthData > 8) return HAL_ERROR;

	  TxHeader.DLC = LengthData;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.TransmitGlobalTime = DISABLE;
	  if(addr <= 0x7ff)
	  {
		  TxHeader.StdId = addr;
		  TxHeader.IDE = CAN_ID_STD;
	  }
	  else
	  {
		  TxHeader.ExtId = addr;
		  TxHeader.IDE = CAN_ID_EXT;
	  }



	  do
	  {
		  status = HAL_CAN_AddTxMessage(hcan, &TxHeader, PtrData, &TxMailbox);
		  SendFault++;
	  }while(status != HAL_OK && SendFault < TRY_SEND_MAX_FAIL);


	  return status;
}

HAL_StatusTypeDef Can20SendLongMsgCrc(CAN_HandleTypeDef *hcan, uint32_t firstAddr, uint8_t *PtrData, uint32_t LengthData)
{
	HAL_StatusTypeDef status;
	uint32_t Crc = 0;
	uint8_t FrameAmount, LastDataLength;

	//dodaj kontrole rozmiaru, czy podzielne przez 4

	FrameAmount = LengthData/8;
	LastDataLength = LengthData % 8;
	if(LastDataLength) FrameAmount++;

	Crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)PtrData, LengthData/4);



	if(LengthData > 8)
	{
		for(uint8_t i=0; i<FrameAmount; i++)
		{
			status = Can20SendSimple(hcan, firstAddr+i, PtrData+(8*i), (FrameAmount-1==i)?LastDataLength:8);
			if(status != 0) return status;
		}
		status = Can20SendSimple(hcan, firstAddr+FrameAmount, (uint8_t*)&Crc, 4);
		return status;
	}
	else
	{
		status = Can20SendSimple(hcan, firstAddr, PtrData, LengthData);
		if(status != 0) return status;
		status = Can20SendSimple(hcan, firstAddr+1, (uint8_t*)&Crc, 4);

		return status;
	}
}

HAL_StatusTypeDef Can20SendCheck(CAN_HandleTypeDef *hcan, uint32_t addr)
{
	HAL_StatusTypeDef status;
	uint32_t TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	uint16_t SendFault = 0;


	  TxHeader.DLC = 0;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.TransmitGlobalTime = DISABLE;

	  if(addr <= 0x7ff)
	  {
		  TxHeader.StdId = addr;
		  TxHeader.IDE = CAN_ID_STD;
	  }
	  else
	  {
		  TxHeader.ExtId = addr;
		  TxHeader.IDE = CAN_ID_EXT;
	  }

	  do
	  {
		  status = HAL_CAN_AddTxMessage(hcan, &TxHeader, NULL, &TxMailbox);
		  SendFault++;
	  }while(status != HAL_OK && SendFault < TRY_SEND_MAX_FAIL);

	  return status;
}

HAL_StatusTypeDef Can20ReceiverCheckFrameInit(CanReceiver_t *CanReceiver, CAN_HandleTypeDef *hcan,
		uint32_t Addr, void(*ActionFunPtr)(CanReceiver_t *))
{
	CanReceiver -> IsCheckFrame  = 1;
	CanReceiver -> MsgSize       = 0;
	CanReceiver -> firstAddr     = Addr;
	CanReceiver -> hcan          = hcan;

	if(ActionFunPtr == NULL)
	{
		return HAL_ERROR;
	}

	CanReceiver -> ActionFunctionPtr = ActionFunPtr;

#if USE_FREERTOS
	if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
	xSemaphoreTake(xMapperModSemaphore, portMAX_DELAY);
#endif
	CanReceiverMapper[CanReceiverCounter] = CanReceiver;
	CanReceiverCounter++;
#if USE_FREERTOS
	if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
	xSemaphoreGive(xMapperModSemaphore);
#endif


	return HAL_OK;

}

HAL_StatusTypeDef Can20ReceiverInit(CanReceiver_t *CanReceiver, size_t MsgDataLength, CAN_HandleTypeDef *hcan,
		uint32_t FirstAddr, uint8_t CrcEnable, void(*ActionFunPtr)(CanReceiver_t *) )
{

#if DYNAMIC_ALLOCATION_BUF
	CanReceiver -> buf_8_ptr = malloc(MsgDataLength);
#else
	CanReceiver -> buf_8_ptr = CanReceiver -> buf_8;
#endif
	CanReceiver -> IsCheckFrame  = 0;
	CanReceiver -> DataIsCorrect = 0;
	CanReceiver -> MsgSize       = MsgDataLength;
	CanReceiver -> hcan          = hcan;
	CanReceiver -> firstAddr     = FirstAddr;
	CanReceiver -> CrcEnabled    = CrcEnable;
	CanReceiver -> ActionFunctionPtr   = ActionFunPtr;

	//dodaj kontrole czy juz jest na liscie
	if(CanReceiver ->buf_8_ptr != NULL)
	{
#if USE_FREERTOS
		if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
		xSemaphoreTake(xMapperModSemaphore, portMAX_DELAY);
#endif
		CanReceiverMapper[CanReceiverCounter] = CanReceiver;
		CanReceiverCounter++;
#if USE_FREERTOS
		if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
		xSemaphoreGive(xMapperModSemaphore);
#endif
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}


}

static size_t CalculateTargetFrameSize(CanReceiver_t *CanReceiver, uint8_t ActualFrame)
{
	size_t MsgSize = CanReceiver -> MsgSize;
	uint8_t FrameAmount;
	if(MsgSize <= 8)
	{
		return MsgSize;
	}
	else
	{
		if(MsgSize%8 == 0)
		{
			return 8U;
		}
		else
		{
			FrameAmount = MsgSize /8;
			if(MsgSize % 8) FrameAmount++;

			if(FrameAmount == ActualFrame+1)
			{
				return MsgSize - (ActualFrame * 8);
			}
			else
			{
				return 8U;
			}
		}
	}
}

CANlib_StatusTypedef Can20ReceiverRoutine(CAN_HandleTypeDef *hcan, uint32_t rxfifo)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint32_t Addr;
	uint8_t RxData[8];
	uint8_t FrameAmount, ActualFrame;
	size_t TargetFrameSize;
	uint32_t CalculatedCRC;

	if(HAL_CAN_GetRxMessage(hcan, rxfifo, &RxHeader, RxData) != HAL_OK) return CANLIB_ERR;
	if(RxHeader.IDE)
	{
		Addr = RxHeader.ExtId;
	}
	else
	{
		Addr = RxHeader.StdId;
	}

	if(CanReceiverCounter)
	{
		for(uint16_t i=0; i<CanReceiverCounter; i++)
		{
			if(CanReceiverMapper[i] -> hcan == hcan) //Check canbus number
			{
				FrameAmount = CanReceiverMapper[i] -> MsgSize /8;
				if(CanReceiverMapper[i] -> MsgSize % 8) FrameAmount++;
				//
				//ControlMsg routine
				//
				if(CanReceiverMapper[i] -> IsCheckFrame && CanReceiverMapper[i] -> firstAddr == Addr)
				{
					CanReceiverMapper[i] -> CpltReceiveTimeStamp = __GET_MS;
					if(CanReceiverMapper[i] -> ActionFunctionPtr != NULL)
					{
						CanReceiverMapper[i] -> ActionFunctionPtr(CanReceiverMapper[i]);
						return CANLIB_OK;
					}
					else return CANLIB_MEMORY_ERR;
				}
				//
				//
				//
				if(Addr >= CanReceiverMapper[i]->firstAddr && Addr < CanReceiverMapper[i]->firstAddr + FrameAmount + (CanReceiverMapper[i]->CrcEnabled)?1:0) //Address pool for msg
				{
					//
					//CRC - calculation and check
					//
					if(Addr == CanReceiverMapper[i]->firstAddr + FrameAmount)
					{
						if(RxHeader.DLC == 4)
						{
							memcpy(&CanReceiverMapper[i]->crc, RxData, 4);
							CalculatedCRC = HAL_CRC_Calculate(&hcrc, CanReceiverMapper[i]->buf_32_ptr, CanReceiverMapper[i]->MsgSize / 4);
						}
						else return CANLIB_SIZE_ERR;

						CanReceiverMapper[i] -> CpltReceiveTimeStamp = __GET_MS;

						if(CalculatedCRC == CanReceiverMapper[i]->crc)
						{
							CanReceiverMapper[i] -> DataIsCorrect = 1;
							if(CanReceiverMapper[i] -> ActionFunctionPtr != NULL)
							{
								CanReceiverMapper[i] -> ActionFunctionPtr(CanReceiverMapper[i]);
							}

							return CANLIB_OK;
						}
						else
						{
							return CANLIB_CRC_ERR;
						}

					}
					//
					//
					//
					else
					{
						//cpy msg frame to buf
						ActualFrame = Addr - (CanReceiverMapper[i] -> firstAddr);
						TargetFrameSize = CalculateTargetFrameSize(CanReceiverMapper[i], ActualFrame);
						if(TargetFrameSize == RxHeader.DLC)
						{
							CanReceiverMapper[i] -> DataIsCorrect = 0;
							memcpy(CanReceiverMapper[i]->buf_8_ptr + (ActualFrame*8), RxData, RxHeader.DLC);
							//
							// If CRC is disable
							//
							if(Addr == CanReceiverMapper[i]->firstAddr + FrameAmount -1 && CanReceiverMapper[i]->CrcEnabled == DISABLE)
							{
								CanReceiverMapper[i] -> ActionFunctionPtr(CanReceiverMapper[i]);
							}
							//
							//
							//
							return CANLIB_OK;
						}
						else
						{
							return CANLIB_SIZE_ERR;
						}
					}
				}
			}
		}
	}
	return CANLIB_NMATCH;
}

#if DIAG_TIME_ROUTINE
CANlib_StatusTypedef Can20ReceiverOvrTimSet(CanReceiver_t *CanReceiver, uint32_t MaxLastReceiveTime, void (*OverTimeFunPtr)())
{
	CanReceiver -> MaxLastReceiveTime = MaxLastReceiveTime;
	if(OverTimeFunPtr != NULL)
	{
	CanReceiver -> OverTimeFunPtr = OverTimeFunPtr;
	return CANLIB_OK;
	}
	return CANLIB_MEMORY_ERR;
}

void Can20ReceiverOvrTimRoutine(void)
{
	if(CanReceiverCounter)
	{
#if USE_FREERTOS
		xSemaphoreTake(xMapperModSemaphore, portMAX_DELAY);
#endif
		for(uint32_t i=0; i < CanReceiverCounter; i++)
		{
			if(CanReceiverMapper[i] -> OverTimeFunPtr != NULL &&
					__GET_MS - CanReceiverMapper[i] -> CpltReceiveTimeStamp > CanReceiverMapper[i]->MaxLastReceiveTime)
			{
#if !DIAG_TIME_FUN_CONT
				CanReceiverMapper[i] -> CpltReceiveTimeStamp = __GET_MS;
#endif
				CanReceiverMapper[i] -> OverTimeFunPtr(CanReceiverMapper[i]);
			}
		}
#if USE_FREERTOS
		xSemaphoreGive(xMapperModSemaphore);
#endif

	}
}

#if USE_FREERTOS
void Can20ReceiverOvrTimTask(void *argument)
{
	for(;;)
	{
		Can20ReceiverOvrTimRoutine();
		vTaskDelay(10);
	}
}
#endif



#endif

#if DYNAMIC_ALLOCATION_BUF
CANlib_StatusTypedef Can20Receiverdeinit(CanReceiver_t *CanReceiver)
{
	uint32_t ReceiverNumMapper;

#if USE_FREERTOS
	xSemaphoreTake(xMapperModSemaphore, portMAX_DELAY);
#endif


	for(uint32_t i=0; i<CanReceiverCounter; i++)
	{
		if(CanReceiverMapper[i] == CanReceiver)
		{
			ReceiverNumMapper = i;
			goto ReturnErrSkip;
		}
	}
	return CANLIB_NMATCH;

	ReturnErrSkip:
	if(CanReceiverCounter - 1 == ReceiverNumMapper)
	{
		free(CanReceiverMapper[ReceiverNumMapper] -> buf_8_ptr);
		memset(CanReceiverMapper[ReceiverNumMapper], 0, sizeof(CanReceiver_t));
		CanReceiverCounter--;
		CanReceiverMapper[CanReceiverCounter] = NULL;
#if USE_FREERTOS
		xSemaphoreGive(xMapperModSemaphore);
#endif
		return CANLIB_OK;
	}
	else
	{
		free(CanReceiverMapper[ReceiverNumMapper] -> buf_8_ptr);
		memset(CanReceiverMapper[ReceiverNumMapper], 0, sizeof(CanReceiver_t));
		memmove(&CanReceiverMapper[ReceiverNumMapper], &CanReceiverMapper[ReceiverNumMapper+1], sizeof(uint32_t) * (CanReceiverCounter-ReceiverNumMapper-1));
		CanReceiverCounter--;
		CanReceiverMapper[CanReceiverCounter] = NULL;
#if USE_FREERTOS
		xSemaphoreGive(xMapperModSemaphore);
#endif
		return CANLIB_OK;
	}
}
#endif



HAL_StatusTypeDef Can20aInit(void)
{
#if STM32_CAN1_ENABLE
	__RETURN_STATUS_NOK(HAL_CAN_Start(&hcan1));
#if STM32_HAL_CAN_RX_INT_FIFO0
	__RETURN_STATUS_NOK(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING));
#endif
#if STM32_HAL_CAN_RX_INT_FIFO1
	__RETURN_STATUS_NOK(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING));
#endif
#endif

#if STM32_CAN2_ENABLE
	__RETURN_STATUS_NOK(HAL_CAN_Start(&hcan2));
#if STM32_HAL_CAN_RX_INT_FIFO0
	__RETURN_STATUS_NOK(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING));
#endif
#if STM32_HAL_CAN_RX_INT_FIFO1
	__RETURN_STATUS_NOK(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING));
#endif
#endif

#if STM32_CAN3_ENABLE
	__RETURN_STATUS_NOK(HAL_CAN_Start(&hcan3));
#if STM32_HAL_CAN_RX_INT_FIFO0
	__RETURN_STATUS_NOK(HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO0_MSG_PENDING));
#endif
#if STM32_HAL_CAN_RX_INT_FIFO1
	__RETURN_STATUS_NOK(HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO1_MSG_PENDING));
#endif
#endif

#if USE_FREERTOS

	BaseType_t xCan20CheckCreated = xTaskCreate(Can20ReceiverOvrTimTask, "Can20CheckTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &xCan20CheckHandle);

	if(xCan20CheckCreated != pdPASS) return HAL_ERROR;


	xMapperModSemaphore = xSemaphoreCreateBinary();
	xSendSemaphore = xSemaphoreCreateBinary();

	//xSemaphoreGive(xMapperModSemaphore);
	//xSemaphoreGive(xSendSemaphore);
#endif

	return HAL_OK;
}


#if STM32_HAL_CAN_RX_INT_FIFO0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	Can20ReceiverRoutine(hcan, CAN_RX_FIFO0);
}
#endif
#if STM32_HAL_CAN_RX_INT_FIFO1
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	Can20ReceiverRoutine(hcan, CAN_RX_FIFO1);
}
#endif


