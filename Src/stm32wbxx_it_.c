/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wbxx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32wbxx_it.h"
#include "tim.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t ssc_rcv_buf[8];
uint8_t volatile tx_cnt;
uint8_t volatile rx_cnt;
volatile uint16_t flag;
extern SPI_HandleTypeDef hspi1;
extern volatile uint8_t idle_frame_tx;
extern uint16_t ssc_txData_buf[510];
extern uint16_t ssc_tx_msg_length;
extern volatile uint16_t ssc_tx_msg_counter;
volatile uint16_t ssc_txFrame_buf[8]   = {0};//, 0, 0x0000, 0x0000, 0xABCD, 0x5A5A, 0, 0};
uint16_t idle_frame_tx_buf[8] = {0, 0, 0xABCD, 0x5A5A, 0xABCD, 0x5A5A, 0, 0};

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
/* USER CODE BEGIN EV */
void SPI1_IRQHandler(void)
{
	if(SPI_CHECK_FLAG( hspi1.Instance->SR, SPI_FLAG_RXNE) != RESET)
	{
		rx_cnt++;
		if(rx_cnt == 8)
		{
		  rx_cnt = 0x00;
			flag = 1;
		}
		ssc_rcv_buf[rx_cnt] = hspi1.Instance->DR;
	}
	if(SPI_CHECK_FLAG( hspi1.Instance->SR, SPI_FLAG_TXE) != RESET)
	{
	  if(idle_frame_tx == 1)  
			hspi1.Instance->DR = idle_frame_tx_buf[tx_cnt++];
		else
			hspi1.Instance->DR = ssc_txFrame_buf[tx_cnt++];
		if(tx_cnt == 8)
		{
		  tx_cnt = 0;
			if(idle_frame_tx == 0)
			{
				if(ssc_tx_msg_counter > ssc_tx_msg_length)
				{
					idle_frame_tx = 1;
					memcpy(ssc_txFrame_buf, idle_frame_tx_buf, 16);
        }
				else
				{
					memcpy(ssc_txFrame_buf + 2, ssc_txData_buf + ssc_tx_msg_counter, 4);
				  ssc_tx_msg_counter += 2;
				}
			}
		}
	}
  /* USER CODE BEGIN SPI1_IRQn 1 */
  /* USER CODE END SPI1_IRQn 1 */
}

//void SPI1_IRQHandler(void)
//{
//	if(SPI_CHECK_FLAG( hspi1.Instance->SR, SPI_FLAG_RXNE) != RESET)
//	{
//		rx_cnt++;
//		if(rx_cnt == 8)
//		{
//		  rx_cnt = 0x00;
//			flag = 1;
//		}
//		ssc_rcv_buf[rx_cnt] = hspi1.Instance->DR;
//	}
//	if(SPI_CHECK_FLAG( hspi1.Instance->SR, SPI_FLAG_TXE) != RESET)
//	{
//	  //if(idle_frame_tx)  
//		//	hspi1.Instance->DR = idle_frame_tx_buf[tx_cnt++];
//		//else
//		hspi1.Instance->DR = ssc_txFrame_buf[tx_cnt++];
//		if(tx_cnt == 8)
//		{
//		  tx_cnt = 0;
//			if(idle_frame_tx == 0)
//			{
//				memcpy((char*)(ssc_txFrame_buf + 2), (char*)(ssc_txData_buf + ssc_tx_msg_counter), 4);
//				ssc_tx_msg_counter += 2;
//      
//				if(ssc_tx_msg_counter > ssc_tx_msg_length)
//				{
//					idle_frame_tx = 1;
//        }
//			}
//			else
//				memcpy((char*)ssc_txFrame_buf,(char*)idle_frame_tx_buf, 16);
//		}
//	}
//  /* USER CODE BEGIN SPI1_IRQn 1 */
//  /* USER CODE END SPI1_IRQn 1 */
//}


uint16_t temp;
void EXTI1_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		if(temp == 40){
			htim2.Instance->CR1 |= TIM_CR1_CEN;
			htim1.Instance->CR1 |= TIM_CR1_CEN;
			hspi1.Instance->CR1 |= SPI_CR1_SPE;
			HAL_NVIC_DisableIRQ(EXTI1_IRQn);
			rx_cnt = 0;
			tx_cnt = 1;
			
		}			
		temp++;
}
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WBxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wbxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles SPI1 global interrupt.
  */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
