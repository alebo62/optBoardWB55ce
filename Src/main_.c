/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "gpio.h"
#include "spi.h"
#include "string.h"
#include "tim.h"
//#include "messages.h"

typedef uint16_t U16;
typedef uint8_t U8;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
это на Nucleo было!!!!!! CN7  5-7 CLOSE for BOOT

stm32wbxx_hal.c 1469-1472 строки закомментировать!!!!!!

PA12- TIM1 ETR input   from ssi clk (USB-D+)            (SB12 CLOSE !!!!)
PA0-  TIM2 ETR input   from ssi clk (CN2-3) 
PA5   SCK 		 input   from ssi clk (CN1-8)

PA1-  EXT_INT1 input   from ssi frame (for start T1, T2)(CN2-10)

PA2-  TIM2 CH3 output(CN2-4)  to NSS ---> PВ2 NSS input (CN1-7)

PA6   MISO 		 output  data to ssi tx (CN1-9) 
PA8  TIM1_CH3 output  to control ssi tx(open output) (CN2-8)0-ACTIVE ---> 

PA7   MOSI 		 input   data from ssi rx (CN1-10)

System CLock 64 MHz

*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CTRL_MSG_FROM_UART 1
#define NO_MSG_RCV 0
#define IS_MSG_RCV 1
const U16 MAX_LEN_BUF = 322;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint16_t flag;
extern volatile uint16_t rx_buf[8];
extern void conn_process(void);
extern uint16_t ssc_txFrame_buf[];
extern volatile U8 idle_frame_tx;
extern uint8_t is_connect;
extern U8 tract_id_my;
extern U8 xcmp_dev_num;
extern U16 mask;
extern U16 ssc_rcv_buf[];
extern U16 XNL_DATA_MSG_ACK[];
extern U16 XNL_DATA_MSG_REPLY[];
extern U16 ssc_tx_msg_length;
extern volatile U16 ssc_tx_msg_counter;
extern U16 ssc_idle_frame[];
extern U16 ssc_txData_buf[];
extern U16 ob_adress;
extern U16 xnl_flags;
extern U16 tract_id_my_hi;
extern U8 uart_ctrl_msg[64];       // = {0xC0,0}; передаются управляющие команды
extern U16 ctrl_msg_uart_ssi[400]; // = {0xABCD,0x0000,0x0000,0x000B,0x0000,0x0006};
extern U16 ssc_rcv_ctrl_msg_len;   // длинна сообщения
extern U16 rcv_msg_counter;        // счетчик принятых данных
extern U16 ssc_rcv_ctrl_msg[180];
extern uint8_t rcv_msg_state; // 1- being receive
extern U8 control_msg_ssi_uart_len;
extern void get_check_sum(U16 *);
U16 i;
extern U8 temp;
U8 control_msg_ssi_uart_len;
U8 uart1_rx_buf[128];
U16 copy_length;
U16 ssc_length;
U16 remain_len;
U8 temp_uart_buf[128];
U16 uart1_rx_msg_len;
U8 ssc_tx_sound; // должен идти звук
U8 need_reply;
volatile uint8_t uart1_rx_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
   /* USER CODE BEGIN 1 */

   /* USER CODE END 1 */

   /* MCU Configuration--------------------------------------------------------*/

   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

   /* USER CODE BEGIN Init */

   /* USER CODE END Init */

   /* Configure the system clock */
   SystemClock_Config();

   /* USER CODE BEGIN SysInit */

   /* USER CODE END SysInit */

   /* Initialize all configured peripherals */
   MX_GPIO_Init();
   MX_TIM2_Init();
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
   MX_TIM1_Init();
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   MX_SPI1_Init();
   __HAL_SPI_ENABLE_IT(&hspi1, SPI_IT_RXNE | SPI_IT_TXE);
   //MX_TIM16_Init();
   //MX_TIM17_Init();
   /* USER CODE BEGIN 2 */
   memcpy((char *)ssc_txFrame_buf, (char *)idle_frame_tx, 16);
   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   while (1)
   {
      if (flag)
      {
         flag = 0;
         if (is_connect)
         {
            switch (rcv_msg_state)
            {

            case NO_MSG_RCV: // начало приема сообщения
               mask = ssc_rcv_buf[3] & 0xF000;
               if (mask == 0x4000)
               {
                  switch (ssc_rcv_buf[3] & 0x0F00)
                  {
                  case 0x0: //simple message
                     ssc_rcv_ctrl_msg_len = ssc_rcv_buf[3] & 0xff;
                     if (ssc_rcv_ctrl_msg_len % 2)
                        ssc_rcv_ctrl_msg_len++;
                     ssc_rcv_ctrl_msg_len += 2;
                     rcv_msg_counter = 0;
                     rcv_msg_state = IS_MSG_RCV; // теперь надо получить остальное message include checksum ...
                     break;
                  case 0x0100: // first fragment for sound
                     break;
                  case 0x0200: // middle fragment for sound
                     break;
                  case 0x0300: // last fragment
                     break;
                  default:
                     break;
                  }
               }
               break; // end NO_MSG_RCV

            case IS_MSG_RCV: //   продолжаем прием сообщения
               memcpy((ssc_rcv_ctrl_msg + (rcv_msg_counter >> 1)), (ssc_rcv_buf + 2), 4);
               rcv_msg_counter += 4;
               if (rcv_msg_counter >= ssc_rcv_ctrl_msg_len)
               { //получили всё сообщение
                  rcv_msg_state = NO_MSG_RCV;

                  //                        else if( (first_time == 1) && (ssc_rcv_ctrl_msg[7] == 0x841D))//&& (ssc_rcv_ctrl_msg[7] == 0x8414))
                  //                        {
                  //                            first_time = 2;
                  //                            //uart1_rx_flag = CTRL_MSG;
                  //                        }

                  switch (ssc_rcv_ctrl_msg[1])
                  {
                  case 0x0B: // xnl data msg  отправляем в уарт
                     control_msg_ssi_uart_len = ssc_rcv_ctrl_msg[6];
                     uart_ctrl_msg[1] = control_msg_ssi_uart_len;
                     if (control_msg_ssi_uart_len % 2)
                     {
                        memcpy(uart_ctrl_msg + 2, ssc_rcv_ctrl_msg + 7, control_msg_ssi_uart_len + 1);
                        for (i = 0; i < (control_msg_ssi_uart_len + 1); i += 2)
                        { // переставляем байты во временном массиве 1234->3412
                           temp = *(uart_ctrl_msg + i + 1 + 2);
                           *(uart_ctrl_msg + i + 1 + 2) = *(uart_ctrl_msg + i + 2);
                           *(uart_ctrl_msg + i + 2) = temp;
                        }
                     }
                     else
                     {
                        memcpy(uart_ctrl_msg + 2, ssc_rcv_ctrl_msg + 7, control_msg_ssi_uart_len);
                        // [0] already = 0xC0 and 0x00BA dont need
                        for (i = 0; i < control_msg_ssi_uart_len; i += 2)
                        { // переставляем байты во временном массиве 1234->3412
                           temp = *(uart_ctrl_msg + i + 1 + 2);
                           *(uart_ctrl_msg + i + 1 + 2) = *(uart_ctrl_msg + i + 2);
                           *(uart_ctrl_msg + i + 2) = temp;
                        }
                     }

                     //USART_WriteBuffer(USART1, uart_ctrl_msg, uart_ctrl_msg[1]+2);
                     //USART_EnableIt(USART1, US_IER_TXBUFE) ;

                     // ***************************** XNL_DATA_MSG_ACK *****************************************

                     if ((ssc_rcv_ctrl_msg[3] == 0) || (ssc_rcv_ctrl_msg[3] == ob_adress))
                     {
                        XNL_DATA_MSG_ACK[4] = ssc_rcv_ctrl_msg[2];
                        XNL_DATA_MSG_ACK[6] = ssc_rcv_ctrl_msg[3]; //ob_adress;
                        XNL_DATA_MSG_ACK[7] = ssc_rcv_ctrl_msg[5];
                        get_check_sum(XNL_DATA_MSG_ACK);
                        ssc_tx_msg_length = 10;
                        ssc_tx_msg_counter = 0;
                        memcpy((ssc_txData_buf), (XNL_DATA_MSG_ACK), 20);
                        while (!idle_frame_tx)
                           ;
                        idle_frame_tx = 0; //  ssc transmit
                        ssc_tx_sound = 0;
                        if (need_reply)
                        {
                           need_reply = 0;
                           ssc_tx_msg_length = 14;
                           ssc_tx_msg_counter = 0;
                           while (!idle_frame_tx)
                              ;
                           memcpy(ssc_txData_buf, XNL_DATA_MSG_REPLY, 28);

                           idle_frame_tx = 0; //  ssc transmit
                           ssc_tx_sound = 0;
                        }
                     }
                     break;

                  case 0x0C: // xnl data ack
                     control_msg_ssi_uart_len = ssc_rcv_ctrl_msg[6];
                     uart_ctrl_msg[1] = control_msg_ssi_uart_len;
                     memcpy(uart_ctrl_msg + 2, ssc_rcv_ctrl_msg + 7, control_msg_ssi_uart_len + 1);
                     //USART_WriteBuffer(USART1, uart_ctrl_msg, uart_ctrl_msg[1]+2);
                     //USART_EnableIt(USART1, US_IER_TXBUFE) ;

                     break;
                  default:
                     break;
                  }
               }
               break; // end IS_MSG_RCV

            default:
               break;
            }
            if (uart1_rx_flag == CTRL_MSG_FROM_UART)
            { //transmit mesage from uart
               uart1_rx_flag = 0;
               {
                  ++xnl_flags;
                  if (xnl_flags == 0x0108)
                     xnl_flags = 0x0100;
                  ++tract_id_my;
                  tract_id_my_hi += tract_id_my;
                  ctrl_msg_uart_ssi[4] = xnl_flags;
                  ctrl_msg_uart_ssi[6] = ob_adress;
                  ctrl_msg_uart_ssi[7] = tract_id_my_hi;
                  // ***********************************************
                  if ((uart1_rx_msg_len - 2) < 241)
                  {
                     //send_ssi_0_240();// 254 - checksum(2) - 12(head)
                     // ***** Первый фрейм****
                     ctrl_msg_uart_ssi[8] = uart1_rx_msg_len - 2;
                     ctrl_msg_uart_ssi[1] = 0x4000 + (12 + uart1_rx_msg_len);
                     if (uart1_rx_msg_len % 2)
                     { // если нечетное - в конец добавим 0х00
                        *(uart1_rx_buf + uart1_rx_msg_len) = 0x00;
                        uart1_rx_msg_len++;
                     }
                     uart1_rx_msg_len -= 2; // длина без 0xC0,0xLen
                     for (i = 0; i < uart1_rx_msg_len; i += 2)
                     {                                                      // переставляем байты во временном массиве 1234->3412
                        *(temp_uart_buf + i) = *(uart1_rx_buf + i + 1 + 2); // start after 0xC0,0xLen
                        *(temp_uart_buf + i + 1) = *(uart1_rx_buf + i + 2);
                     }
                     memcpy(ctrl_msg_uart_ssi + 9, temp_uart_buf, uart1_rx_msg_len); //+ 2);// -2 + 4 (00ba 0000)
                     ssc_tx_msg_length = uart1_rx_msg_len >> 1;                      // длинна теперь в словах
                     ctrl_msg_uart_ssi[ssc_tx_msg_length + 9] = 0x00BA;              // +терминатор
                     if (ssc_tx_msg_length % 2)
                     { // для окончания фрейма добавить 0х0000 , если нечетное к-во слов
                        ctrl_msg_uart_ssi[ssc_tx_msg_length + 10] = 0x0000;
                        ssc_tx_msg_length++;
                     }
                     uart1_rx_msg_len = MAX_LEN_BUF; // подготовка для следующего приема
                     get_check_sum(ctrl_msg_uart_ssi);
                     // отправка сформированного сообщения
                     ssc_tx_msg_length += 10; // 3(ssi)+6(xnml)+1(00BA)
                     ssc_tx_msg_counter = 0;
                     memcpy((ssc_txData_buf), (ctrl_msg_uart_ssi), (ssc_tx_msg_length << 1));
                     ssc_txFrame_buf[4] = 0xABCD;
                     ssc_txFrame_buf[5] = 0x5A5A;
                     ssc_txFrame_buf[6] = 0;
                     ssc_txFrame_buf[7] = 0;
                     while (!idle_frame_tx)
                        ;
                     idle_frame_tx = 0;
                  }
                  else if ((uart1_rx_msg_len - 2) < 493)
                  {
                     //send_ssi_0_492();//  (2*254 - checksum(2))*2 - 12(head)
                     // ***** Первый фрейм****
                     ctrl_msg_uart_ssi[8] = uart1_rx_msg_len - 2; // общая длинна сообщения
                     ctrl_msg_uart_ssi[1] = 0x41FE;               // + (2 + 12 + 240 );//first fragment chs(2)+head(12)+maxlen;
                     if (uart1_rx_msg_len % 2)
                     { // если нечетное - в конец добавим 0х00
                        *(uart1_rx_buf + uart1_rx_msg_len) = 0x00;
                        uart1_rx_msg_len++;
                     }
                     uart1_rx_msg_len -= 2; // длина без 0xC1,0xХХ
                     for (i = 0; i < 240; i += 2)
                     {                                                      // переставляем байты во временном массиве 1234->3412
                        *(temp_uart_buf + i) = *(uart1_rx_buf + i + 1 + 2); // start after 0xC0,0xLen
                        *(temp_uart_buf + i + 1) = *(uart1_rx_buf + i + 2);
                     }
                     memcpy(ctrl_msg_uart_ssi + 9, temp_uart_buf, 240); //+ 2);// -2 + 4 (00ba 0000)

                     ctrl_msg_uart_ssi[129] = 0x00BA;
                     get_check_sum(ctrl_msg_uart_ssi);

                     remain_len = uart1_rx_msg_len - 240; // остаток сообщения
                     // ***** Второй фрейм****
                     ctrl_msg_uart_ssi[130] = 0xABCD;
                     ctrl_msg_uart_ssi[131] = 0x4300 + 2 + remain_len; //chs(2) + remain_len

                     for (i = 0; i < remain_len; i += 2)
                     {                                                            // переставляем байты во временном массиве 1234->3412
                        *(temp_uart_buf + i) = *(uart1_rx_buf + i + 1 + 2 + 240); // start after 0xC0,0xLen
                        *(temp_uart_buf + i + 1) = *(uart1_rx_buf + i + 2 + 240);
                     }
                     memcpy(ctrl_msg_uart_ssi + 133, temp_uart_buf, remain_len); //+ 2);// -2 + 4 (00ba 0000)
                     ssc_length = remain_len >> 1;
                     //ssc_tx_msg_length = remain_len >> 1;
                     ctrl_msg_uart_ssi[130 + 3 + ssc_length] = 0x00BA; // +терминатор
                     //ctrl_msg_uart_ssi[130 + ssc_tx_msg_length + 3] = 0x00BA; // +терминатор
                     if (ssc_length % 2)
                     { // для окончания фрейма добавить 0х0000 , если нечетное к-во слов
                        ctrl_msg_uart_ssi[130 + 3 + ssc_length + 1] = 0x0000;
                        ssc_length++;
                     }
                     uart1_rx_msg_len = MAX_LEN_BUF; // для нового приема по уарт
                     get_check_sum(ctrl_msg_uart_ssi + 130);
                     ssc_length = 130 + 3 + ssc_length + 1;
                     ssc_tx_msg_length = ssc_length;
                     ssc_tx_msg_counter = 0;
                     memcpy(ssc_txData_buf, ctrl_msg_uart_ssi, ssc_tx_msg_length << 1);
                     ssc_txFrame_buf[4] = 0xABCD;
                     ssc_txFrame_buf[5] = 0x5A5A;
                     ssc_txFrame_buf[6] = 0;
                     ssc_txFrame_buf[7] = 0;
                     while (!idle_frame_tx)
                        ;
                     idle_frame_tx = 0;
                  }
                  else if ((uart1_rx_msg_len - 2) < 745)
                  {
                     // ***** Первый фрейм****
                     copy_length = uart1_rx_msg_len - 2;
                     if (uart1_rx_buf[0] > 0xC2)
                        uart1_rx_msg_len -= 3; // был добавлен байт 0х00 для выравнивания
                     else
                        uart1_rx_msg_len -= 2;

                     ctrl_msg_uart_ssi[8] = uart1_rx_msg_len;
                     ctrl_msg_uart_ssi[1] = 0x41FE;
                     //													if(uart1_rx_msg_len % 2)
                     //													{// если нечетное - в конец добавим 0х00
                     //															//*(uart1_rx_buf + uart1_rx_msg_len) = 0x00;
                     //															uart1_rx_msg_len++;
                     //													}
                     // длина без 0xC1,0xLen

                     //													for(i = 0 ; i < 240; i+=2)
                     //													{// переставляем байты во временном массиве 1234->3412
                     //															*(temp_uart_buf + i ) = *(uart1_rx_buf + i + 1 + 2);// start after 0xC0,0xLen
                     //															*(temp_uart_buf + i + 1) = *(uart1_rx_buf + i + 2 );
                     //													}
                     memcpy(ctrl_msg_uart_ssi + 9, uart1_rx_buf + 2, 240); //+ 2);// -2 + 4 (00ba 0000)
                     ctrl_msg_uart_ssi[129] = 0x00BA;
                     get_check_sum(ctrl_msg_uart_ssi);

                     // ***** Второй фрейм****
                     ctrl_msg_uart_ssi[130] = 0xABCD;
                     ctrl_msg_uart_ssi[131] = 0x42FE;

                     //													for(i = 0 ; i < 252; i+=2)
                     //													{// переставляем байты во временном массиве 1234->3412
                     //															*(temp_uart_buf + i)  = *(uart1_rx_buf + i + 1 + 2 + 240);// start after 0xC0,0xLen
                     //															*(temp_uart_buf + i + 1) = *(uart1_rx_buf + i + 2 + 240);
                     //													}
                     memcpy(ctrl_msg_uart_ssi + 133, uart1_rx_buf + 240 + 2, 252); //+ 2);// -2 + 4 (00ba 0000)
                     ctrl_msg_uart_ssi[130 + 129] = 0x00BA;                        // +терминатор
                     get_check_sum(ctrl_msg_uart_ssi + 130);

                     // ***** Третий фрейм****
                     ctrl_msg_uart_ssi[130 + 130] = 0xABCD;
                     remain_len = uart1_rx_msg_len - 240 - 252; // столько будет payload в последнем фрейме
                     ctrl_msg_uart_ssi[130 + 131] = 0x4300 + 2 + remain_len;

                     copy_length -= (240 + 252); // столько будем копировать в ctrl_msg_uart_ssi

                     // ********** вставили 27.02.19****************
                     //													for(i = 0 ; i < remain_len; i += 2)
                     //													{// переставляем байты во временном массиве 1234->3412
                     //															*(temp_uart_buf + i)  = *(uart1_rx_buf + i + 1 + 2 + 240 + 252);// start after 0xC0,0xLen
                     //															*(temp_uart_buf + i + 1) = *(uart1_rx_buf + i + 2 + 240 + 252);
                     //													}
                     memcpy(ctrl_msg_uart_ssi + 130 + 133, uart1_rx_buf + 240 + 252 + 2, copy_length); //+ 2);// -2 + 4 (00ba 0000)

                     ssc_length = copy_length >> 1;
                     ctrl_msg_uart_ssi[130 + 130 + 3 + ssc_length] = 0x00BA; // +терминатор

                     ssc_tx_msg_length = (ssc_length) + (130 + 130); //
                     if (ssc_tx_msg_length % 2)
                     { // для окончания фрейма добавить 0х0000 , если нечетное к-во слов
                        ctrl_msg_uart_ssi[130 + 130 + 3 + ssc_tx_msg_length + 1] = 0x0000;
                        ssc_tx_msg_length++;
                     }
                     get_check_sum(ctrl_msg_uart_ssi + 130 + 130);
                     ssc_tx_msg_length += 4;

                     //uart1_rx_msg_len = MAX_LEN_BUF;
                     ssc_tx_msg_counter = 0;
                     memcpy((ssc_txData_buf), (ctrl_msg_uart_ssi), ssc_tx_msg_length << 1);
                     ssc_txFrame_buf[4] = 0xABCD;
                     ssc_txFrame_buf[5] = 0x5A5A;
                     ssc_txFrame_buf[6] = 0;
                     ssc_txFrame_buf[7] = 0;
                     while (!idle_frame_tx)
                        ;
                     idle_frame_tx = 0;
                  }
               }
            }

            else
               memcpy(ssc_idle_frame + 4, ssc_rcv_buf + 4, 8); // это пропускаем через опт. боард в станцию
         }
         else
            conn_process();
      }
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
   }
   /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
   /* USER CODE BEGIN Error_Handler_Debug */
   /* User can add his own implementation to report the HAL error return state */
   __disable_irq();
   while (1)
   {
   }
   /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
   /* USER CODE BEGIN 6 */
   /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
   /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/