/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define p2_Pin GPIO_PIN_0
#define p2_GPIO_Port GPIOC
#define p3_Pin GPIO_PIN_1
#define p3_GPIO_Port GPIOC
#define p4_Pin GPIO_PIN_2
#define p4_GPIO_Port GPIOC
#define p5_Pin GPIO_PIN_3
#define p5_GPIO_Port GPIOC
#define p6_Pin GPIO_PIN_0
#define p6_GPIO_Port GPIOA
#define p7_Pin GPIO_PIN_1
#define p7_GPIO_Port GPIOA
#define p8_Pin GPIO_PIN_2
#define p8_GPIO_Port GPIOA
#define p9_Pin GPIO_PIN_3
#define p9_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_4
#define SCK_GPIO_Port GPIOA
#define MO_Pin GPIO_PIN_5
#define MO_GPIO_Port GPIOA
#define STRn_Pin GPIO_PIN_6
#define STRn_GPIO_Port GPIOA
#define p14_Pin GPIO_PIN_7
#define p14_GPIO_Port GPIOA
#define p15_Pin GPIO_PIN_4
#define p15_GPIO_Port GPIOC
#define p20_Pin GPIO_PIN_5
#define p20_GPIO_Port GPIOC
#define p21_Pin GPIO_PIN_0
#define p21_GPIO_Port GPIOB
#define p22_Pin GPIO_PIN_1
#define p22_GPIO_Port GPIOB
#define p2pr_Pin GPIO_PIN_2
#define p2pr_GPIO_Port GPIOB
#define p4pr_Pin GPIO_PIN_10
#define p4pr_GPIO_Port GPIOB
#define p22v_Pin GPIO_PIN_11
#define p22v_GPIO_Port GPIOB
#define p24v_Pin GPIO_PIN_12
#define p24v_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
