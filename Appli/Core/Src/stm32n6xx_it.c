/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32n6xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32n6xx_it.h"
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
__attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32)))
volatile uint32_t g_hardfault_regs[16];
__attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32)))
volatile uint32_t g_hardfault_stack[8];
__attribute__((section("noncacheable_buffer"))) __attribute__((aligned(32)))
volatile uint32_t g_hardfault_valid = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_NodeTypeDef Node_GPDMA1_Channel0;
extern DMA_QListTypeDef List_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern ETH_HandleTypeDef heth1;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

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
  uint32_t *stack = 0;
  uint32_t exc_return;
  __asm volatile ("mov %0, lr" : "=r"(exc_return));
  if ((exc_return & 0x4U) != 0U)
  {
    __asm volatile ("mrs %0, psp" : "=r"(stack));
  }
  else
  {
    __asm volatile ("mrs %0, msp" : "=r"(stack));
  }

  g_hardfault_valid = 0xDEAD0001U;
  g_hardfault_stack[0] = stack ? stack[0] : 0U; /* R0  */
  g_hardfault_stack[1] = stack ? stack[1] : 0U; /* R1  */
  g_hardfault_stack[2] = stack ? stack[2] : 0U; /* R2  */
  g_hardfault_stack[3] = stack ? stack[3] : 0U; /* R3  */
  g_hardfault_stack[4] = stack ? stack[4] : 0U; /* R12 */
  g_hardfault_stack[5] = stack ? stack[5] : 0U; /* LR  */
  g_hardfault_stack[6] = stack ? stack[6] : 0U; /* PC  */
  g_hardfault_stack[7] = stack ? stack[7] : 0U; /* xPSR */

  g_hardfault_regs[0] = (uint32_t)stack;
  g_hardfault_regs[1] = exc_return;
  g_hardfault_regs[2] = SCB->HFSR;
  g_hardfault_regs[3] = SCB->CFSR;
  g_hardfault_regs[4] = SCB->BFAR;
  g_hardfault_regs[5] = SCB->MMFAR;
  g_hardfault_regs[6] = SCB->DFSR;
  g_hardfault_regs[7] = SCB->AFSR;
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
  * @brief This function handles Secure fault.
  */
void SecureFault_Handler(void)
{
  /* USER CODE BEGIN SecureFault_IRQn 0 */

  /* USER CODE END SecureFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_SecureFault_IRQn 0 */
    /* USER CODE END W1_SecureFault_IRQn 0 */
  }
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

/******************************************************************************/
/* STM32N6xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32n6xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 0 */
  g_audio_dbg.dma_irq_cnt++;
  /* USER CODE END GPDMA1_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 1 */

  /* USER CODE END GPDMA1_Channel0_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles ETH1 global interrupt.
  */
void ETH1_IRQHandler(void)
{
  /* USER CODE BEGIN ETH1_IRQn 0 */

  /* USER CODE END ETH1_IRQn 0 */
  HAL_ETH_IRQHandler(&heth1);
  /* USER CODE BEGIN ETH1_IRQn 1 */

  /* USER CODE END ETH1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

