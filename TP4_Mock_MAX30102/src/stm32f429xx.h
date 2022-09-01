/**
 ******************************************************************************
 * @file    stm32f429xx.h
 * @author  MCD Application Team
 * @brief   CMSIS STM32F429xx Device Peripheral Access Layer Header File.
 *
 *          This file contains:
 *           - Data structures and the address mapping for all peripherals
 *           - peripherals registers declarations and bits definition
 *           - Macros to access peripheral’s registers hardware
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/** @addtogroup CMSIS_Device
 * @{
 */

/** @addtogroup stm32f429xx
 * @{
 */

#ifndef __STM32F429xx_H
#define __STM32F429xx_H

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

  /** @addtogroup Configuration_section_for_CMSIS
   * @{
   */

/** @defgroup I2C_duty_cycle_in_fast_mode I2C duty cycle in fast mode
 * @{
 */
#define I2C_DUTYCYCLE_2 0x00000000U

/** @defgroup I2C_addressing_mode I2C addressing mode
 * @{
 */
#define I2C_ADDRESSINGMODE_7BIT 0x00004000U

/** @defgroup I2C_general_call_addressing_mode I2C general call addressing mode
 * @{
 */
#define I2C_GENERALCALL_DISABLE 0x00000000U

/** @defgroup I2C_dual_addressing_mode  I2C dual addressing mode
 * @{
 */
#define I2C_DUALADDRESS_DISABLE 0x00000000U

/** @defgroup I2C_nostretch_mode I2C nostretch mode
 * @{
 */
#define I2C_NOSTRETCH_DISABLE 0x00000000U

/** @defgroup I2CEx_Analog_Filter I2C Analog Filter
 * @{
 */
#define I2C_ANALOGFILTER_ENABLE 0x00000000U

/** @defgroup I2C_Memory_Address_Size I2C Memory Address Size
 * @{
 */
#define I2C_MEMADD_SIZE_8BIT 0x00000001U
#define I2C_MEMADD_SIZE_16BIT 0x00000010U

#define PERIPH_BASE 0x40000000UL /*!< Peripheral base address in the alias region \
/*!< Peripheral memory map */
#define APB1PERIPH_BASE PERIPH_BASE
#define I2C1_BASE (APB1PERIPH_BASE + 0x5400UL)

// /** @addtogroup Peripheral_declaration
//   * @{
//   */
#define I2C1 ((I2C_TypeDef *)I2C1_BASE)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F429xx_H */
