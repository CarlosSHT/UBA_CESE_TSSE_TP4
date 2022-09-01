/**
 ******************************************************************************
 * @file    stm32f4xx_hal_i2c.h
 * @author  MCD Application Team
 * @brief   Header file of I2C HAL module.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_I2C_H
#define __STM32F4xx_HAL_I2C_H

#ifdef __cplusplus
extern "C"
{
#endif

#define __IO volatile /*!< Defines 'read / write' permissions */

  typedef enum
  {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
  } HAL_StatusTypeDef;

  /**
   * @brief Inter-integrated Circuit Interface
   */

  typedef struct
  {
    __IO uint32_t CR1;   /*!< I2C Control register 1,     Address offset: 0x00 */
    __IO uint32_t CR2;   /*!< I2C Control register 2,     Address offset: 0x04 */
    __IO uint32_t OAR1;  /*!< I2C Own address register 1, Address offset: 0x08 */
    __IO uint32_t OAR2;  /*!< I2C Own address register 2, Address offset: 0x0C */
    __IO uint32_t DR;    /*!< I2C Data register,          Address offset: 0x10 */
    __IO uint32_t SR1;   /*!< I2C Status register 1,      Address offset: 0x14 */
    __IO uint32_t SR2;   /*!< I2C Status register 2,      Address offset: 0x18 */
    __IO uint32_t CCR;   /*!< I2C Clock control register, Address offset: 0x1C */
    __IO uint32_t TRISE; /*!< I2C TRISE register,         Address offset: 0x20 */
    __IO uint32_t FLTR;  /*!< I2C FLTR register,          Address offset: 0x24 */
  } I2C_TypeDef;

  /** @defgroup I2C_Configuration_Structure_definition I2C Configuration Structure definition
   * @brief  I2C Configuration Structure definition
   * @{
   */
  typedef struct
  {
    uint32_t ClockSpeed; /*!< Specifies the clock frequency.
                              This parameter must be set to a value lower than 400kHz */

    uint32_t DutyCycle; /*!< Specifies the I2C fast mode duty cycle.
                             This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

    uint32_t OwnAddress1; /*!< Specifies the first device own address.
                               This parameter can be a 7-bit or 10-bit address. */

    uint32_t AddressingMode; /*!< Specifies if 7-bit or 10-bit addressing mode is selected.
                                  This parameter can be a value of @ref I2C_addressing_mode */

    uint32_t DualAddressMode; /*!< Specifies if dual addressing mode is selected.
                                   This parameter can be a value of @ref I2C_dual_addressing_mode */

    uint32_t OwnAddress2; /*!< Specifies the second device own address if dual addressing mode is selected
                               This parameter can be a 7-bit address. */

    uint32_t GeneralCallMode; /*!< Specifies if general call mode is selected.
                                   This parameter can be a value of @ref I2C_general_call_addressing_mode */

    uint32_t NoStretchMode; /*!< Specifies if nostretch mode is selected.
                                 This parameter can be a value of @ref I2C_nostretch_mode */

  } I2C_InitTypeDef;

  /** @defgroup HAL_mode_structure_definition HAL mode structure definition
   * @brief  HAL Mode structure definition
   * @note  HAL I2C Mode value coding follow below described bitmap :\n
   *          b7     (not used)\n
   *             x  : Should be set to 0\n
   *          b6\n
   *             0  : None\n
   *             1  : Memory (HAL I2C communication is in Memory Mode)\n
   *          b5\n
   *             0  : None\n
   *             1  : Slave (HAL I2C communication is in Slave Mode)\n
   *          b4\n
   *             0  : None\n
   *             1  : Master (HAL I2C communication is in Master Mode)\n
   *          b3-b2-b1-b0  (not used)\n
   *             xxxx : Should be set to 0000
   * @{
   */
  typedef enum
  {
    HAL_I2C_MODE_NONE = 0x00U,   /*!< No I2C communication on going             */
    HAL_I2C_MODE_MASTER = 0x10U, /*!< I2C communication is in Master Mode       */
    HAL_I2C_MODE_SLAVE = 0x20U,  /*!< I2C communication is in Slave Mode        */
    HAL_I2C_MODE_MEM = 0x40U     /*!< I2C communication is in Memory Mode       */

  } HAL_I2C_ModeTypeDef;

  /** @defgroup HAL_state_structure_definition HAL state structure definition
   * @brief  HAL State structure definition
   * @note  HAL I2C State value coding follow below described bitmap :
   *          b7-b6  Error information
   *             00 : No Error
   *             01 : Abort (Abort user request on going)
   *             10 : Timeout
   *             11 : Error
   *          b5     Peripheral initialization status
   *             0  : Reset (Peripheral not initialized)
   *             1  : Init done (Peripheral initialized and ready to use. HAL I2C Init function called)
   *          b4     (not used)
   *             x  : Should be set to 0
   *          b3
   *             0  : Ready or Busy (No Listen mode ongoing)
   *             1  : Listen (Peripheral in Address Listen Mode)
   *          b2     Intrinsic process state
   *             0  : Ready
   *             1  : Busy (Peripheral busy with some configuration or internal operations)
   *          b1     Rx state
   *             0  : Ready (no Rx operation ongoing)
   *             1  : Busy (Rx operation ongoing)
   *          b0     Tx state
   *             0  : Ready (no Tx operation ongoing)
   *             1  : Busy (Tx operation ongoing)
   * @{
   */
  typedef enum
  {
    HAL_I2C_STATE_RESET = 0x00U,          /*!< Peripheral is not yet Initialized         */
    HAL_I2C_STATE_READY = 0x20U,          /*!< Peripheral Initialized and ready for use  */
    HAL_I2C_STATE_BUSY = 0x24U,           /*!< An internal process is ongoing            */
    HAL_I2C_STATE_BUSY_TX = 0x21U,        /*!< Data Transmission process is ongoing      */
    HAL_I2C_STATE_BUSY_RX = 0x22U,        /*!< Data Reception process is ongoing         */
    HAL_I2C_STATE_LISTEN = 0x28U,         /*!< Address Listen Mode is ongoing            */
    HAL_I2C_STATE_BUSY_TX_LISTEN = 0x29U, /*!< Address Listen Mode and Data Transmission
                                              process is ongoing                         */
    HAL_I2C_STATE_BUSY_RX_LISTEN = 0x2AU, /*!< Address Listen Mode and Data Reception
                                              process is ongoing                         */
    HAL_I2C_STATE_ABORT = 0x60U,          /*!< Abort user request ongoing                */
    HAL_I2C_STATE_TIMEOUT = 0xA0U,        /*!< Timeout state                             */
    HAL_I2C_STATE_ERROR = 0xE0U           /*!< Error                                     */

  } HAL_I2C_StateTypeDef;

  /**
   * @brief  HAL Lock structures definition
   */
  typedef enum
  {
    HAL_UNLOCKED = 0x00U,
    HAL_LOCKED = 0x01U
  } HAL_LockTypeDef;

  /**
   * @brief DMA Controller
   */

  typedef struct
  {
    __IO uint32_t CR;   /*!< DMA stream x configuration register      */
    __IO uint32_t NDTR; /*!< DMA stream x number of data register     */
    __IO uint32_t PAR;  /*!< DMA stream x peripheral address register */
    __IO uint32_t M0AR; /*!< DMA stream x memory 0 address register   */
    __IO uint32_t M1AR; /*!< DMA stream x memory 1 address register   */
    __IO uint32_t FCR;  /*!< DMA stream x FIFO control register       */
  } DMA_Stream_TypeDef;

  /**
   * @brief  DMA Configuration Structure definition
   */
  typedef struct
  {
    uint32_t Channel; /*!< Specifies the channel used for the specified stream.
                           This parameter can be a value of @ref DMA_Channel_selection                    */

    uint32_t Direction; /*!< Specifies if the data will be transferred from memory to peripheral,
                             from memory to memory or from peripheral to memory.
                             This parameter can be a value of @ref DMA_Data_transfer_direction              */

    uint32_t PeriphInc; /*!< Specifies whether the Peripheral address register should be incremented or not.
                             This parameter can be a value of @ref DMA_Peripheral_incremented_mode          */

    uint32_t MemInc; /*!< Specifies whether the memory address register should be incremented or not.
                          This parameter can be a value of @ref DMA_Memory_incremented_mode              */

    uint32_t PeriphDataAlignment; /*!< Specifies the Peripheral data width.
                                       This parameter can be a value of @ref DMA_Peripheral_data_size                 */

    uint32_t MemDataAlignment; /*!< Specifies the Memory data width.
                                    This parameter can be a value of @ref DMA_Memory_data_size                     */

    uint32_t Mode; /*!< Specifies the operation mode of the DMAy Streamx.
                        This parameter can be a value of @ref DMA_mode
                        @note The circular buffer mode cannot be used if the memory-to-memory
                              data transfer is configured on the selected Stream                        */

    uint32_t Priority; /*!< Specifies the software priority for the DMAy Streamx.
                            This parameter can be a value of @ref DMA_Priority_level                       */

    uint32_t FIFOMode; /*!< Specifies if the FIFO mode or Direct mode will be used for the specified stream.
                            This parameter can be a value of @ref DMA_FIFO_direct_mode
                            @note The Direct mode (FIFO mode disabled) cannot be used if the
                                  memory-to-memory data transfer is configured on the selected stream       */

    uint32_t FIFOThreshold; /*!< Specifies the FIFO threshold level.
                                 This parameter can be a value of @ref DMA_FIFO_threshold_level                  */

    uint32_t MemBurst; /*!< Specifies the Burst transfer configuration for the memory transfers.
                            It specifies the amount of data to be transferred in a single non interruptible
                            transaction.
                            This parameter can be a value of @ref DMA_Memory_burst
                            @note The burst mode is possible only if the address Increment mode is enabled. */

    uint32_t PeriphBurst; /*!< Specifies the Burst transfer configuration for the peripheral transfers.
                               It specifies the amount of data to be transferred in a single non interruptible
                               transaction.
                               This parameter can be a value of @ref DMA_Peripheral_burst
                               @note The burst mode is possible only if the address Increment mode is enabled. */
  } DMA_InitTypeDef;

  /**
   * @brief  HAL DMA State structures definition
   */
  typedef enum
  {
    HAL_DMA_STATE_RESET = 0x00U,   /*!< DMA not yet initialized or disabled */
    HAL_DMA_STATE_READY = 0x01U,   /*!< DMA initialized and ready for use   */
    HAL_DMA_STATE_BUSY = 0x02U,    /*!< DMA process is ongoing              */
    HAL_DMA_STATE_TIMEOUT = 0x03U, /*!< DMA timeout state                   */
    HAL_DMA_STATE_ERROR = 0x04U,   /*!< DMA error state                     */
    HAL_DMA_STATE_ABORT = 0x05U,   /*!< DMA Abort state                     */
  } HAL_DMA_StateTypeDef;
  /**
   * @brief  DMA handle Structure definition
   */
  typedef struct __DMA_HandleTypeDef
  {
    DMA_Stream_TypeDef *Instance; /*!< Register base address                  */

    DMA_InitTypeDef Init; /*!< DMA communication parameters           */

    HAL_LockTypeDef Lock; /*!< DMA locking object                     */

    __IO HAL_DMA_StateTypeDef State; /*!< DMA transfer state                     */

    void *Parent; /*!< Parent object state                    */

    void (*XferCpltCallback)(struct __DMA_HandleTypeDef *hdma); /*!< DMA transfer complete callback         */

    void (*XferHalfCpltCallback)(struct __DMA_HandleTypeDef *hdma); /*!< DMA Half transfer complete callback    */

    void (*XferM1CpltCallback)(struct __DMA_HandleTypeDef *hdma); /*!< DMA transfer complete Memory1 callback */

    void (*XferM1HalfCpltCallback)(struct __DMA_HandleTypeDef *hdma); /*!< DMA transfer Half complete Memory1 callback */

    void (*XferErrorCallback)(struct __DMA_HandleTypeDef *hdma); /*!< DMA transfer error callback            */

    void (*XferAbortCallback)(struct __DMA_HandleTypeDef *hdma); /*!< DMA transfer Abort callback            */

    __IO uint32_t ErrorCode; /*!< DMA Error code                          */

    uint32_t StreamBaseAddress; /*!< DMA Stream Base Address                */

    uint32_t StreamIndex; /*!< DMA Stream Index                       */

  } DMA_HandleTypeDef;

//   /* Includes ------------------------------------------------------------------*/
//   //#include "stm32f4xx_hal_def.h"

//   /** @addtogroup STM32F4xx_HAL_Driver
//    * @{
//    */

//   /** @addtogroup I2C
//    * @{
//    */

//   /* Exported types ------------------------------------------------------------*/
//   /** @defgroup I2C_Exported_Types I2C Exported Types
//    * @{
//    */

//   /** @defgroup I2C_Configuration_Structure_definition I2C Configuration Structure definition
//    * @brief  I2C Configuration Structure definition
//    * @{
//    */
//   typedef struct
//   {
//     uint32_t ClockSpeed; /*!< Specifies the clock frequency.
//                               This parameter must be set to a value lower than 400kHz */

//     uint32_t DutyCycle; /*!< Specifies the I2C fast mode duty cycle.
//                              This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

//     uint32_t OwnAddress1; /*!< Specifies the first device own address.
//                                This parameter can be a 7-bit or 10-bit address. */

//     uint32_t AddressingMode; /*!< Specifies if 7-bit or 10-bit addressing mode is selected.
//                                   This parameter can be a value of @ref I2C_addressing_mode */

//     uint32_t DualAddressMode; /*!< Specifies if dual addressing mode is selected.
//                                    This parameter can be a value of @ref I2C_dual_addressing_mode */

//     uint32_t OwnAddress2; /*!< Specifies the second device own address if dual addressing mode is selected
//                                This parameter can be a 7-bit address. */

//     uint32_t GeneralCallMode; /*!< Specifies if general call mode is selected.
//                                    This parameter can be a value of @ref I2C_general_call_addressing_mode */

//     uint32_t NoStretchMode; /*!< Specifies if nostretch mode is selected.
//                                  This parameter can be a value of @ref I2C_nostretch_mode */

//   } I2C_InitTypeDef;

//   /**
//    * @}
//    */

//   /** @defgroup HAL_state_structure_definition HAL state structure definition
//    * @brief  HAL State structure definition
//    * @note  HAL I2C State value coding follow below described bitmap :
//    *          b7-b6  Error information
//    *             00 : No Error
//    *             01 : Abort (Abort user request on going)
//    *             10 : Timeout
//    *             11 : Error
//    *          b5     Peripheral initialization status
//    *             0  : Reset (Peripheral not initialized)
//    *             1  : Init done (Peripheral initialized and ready to use. HAL I2C Init function called)
//    *          b4     (not used)
//    *             x  : Should be set to 0
//    *          b3
//    *             0  : Ready or Busy (No Listen mode ongoing)
//    *             1  : Listen (Peripheral in Address Listen Mode)
//    *          b2     Intrinsic process state
//    *             0  : Ready
//    *             1  : Busy (Peripheral busy with some configuration or internal operations)
//    *          b1     Rx state
//    *             0  : Ready (no Rx operation ongoing)
//    *             1  : Busy (Rx operation ongoing)
//    *          b0     Tx state
//    *             0  : Ready (no Tx operation ongoing)
//    *             1  : Busy (Tx operation ongoing)
//    * @{
//    */
//   typedef enum
//   {
//     HAL_I2C_STATE_RESET = 0x00U,          /*!< Peripheral is not yet Initialized         */
//     HAL_I2C_STATE_READY = 0x20U,          /*!< Peripheral Initialized and ready for use  */
//     HAL_I2C_STATE_BUSY = 0x24U,           /*!< An internal process is ongoing            */
//     HAL_I2C_STATE_BUSY_TX = 0x21U,        /*!< Data Transmission process is ongoing      */
//     HAL_I2C_STATE_BUSY_RX = 0x22U,        /*!< Data Reception process is ongoing         */
//     HAL_I2C_STATE_LISTEN = 0x28U,         /*!< Address Listen Mode is ongoing            */
//     HAL_I2C_STATE_BUSY_TX_LISTEN = 0x29U, /*!< Address Listen Mode and Data Transmission
//                                               process is ongoing                         */
//     HAL_I2C_STATE_BUSY_RX_LISTEN = 0x2AU, /*!< Address Listen Mode and Data Reception
//                                               process is ongoing                         */
//     HAL_I2C_STATE_ABORT = 0x60U,          /*!< Abort user request ongoing                */
//     HAL_I2C_STATE_TIMEOUT = 0xA0U,        /*!< Timeout state                             */
//     HAL_I2C_STATE_ERROR = 0xE0U           /*!< Error                                     */

//   } HAL_I2C_StateTypeDef;

//   /**
//    * @}
//    */

//   /** @defgroup HAL_mode_structure_definition HAL mode structure definition
//    * @brief  HAL Mode structure definition
//    * @note  HAL I2C Mode value coding follow below described bitmap :\n
//    *          b7     (not used)\n
//    *             x  : Should be set to 0\n
//    *          b6\n
//    *             0  : None\n
//    *             1  : Memory (HAL I2C communication is in Memory Mode)\n
//    *          b5\n
//    *             0  : None\n
//    *             1  : Slave (HAL I2C communication is in Slave Mode)\n
//    *          b4\n
//    *             0  : None\n
//    *             1  : Master (HAL I2C communication is in Master Mode)\n
//    *          b3-b2-b1-b0  (not used)\n
//    *             xxxx : Should be set to 0000
//    * @{
//    */
//   typedef enum
//   {
//     HAL_I2C_MODE_NONE = 0x00U,   /*!< No I2C communication on going             */
//     HAL_I2C_MODE_MASTER = 0x10U, /*!< I2C communication is in Master Mode       */
//     HAL_I2C_MODE_SLAVE = 0x20U,  /*!< I2C communication is in Slave Mode        */
//     HAL_I2C_MODE_MEM = 0x40U     /*!< I2C communication is in Memory Mode       */

//   } HAL_I2C_ModeTypeDef;

// /**
//  * @}
//  */

// /** @defgroup I2C_Error_Code_definition I2C Error Code definition
//  * @brief  I2C Error Code definition
//  * @{
//  */
// #define HAL_I2C_ERROR_NONE 0x00000000U      /*!< No error              */
// #define HAL_I2C_ERROR_BERR 0x00000001U      /*!< BERR error            */
// #define HAL_I2C_ERROR_ARLO 0x00000002U      /*!< ARLO error            */
// #define HAL_I2C_ERROR_AF 0x00000004U        /*!< AF error              */
// #define HAL_I2C_ERROR_OVR 0x00000008U       /*!< OVR error             */
// #define HAL_I2C_ERROR_DMA 0x00000010U       /*!< DMA transfer error    */
// #define HAL_I2C_ERROR_TIMEOUT 0x00000020U   /*!< Timeout Error         */
// #define HAL_I2C_ERROR_SIZE 0x00000040U      /*!< Size Management error */
// #define HAL_I2C_ERROR_DMA_PARAM 0x00000080U /*!< DMA Parameter Error   */
// #define HAL_I2C_WRONG_START 0x00000200U     /*!< Wrong start Error     */
// #if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
// #define HAL_I2C_ERROR_INVALID_CALLBACK 0x00000100U /*!< Invalid Callback error */
// #endif                                             /* USE_HAL_I2C_REGISTER_CALLBACKS */
// /**
//  * @}
//  */

/** @defgroup I2C_handle_Structure_definition I2C handle Structure definition
 * @brief  I2C handle Structure definition
 * @{
 */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
  typedef struct __I2C_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  {
    I2C_TypeDef *Instance; /*!< I2C registers base address               */

    I2C_InitTypeDef Init; /*!< I2C communication parameters             */

    uint8_t *pBuffPtr; /*!< Pointer to I2C transfer buffer           */

    uint16_t XferSize; /*!< I2C transfer size                        */

    __IO uint16_t XferCount; /*!< I2C transfer counter                     */

    __IO uint32_t XferOptions; /*!< I2C transfer options                     */

    __IO uint32_t PreviousState; /*!< I2C communication Previous state and mode
                                      context for internal usage               */

    DMA_HandleTypeDef *hdmatx; /*!< I2C Tx DMA handle parameters             */

    DMA_HandleTypeDef *hdmarx; /*!< I2C Rx DMA handle parameters             */

    HAL_LockTypeDef Lock; /*!< I2C locking object                       */

    __IO HAL_I2C_StateTypeDef State; /*!< I2C communication state                  */

    __IO HAL_I2C_ModeTypeDef Mode; /*!< I2C communication mode                   */

    __IO uint32_t ErrorCode; /*!< I2C Error code                           */

    __IO uint32_t Devaddress; /*!< I2C Target device address                */

    __IO uint32_t Memaddress; /*!< I2C Target memory address                */

    __IO uint32_t MemaddSize; /*!< I2C Target memory address  size          */

    __IO uint32_t EventCount; /*!< I2C Event counter                        */

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    void (*MasterTxCpltCallback)(struct __I2C_HandleTypeDef *hi2c); /*!< I2C Master Tx Transfer completed callback */
    void (*MasterRxCpltCallback)(struct __I2C_HandleTypeDef *hi2c); /*!< I2C Master Rx Transfer completed callback */
    void (*SlaveTxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);  /*!< I2C Slave Tx Transfer completed callback  */
    void (*SlaveRxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);  /*!< I2C Slave Rx Transfer completed callback  */
    void (*ListenCpltCallback)(struct __I2C_HandleTypeDef *hi2c);   /*!< I2C Listen Complete callback              */
    void (*MemTxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);    /*!< I2C Memory Tx Transfer completed callback */
    void (*MemRxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);    /*!< I2C Memory Rx Transfer completed callback */
    void (*ErrorCallback)(struct __I2C_HandleTypeDef *hi2c);        /*!< I2C Error callback                        */
    void (*AbortCpltCallback)(struct __I2C_HandleTypeDef *hi2c);    /*!< I2C Abort callback                        */

    void (*AddrCallback)(struct __I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode); /*!< I2C Slave Address Match callback */

    void (*MspInitCallback)(struct __I2C_HandleTypeDef *hi2c);   /*!< I2C Msp Init callback                     */
    void (*MspDeInitCallback)(struct __I2C_HandleTypeDef *hi2c); /*!< I2C Msp DeInit callback                   */

#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
  } I2C_HandleTypeDef;

  //   /** @addtogroup I2C_Exported_Functions_Group2 Input and Output operation functions
  //    * @{
  //    */
  //   /* IO operation functions  ****************************************************/
  //   /******* Blocking mode: Polling */
  HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
  HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
  HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_I2C_H */
