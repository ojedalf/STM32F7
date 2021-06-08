/*-----------------------------------------------------------------------------
   LCD-TFT Display Controller (LTDC)
-------------------------------------------------------------------------------
   Author: Fernando Ojeda L.                                          Jan-2020
-------------------------------------------------------------------------------
   Description:
-------------------------------------------------------------------------------
   Interface:
-------------------------------------------------------------------------------*/


#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

/*--------------------------------------------------------
  Include Files
 *------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>                
#include "ltdc.h"


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/*--------------------------------------------------------
  Macros
*--------------------------------------------------------*/



/*--------------------------------------------------------
  General Purpose I/O Registers
*--------------------------------------------------------*/
typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;


 /*--------------------------------------------------------
  Reset and Clock Control Registers
*--------------------------------------------------------*/
typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  __IO uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
  __IO uint32_t DCKCFGR1;      /*!< RCC Dedicated Clocks configuration register1,                 Address offset: 0x8C */
  __IO uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x90 */

} RCC_TypeDef;

 /*--------------------------------------------------------
  FLASH Registers
*--------------------------------------------------------*/

typedef struct
{
  __IO uint32_t ACR;      /*!< FLASH access control register,     Address offset: 0x00 */
  __IO uint32_t KEYR;     /*!< FLASH key register,                Address offset: 0x04 */
  __IO uint32_t OPTKEYR;  /*!< FLASH option key register,         Address offset: 0x08 */
  __IO uint32_t SR;       /*!< FLASH status register,             Address offset: 0x0C */
  __IO uint32_t CR;       /*!< FLASH control register,            Address offset: 0x10 */
  __IO uint32_t OPTCR;    /*!< FLASH option control register ,    Address offset: 0x14 */
  __IO uint32_t OPTCR1;   /*!< FLASH option control register 1 ,  Address offset: 0x18 */
} FLASH_TypeDef;

/*--------------------------------------------------------
  Peripheral memory map
*--------------------------------------------------------*/
#define PERIPH_BASE            0x40000000U /*!< Base address of : AHB/ABP Peripherals */

#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000U)

#define LTDC_BASE             (APB2PERIPH_BASE + 0x6800U)
#define LTDC_Layer1_BASE      (LTDC_BASE + 0x84U)
#define LTDC_Layer2_BASE      (LTDC_BASE + 0x104U)


/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000U)
#define GPIOJ_BASE            (AHB1PERIPH_BASE + 0x2400U)
#define GPIOK_BASE            (AHB1PERIPH_BASE + 0x2800U)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000U)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800U)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00U)

/* Peripheral_declaration */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
#define GPIOJ               ((GPIO_TypeDef *) GPIOJ_BASE)
#define GPIOK               ((GPIO_TypeDef *) GPIOK_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0_Pos            (0U)                                  
#define GPIO_MODER_MODER0_Msk            (0x3U << GPIO_MODER_MODER0_Pos)       /*!< 0x00000003 */
#define GPIO_MODER_MODER0                GPIO_MODER_MODER0_Msk                 
#define GPIO_MODER_MODER0_0              (0x1U << GPIO_MODER_MODER0_Pos)       /*!< 0x00000001 */
#define GPIO_MODER_MODER0_1              (0x2U << GPIO_MODER_MODER0_Pos)       /*!< 0x00000002 */
#define GPIO_MODER_MODER1_Pos            (2U)                                  
#define GPIO_MODER_MODER1_Msk            (0x3U << GPIO_MODER_MODER1_Pos)       /*!< 0x0000000C */
#define GPIO_MODER_MODER1                GPIO_MODER_MODER1_Msk                 
#define GPIO_MODER_MODER1_0              (0x1U << GPIO_MODER_MODER1_Pos)       /*!< 0x00000004 */
#define GPIO_MODER_MODER1_1              (0x2U << GPIO_MODER_MODER1_Pos)       /*!< 0x00000008 */
#define GPIO_MODER_MODER2_Pos            (4U)                                  
#define GPIO_MODER_MODER2_Msk            (0x3U << GPIO_MODER_MODER2_Pos)       /*!< 0x00000030 */
#define GPIO_MODER_MODER2                GPIO_MODER_MODER2_Msk                 
#define GPIO_MODER_MODER2_0              (0x1U << GPIO_MODER_MODER2_Pos)       /*!< 0x00000010 */
#define GPIO_MODER_MODER2_1              (0x2U << GPIO_MODER_MODER2_Pos)       /*!< 0x00000020 */
#define GPIO_MODER_MODER3_Pos            (6U)                                  
#define GPIO_MODER_MODER3_Msk            (0x3U << GPIO_MODER_MODER3_Pos)       /*!< 0x000000C0 */
#define GPIO_MODER_MODER3                GPIO_MODER_MODER3_Msk                 
#define GPIO_MODER_MODER3_0              (0x1U << GPIO_MODER_MODER3_Pos)       /*!< 0x00000040 */
#define GPIO_MODER_MODER3_1              (0x2U << GPIO_MODER_MODER3_Pos)       /*!< 0x00000080 */
#define GPIO_MODER_MODER4_Pos            (8U)                                  
#define GPIO_MODER_MODER4_Msk            (0x3U << GPIO_MODER_MODER4_Pos)       /*!< 0x00000300 */
#define GPIO_MODER_MODER4                GPIO_MODER_MODER4_Msk                 
#define GPIO_MODER_MODER4_0              (0x1U << GPIO_MODER_MODER4_Pos)       /*!< 0x00000100 */
#define GPIO_MODER_MODER4_1              (0x2U << GPIO_MODER_MODER4_Pos)       /*!< 0x00000200 */
#define GPIO_MODER_MODER5_Pos            (10U)                                 
#define GPIO_MODER_MODER5_Msk            (0x3U << GPIO_MODER_MODER5_Pos)       /*!< 0x00000C00 */
#define GPIO_MODER_MODER5                GPIO_MODER_MODER5_Msk                 
#define GPIO_MODER_MODER5_0              (0x1U << GPIO_MODER_MODER5_Pos)       /*!< 0x00000400 */
#define GPIO_MODER_MODER5_1              (0x2U << GPIO_MODER_MODER5_Pos)       /*!< 0x00000800 */
#define GPIO_MODER_MODER6_Pos            (12U)                                 
#define GPIO_MODER_MODER6_Msk            (0x3U << GPIO_MODER_MODER6_Pos)       /*!< 0x00003000 */
#define GPIO_MODER_MODER6                GPIO_MODER_MODER6_Msk                 
#define GPIO_MODER_MODER6_0              (0x1U << GPIO_MODER_MODER6_Pos)       /*!< 0x00001000 */
#define GPIO_MODER_MODER6_1              (0x2U << GPIO_MODER_MODER6_Pos)       /*!< 0x00002000 */
#define GPIO_MODER_MODER7_Pos            (14U)                                 
#define GPIO_MODER_MODER7_Msk            (0x3U << GPIO_MODER_MODER7_Pos)       /*!< 0x0000C000 */
#define GPIO_MODER_MODER7                GPIO_MODER_MODER7_Msk                 
#define GPIO_MODER_MODER7_0              (0x1U << GPIO_MODER_MODER7_Pos)       /*!< 0x00004000 */
#define GPIO_MODER_MODER7_1              (0x2U << GPIO_MODER_MODER7_Pos)       /*!< 0x00008000 */
#define GPIO_MODER_MODER8_Pos            (16U)                                 
#define GPIO_MODER_MODER8_Msk            (0x3U << GPIO_MODER_MODER8_Pos)       /*!< 0x00030000 */
#define GPIO_MODER_MODER8                GPIO_MODER_MODER8_Msk                 
#define GPIO_MODER_MODER8_0              (0x1U << GPIO_MODER_MODER8_Pos)       /*!< 0x00010000 */
#define GPIO_MODER_MODER8_1              (0x2U << GPIO_MODER_MODER8_Pos)       /*!< 0x00020000 */
#define GPIO_MODER_MODER9_Pos            (18U)                                 
#define GPIO_MODER_MODER9_Msk            (0x3U << GPIO_MODER_MODER9_Pos)       /*!< 0x000C0000 */
#define GPIO_MODER_MODER9                GPIO_MODER_MODER9_Msk                 
#define GPIO_MODER_MODER9_0              (0x1U << GPIO_MODER_MODER9_Pos)       /*!< 0x00040000 */
#define GPIO_MODER_MODER9_1              (0x2U << GPIO_MODER_MODER9_Pos)       /*!< 0x00080000 */
#define GPIO_MODER_MODER10_Pos           (20U)                                 
#define GPIO_MODER_MODER10_Msk           (0x3U << GPIO_MODER_MODER10_Pos)      /*!< 0x00300000 */
#define GPIO_MODER_MODER10               GPIO_MODER_MODER10_Msk                
#define GPIO_MODER_MODER10_0             (0x1U << GPIO_MODER_MODER10_Pos)      /*!< 0x00100000 */
#define GPIO_MODER_MODER10_1             (0x2U << GPIO_MODER_MODER10_Pos)      /*!< 0x00200000 */
#define GPIO_MODER_MODER11_Pos           (22U)                                 
#define GPIO_MODER_MODER11_Msk           (0x3U << GPIO_MODER_MODER11_Pos)      /*!< 0x00C00000 */
#define GPIO_MODER_MODER11               GPIO_MODER_MODER11_Msk                
#define GPIO_MODER_MODER11_0             (0x1U << GPIO_MODER_MODER11_Pos)      /*!< 0x00400000 */
#define GPIO_MODER_MODER11_1             (0x2U << GPIO_MODER_MODER11_Pos)      /*!< 0x00800000 */
#define GPIO_MODER_MODER12_Pos           (24U)                                 
#define GPIO_MODER_MODER12_Msk           (0x3U << GPIO_MODER_MODER12_Pos)      /*!< 0x03000000 */
#define GPIO_MODER_MODER12               GPIO_MODER_MODER12_Msk                
#define GPIO_MODER_MODER12_0             (0x1U << GPIO_MODER_MODER12_Pos)      /*!< 0x01000000 */
#define GPIO_MODER_MODER12_1             (0x2U << GPIO_MODER_MODER12_Pos)      /*!< 0x02000000 */
#define GPIO_MODER_MODER13_Pos           (26U)                                 
#define GPIO_MODER_MODER13_Msk           (0x3U << GPIO_MODER_MODER13_Pos)      /*!< 0x0C000000 */
#define GPIO_MODER_MODER13               GPIO_MODER_MODER13_Msk                
#define GPIO_MODER_MODER13_0             (0x1U << GPIO_MODER_MODER13_Pos)      /*!< 0x04000000 */
#define GPIO_MODER_MODER13_1             (0x2U << GPIO_MODER_MODER13_Pos)      /*!< 0x08000000 */
#define GPIO_MODER_MODER14_Pos           (28U)                                 
#define GPIO_MODER_MODER14_Msk           (0x3U << GPIO_MODER_MODER14_Pos)      /*!< 0x30000000 */
#define GPIO_MODER_MODER14               GPIO_MODER_MODER14_Msk                
#define GPIO_MODER_MODER14_0             (0x1U << GPIO_MODER_MODER14_Pos)      /*!< 0x10000000 */
#define GPIO_MODER_MODER14_1             (0x2U << GPIO_MODER_MODER14_Pos)      /*!< 0x20000000 */
#define GPIO_MODER_MODER15_Pos           (30U)                                 
#define GPIO_MODER_MODER15_Msk           (0x3U << GPIO_MODER_MODER15_Pos)      /*!< 0xC0000000 */
#define GPIO_MODER_MODER15               GPIO_MODER_MODER15_Msk                
#define GPIO_MODER_MODER15_0             (0x1U << GPIO_MODER_MODER15_Pos)      /*!< 0x40000000 */
#define GPIO_MODER_MODER15_1             (0x2U << GPIO_MODER_MODER15_Pos)      /*!< 0x80000000 */

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT_0                 0x00000001U                           
#define GPIO_OTYPER_OT_1                 0x00000002U                           
#define GPIO_OTYPER_OT_2                 0x00000004U                           
#define GPIO_OTYPER_OT_3                 0x00000008U                           
#define GPIO_OTYPER_OT_4                 0x00000010U                           
#define GPIO_OTYPER_OT_5                 0x00000020U                           
#define GPIO_OTYPER_OT_6                 0x00000040U                           
#define GPIO_OTYPER_OT_7                 0x00000080U                           
#define GPIO_OTYPER_OT_8                 0x00000100U                           
#define GPIO_OTYPER_OT_9                 0x00000200U                           
#define GPIO_OTYPER_OT_10                0x00000400U                           
#define GPIO_OTYPER_OT_11                0x00000800U                           
#define GPIO_OTYPER_OT_12                0x00001000U                           
#define GPIO_OTYPER_OT_13                0x00002000U                           
#define GPIO_OTYPER_OT_14                0x00004000U                           
#define GPIO_OTYPER_OT_15                0x00008000U                           

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDER_OSPEEDR0_Pos       (0U)                                  
#define GPIO_OSPEEDER_OSPEEDR0_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR0_Pos)  /*!< 0x00000003 */
#define GPIO_OSPEEDER_OSPEEDR0           GPIO_OSPEEDER_OSPEEDR0_Msk            
#define GPIO_OSPEEDER_OSPEEDR0_0         (0x1U << GPIO_OSPEEDER_OSPEEDR0_Pos)  /*!< 0x00000001 */
#define GPIO_OSPEEDER_OSPEEDR0_1         (0x2U << GPIO_OSPEEDER_OSPEEDR0_Pos)  /*!< 0x00000002 */
#define GPIO_OSPEEDER_OSPEEDR1_Pos       (2U)                                  
#define GPIO_OSPEEDER_OSPEEDR1_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR1_Pos)  /*!< 0x0000000C */
#define GPIO_OSPEEDER_OSPEEDR1           GPIO_OSPEEDER_OSPEEDR1_Msk            
#define GPIO_OSPEEDER_OSPEEDR1_0         (0x1U << GPIO_OSPEEDER_OSPEEDR1_Pos)  /*!< 0x00000004 */
#define GPIO_OSPEEDER_OSPEEDR1_1         (0x2U << GPIO_OSPEEDER_OSPEEDR1_Pos)  /*!< 0x00000008 */
#define GPIO_OSPEEDER_OSPEEDR2_Pos       (4U)                                  
#define GPIO_OSPEEDER_OSPEEDR2_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR2_Pos)  /*!< 0x00000030 */
#define GPIO_OSPEEDER_OSPEEDR2           GPIO_OSPEEDER_OSPEEDR2_Msk            
#define GPIO_OSPEEDER_OSPEEDR2_0         (0x1U << GPIO_OSPEEDER_OSPEEDR2_Pos)  /*!< 0x00000010 */
#define GPIO_OSPEEDER_OSPEEDR2_1         (0x2U << GPIO_OSPEEDER_OSPEEDR2_Pos)  /*!< 0x00000020 */
#define GPIO_OSPEEDER_OSPEEDR3_Pos       (6U)                                  
#define GPIO_OSPEEDER_OSPEEDR3_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR3_Pos)  /*!< 0x000000C0 */
#define GPIO_OSPEEDER_OSPEEDR3           GPIO_OSPEEDER_OSPEEDR3_Msk            
#define GPIO_OSPEEDER_OSPEEDR3_0         (0x1U << GPIO_OSPEEDER_OSPEEDR3_Pos)  /*!< 0x00000040 */
#define GPIO_OSPEEDER_OSPEEDR3_1         (0x2U << GPIO_OSPEEDER_OSPEEDR3_Pos)  /*!< 0x00000080 */
#define GPIO_OSPEEDER_OSPEEDR4_Pos       (8U)                                  
#define GPIO_OSPEEDER_OSPEEDR4_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR4_Pos)  /*!< 0x00000300 */
#define GPIO_OSPEEDER_OSPEEDR4           GPIO_OSPEEDER_OSPEEDR4_Msk            
#define GPIO_OSPEEDER_OSPEEDR4_0         (0x1U << GPIO_OSPEEDER_OSPEEDR4_Pos)  /*!< 0x00000100 */
#define GPIO_OSPEEDER_OSPEEDR4_1         (0x2U << GPIO_OSPEEDER_OSPEEDR4_Pos)  /*!< 0x00000200 */
#define GPIO_OSPEEDER_OSPEEDR5_Pos       (10U)                                 
#define GPIO_OSPEEDER_OSPEEDR5_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR5_Pos)  /*!< 0x00000C00 */
#define GPIO_OSPEEDER_OSPEEDR5           GPIO_OSPEEDER_OSPEEDR5_Msk            
#define GPIO_OSPEEDER_OSPEEDR5_0         (0x1U << GPIO_OSPEEDER_OSPEEDR5_Pos)  /*!< 0x00000400 */
#define GPIO_OSPEEDER_OSPEEDR5_1         (0x2U << GPIO_OSPEEDER_OSPEEDR5_Pos)  /*!< 0x00000800 */
#define GPIO_OSPEEDER_OSPEEDR6_Pos       (12U)                                 
#define GPIO_OSPEEDER_OSPEEDR6_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR6_Pos)  /*!< 0x00003000 */
#define GPIO_OSPEEDER_OSPEEDR6           GPIO_OSPEEDER_OSPEEDR6_Msk            
#define GPIO_OSPEEDER_OSPEEDR6_0         (0x1U << GPIO_OSPEEDER_OSPEEDR6_Pos)  /*!< 0x00001000 */
#define GPIO_OSPEEDER_OSPEEDR6_1         (0x2U << GPIO_OSPEEDER_OSPEEDR6_Pos)  /*!< 0x00002000 */
#define GPIO_OSPEEDER_OSPEEDR7_Pos       (14U)                                 
#define GPIO_OSPEEDER_OSPEEDR7_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR7_Pos)  /*!< 0x0000C000 */
#define GPIO_OSPEEDER_OSPEEDR7           GPIO_OSPEEDER_OSPEEDR7_Msk            
#define GPIO_OSPEEDER_OSPEEDR7_0         (0x1U << GPIO_OSPEEDER_OSPEEDR7_Pos)  /*!< 0x00004000 */
#define GPIO_OSPEEDER_OSPEEDR7_1         (0x2U << GPIO_OSPEEDER_OSPEEDR7_Pos)  /*!< 0x00008000 */
#define GPIO_OSPEEDER_OSPEEDR8_Pos       (16U)                                 
#define GPIO_OSPEEDER_OSPEEDR8_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR8_Pos)  /*!< 0x00030000 */
#define GPIO_OSPEEDER_OSPEEDR8           GPIO_OSPEEDER_OSPEEDR8_Msk            
#define GPIO_OSPEEDER_OSPEEDR8_0         (0x1U << GPIO_OSPEEDER_OSPEEDR8_Pos)  /*!< 0x00010000 */
#define GPIO_OSPEEDER_OSPEEDR8_1         (0x2U << GPIO_OSPEEDER_OSPEEDR8_Pos)  /*!< 0x00020000 */
#define GPIO_OSPEEDER_OSPEEDR9_Pos       (18U)                                 
#define GPIO_OSPEEDER_OSPEEDR9_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR9_Pos)  /*!< 0x000C0000 */
#define GPIO_OSPEEDER_OSPEEDR9           GPIO_OSPEEDER_OSPEEDR9_Msk            
#define GPIO_OSPEEDER_OSPEEDR9_0         (0x1U << GPIO_OSPEEDER_OSPEEDR9_Pos)  /*!< 0x00040000 */
#define GPIO_OSPEEDER_OSPEEDR9_1         (0x2U << GPIO_OSPEEDER_OSPEEDR9_Pos)  /*!< 0x00080000 */
#define GPIO_OSPEEDER_OSPEEDR10_Pos      (20U)                                 
#define GPIO_OSPEEDER_OSPEEDR10_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR10_Pos) /*!< 0x00300000 */
#define GPIO_OSPEEDER_OSPEEDR10          GPIO_OSPEEDER_OSPEEDR10_Msk           
#define GPIO_OSPEEDER_OSPEEDR10_0        (0x1U << GPIO_OSPEEDER_OSPEEDR10_Pos) /*!< 0x00100000 */
#define GPIO_OSPEEDER_OSPEEDR10_1        (0x2U << GPIO_OSPEEDER_OSPEEDR10_Pos) /*!< 0x00200000 */
#define GPIO_OSPEEDER_OSPEEDR11_Pos      (22U)                                 
#define GPIO_OSPEEDER_OSPEEDR11_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR11_Pos) /*!< 0x00C00000 */
#define GPIO_OSPEEDER_OSPEEDR11          GPIO_OSPEEDER_OSPEEDR11_Msk           
#define GPIO_OSPEEDER_OSPEEDR11_0        (0x1U << GPIO_OSPEEDER_OSPEEDR11_Pos) /*!< 0x00400000 */
#define GPIO_OSPEEDER_OSPEEDR11_1        (0x2U << GPIO_OSPEEDER_OSPEEDR11_Pos) /*!< 0x00800000 */
#define GPIO_OSPEEDER_OSPEEDR12_Pos      (24U)                                 
#define GPIO_OSPEEDER_OSPEEDR12_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR12_Pos) /*!< 0x03000000 */
#define GPIO_OSPEEDER_OSPEEDR12          GPIO_OSPEEDER_OSPEEDR12_Msk           
#define GPIO_OSPEEDER_OSPEEDR12_0        (0x1U << GPIO_OSPEEDER_OSPEEDR12_Pos) /*!< 0x01000000 */
#define GPIO_OSPEEDER_OSPEEDR12_1        (0x2U << GPIO_OSPEEDER_OSPEEDR12_Pos) /*!< 0x02000000 */
#define GPIO_OSPEEDER_OSPEEDR13_Pos      (26U)                                 
#define GPIO_OSPEEDER_OSPEEDR13_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR13_Pos) /*!< 0x0C000000 */
#define GPIO_OSPEEDER_OSPEEDR13          GPIO_OSPEEDER_OSPEEDR13_Msk           
#define GPIO_OSPEEDER_OSPEEDR13_0        (0x1U << GPIO_OSPEEDER_OSPEEDR13_Pos) /*!< 0x04000000 */
#define GPIO_OSPEEDER_OSPEEDR13_1        (0x2U << GPIO_OSPEEDER_OSPEEDR13_Pos) /*!< 0x08000000 */
#define GPIO_OSPEEDER_OSPEEDR14_Pos      (28U)                                 
#define GPIO_OSPEEDER_OSPEEDR14_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR14_Pos) /*!< 0x30000000 */
#define GPIO_OSPEEDER_OSPEEDR14          GPIO_OSPEEDER_OSPEEDR14_Msk           
#define GPIO_OSPEEDER_OSPEEDR14_0        (0x1U << GPIO_OSPEEDER_OSPEEDR14_Pos) /*!< 0x10000000 */
#define GPIO_OSPEEDER_OSPEEDR14_1        (0x2U << GPIO_OSPEEDER_OSPEEDR14_Pos) /*!< 0x20000000 */
#define GPIO_OSPEEDER_OSPEEDR15_Pos      (30U)                                 
#define GPIO_OSPEEDER_OSPEEDR15_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR15_Pos) /*!< 0xC0000000 */
#define GPIO_OSPEEDER_OSPEEDR15          GPIO_OSPEEDER_OSPEEDR15_Msk           
#define GPIO_OSPEEDER_OSPEEDR15_0        (0x1U << GPIO_OSPEEDER_OSPEEDR15_Pos) /*!< 0x40000000 */
#define GPIO_OSPEEDER_OSPEEDR15_1        (0x2U << GPIO_OSPEEDER_OSPEEDR15_Pos) /*!< 0x80000000 */

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPDR0_Pos            (0U)                                  
#define GPIO_PUPDR_PUPDR0_Msk            (0x3U << GPIO_PUPDR_PUPDR0_Pos)       /*!< 0x00000003 */
#define GPIO_PUPDR_PUPDR0                GPIO_PUPDR_PUPDR0_Msk                 
#define GPIO_PUPDR_PUPDR0_0              (0x1U << GPIO_PUPDR_PUPDR0_Pos)       /*!< 0x00000001 */
#define GPIO_PUPDR_PUPDR0_1              (0x2U << GPIO_PUPDR_PUPDR0_Pos)       /*!< 0x00000002 */
#define GPIO_PUPDR_PUPDR1_Pos            (2U)                                  
#define GPIO_PUPDR_PUPDR1_Msk            (0x3U << GPIO_PUPDR_PUPDR1_Pos)       /*!< 0x0000000C */
#define GPIO_PUPDR_PUPDR1                GPIO_PUPDR_PUPDR1_Msk                 
#define GPIO_PUPDR_PUPDR1_0              (0x1U << GPIO_PUPDR_PUPDR1_Pos)       /*!< 0x00000004 */
#define GPIO_PUPDR_PUPDR1_1              (0x2U << GPIO_PUPDR_PUPDR1_Pos)       /*!< 0x00000008 */
#define GPIO_PUPDR_PUPDR2_Pos            (4U)                                  
#define GPIO_PUPDR_PUPDR2_Msk            (0x3U << GPIO_PUPDR_PUPDR2_Pos)       /*!< 0x00000030 */
#define GPIO_PUPDR_PUPDR2                GPIO_PUPDR_PUPDR2_Msk                 
#define GPIO_PUPDR_PUPDR2_0              (0x1U << GPIO_PUPDR_PUPDR2_Pos)       /*!< 0x00000010 */
#define GPIO_PUPDR_PUPDR2_1              (0x2U << GPIO_PUPDR_PUPDR2_Pos)       /*!< 0x00000020 */
#define GPIO_PUPDR_PUPDR3_Pos            (6U)                                  
#define GPIO_PUPDR_PUPDR3_Msk            (0x3U << GPIO_PUPDR_PUPDR3_Pos)       /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPDR3                GPIO_PUPDR_PUPDR3_Msk                 
#define GPIO_PUPDR_PUPDR3_0              (0x1U << GPIO_PUPDR_PUPDR3_Pos)       /*!< 0x00000040 */
#define GPIO_PUPDR_PUPDR3_1              (0x2U << GPIO_PUPDR_PUPDR3_Pos)       /*!< 0x00000080 */
#define GPIO_PUPDR_PUPDR4_Pos            (8U)                                  
#define GPIO_PUPDR_PUPDR4_Msk            (0x3U << GPIO_PUPDR_PUPDR4_Pos)       /*!< 0x00000300 */
#define GPIO_PUPDR_PUPDR4                GPIO_PUPDR_PUPDR4_Msk                 
#define GPIO_PUPDR_PUPDR4_0              (0x1U << GPIO_PUPDR_PUPDR4_Pos)       /*!< 0x00000100 */
#define GPIO_PUPDR_PUPDR4_1              (0x2U << GPIO_PUPDR_PUPDR4_Pos)       /*!< 0x00000200 */
#define GPIO_PUPDR_PUPDR5_Pos            (10U)                                 
#define GPIO_PUPDR_PUPDR5_Msk            (0x3U << GPIO_PUPDR_PUPDR5_Pos)       /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPDR5                GPIO_PUPDR_PUPDR5_Msk                 
#define GPIO_PUPDR_PUPDR5_0              (0x1U << GPIO_PUPDR_PUPDR5_Pos)       /*!< 0x00000400 */
#define GPIO_PUPDR_PUPDR5_1              (0x2U << GPIO_PUPDR_PUPDR5_Pos)       /*!< 0x00000800 */
#define GPIO_PUPDR_PUPDR6_Pos            (12U)                                 
#define GPIO_PUPDR_PUPDR6_Msk            (0x3U << GPIO_PUPDR_PUPDR6_Pos)       /*!< 0x00003000 */
#define GPIO_PUPDR_PUPDR6                GPIO_PUPDR_PUPDR6_Msk                 
#define GPIO_PUPDR_PUPDR6_0              (0x1U << GPIO_PUPDR_PUPDR6_Pos)       /*!< 0x00001000 */
#define GPIO_PUPDR_PUPDR6_1              (0x2U << GPIO_PUPDR_PUPDR6_Pos)       /*!< 0x00002000 */
#define GPIO_PUPDR_PUPDR7_Pos            (14U)                                 
#define GPIO_PUPDR_PUPDR7_Msk            (0x3U << GPIO_PUPDR_PUPDR7_Pos)       /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPDR7                GPIO_PUPDR_PUPDR7_Msk                 
#define GPIO_PUPDR_PUPDR7_0              (0x1U << GPIO_PUPDR_PUPDR7_Pos)       /*!< 0x00004000 */
#define GPIO_PUPDR_PUPDR7_1              (0x2U << GPIO_PUPDR_PUPDR7_Pos)       /*!< 0x00008000 */
#define GPIO_PUPDR_PUPDR8_Pos            (16U)                                 
#define GPIO_PUPDR_PUPDR8_Msk            (0x3U << GPIO_PUPDR_PUPDR8_Pos)       /*!< 0x00030000 */
#define GPIO_PUPDR_PUPDR8                GPIO_PUPDR_PUPDR8_Msk                 
#define GPIO_PUPDR_PUPDR8_0              (0x1U << GPIO_PUPDR_PUPDR8_Pos)       /*!< 0x00010000 */
#define GPIO_PUPDR_PUPDR8_1              (0x2U << GPIO_PUPDR_PUPDR8_Pos)       /*!< 0x00020000 */
#define GPIO_PUPDR_PUPDR9_Pos            (18U)                                 
#define GPIO_PUPDR_PUPDR9_Msk            (0x3U << GPIO_PUPDR_PUPDR9_Pos)       /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPDR9                GPIO_PUPDR_PUPDR9_Msk                 
#define GPIO_PUPDR_PUPDR9_0              (0x1U << GPIO_PUPDR_PUPDR9_Pos)       /*!< 0x00040000 */
#define GPIO_PUPDR_PUPDR9_1              (0x2U << GPIO_PUPDR_PUPDR9_Pos)       /*!< 0x00080000 */
#define GPIO_PUPDR_PUPDR10_Pos           (20U)                                 
#define GPIO_PUPDR_PUPDR10_Msk           (0x3U << GPIO_PUPDR_PUPDR10_Pos)      /*!< 0x00300000 */
#define GPIO_PUPDR_PUPDR10               GPIO_PUPDR_PUPDR10_Msk                
#define GPIO_PUPDR_PUPDR10_0             (0x1U << GPIO_PUPDR_PUPDR10_Pos)      /*!< 0x00100000 */
#define GPIO_PUPDR_PUPDR10_1             (0x2U << GPIO_PUPDR_PUPDR10_Pos)      /*!< 0x00200000 */
#define GPIO_PUPDR_PUPDR11_Pos           (22U)                                 
#define GPIO_PUPDR_PUPDR11_Msk           (0x3U << GPIO_PUPDR_PUPDR11_Pos)      /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPDR11               GPIO_PUPDR_PUPDR11_Msk                
#define GPIO_PUPDR_PUPDR11_0             (0x1U << GPIO_PUPDR_PUPDR11_Pos)      /*!< 0x00400000 */
#define GPIO_PUPDR_PUPDR11_1             (0x2U << GPIO_PUPDR_PUPDR11_Pos)      /*!< 0x00800000 */
#define GPIO_PUPDR_PUPDR12_Pos           (24U)                                 
#define GPIO_PUPDR_PUPDR12_Msk           (0x3U << GPIO_PUPDR_PUPDR12_Pos)      /*!< 0x03000000 */
#define GPIO_PUPDR_PUPDR12               GPIO_PUPDR_PUPDR12_Msk                
#define GPIO_PUPDR_PUPDR12_0             (0x1U << GPIO_PUPDR_PUPDR12_Pos)      /*!< 0x01000000 */
#define GPIO_PUPDR_PUPDR12_1             (0x2U << GPIO_PUPDR_PUPDR12_Pos)      /*!< 0x02000000 */
#define GPIO_PUPDR_PUPDR13_Pos           (26U)                                 
#define GPIO_PUPDR_PUPDR13_Msk           (0x3U << GPIO_PUPDR_PUPDR13_Pos)      /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPDR13               GPIO_PUPDR_PUPDR13_Msk                
#define GPIO_PUPDR_PUPDR13_0             (0x1U << GPIO_PUPDR_PUPDR13_Pos)      /*!< 0x04000000 */
#define GPIO_PUPDR_PUPDR13_1             (0x2U << GPIO_PUPDR_PUPDR13_Pos)      /*!< 0x08000000 */
#define GPIO_PUPDR_PUPDR14_Pos           (28U)                                 
#define GPIO_PUPDR_PUPDR14_Msk           (0x3U << GPIO_PUPDR_PUPDR14_Pos)      /*!< 0x30000000 */
#define GPIO_PUPDR_PUPDR14               GPIO_PUPDR_PUPDR14_Msk                
#define GPIO_PUPDR_PUPDR14_0             (0x1U << GPIO_PUPDR_PUPDR14_Pos)      /*!< 0x10000000 */
#define GPIO_PUPDR_PUPDR14_1             (0x2U << GPIO_PUPDR_PUPDR14_Pos)      /*!< 0x20000000 */
#define GPIO_PUPDR_PUPDR15_Pos           (30U)                                 
#define GPIO_PUPDR_PUPDR15_Msk           (0x3U << GPIO_PUPDR_PUPDR15_Pos)      /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPDR15               GPIO_PUPDR_PUPDR15_Msk                
#define GPIO_PUPDR_PUPDR15_0             (0x1U << GPIO_PUPDR_PUPDR15_Pos)      /*!< 0x40000000 */
#define GPIO_PUPDR_PUPDR15_1             (0x2U << GPIO_PUPDR_PUPDR15_Pos)      /*!< 0x80000000 */

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR_0                   0x00000001U                           
#define GPIO_IDR_IDR_1                   0x00000002U                           
#define GPIO_IDR_IDR_2                   0x00000004U                           
#define GPIO_IDR_IDR_3                   0x00000008U                           
#define GPIO_IDR_IDR_4                   0x00000010U                           
#define GPIO_IDR_IDR_5                   0x00000020U                           
#define GPIO_IDR_IDR_6                   0x00000040U                           
#define GPIO_IDR_IDR_7                   0x00000080U                           
#define GPIO_IDR_IDR_8                   0x00000100U                           
#define GPIO_IDR_IDR_9                   0x00000200U                           
#define GPIO_IDR_IDR_10                  0x00000400U                           
#define GPIO_IDR_IDR_11                  0x00000800U                           
#define GPIO_IDR_IDR_12                  0x00001000U                           
#define GPIO_IDR_IDR_13                  0x00002000U                           
#define GPIO_IDR_IDR_14                  0x00004000U                           
#define GPIO_IDR_IDR_15                  0x00008000U                           

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR_0                   0x00000001U                           
#define GPIO_ODR_ODR_1                   0x00000002U                           
#define GPIO_ODR_ODR_2                   0x00000004U                           
#define GPIO_ODR_ODR_3                   0x00000008U                           
#define GPIO_ODR_ODR_4                   0x00000010U                           
#define GPIO_ODR_ODR_5                   0x00000020U                           
#define GPIO_ODR_ODR_6                   0x00000040U                           
#define GPIO_ODR_ODR_7                   0x00000080U                           
#define GPIO_ODR_ODR_8                   0x00000100U                           
#define GPIO_ODR_ODR_9                   0x00000200U                           
#define GPIO_ODR_ODR_10                  0x00000400U                           
#define GPIO_ODR_ODR_11                  0x00000800U                           
#define GPIO_ODR_ODR_12                  0x00001000U                           
#define GPIO_ODR_ODR_13                  0x00002000U                           
#define GPIO_ODR_ODR_14                  0x00004000U                           
#define GPIO_ODR_ODR_15                  0x00008000U                           

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS_0                   0x00000001U                           
#define GPIO_BSRR_BS_1                   0x00000002U                           
#define GPIO_BSRR_BS_2                   0x00000004U                           
#define GPIO_BSRR_BS_3                   0x00000008U                           
#define GPIO_BSRR_BS_4                   0x00000010U                           
#define GPIO_BSRR_BS_5                   0x00000020U                           
#define GPIO_BSRR_BS_6                   0x00000040U                           
#define GPIO_BSRR_BS_7                   0x00000080U                           
#define GPIO_BSRR_BS_8                   0x00000100U                           
#define GPIO_BSRR_BS_9                   0x00000200U                           
#define GPIO_BSRR_BS_10                  0x00000400U                           
#define GPIO_BSRR_BS_11                  0x00000800U                           
#define GPIO_BSRR_BS_12                  0x00001000U                           
#define GPIO_BSRR_BS_13                  0x00002000U                           
#define GPIO_BSRR_BS_14                  0x00004000U                           
#define GPIO_BSRR_BS_15                  0x00008000U                           
#define GPIO_BSRR_BR_0                   0x00010000U                           
#define GPIO_BSRR_BR_1                   0x00020000U                           
#define GPIO_BSRR_BR_2                   0x00040000U                           
#define GPIO_BSRR_BR_3                   0x00080000U                           
#define GPIO_BSRR_BR_4                   0x00100000U                           
#define GPIO_BSRR_BR_5                   0x00200000U                           
#define GPIO_BSRR_BR_6                   0x00400000U                           
#define GPIO_BSRR_BR_7                   0x00800000U                           
#define GPIO_BSRR_BR_8                   0x01000000U                           
#define GPIO_BSRR_BR_9                   0x02000000U                           
#define GPIO_BSRR_BR_10                  0x04000000U                           
#define GPIO_BSRR_BR_11                  0x08000000U                           
#define GPIO_BSRR_BR_12                  0x10000000U                           
#define GPIO_BSRR_BR_13                  0x20000000U                           
#define GPIO_BSRR_BR_14                  0x40000000U                           
#define GPIO_BSRR_BR_15                  0x80000000U                           

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0_Pos               (0U)                                  
#define GPIO_LCKR_LCK0_Msk               (0x1U << GPIO_LCKR_LCK0_Pos)          /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                   GPIO_LCKR_LCK0_Msk                    
#define GPIO_LCKR_LCK1_Pos               (1U)                                  
#define GPIO_LCKR_LCK1_Msk               (0x1U << GPIO_LCKR_LCK1_Pos)          /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                   GPIO_LCKR_LCK1_Msk                    
#define GPIO_LCKR_LCK2_Pos               (2U)                                  
#define GPIO_LCKR_LCK2_Msk               (0x1U << GPIO_LCKR_LCK2_Pos)          /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                   GPIO_LCKR_LCK2_Msk                    
#define GPIO_LCKR_LCK3_Pos               (3U)                                  
#define GPIO_LCKR_LCK3_Msk               (0x1U << GPIO_LCKR_LCK3_Pos)          /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                   GPIO_LCKR_LCK3_Msk                    
#define GPIO_LCKR_LCK4_Pos               (4U)                                  
#define GPIO_LCKR_LCK4_Msk               (0x1U << GPIO_LCKR_LCK4_Pos)          /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                   GPIO_LCKR_LCK4_Msk                    
#define GPIO_LCKR_LCK5_Pos               (5U)                                  
#define GPIO_LCKR_LCK5_Msk               (0x1U << GPIO_LCKR_LCK5_Pos)          /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                   GPIO_LCKR_LCK5_Msk                    
#define GPIO_LCKR_LCK6_Pos               (6U)                                  
#define GPIO_LCKR_LCK6_Msk               (0x1U << GPIO_LCKR_LCK6_Pos)          /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                   GPIO_LCKR_LCK6_Msk                    
#define GPIO_LCKR_LCK7_Pos               (7U)                                  
#define GPIO_LCKR_LCK7_Msk               (0x1U << GPIO_LCKR_LCK7_Pos)          /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                   GPIO_LCKR_LCK7_Msk                    
#define GPIO_LCKR_LCK8_Pos               (8U)                                  
#define GPIO_LCKR_LCK8_Msk               (0x1U << GPIO_LCKR_LCK8_Pos)          /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                   GPIO_LCKR_LCK8_Msk                    
#define GPIO_LCKR_LCK9_Pos               (9U)                                  
#define GPIO_LCKR_LCK9_Msk               (0x1U << GPIO_LCKR_LCK9_Pos)          /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                   GPIO_LCKR_LCK9_Msk                    
#define GPIO_LCKR_LCK10_Pos              (10U)                                 
#define GPIO_LCKR_LCK10_Msk              (0x1U << GPIO_LCKR_LCK10_Pos)         /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                  GPIO_LCKR_LCK10_Msk                   
#define GPIO_LCKR_LCK11_Pos              (11U)                                 
#define GPIO_LCKR_LCK11_Msk              (0x1U << GPIO_LCKR_LCK11_Pos)         /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                  GPIO_LCKR_LCK11_Msk                   
#define GPIO_LCKR_LCK12_Pos              (12U)                                 
#define GPIO_LCKR_LCK12_Msk              (0x1U << GPIO_LCKR_LCK12_Pos)         /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                  GPIO_LCKR_LCK12_Msk                   
#define GPIO_LCKR_LCK13_Pos              (13U)                                 
#define GPIO_LCKR_LCK13_Msk              (0x1U << GPIO_LCKR_LCK13_Pos)         /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                  GPIO_LCKR_LCK13_Msk                   
#define GPIO_LCKR_LCK14_Pos              (14U)                                 
#define GPIO_LCKR_LCK14_Msk              (0x1U << GPIO_LCKR_LCK14_Pos)         /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                  GPIO_LCKR_LCK14_Msk                   
#define GPIO_LCKR_LCK15_Pos              (15U)                                 
#define GPIO_LCKR_LCK15_Msk              (0x1U << GPIO_LCKR_LCK15_Pos)         /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                  GPIO_LCKR_LCK15_Msk                   
#define GPIO_LCKR_LCKK_Pos               (16U)                                 
#define GPIO_LCKR_LCKK_Msk               (0x1U << GPIO_LCKR_LCKK_Pos)          /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                   GPIO_LCKR_LCKK_Msk                    

/****************** Bit definition for GPIO_AFRL register *********************/
#define GPIO_AFRL_AFRL0_Pos              (0U)                                  
#define GPIO_AFRL_AFRL0_Msk              (0xFU << GPIO_AFRL_AFRL0_Pos)         /*!< 0x0000000F */
#define GPIO_AFRL_AFRL0                  GPIO_AFRL_AFRL0_Msk                   
#define GPIO_AFRL_AFRL0_0                (0x1U << GPIO_AFRL_AFRL0_Pos)         /*!< 0x00000001 */
#define GPIO_AFRL_AFRL0_1                (0x2U << GPIO_AFRL_AFRL0_Pos)         /*!< 0x00000002 */
#define GPIO_AFRL_AFRL0_2                (0x4U << GPIO_AFRL_AFRL0_Pos)         /*!< 0x00000004 */
#define GPIO_AFRL_AFRL0_3                (0x8U << GPIO_AFRL_AFRL0_Pos)         /*!< 0x00000008 */
#define GPIO_AFRL_AFRL1_Pos              (4U)                                  
#define GPIO_AFRL_AFRL1_Msk              (0xFU << GPIO_AFRL_AFRL1_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRL_AFRL1                  GPIO_AFRL_AFRL1_Msk                   
#define GPIO_AFRL_AFRL1_0                (0x1U << GPIO_AFRL_AFRL1_Pos)         /*!< 0x00000010 */
#define GPIO_AFRL_AFRL1_1                (0x2U << GPIO_AFRL_AFRL1_Pos)         /*!< 0x00000020 */
#define GPIO_AFRL_AFRL1_2                (0x4U << GPIO_AFRL_AFRL1_Pos)         /*!< 0x00000040 */
#define GPIO_AFRL_AFRL1_3                (0x8U << GPIO_AFRL_AFRL1_Pos)         /*!< 0x00000080 */
#define GPIO_AFRL_AFRL2_Pos              (8U)                                  
#define GPIO_AFRL_AFRL2_Msk              (0xFU << GPIO_AFRL_AFRL2_Pos)         /*!< 0x00000F00 */
#define GPIO_AFRL_AFRL2                  GPIO_AFRL_AFRL2_Msk                   
#define GPIO_AFRL_AFRL2_0                (0x1U << GPIO_AFRL_AFRL2_Pos)         /*!< 0x00000100 */
#define GPIO_AFRL_AFRL2_1                (0x2U << GPIO_AFRL_AFRL2_Pos)         /*!< 0x00000200 */
#define GPIO_AFRL_AFRL2_2                (0x4U << GPIO_AFRL_AFRL2_Pos)         /*!< 0x00000400 */
#define GPIO_AFRL_AFRL2_3                (0x8U << GPIO_AFRL_AFRL2_Pos)         /*!< 0x00000800 */
#define GPIO_AFRL_AFRL3_Pos              (12U)                                 
#define GPIO_AFRL_AFRL3_Msk              (0xFU << GPIO_AFRL_AFRL3_Pos)         /*!< 0x0000F000 */
#define GPIO_AFRL_AFRL3                  GPIO_AFRL_AFRL3_Msk                   
#define GPIO_AFRL_AFRL3_0                (0x1U << GPIO_AFRL_AFRL3_Pos)         /*!< 0x00001000 */
#define GPIO_AFRL_AFRL3_1                (0x2U << GPIO_AFRL_AFRL3_Pos)         /*!< 0x00002000 */
#define GPIO_AFRL_AFRL3_2                (0x4U << GPIO_AFRL_AFRL3_Pos)         /*!< 0x00004000 */
#define GPIO_AFRL_AFRL3_3                (0x8U << GPIO_AFRL_AFRL3_Pos)         /*!< 0x00008000 */
#define GPIO_AFRL_AFRL4_Pos              (16U)                                 
#define GPIO_AFRL_AFRL4_Msk              (0xFU << GPIO_AFRL_AFRL4_Pos)         /*!< 0x000F0000 */
#define GPIO_AFRL_AFRL4                  GPIO_AFRL_AFRL4_Msk                   
#define GPIO_AFRL_AFRL4_0                (0x1U << GPIO_AFRL_AFRL4_Pos)         /*!< 0x00010000 */
#define GPIO_AFRL_AFRL4_1                (0x2U << GPIO_AFRL_AFRL4_Pos)         /*!< 0x00020000 */
#define GPIO_AFRL_AFRL4_2                (0x4U << GPIO_AFRL_AFRL4_Pos)         /*!< 0x00040000 */
#define GPIO_AFRL_AFRL4_3                (0x8U << GPIO_AFRL_AFRL4_Pos)         /*!< 0x00080000 */
#define GPIO_AFRL_AFRL5_Pos              (20U)                                 
#define GPIO_AFRL_AFRL5_Msk              (0xFU << GPIO_AFRL_AFRL5_Pos)         /*!< 0x00F00000 */
#define GPIO_AFRL_AFRL5                  GPIO_AFRL_AFRL5_Msk                   
#define GPIO_AFRL_AFRL5_0                (0x1U << GPIO_AFRL_AFRL5_Pos)         /*!< 0x00100000 */
#define GPIO_AFRL_AFRL5_1                (0x2U << GPIO_AFRL_AFRL5_Pos)         /*!< 0x00200000 */
#define GPIO_AFRL_AFRL5_2                (0x4U << GPIO_AFRL_AFRL5_Pos)         /*!< 0x00400000 */
#define GPIO_AFRL_AFRL5_3                (0x8U << GPIO_AFRL_AFRL5_Pos)         /*!< 0x00800000 */
#define GPIO_AFRL_AFRL6_Pos              (24U)                                 
#define GPIO_AFRL_AFRL6_Msk              (0xFU << GPIO_AFRL_AFRL6_Pos)         /*!< 0x0F000000 */
#define GPIO_AFRL_AFRL6                  GPIO_AFRL_AFRL6_Msk                   
#define GPIO_AFRL_AFRL6_0                (0x1U << GPIO_AFRL_AFRL6_Pos)         /*!< 0x01000000 */
#define GPIO_AFRL_AFRL6_1                (0x2U << GPIO_AFRL_AFRL6_Pos)         /*!< 0x02000000 */
#define GPIO_AFRL_AFRL6_2                (0x4U << GPIO_AFRL_AFRL6_Pos)         /*!< 0x04000000 */
#define GPIO_AFRL_AFRL6_3                (0x8U << GPIO_AFRL_AFRL6_Pos)         /*!< 0x08000000 */
#define GPIO_AFRL_AFRL7_Pos              (28U)                                 
#define GPIO_AFRL_AFRL7_Msk              (0xFU << GPIO_AFRL_AFRL7_Pos)         /*!< 0xF0000000 */
#define GPIO_AFRL_AFRL7                  GPIO_AFRL_AFRL7_Msk                   
#define GPIO_AFRL_AFRL7_0                (0x1U << GPIO_AFRL_AFRL7_Pos)         /*!< 0x10000000 */
#define GPIO_AFRL_AFRL7_1                (0x2U << GPIO_AFRL_AFRL7_Pos)         /*!< 0x20000000 */
#define GPIO_AFRL_AFRL7_2                (0x4U << GPIO_AFRL_AFRL7_Pos)         /*!< 0x40000000 */
#define GPIO_AFRL_AFRL7_3                (0x8U << GPIO_AFRL_AFRL7_Pos)         /*!< 0x80000000 */

/****************** Bit definition for GPIO_AFRH register *********************/
#define GPIO_AFRH_AFRH0_Pos              (0U)                                  
#define GPIO_AFRH_AFRH0_Msk              (0xFU << GPIO_AFRH_AFRH0_Pos)         /*!< 0x0000000F */
#define GPIO_AFRH_AFRH0                  GPIO_AFRH_AFRH0_Msk                   
#define GPIO_AFRH_AFRH0_0                (0x1U << GPIO_AFRH_AFRH0_Pos)         /*!< 0x00000001 */
#define GPIO_AFRH_AFRH0_1                (0x2U << GPIO_AFRH_AFRH0_Pos)         /*!< 0x00000002 */
#define GPIO_AFRH_AFRH0_2                (0x4U << GPIO_AFRH_AFRH0_Pos)         /*!< 0x00000004 */
#define GPIO_AFRH_AFRH0_3                (0x8U << GPIO_AFRH_AFRH0_Pos)         /*!< 0x00000008 */
#define GPIO_AFRH_AFRH1_Pos              (4U)                                  
#define GPIO_AFRH_AFRH1_Msk              (0xFU << GPIO_AFRH_AFRH1_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRH_AFRH1                  GPIO_AFRH_AFRH1_Msk                   
#define GPIO_AFRH_AFRH1_0                (0x1U << GPIO_AFRH_AFRH1_Pos)         /*!< 0x00000010 */
#define GPIO_AFRH_AFRH1_1                (0x2U << GPIO_AFRH_AFRH1_Pos)         /*!< 0x00000020 */
#define GPIO_AFRH_AFRH1_2                (0x4U << GPIO_AFRH_AFRH1_Pos)         /*!< 0x00000040 */
#define GPIO_AFRH_AFRH1_3                (0x8U << GPIO_AFRH_AFRH1_Pos)         /*!< 0x00000080 */
#define GPIO_AFRH_AFRH2_Pos              (8U)                                  
#define GPIO_AFRH_AFRH2_Msk              (0xFU << GPIO_AFRH_AFRH2_Pos)         /*!< 0x00000F00 */
#define GPIO_AFRH_AFRH2                  GPIO_AFRH_AFRH2_Msk                   
#define GPIO_AFRH_AFRH2_0                (0x1U << GPIO_AFRH_AFRH2_Pos)         /*!< 0x00000100 */
#define GPIO_AFRH_AFRH2_1                (0x2U << GPIO_AFRH_AFRH2_Pos)         /*!< 0x00000200 */
#define GPIO_AFRH_AFRH2_2                (0x4U << GPIO_AFRH_AFRH2_Pos)         /*!< 0x00000400 */
#define GPIO_AFRH_AFRH2_3                (0x8U << GPIO_AFRH_AFRH2_Pos)         /*!< 0x00000800 */
#define GPIO_AFRH_AFRH3_Pos              (12U)                                 
#define GPIO_AFRH_AFRH3_Msk              (0xFU << GPIO_AFRH_AFRH3_Pos)         /*!< 0x0000F000 */
#define GPIO_AFRH_AFRH3                  GPIO_AFRH_AFRH3_Msk                   
#define GPIO_AFRH_AFRH3_0                (0x1U << GPIO_AFRH_AFRH3_Pos)         /*!< 0x00001000 */
#define GPIO_AFRH_AFRH3_1                (0x2U << GPIO_AFRH_AFRH3_Pos)         /*!< 0x00002000 */
#define GPIO_AFRH_AFRH3_2                (0x4U << GPIO_AFRH_AFRH3_Pos)         /*!< 0x00004000 */
#define GPIO_AFRH_AFRH3_3                (0x8U << GPIO_AFRH_AFRH3_Pos)         /*!< 0x00008000 */
#define GPIO_AFRH_AFRH4_Pos              (16U)                                 
#define GPIO_AFRH_AFRH4_Msk              (0xFU << GPIO_AFRH_AFRH4_Pos)         /*!< 0x000F0000 */
#define GPIO_AFRH_AFRH4                  GPIO_AFRH_AFRH4_Msk                   
#define GPIO_AFRH_AFRH4_0                (0x1U << GPIO_AFRH_AFRH4_Pos)         /*!< 0x00010000 */
#define GPIO_AFRH_AFRH4_1                (0x2U << GPIO_AFRH_AFRH4_Pos)         /*!< 0x00020000 */
#define GPIO_AFRH_AFRH4_2                (0x4U << GPIO_AFRH_AFRH4_Pos)         /*!< 0x00040000 */
#define GPIO_AFRH_AFRH4_3                (0x8U << GPIO_AFRH_AFRH4_Pos)         /*!< 0x00080000 */
#define GPIO_AFRH_AFRH5_Pos              (20U)                                 
#define GPIO_AFRH_AFRH5_Msk              (0xFU << GPIO_AFRH_AFRH5_Pos)         /*!< 0x00F00000 */
#define GPIO_AFRH_AFRH5                  GPIO_AFRH_AFRH5_Msk                   
#define GPIO_AFRH_AFRH5_0                (0x1U << GPIO_AFRH_AFRH5_Pos)         /*!< 0x00100000 */
#define GPIO_AFRH_AFRH5_1                (0x2U << GPIO_AFRH_AFRH5_Pos)         /*!< 0x00200000 */
#define GPIO_AFRH_AFRH5_2                (0x4U << GPIO_AFRH_AFRH5_Pos)         /*!< 0x00400000 */
#define GPIO_AFRH_AFRH5_3                (0x8U << GPIO_AFRH_AFRH5_Pos)         /*!< 0x00800000 */
#define GPIO_AFRH_AFRH6_Pos              (24U)                                 
#define GPIO_AFRH_AFRH6_Msk              (0xFU << GPIO_AFRH_AFRH6_Pos)         /*!< 0x0F000000 */
#define GPIO_AFRH_AFRH6                  GPIO_AFRH_AFRH6_Msk                   
#define GPIO_AFRH_AFRH6_0                (0x1U << GPIO_AFRH_AFRH6_Pos)         /*!< 0x01000000 */
#define GPIO_AFRH_AFRH6_1                (0x2U << GPIO_AFRH_AFRH6_Pos)         /*!< 0x02000000 */
#define GPIO_AFRH_AFRH6_2                (0x4U << GPIO_AFRH_AFRH6_Pos)         /*!< 0x04000000 */
#define GPIO_AFRH_AFRH6_3                (0x8U << GPIO_AFRH_AFRH6_Pos)         /*!< 0x08000000 */
#define GPIO_AFRH_AFRH7_Pos              (28U)                                 
#define GPIO_AFRH_AFRH7_Msk              (0xFU << GPIO_AFRH_AFRH7_Pos)         /*!< 0xF0000000 */
#define GPIO_AFRH_AFRH7                  GPIO_AFRH_AFRH7_Msk                   
#define GPIO_AFRH_AFRH7_0                (0x1U << GPIO_AFRH_AFRH7_Pos)         /*!< 0x10000000 */
#define GPIO_AFRH_AFRH7_1                (0x2U << GPIO_AFRH_AFRH7_Pos)         /*!< 0x20000000 */
#define GPIO_AFRH_AFRH7_2                (0x4U << GPIO_AFRH_AFRH7_Pos)         /*!< 0x40000000 */
#define GPIO_AFRH_AFRH7_3                (0x8U << GPIO_AFRH_AFRH7_Pos)         /*!< 0x80000000 */


/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION_Pos                   (0U)                                
#define RCC_CR_HSION_Msk                   (0x1U << RCC_CR_HSION_Pos)          /*!< 0x00000001 */
#define RCC_CR_HSION                       RCC_CR_HSION_Msk                    
#define RCC_CR_HSIRDY_Pos                  (1U)                                
#define RCC_CR_HSIRDY_Msk                  (0x1U << RCC_CR_HSIRDY_Pos)         /*!< 0x00000002 */
#define RCC_CR_HSIRDY                      RCC_CR_HSIRDY_Msk                   
#define RCC_CR_HSITRIM_Pos                 (3U)                                
#define RCC_CR_HSITRIM_Msk                 (0x1FU << RCC_CR_HSITRIM_Pos)       /*!< 0x000000F8 */
#define RCC_CR_HSITRIM                     RCC_CR_HSITRIM_Msk                  
#define RCC_CR_HSITRIM_0                   (0x01U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000008 */
#define RCC_CR_HSITRIM_1                   (0x02U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000010 */
#define RCC_CR_HSITRIM_2                   (0x04U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000020 */
#define RCC_CR_HSITRIM_3                   (0x08U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000040 */
#define RCC_CR_HSITRIM_4                   (0x10U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000080 */
#define RCC_CR_HSICAL_Pos                  (8U)                                
#define RCC_CR_HSICAL_Msk                  (0xFFU << RCC_CR_HSICAL_Pos)        /*!< 0x0000FF00 */
#define RCC_CR_HSICAL                      RCC_CR_HSICAL_Msk                   
#define RCC_CR_HSICAL_0                    (0x01U << RCC_CR_HSICAL_Pos)        /*!< 0x00000100 */
#define RCC_CR_HSICAL_1                    (0x02U << RCC_CR_HSICAL_Pos)        /*!< 0x00000200 */
#define RCC_CR_HSICAL_2                    (0x04U << RCC_CR_HSICAL_Pos)        /*!< 0x00000400 */
#define RCC_CR_HSICAL_3                    (0x08U << RCC_CR_HSICAL_Pos)        /*!< 0x00000800 */
#define RCC_CR_HSICAL_4                    (0x10U << RCC_CR_HSICAL_Pos)        /*!< 0x00001000 */
#define RCC_CR_HSICAL_5                    (0x20U << RCC_CR_HSICAL_Pos)        /*!< 0x00002000 */
#define RCC_CR_HSICAL_6                    (0x40U << RCC_CR_HSICAL_Pos)        /*!< 0x00004000 */
#define RCC_CR_HSICAL_7                    (0x80U << RCC_CR_HSICAL_Pos)        /*!< 0x00008000 */
#define RCC_CR_HSEON_Pos                   (16U)                               
#define RCC_CR_HSEON_Msk                   (0x1U << RCC_CR_HSEON_Pos)          /*!< 0x00010000 */
#define RCC_CR_HSEON                       RCC_CR_HSEON_Msk                    
#define RCC_CR_HSERDY_Pos                  (17U)                               
#define RCC_CR_HSERDY_Msk                  (0x1U << RCC_CR_HSERDY_Pos)         /*!< 0x00020000 */
#define RCC_CR_HSERDY                      RCC_CR_HSERDY_Msk                   
#define RCC_CR_HSEBYP_Pos                  (18U)                               
#define RCC_CR_HSEBYP_Msk                  (0x1U << RCC_CR_HSEBYP_Pos)         /*!< 0x00040000 */
#define RCC_CR_HSEBYP                      RCC_CR_HSEBYP_Msk                   
#define RCC_CR_CSSON_Pos                   (19U)                               
#define RCC_CR_CSSON_Msk                   (0x1U << RCC_CR_CSSON_Pos)          /*!< 0x00080000 */
#define RCC_CR_CSSON                       RCC_CR_CSSON_Msk                    
#define RCC_CR_PLLON_Pos                   (24U)                               
#define RCC_CR_PLLON_Msk                   (0x1U << RCC_CR_PLLON_Pos)          /*!< 0x01000000 */
#define RCC_CR_PLLON                       RCC_CR_PLLON_Msk                    
#define RCC_CR_PLLRDY_Pos                  (25U)                               
#define RCC_CR_PLLRDY_Msk                  (0x1U << RCC_CR_PLLRDY_Pos)         /*!< 0x02000000 */
#define RCC_CR_PLLRDY                      RCC_CR_PLLRDY_Msk                   
#define RCC_CR_PLLI2SON_Pos                (26U)                               
#define RCC_CR_PLLI2SON_Msk                (0x1U << RCC_CR_PLLI2SON_Pos)       /*!< 0x04000000 */
#define RCC_CR_PLLI2SON                    RCC_CR_PLLI2SON_Msk                 
#define RCC_CR_PLLI2SRDY_Pos               (27U)                               
#define RCC_CR_PLLI2SRDY_Msk               (0x1U << RCC_CR_PLLI2SRDY_Pos)      /*!< 0x08000000 */
#define RCC_CR_PLLI2SRDY                   RCC_CR_PLLI2SRDY_Msk                
#define RCC_CR_PLLSAION_Pos                (28U)                               
#define RCC_CR_PLLSAION_Msk                (0x1U << RCC_CR_PLLSAION_Pos)       /*!< 0x10000000 */
#define RCC_CR_PLLSAION                    RCC_CR_PLLSAION_Msk                 
#define RCC_CR_PLLSAIRDY_Pos               (29U)                               
#define RCC_CR_PLLSAIRDY_Msk               (0x1U << RCC_CR_PLLSAIRDY_Pos)      /*!< 0x20000000 */
#define RCC_CR_PLLSAIRDY                   RCC_CR_PLLSAIRDY_Msk                

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLM_Pos               (0U)                                
#define RCC_PLLCFGR_PLLM_Msk               (0x3FU << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x0000003F */
#define RCC_PLLCFGR_PLLM                   RCC_PLLCFGR_PLLM_Msk                
#define RCC_PLLCFGR_PLLM_0                 (0x01U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000001 */
#define RCC_PLLCFGR_PLLM_1                 (0x02U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000002 */
#define RCC_PLLCFGR_PLLM_2                 (0x04U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000004 */
#define RCC_PLLCFGR_PLLM_3                 (0x08U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000008 */
#define RCC_PLLCFGR_PLLM_4                 (0x10U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000010 */
#define RCC_PLLCFGR_PLLM_5                 (0x20U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000020 */
#define RCC_PLLCFGR_PLLN_Pos               (6U)                                
#define RCC_PLLCFGR_PLLN_Msk               (0x1FFU << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00007FC0 */
#define RCC_PLLCFGR_PLLN                   RCC_PLLCFGR_PLLN_Msk                
#define RCC_PLLCFGR_PLLN_0                 (0x001U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000040 */
#define RCC_PLLCFGR_PLLN_1                 (0x002U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000080 */
#define RCC_PLLCFGR_PLLN_2                 (0x004U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000100 */
#define RCC_PLLCFGR_PLLN_3                 (0x008U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000200 */
#define RCC_PLLCFGR_PLLN_4                 (0x010U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000400 */
#define RCC_PLLCFGR_PLLN_5                 (0x020U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000800 */
#define RCC_PLLCFGR_PLLN_6                 (0x040U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00001000 */
#define RCC_PLLCFGR_PLLN_7                 (0x080U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00002000 */
#define RCC_PLLCFGR_PLLN_8                 (0x100U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00004000 */
#define RCC_PLLCFGR_PLLP_Pos               (16U)                               
#define RCC_PLLCFGR_PLLP_Msk               (0x3U << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00030000 */
#define RCC_PLLCFGR_PLLP                   RCC_PLLCFGR_PLLP_Msk                
#define RCC_PLLCFGR_PLLP_0                 (0x1U << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLP_1                 (0x2U << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00020000 */
#define RCC_PLLCFGR_PLLSRC_Pos             (22U)                               
#define RCC_PLLCFGR_PLLSRC_Msk             (0x1U << RCC_PLLCFGR_PLLSRC_Pos)    /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC                 RCC_PLLCFGR_PLLSRC_Msk              
#define RCC_PLLCFGR_PLLSRC_HSE_Pos         (22U)                               
#define RCC_PLLCFGR_PLLSRC_HSE_Msk         (0x1U << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE             RCC_PLLCFGR_PLLSRC_HSE_Msk          
#define RCC_PLLCFGR_PLLSRC_HSI             0x00000000U                         
#define RCC_PLLCFGR_PLLQ_Pos               (24U)                               
#define RCC_PLLCFGR_PLLQ_Msk               (0xFU << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x0F000000 */
#define RCC_PLLCFGR_PLLQ                   RCC_PLLCFGR_PLLQ_Msk                
#define RCC_PLLCFGR_PLLQ_0                 (0x1U << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLQ_1                 (0x2U << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x02000000 */
#define RCC_PLLCFGR_PLLQ_2                 (0x4U << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x04000000 */
#define RCC_PLLCFGR_PLLQ_3                 (0x8U << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x08000000 */

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                    (0U)                                
#define RCC_CFGR_SW_Msk                    (0x3U << RCC_CFGR_SW_Pos)           /*!< 0x00000003 */
#define RCC_CFGR_SW                        RCC_CFGR_SW_Msk                     /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                      (0x1U << RCC_CFGR_SW_Pos)           /*!< 0x00000001 */
#define RCC_CFGR_SW_1                      (0x2U << RCC_CFGR_SW_Pos)           /*!< 0x00000002 */
#define RCC_CFGR_SW_HSI                    0x00000000U                         /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                    0x00000001U                         /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                    0x00000002U                         /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                   (2U)                                
#define RCC_CFGR_SWS_Msk                   (0x3U << RCC_CFGR_SWS_Pos)          /*!< 0x0000000C */
#define RCC_CFGR_SWS                       RCC_CFGR_SWS_Msk                    /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                     (0x1U << RCC_CFGR_SWS_Pos)          /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                     (0x2U << RCC_CFGR_SWS_Pos)          /*!< 0x00000008 */
#define RCC_CFGR_SWS_HSI                   0x00000000U                         /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                   0x00000004U                         /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                   0x00000008U                         /*!< PLL used as system clock */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                  (4U)                                
#define RCC_CFGR_HPRE_Msk                  (0xFU << RCC_CFGR_HPRE_Pos)         /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                      RCC_CFGR_HPRE_Msk                   /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                    (0x1U << RCC_CFGR_HPRE_Pos)         /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                    (0x2U << RCC_CFGR_HPRE_Pos)         /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                    (0x4U << RCC_CFGR_HPRE_Pos)         /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                    (0x8U << RCC_CFGR_HPRE_Pos)         /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2                 0x00000080U                         /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4                 0x00000090U                         /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8                 0x000000A0U                         /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16                0x000000B0U                         /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64                0x000000C0U                         /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128               0x000000D0U                         /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256               0x000000E0U                         /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512               0x000000F0U                         /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                 (10U)                               
#define RCC_CFGR_PPRE1_Msk                 (0x7U << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1                     RCC_CFGR_PPRE1_Msk                  /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                   (0x1U << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000400 */
#define RCC_CFGR_PPRE1_1                   (0x2U << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000800 */
#define RCC_CFGR_PPRE1_2                   (0x4U << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001000 */

#define RCC_CFGR_PPRE1_DIV1                0x00000000U                         /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV2                0x00001000U                         /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4                0x00001400U                         /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8                0x00001800U                         /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16               0x00001C00U                         /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                 (13U)                               
#define RCC_CFGR_PPRE2_Msk                 (0x7U << RCC_CFGR_PPRE2_Pos)        /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2                     RCC_CFGR_PPRE2_Msk                  /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                   (0x1U << RCC_CFGR_PPRE2_Pos)        /*!< 0x00002000 */
#define RCC_CFGR_PPRE2_1                   (0x2U << RCC_CFGR_PPRE2_Pos)        /*!< 0x00004000 */
#define RCC_CFGR_PPRE2_2                   (0x4U << RCC_CFGR_PPRE2_Pos)        /*!< 0x00008000 */

#define RCC_CFGR_PPRE2_DIV1                0x00000000U                         /*!< HCLK not divided */
#define RCC_CFGR_PPRE2_DIV2                0x00008000U                         /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4                0x0000A000U                         /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8                0x0000C000U                         /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16               0x0000E000U                         /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define RCC_CFGR_RTCPRE_Pos                (16U)                               
#define RCC_CFGR_RTCPRE_Msk                (0x1FU << RCC_CFGR_RTCPRE_Pos)      /*!< 0x001F0000 */
#define RCC_CFGR_RTCPRE                    RCC_CFGR_RTCPRE_Msk                 
#define RCC_CFGR_RTCPRE_0                  (0x01U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00010000 */
#define RCC_CFGR_RTCPRE_1                  (0x02U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00020000 */
#define RCC_CFGR_RTCPRE_2                  (0x04U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00040000 */
#define RCC_CFGR_RTCPRE_3                  (0x08U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00080000 */
#define RCC_CFGR_RTCPRE_4                  (0x10U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00100000 */

/*!< MCO1 configuration */
#define RCC_CFGR_MCO1_Pos                  (21U)                               
#define RCC_CFGR_MCO1_Msk                  (0x3U << RCC_CFGR_MCO1_Pos)         /*!< 0x00600000 */
#define RCC_CFGR_MCO1                      RCC_CFGR_MCO1_Msk                   
#define RCC_CFGR_MCO1_0                    (0x1U << RCC_CFGR_MCO1_Pos)         /*!< 0x00200000 */
#define RCC_CFGR_MCO1_1                    (0x2U << RCC_CFGR_MCO1_Pos)         /*!< 0x00400000 */

#define RCC_CFGR_I2SSRC_Pos                (23U)                               
#define RCC_CFGR_I2SSRC_Msk                (0x1U << RCC_CFGR_I2SSRC_Pos)       /*!< 0x00800000 */
#define RCC_CFGR_I2SSRC                    RCC_CFGR_I2SSRC_Msk                 

#define RCC_CFGR_MCO1PRE_Pos               (24U)                               
#define RCC_CFGR_MCO1PRE_Msk               (0x7U << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x07000000 */
#define RCC_CFGR_MCO1PRE                   RCC_CFGR_MCO1PRE_Msk                
#define RCC_CFGR_MCO1PRE_0                 (0x1U << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x01000000 */
#define RCC_CFGR_MCO1PRE_1                 (0x2U << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x02000000 */
#define RCC_CFGR_MCO1PRE_2                 (0x4U << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x04000000 */

#define RCC_CFGR_MCO2PRE_Pos               (27U)                               
#define RCC_CFGR_MCO2PRE_Msk               (0x7U << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x38000000 */
#define RCC_CFGR_MCO2PRE                   RCC_CFGR_MCO2PRE_Msk                
#define RCC_CFGR_MCO2PRE_0                 (0x1U << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x08000000 */
#define RCC_CFGR_MCO2PRE_1                 (0x2U << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x10000000 */
#define RCC_CFGR_MCO2PRE_2                 (0x4U << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x20000000 */

#define RCC_CFGR_MCO2_Pos                  (30U)                               
#define RCC_CFGR_MCO2_Msk                  (0x3U << RCC_CFGR_MCO2_Pos)         /*!< 0xC0000000 */
#define RCC_CFGR_MCO2                      RCC_CFGR_MCO2_Msk                   
#define RCC_CFGR_MCO2_0                    (0x1U << RCC_CFGR_MCO2_Pos)         /*!< 0x40000000 */
#define RCC_CFGR_MCO2_1                    (0x2U << RCC_CFGR_MCO2_Pos)         /*!< 0x80000000 */

/********************  Bit definition for RCC_CIR register  *******************/
#define RCC_CIR_LSIRDYF_Pos                (0U)                                
#define RCC_CIR_LSIRDYF_Msk                (0x1U << RCC_CIR_LSIRDYF_Pos)       /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF                    RCC_CIR_LSIRDYF_Msk                 
#define RCC_CIR_LSERDYF_Pos                (1U)                                
#define RCC_CIR_LSERDYF_Msk                (0x1U << RCC_CIR_LSERDYF_Pos)       /*!< 0x00000002 */
#define RCC_CIR_LSERDYF                    RCC_CIR_LSERDYF_Msk                 
#define RCC_CIR_HSIRDYF_Pos                (2U)                                
#define RCC_CIR_HSIRDYF_Msk                (0x1U << RCC_CIR_HSIRDYF_Pos)       /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF                    RCC_CIR_HSIRDYF_Msk                 
#define RCC_CIR_HSERDYF_Pos                (3U)                                
#define RCC_CIR_HSERDYF_Msk                (0x1U << RCC_CIR_HSERDYF_Pos)       /*!< 0x00000008 */
#define RCC_CIR_HSERDYF                    RCC_CIR_HSERDYF_Msk                 
#define RCC_CIR_PLLRDYF_Pos                (4U)                                
#define RCC_CIR_PLLRDYF_Msk                (0x1U << RCC_CIR_PLLRDYF_Pos)       /*!< 0x00000010 */
#define RCC_CIR_PLLRDYF                    RCC_CIR_PLLRDYF_Msk                 
#define RCC_CIR_PLLI2SRDYF_Pos             (5U)                                
#define RCC_CIR_PLLI2SRDYF_Msk             (0x1U << RCC_CIR_PLLI2SRDYF_Pos)    /*!< 0x00000020 */
#define RCC_CIR_PLLI2SRDYF                 RCC_CIR_PLLI2SRDYF_Msk              
#define RCC_CIR_PLLSAIRDYF_Pos             (6U)                                
#define RCC_CIR_PLLSAIRDYF_Msk             (0x1U << RCC_CIR_PLLSAIRDYF_Pos)    /*!< 0x00000040 */
#define RCC_CIR_PLLSAIRDYF                 RCC_CIR_PLLSAIRDYF_Msk              
#define RCC_CIR_CSSF_Pos                   (7U)                                
#define RCC_CIR_CSSF_Msk                   (0x1U << RCC_CIR_CSSF_Pos)          /*!< 0x00000080 */
#define RCC_CIR_CSSF                       RCC_CIR_CSSF_Msk                    
#define RCC_CIR_LSIRDYIE_Pos               (8U)                                
#define RCC_CIR_LSIRDYIE_Msk               (0x1U << RCC_CIR_LSIRDYIE_Pos)      /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE                   RCC_CIR_LSIRDYIE_Msk                
#define RCC_CIR_LSERDYIE_Pos               (9U)                                
#define RCC_CIR_LSERDYIE_Msk               (0x1U << RCC_CIR_LSERDYIE_Pos)      /*!< 0x00000200 */
#define RCC_CIR_LSERDYIE                   RCC_CIR_LSERDYIE_Msk                
#define RCC_CIR_HSIRDYIE_Pos               (10U)                               
#define RCC_CIR_HSIRDYIE_Msk               (0x1U << RCC_CIR_HSIRDYIE_Pos)      /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE                   RCC_CIR_HSIRDYIE_Msk                
#define RCC_CIR_HSERDYIE_Pos               (11U)                               
#define RCC_CIR_HSERDYIE_Msk               (0x1U << RCC_CIR_HSERDYIE_Pos)      /*!< 0x00000800 */
#define RCC_CIR_HSERDYIE                   RCC_CIR_HSERDYIE_Msk                
#define RCC_CIR_PLLRDYIE_Pos               (12U)                               
#define RCC_CIR_PLLRDYIE_Msk               (0x1U << RCC_CIR_PLLRDYIE_Pos)      /*!< 0x00001000 */
#define RCC_CIR_PLLRDYIE                   RCC_CIR_PLLRDYIE_Msk                
#define RCC_CIR_PLLI2SRDYIE_Pos            (13U)                               
#define RCC_CIR_PLLI2SRDYIE_Msk            (0x1U << RCC_CIR_PLLI2SRDYIE_Pos)   /*!< 0x00002000 */
#define RCC_CIR_PLLI2SRDYIE                RCC_CIR_PLLI2SRDYIE_Msk             
#define RCC_CIR_PLLSAIRDYIE_Pos            (14U)                               
#define RCC_CIR_PLLSAIRDYIE_Msk            (0x1U << RCC_CIR_PLLSAIRDYIE_Pos)   /*!< 0x00004000 */
#define RCC_CIR_PLLSAIRDYIE                RCC_CIR_PLLSAIRDYIE_Msk             
#define RCC_CIR_LSIRDYC_Pos                (16U)                               
#define RCC_CIR_LSIRDYC_Msk                (0x1U << RCC_CIR_LSIRDYC_Pos)       /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC                    RCC_CIR_LSIRDYC_Msk                 
#define RCC_CIR_LSERDYC_Pos                (17U)                               
#define RCC_CIR_LSERDYC_Msk                (0x1U << RCC_CIR_LSERDYC_Pos)       /*!< 0x00020000 */
#define RCC_CIR_LSERDYC                    RCC_CIR_LSERDYC_Msk                 
#define RCC_CIR_HSIRDYC_Pos                (18U)                               
#define RCC_CIR_HSIRDYC_Msk                (0x1U << RCC_CIR_HSIRDYC_Pos)       /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC                    RCC_CIR_HSIRDYC_Msk                 
#define RCC_CIR_HSERDYC_Pos                (19U)                               
#define RCC_CIR_HSERDYC_Msk                (0x1U << RCC_CIR_HSERDYC_Pos)       /*!< 0x00080000 */
#define RCC_CIR_HSERDYC                    RCC_CIR_HSERDYC_Msk                 
#define RCC_CIR_PLLRDYC_Pos                (20U)                               
#define RCC_CIR_PLLRDYC_Msk                (0x1U << RCC_CIR_PLLRDYC_Pos)       /*!< 0x00100000 */
#define RCC_CIR_PLLRDYC                    RCC_CIR_PLLRDYC_Msk                 
#define RCC_CIR_PLLI2SRDYC_Pos             (21U)                               
#define RCC_CIR_PLLI2SRDYC_Msk             (0x1U << RCC_CIR_PLLI2SRDYC_Pos)    /*!< 0x00200000 */
#define RCC_CIR_PLLI2SRDYC                 RCC_CIR_PLLI2SRDYC_Msk              
#define RCC_CIR_PLLSAIRDYC_Pos             (22U)                               
#define RCC_CIR_PLLSAIRDYC_Msk             (0x1U << RCC_CIR_PLLSAIRDYC_Pos)    /*!< 0x00400000 */
#define RCC_CIR_PLLSAIRDYC                 RCC_CIR_PLLSAIRDYC_Msk              
#define RCC_CIR_CSSC_Pos                   (23U)                               
#define RCC_CIR_CSSC_Msk                   (0x1U << RCC_CIR_CSSC_Pos)          /*!< 0x00800000 */
#define RCC_CIR_CSSC                       RCC_CIR_CSSC_Msk                    

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define RCC_AHB1ENR_GPIOAEN_Pos            (0U)                                
#define RCC_AHB1ENR_GPIOAEN_Msk            (0x1U << RCC_AHB1ENR_GPIOAEN_Pos)   /*!< 0x00000001 */
#define RCC_AHB1ENR_GPIOAEN                RCC_AHB1ENR_GPIOAEN_Msk             
#define RCC_AHB1ENR_GPIOBEN_Pos            (1U)                                
#define RCC_AHB1ENR_GPIOBEN_Msk            (0x1U << RCC_AHB1ENR_GPIOBEN_Pos)   /*!< 0x00000002 */
#define RCC_AHB1ENR_GPIOBEN                RCC_AHB1ENR_GPIOBEN_Msk             
#define RCC_AHB1ENR_GPIOCEN_Pos            (2U)                                
#define RCC_AHB1ENR_GPIOCEN_Msk            (0x1U << RCC_AHB1ENR_GPIOCEN_Pos)   /*!< 0x00000004 */
#define RCC_AHB1ENR_GPIOCEN                RCC_AHB1ENR_GPIOCEN_Msk             
#define RCC_AHB1ENR_GPIODEN_Pos            (3U)                                
#define RCC_AHB1ENR_GPIODEN_Msk            (0x1U << RCC_AHB1ENR_GPIODEN_Pos)   /*!< 0x00000008 */
#define RCC_AHB1ENR_GPIODEN                RCC_AHB1ENR_GPIODEN_Msk             
#define RCC_AHB1ENR_GPIOEEN_Pos            (4U)                                
#define RCC_AHB1ENR_GPIOEEN_Msk            (0x1U << RCC_AHB1ENR_GPIOEEN_Pos)   /*!< 0x00000010 */
#define RCC_AHB1ENR_GPIOEEN                RCC_AHB1ENR_GPIOEEN_Msk             
#define RCC_AHB1ENR_GPIOFEN_Pos            (5U)                                
#define RCC_AHB1ENR_GPIOFEN_Msk            (0x1U << RCC_AHB1ENR_GPIOFEN_Pos)   /*!< 0x00000020 */
#define RCC_AHB1ENR_GPIOFEN                RCC_AHB1ENR_GPIOFEN_Msk             
#define RCC_AHB1ENR_GPIOGEN_Pos            (6U)                                
#define RCC_AHB1ENR_GPIOGEN_Msk            (0x1U << RCC_AHB1ENR_GPIOGEN_Pos)   /*!< 0x00000040 */
#define RCC_AHB1ENR_GPIOGEN                RCC_AHB1ENR_GPIOGEN_Msk             
#define RCC_AHB1ENR_GPIOHEN_Pos            (7U)                                
#define RCC_AHB1ENR_GPIOHEN_Msk            (0x1U << RCC_AHB1ENR_GPIOHEN_Pos)   /*!< 0x00000080 */
#define RCC_AHB1ENR_GPIOHEN                RCC_AHB1ENR_GPIOHEN_Msk             
#define RCC_AHB1ENR_GPIOIEN_Pos            (8U)                                
#define RCC_AHB1ENR_GPIOIEN_Msk            (0x1U << RCC_AHB1ENR_GPIOIEN_Pos)   /*!< 0x00000100 */
#define RCC_AHB1ENR_GPIOIEN                RCC_AHB1ENR_GPIOIEN_Msk             
#define RCC_AHB1ENR_GPIOJEN_Pos            (9U)                                
#define RCC_AHB1ENR_GPIOJEN_Msk            (0x1U << RCC_AHB1ENR_GPIOJEN_Pos)   /*!< 0x00000200 */
#define RCC_AHB1ENR_GPIOJEN                RCC_AHB1ENR_GPIOJEN_Msk             
#define RCC_AHB1ENR_GPIOKEN_Pos            (10U)                               
#define RCC_AHB1ENR_GPIOKEN_Msk            (0x1U << RCC_AHB1ENR_GPIOKEN_Pos)   /*!< 0x00000400 */
#define RCC_AHB1ENR_GPIOKEN                RCC_AHB1ENR_GPIOKEN_Msk             
#define RCC_AHB1ENR_CRCEN_Pos              (12U)                               
#define RCC_AHB1ENR_CRCEN_Msk              (0x1U << RCC_AHB1ENR_CRCEN_Pos)     /*!< 0x00001000 */
#define RCC_AHB1ENR_CRCEN                  RCC_AHB1ENR_CRCEN_Msk               
#define RCC_AHB1ENR_BKPSRAMEN_Pos          (18U)                               
#define RCC_AHB1ENR_BKPSRAMEN_Msk          (0x1U << RCC_AHB1ENR_BKPSRAMEN_Pos) /*!< 0x00040000 */
#define RCC_AHB1ENR_BKPSRAMEN              RCC_AHB1ENR_BKPSRAMEN_Msk           
#define RCC_AHB1ENR_DTCMRAMEN_Pos          (20U)                               
#define RCC_AHB1ENR_DTCMRAMEN_Msk          (0x1U << RCC_AHB1ENR_DTCMRAMEN_Pos) /*!< 0x00100000 */
#define RCC_AHB1ENR_DTCMRAMEN              RCC_AHB1ENR_DTCMRAMEN_Msk           
#define RCC_AHB1ENR_DMA1EN_Pos             (21U)                               
#define RCC_AHB1ENR_DMA1EN_Msk             (0x1U << RCC_AHB1ENR_DMA1EN_Pos)    /*!< 0x00200000 */
#define RCC_AHB1ENR_DMA1EN                 RCC_AHB1ENR_DMA1EN_Msk              
#define RCC_AHB1ENR_DMA2EN_Pos             (22U)                               
#define RCC_AHB1ENR_DMA2EN_Msk             (0x1U << RCC_AHB1ENR_DMA2EN_Pos)    /*!< 0x00400000 */
#define RCC_AHB1ENR_DMA2EN                 RCC_AHB1ENR_DMA2EN_Msk              
#define RCC_AHB1ENR_DMA2DEN_Pos            (23U)                               
#define RCC_AHB1ENR_DMA2DEN_Msk            (0x1U << RCC_AHB1ENR_DMA2DEN_Pos)   /*!< 0x00800000 */
#define RCC_AHB1ENR_DMA2DEN                RCC_AHB1ENR_DMA2DEN_Msk             
#define RCC_AHB1ENR_ETHMACEN_Pos           (25U)                               
#define RCC_AHB1ENR_ETHMACEN_Msk           (0x1U << RCC_AHB1ENR_ETHMACEN_Pos)  /*!< 0x02000000 */
#define RCC_AHB1ENR_ETHMACEN               RCC_AHB1ENR_ETHMACEN_Msk            
#define RCC_AHB1ENR_ETHMACTXEN_Pos         (26U)                               
#define RCC_AHB1ENR_ETHMACTXEN_Msk         (0x1U << RCC_AHB1ENR_ETHMACTXEN_Pos) /*!< 0x04000000 */
#define RCC_AHB1ENR_ETHMACTXEN             RCC_AHB1ENR_ETHMACTXEN_Msk          
#define RCC_AHB1ENR_ETHMACRXEN_Pos         (27U)                               
#define RCC_AHB1ENR_ETHMACRXEN_Msk         (0x1U << RCC_AHB1ENR_ETHMACRXEN_Pos) /*!< 0x08000000 */
#define RCC_AHB1ENR_ETHMACRXEN             RCC_AHB1ENR_ETHMACRXEN_Msk          
#define RCC_AHB1ENR_ETHMACPTPEN_Pos        (28U)                               
#define RCC_AHB1ENR_ETHMACPTPEN_Msk        (0x1U << RCC_AHB1ENR_ETHMACPTPEN_Pos) /*!< 0x10000000 */
#define RCC_AHB1ENR_ETHMACPTPEN            RCC_AHB1ENR_ETHMACPTPEN_Msk         
#define RCC_AHB1ENR_OTGHSEN_Pos            (29U)                               
#define RCC_AHB1ENR_OTGHSEN_Msk            (0x1U << RCC_AHB1ENR_OTGHSEN_Pos)   /*!< 0x20000000 */
#define RCC_AHB1ENR_OTGHSEN                RCC_AHB1ENR_OTGHSEN_Msk             
#define RCC_AHB1ENR_OTGHSULPIEN_Pos        (30U)                               
#define RCC_AHB1ENR_OTGHSULPIEN_Msk        (0x1U << RCC_AHB1ENR_OTGHSULPIEN_Pos) /*!< 0x40000000 */
#define RCC_AHB1ENR_OTGHSULPIEN            RCC_AHB1ENR_OTGHSULPIEN_Msk         

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define RCC_AHB2ENR_DCMIEN_Pos             (0U)                                
#define RCC_AHB2ENR_DCMIEN_Msk             (0x1U << RCC_AHB2ENR_DCMIEN_Pos)    /*!< 0x00000001 */
#define RCC_AHB2ENR_DCMIEN                 RCC_AHB2ENR_DCMIEN_Msk              
#define RCC_AHB2ENR_RNGEN_Pos              (6U)                                
#define RCC_AHB2ENR_RNGEN_Msk              (0x1U << RCC_AHB2ENR_RNGEN_Pos)     /*!< 0x00000040 */
#define RCC_AHB2ENR_RNGEN                  RCC_AHB2ENR_RNGEN_Msk               
#define RCC_AHB2ENR_OTGFSEN_Pos            (7U)                                
#define RCC_AHB2ENR_OTGFSEN_Msk            (0x1U << RCC_AHB2ENR_OTGFSEN_Pos)   /*!< 0x00000080 */
#define RCC_AHB2ENR_OTGFSEN                RCC_AHB2ENR_OTGFSEN_Msk             

/********************  Bit definition for RCC_AHB3ENR register  ***************/
#define RCC_AHB3ENR_FMCEN_Pos              (0U)                                
#define RCC_AHB3ENR_FMCEN_Msk              (0x1U << RCC_AHB3ENR_FMCEN_Pos)     /*!< 0x00000001 */
#define RCC_AHB3ENR_FMCEN                  RCC_AHB3ENR_FMCEN_Msk               
#define RCC_AHB3ENR_QSPIEN_Pos             (1U)                                
#define RCC_AHB3ENR_QSPIEN_Msk             (0x1U << RCC_AHB3ENR_QSPIEN_Pos)    /*!< 0x00000002 */
#define RCC_AHB3ENR_QSPIEN                 RCC_AHB3ENR_QSPIEN_Msk              

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define RCC_APB1ENR_TIM2EN_Pos             (0U)                                
#define RCC_APB1ENR_TIM2EN_Msk             (0x1U << RCC_APB1ENR_TIM2EN_Pos)    /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN                 RCC_APB1ENR_TIM2EN_Msk              
#define RCC_APB1ENR_TIM3EN_Pos             (1U)                                
#define RCC_APB1ENR_TIM3EN_Msk             (0x1U << RCC_APB1ENR_TIM3EN_Pos)    /*!< 0x00000002 */
#define RCC_APB1ENR_TIM3EN                 RCC_APB1ENR_TIM3EN_Msk              
#define RCC_APB1ENR_TIM4EN_Pos             (2U)                                
#define RCC_APB1ENR_TIM4EN_Msk             (0x1U << RCC_APB1ENR_TIM4EN_Pos)    /*!< 0x00000004 */
#define RCC_APB1ENR_TIM4EN                 RCC_APB1ENR_TIM4EN_Msk              
#define RCC_APB1ENR_TIM5EN_Pos             (3U)                                
#define RCC_APB1ENR_TIM5EN_Msk             (0x1U << RCC_APB1ENR_TIM5EN_Pos)    /*!< 0x00000008 */
#define RCC_APB1ENR_TIM5EN                 RCC_APB1ENR_TIM5EN_Msk              
#define RCC_APB1ENR_TIM6EN_Pos             (4U)                                
#define RCC_APB1ENR_TIM6EN_Msk             (0x1U << RCC_APB1ENR_TIM6EN_Pos)    /*!< 0x00000010 */
#define RCC_APB1ENR_TIM6EN                 RCC_APB1ENR_TIM6EN_Msk              
#define RCC_APB1ENR_TIM7EN_Pos             (5U)                                
#define RCC_APB1ENR_TIM7EN_Msk             (0x1U << RCC_APB1ENR_TIM7EN_Pos)    /*!< 0x00000020 */
#define RCC_APB1ENR_TIM7EN                 RCC_APB1ENR_TIM7EN_Msk              
#define RCC_APB1ENR_TIM12EN_Pos            (6U)                                
#define RCC_APB1ENR_TIM12EN_Msk            (0x1U << RCC_APB1ENR_TIM12EN_Pos)   /*!< 0x00000040 */
#define RCC_APB1ENR_TIM12EN                RCC_APB1ENR_TIM12EN_Msk             
#define RCC_APB1ENR_TIM13EN_Pos            (7U)                                
#define RCC_APB1ENR_TIM13EN_Msk            (0x1U << RCC_APB1ENR_TIM13EN_Pos)   /*!< 0x00000080 */
#define RCC_APB1ENR_TIM13EN                RCC_APB1ENR_TIM13EN_Msk             
#define RCC_APB1ENR_TIM14EN_Pos            (8U)                                
#define RCC_APB1ENR_TIM14EN_Msk            (0x1U << RCC_APB1ENR_TIM14EN_Pos)   /*!< 0x00000100 */
#define RCC_APB1ENR_TIM14EN                RCC_APB1ENR_TIM14EN_Msk             
#define RCC_APB1ENR_LPTIM1EN_Pos           (9U)                                
#define RCC_APB1ENR_LPTIM1EN_Msk           (0x1U << RCC_APB1ENR_LPTIM1EN_Pos)  /*!< 0x00000200 */
#define RCC_APB1ENR_LPTIM1EN               RCC_APB1ENR_LPTIM1EN_Msk            
#define RCC_APB1ENR_WWDGEN_Pos             (11U)                               
#define RCC_APB1ENR_WWDGEN_Msk             (0x1U << RCC_APB1ENR_WWDGEN_Pos)    /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN                 RCC_APB1ENR_WWDGEN_Msk              
#define RCC_APB1ENR_SPI2EN_Pos             (14U)                               
#define RCC_APB1ENR_SPI2EN_Msk             (0x1U << RCC_APB1ENR_SPI2EN_Pos)    /*!< 0x00004000 */
#define RCC_APB1ENR_SPI2EN                 RCC_APB1ENR_SPI2EN_Msk              
#define RCC_APB1ENR_SPI3EN_Pos             (15U)                               
#define RCC_APB1ENR_SPI3EN_Msk             (0x1U << RCC_APB1ENR_SPI3EN_Pos)    /*!< 0x00008000 */
#define RCC_APB1ENR_SPI3EN                 RCC_APB1ENR_SPI3EN_Msk              
#define RCC_APB1ENR_SPDIFRXEN_Pos          (16U)                               
#define RCC_APB1ENR_SPDIFRXEN_Msk          (0x1U << RCC_APB1ENR_SPDIFRXEN_Pos) /*!< 0x00010000 */
#define RCC_APB1ENR_SPDIFRXEN              RCC_APB1ENR_SPDIFRXEN_Msk           
#define RCC_APB1ENR_USART2EN_Pos           (17U)                               
#define RCC_APB1ENR_USART2EN_Msk           (0x1U << RCC_APB1ENR_USART2EN_Pos)  /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN               RCC_APB1ENR_USART2EN_Msk            
#define RCC_APB1ENR_USART3EN_Pos           (18U)                               
#define RCC_APB1ENR_USART3EN_Msk           (0x1U << RCC_APB1ENR_USART3EN_Pos)  /*!< 0x00040000 */
#define RCC_APB1ENR_USART3EN               RCC_APB1ENR_USART3EN_Msk            
#define RCC_APB1ENR_UART4EN_Pos            (19U)                               
#define RCC_APB1ENR_UART4EN_Msk            (0x1U << RCC_APB1ENR_UART4EN_Pos)   /*!< 0x00080000 */
#define RCC_APB1ENR_UART4EN                RCC_APB1ENR_UART4EN_Msk             
#define RCC_APB1ENR_UART5EN_Pos            (20U)                               
#define RCC_APB1ENR_UART5EN_Msk            (0x1U << RCC_APB1ENR_UART5EN_Pos)   /*!< 0x00100000 */
#define RCC_APB1ENR_UART5EN                RCC_APB1ENR_UART5EN_Msk             
#define RCC_APB1ENR_I2C1EN_Pos             (21U)                               
#define RCC_APB1ENR_I2C1EN_Msk             (0x1U << RCC_APB1ENR_I2C1EN_Pos)    /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN                 RCC_APB1ENR_I2C1EN_Msk              
#define RCC_APB1ENR_I2C2EN_Pos             (22U)                               
#define RCC_APB1ENR_I2C2EN_Msk             (0x1U << RCC_APB1ENR_I2C2EN_Pos)    /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN                 RCC_APB1ENR_I2C2EN_Msk              
#define RCC_APB1ENR_I2C3EN_Pos             (23U)                               
#define RCC_APB1ENR_I2C3EN_Msk             (0x1U << RCC_APB1ENR_I2C3EN_Pos)    /*!< 0x00800000 */
#define RCC_APB1ENR_I2C3EN                 RCC_APB1ENR_I2C3EN_Msk              
#define RCC_APB1ENR_I2C4EN_Pos             (24U)                               
#define RCC_APB1ENR_I2C4EN_Msk             (0x1U << RCC_APB1ENR_I2C4EN_Pos)    /*!< 0x01000000 */
#define RCC_APB1ENR_I2C4EN                 RCC_APB1ENR_I2C4EN_Msk              
#define RCC_APB1ENR_CAN1EN_Pos             (25U)                               
#define RCC_APB1ENR_CAN1EN_Msk             (0x1U << RCC_APB1ENR_CAN1EN_Pos)    /*!< 0x02000000 */
#define RCC_APB1ENR_CAN1EN                 RCC_APB1ENR_CAN1EN_Msk              
#define RCC_APB1ENR_CAN2EN_Pos             (26U)                               
#define RCC_APB1ENR_CAN2EN_Msk             (0x1U << RCC_APB1ENR_CAN2EN_Pos)    /*!< 0x04000000 */
#define RCC_APB1ENR_CAN2EN                 RCC_APB1ENR_CAN2EN_Msk              
#define RCC_APB1ENR_CECEN_Pos              (27U)                               
#define RCC_APB1ENR_CECEN_Msk              (0x1U << RCC_APB1ENR_CECEN_Pos)     /*!< 0x08000000 */
#define RCC_APB1ENR_CECEN                  RCC_APB1ENR_CECEN_Msk               
#define RCC_APB1ENR_PWREN_Pos              (28U)                               
#define RCC_APB1ENR_PWREN_Msk              (0x1U << RCC_APB1ENR_PWREN_Pos)     /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                  RCC_APB1ENR_PWREN_Msk               
#define RCC_APB1ENR_DACEN_Pos              (29U)                               
#define RCC_APB1ENR_DACEN_Msk              (0x1U << RCC_APB1ENR_DACEN_Pos)     /*!< 0x20000000 */
#define RCC_APB1ENR_DACEN                  RCC_APB1ENR_DACEN_Msk               
#define RCC_APB1ENR_UART7EN_Pos            (30U)                               
#define RCC_APB1ENR_UART7EN_Msk            (0x1U << RCC_APB1ENR_UART7EN_Pos)   /*!< 0x40000000 */
#define RCC_APB1ENR_UART7EN                RCC_APB1ENR_UART7EN_Msk             
#define RCC_APB1ENR_UART8EN_Pos            (31U)                               
#define RCC_APB1ENR_UART8EN_Msk            (0x1U << RCC_APB1ENR_UART8EN_Pos)   /*!< 0x80000000 */
#define RCC_APB1ENR_UART8EN                RCC_APB1ENR_UART8EN_Msk             

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define RCC_APB2ENR_TIM1EN_Pos             (0U)                                
#define RCC_APB2ENR_TIM1EN_Msk             (0x1U << RCC_APB2ENR_TIM1EN_Pos)    /*!< 0x00000001 */
#define RCC_APB2ENR_TIM1EN                 RCC_APB2ENR_TIM1EN_Msk              
#define RCC_APB2ENR_TIM8EN_Pos             (1U)                                
#define RCC_APB2ENR_TIM8EN_Msk             (0x1U << RCC_APB2ENR_TIM8EN_Pos)    /*!< 0x00000002 */
#define RCC_APB2ENR_TIM8EN                 RCC_APB2ENR_TIM8EN_Msk              
#define RCC_APB2ENR_USART1EN_Pos           (4U)                                
#define RCC_APB2ENR_USART1EN_Msk           (0x1U << RCC_APB2ENR_USART1EN_Pos)  /*!< 0x00000010 */
#define RCC_APB2ENR_USART1EN               RCC_APB2ENR_USART1EN_Msk            
#define RCC_APB2ENR_USART6EN_Pos           (5U)                                
#define RCC_APB2ENR_USART6EN_Msk           (0x1U << RCC_APB2ENR_USART6EN_Pos)  /*!< 0x00000020 */
#define RCC_APB2ENR_USART6EN               RCC_APB2ENR_USART6EN_Msk            
#define RCC_APB2ENR_ADC1EN_Pos             (8U)                                
#define RCC_APB2ENR_ADC1EN_Msk             (0x1U << RCC_APB2ENR_ADC1EN_Pos)    /*!< 0x00000100 */
#define RCC_APB2ENR_ADC1EN                 RCC_APB2ENR_ADC1EN_Msk              
#define RCC_APB2ENR_ADC2EN_Pos             (9U)                                
#define RCC_APB2ENR_ADC2EN_Msk             (0x1U << RCC_APB2ENR_ADC2EN_Pos)    /*!< 0x00000200 */
#define RCC_APB2ENR_ADC2EN                 RCC_APB2ENR_ADC2EN_Msk              
#define RCC_APB2ENR_ADC3EN_Pos             (10U)                               
#define RCC_APB2ENR_ADC3EN_Msk             (0x1U << RCC_APB2ENR_ADC3EN_Pos)    /*!< 0x00000400 */
#define RCC_APB2ENR_ADC3EN                 RCC_APB2ENR_ADC3EN_Msk              
#define RCC_APB2ENR_SDMMC1EN_Pos           (11U)                               
#define RCC_APB2ENR_SDMMC1EN_Msk           (0x1U << RCC_APB2ENR_SDMMC1EN_Pos)  /*!< 0x00000800 */
#define RCC_APB2ENR_SDMMC1EN               RCC_APB2ENR_SDMMC1EN_Msk            
#define RCC_APB2ENR_SPI1EN_Pos             (12U)                               
#define RCC_APB2ENR_SPI1EN_Msk             (0x1U << RCC_APB2ENR_SPI1EN_Pos)    /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                 RCC_APB2ENR_SPI1EN_Msk              
#define RCC_APB2ENR_SPI4EN_Pos             (13U)                               
#define RCC_APB2ENR_SPI4EN_Msk             (0x1U << RCC_APB2ENR_SPI4EN_Pos)    /*!< 0x00002000 */
#define RCC_APB2ENR_SPI4EN                 RCC_APB2ENR_SPI4EN_Msk              
#define RCC_APB2ENR_SYSCFGEN_Pos           (14U)                               
#define RCC_APB2ENR_SYSCFGEN_Msk           (0x1U << RCC_APB2ENR_SYSCFGEN_Pos)  /*!< 0x00004000 */
#define RCC_APB2ENR_SYSCFGEN               RCC_APB2ENR_SYSCFGEN_Msk            
#define RCC_APB2ENR_TIM9EN_Pos             (16U)                               
#define RCC_APB2ENR_TIM9EN_Msk             (0x1U << RCC_APB2ENR_TIM9EN_Pos)    /*!< 0x00010000 */
#define RCC_APB2ENR_TIM9EN                 RCC_APB2ENR_TIM9EN_Msk              
#define RCC_APB2ENR_TIM10EN_Pos            (17U)                               
#define RCC_APB2ENR_TIM10EN_Msk            (0x1U << RCC_APB2ENR_TIM10EN_Pos)   /*!< 0x00020000 */
#define RCC_APB2ENR_TIM10EN                RCC_APB2ENR_TIM10EN_Msk             
#define RCC_APB2ENR_TIM11EN_Pos            (18U)                               
#define RCC_APB2ENR_TIM11EN_Msk            (0x1U << RCC_APB2ENR_TIM11EN_Pos)   /*!< 0x00040000 */
#define RCC_APB2ENR_TIM11EN                RCC_APB2ENR_TIM11EN_Msk             
#define RCC_APB2ENR_SPI5EN_Pos             (20U)                               
#define RCC_APB2ENR_SPI5EN_Msk             (0x1U << RCC_APB2ENR_SPI5EN_Pos)    /*!< 0x00100000 */
#define RCC_APB2ENR_SPI5EN                 RCC_APB2ENR_SPI5EN_Msk              
#define RCC_APB2ENR_SPI6EN_Pos             (21U)                               
#define RCC_APB2ENR_SPI6EN_Msk             (0x1U << RCC_APB2ENR_SPI6EN_Pos)    /*!< 0x00200000 */
#define RCC_APB2ENR_SPI6EN                 RCC_APB2ENR_SPI6EN_Msk              
#define RCC_APB2ENR_SAI1EN_Pos             (22U)                               
#define RCC_APB2ENR_SAI1EN_Msk             (0x1U << RCC_APB2ENR_SAI1EN_Pos)    /*!< 0x00400000 */
#define RCC_APB2ENR_SAI1EN                 RCC_APB2ENR_SAI1EN_Msk              
#define RCC_APB2ENR_SAI2EN_Pos             (23U)                               
#define RCC_APB2ENR_SAI2EN_Msk             (0x1U << RCC_APB2ENR_SAI2EN_Pos)    /*!< 0x00800000 */
#define RCC_APB2ENR_SAI2EN                 RCC_APB2ENR_SAI2EN_Msk              
#define RCC_APB2ENR_LTDCEN_Pos             (26U)                               
#define RCC_APB2ENR_LTDCEN_Msk             (0x1U << RCC_APB2ENR_LTDCEN_Pos)    /*!< 0x04000000 */
#define RCC_APB2ENR_LTDCEN                 RCC_APB2ENR_LTDCEN_Msk              

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define RCC_AHB1LPENR_GPIOALPEN_Pos        (0U)                                
#define RCC_AHB1LPENR_GPIOALPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOALPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB1LPENR_GPIOALPEN            RCC_AHB1LPENR_GPIOALPEN_Msk         
#define RCC_AHB1LPENR_GPIOBLPEN_Pos        (1U)                                
#define RCC_AHB1LPENR_GPIOBLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOBLPEN_Pos) /*!< 0x00000002 */
#define RCC_AHB1LPENR_GPIOBLPEN            RCC_AHB1LPENR_GPIOBLPEN_Msk         
#define RCC_AHB1LPENR_GPIOCLPEN_Pos        (2U)                                
#define RCC_AHB1LPENR_GPIOCLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOCLPEN_Pos) /*!< 0x00000004 */
#define RCC_AHB1LPENR_GPIOCLPEN            RCC_AHB1LPENR_GPIOCLPEN_Msk         
#define RCC_AHB1LPENR_GPIODLPEN_Pos        (3U)                                
#define RCC_AHB1LPENR_GPIODLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIODLPEN_Pos) /*!< 0x00000008 */
#define RCC_AHB1LPENR_GPIODLPEN            RCC_AHB1LPENR_GPIODLPEN_Msk         
#define RCC_AHB1LPENR_GPIOELPEN_Pos        (4U)                                
#define RCC_AHB1LPENR_GPIOELPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOELPEN_Pos) /*!< 0x00000010 */
#define RCC_AHB1LPENR_GPIOELPEN            RCC_AHB1LPENR_GPIOELPEN_Msk         
#define RCC_AHB1LPENR_GPIOFLPEN_Pos        (5U)                                
#define RCC_AHB1LPENR_GPIOFLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOFLPEN_Pos) /*!< 0x00000020 */
#define RCC_AHB1LPENR_GPIOFLPEN            RCC_AHB1LPENR_GPIOFLPEN_Msk         
#define RCC_AHB1LPENR_GPIOGLPEN_Pos        (6U)                                
#define RCC_AHB1LPENR_GPIOGLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOGLPEN_Pos) /*!< 0x00000040 */
#define RCC_AHB1LPENR_GPIOGLPEN            RCC_AHB1LPENR_GPIOGLPEN_Msk         
#define RCC_AHB1LPENR_GPIOHLPEN_Pos        (7U)                                
#define RCC_AHB1LPENR_GPIOHLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOHLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB1LPENR_GPIOHLPEN            RCC_AHB1LPENR_GPIOHLPEN_Msk         
#define RCC_AHB1LPENR_GPIOILPEN_Pos        (8U)                                
#define RCC_AHB1LPENR_GPIOILPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOILPEN_Pos) /*!< 0x00000100 */
#define RCC_AHB1LPENR_GPIOILPEN            RCC_AHB1LPENR_GPIOILPEN_Msk         
#define RCC_AHB1LPENR_GPIOJLPEN_Pos        (9U)                                
#define RCC_AHB1LPENR_GPIOJLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOJLPEN_Pos) /*!< 0x00000200 */
#define RCC_AHB1LPENR_GPIOJLPEN            RCC_AHB1LPENR_GPIOJLPEN_Msk         
#define RCC_AHB1LPENR_GPIOKLPEN_Pos        (10U)                               
#define RCC_AHB1LPENR_GPIOKLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOKLPEN_Pos) /*!< 0x00000400 */
#define RCC_AHB1LPENR_GPIOKLPEN            RCC_AHB1LPENR_GPIOKLPEN_Msk         
#define RCC_AHB1LPENR_CRCLPEN_Pos          (12U)                               
#define RCC_AHB1LPENR_CRCLPEN_Msk          (0x1U << RCC_AHB1LPENR_CRCLPEN_Pos) /*!< 0x00001000 */
#define RCC_AHB1LPENR_CRCLPEN              RCC_AHB1LPENR_CRCLPEN_Msk           
#define RCC_AHB1LPENR_AXILPEN_Pos          (13U)                               
#define RCC_AHB1LPENR_AXILPEN_Msk          (0x1U << RCC_AHB1LPENR_AXILPEN_Pos) /*!< 0x00002000 */
#define RCC_AHB1LPENR_AXILPEN              RCC_AHB1LPENR_AXILPEN_Msk           
#define RCC_AHB1LPENR_FLITFLPEN_Pos        (15U)                               
#define RCC_AHB1LPENR_FLITFLPEN_Msk        (0x1U << RCC_AHB1LPENR_FLITFLPEN_Pos) /*!< 0x00008000 */
#define RCC_AHB1LPENR_FLITFLPEN            RCC_AHB1LPENR_FLITFLPEN_Msk         
#define RCC_AHB1LPENR_SRAM1LPEN_Pos        (16U)                               
#define RCC_AHB1LPENR_SRAM1LPEN_Msk        (0x1U << RCC_AHB1LPENR_SRAM1LPEN_Pos) /*!< 0x00010000 */
#define RCC_AHB1LPENR_SRAM1LPEN            RCC_AHB1LPENR_SRAM1LPEN_Msk         
#define RCC_AHB1LPENR_SRAM2LPEN_Pos        (17U)                               
#define RCC_AHB1LPENR_SRAM2LPEN_Msk        (0x1U << RCC_AHB1LPENR_SRAM2LPEN_Pos) /*!< 0x00020000 */
#define RCC_AHB1LPENR_SRAM2LPEN            RCC_AHB1LPENR_SRAM2LPEN_Msk         
#define RCC_AHB1LPENR_BKPSRAMLPEN_Pos      (18U)                               
#define RCC_AHB1LPENR_BKPSRAMLPEN_Msk      (0x1U << RCC_AHB1LPENR_BKPSRAMLPEN_Pos) /*!< 0x00040000 */
#define RCC_AHB1LPENR_BKPSRAMLPEN          RCC_AHB1LPENR_BKPSRAMLPEN_Msk       
#define RCC_AHB1LPENR_DTCMLPEN_Pos         (20U)                               
#define RCC_AHB1LPENR_DTCMLPEN_Msk         (0x1U << RCC_AHB1LPENR_DTCMLPEN_Pos) /*!< 0x00100000 */
#define RCC_AHB1LPENR_DTCMLPEN             RCC_AHB1LPENR_DTCMLPEN_Msk          
#define RCC_AHB1LPENR_DMA1LPEN_Pos         (21U)                               
#define RCC_AHB1LPENR_DMA1LPEN_Msk         (0x1U << RCC_AHB1LPENR_DMA1LPEN_Pos) /*!< 0x00200000 */
#define RCC_AHB1LPENR_DMA1LPEN             RCC_AHB1LPENR_DMA1LPEN_Msk          
#define RCC_AHB1LPENR_DMA2LPEN_Pos         (22U)                               
#define RCC_AHB1LPENR_DMA2LPEN_Msk         (0x1U << RCC_AHB1LPENR_DMA2LPEN_Pos) /*!< 0x00400000 */
#define RCC_AHB1LPENR_DMA2LPEN             RCC_AHB1LPENR_DMA2LPEN_Msk          
#define RCC_AHB1LPENR_DMA2DLPEN_Pos        (23U)                               
#define RCC_AHB1LPENR_DMA2DLPEN_Msk        (0x1U << RCC_AHB1LPENR_DMA2DLPEN_Pos) /*!< 0x00800000 */
#define RCC_AHB1LPENR_DMA2DLPEN            RCC_AHB1LPENR_DMA2DLPEN_Msk         
#define RCC_AHB1LPENR_ETHMACLPEN_Pos       (25U)                               
#define RCC_AHB1LPENR_ETHMACLPEN_Msk       (0x1U << RCC_AHB1LPENR_ETHMACLPEN_Pos) /*!< 0x02000000 */
#define RCC_AHB1LPENR_ETHMACLPEN           RCC_AHB1LPENR_ETHMACLPEN_Msk        
#define RCC_AHB1LPENR_ETHMACTXLPEN_Pos     (26U)                               
#define RCC_AHB1LPENR_ETHMACTXLPEN_Msk     (0x1U << RCC_AHB1LPENR_ETHMACTXLPEN_Pos) /*!< 0x04000000 */
#define RCC_AHB1LPENR_ETHMACTXLPEN         RCC_AHB1LPENR_ETHMACTXLPEN_Msk      
#define RCC_AHB1LPENR_ETHMACRXLPEN_Pos     (27U)                               
#define RCC_AHB1LPENR_ETHMACRXLPEN_Msk     (0x1U << RCC_AHB1LPENR_ETHMACRXLPEN_Pos) /*!< 0x08000000 */
#define RCC_AHB1LPENR_ETHMACRXLPEN         RCC_AHB1LPENR_ETHMACRXLPEN_Msk      
#define RCC_AHB1LPENR_ETHMACPTPLPEN_Pos    (28U)                               
#define RCC_AHB1LPENR_ETHMACPTPLPEN_Msk    (0x1U << RCC_AHB1LPENR_ETHMACPTPLPEN_Pos) /*!< 0x10000000 */
#define RCC_AHB1LPENR_ETHMACPTPLPEN        RCC_AHB1LPENR_ETHMACPTPLPEN_Msk     
#define RCC_AHB1LPENR_OTGHSLPEN_Pos        (29U)                               
#define RCC_AHB1LPENR_OTGHSLPEN_Msk        (0x1U << RCC_AHB1LPENR_OTGHSLPEN_Pos) /*!< 0x20000000 */
#define RCC_AHB1LPENR_OTGHSLPEN            RCC_AHB1LPENR_OTGHSLPEN_Msk         
#define RCC_AHB1LPENR_OTGHSULPILPEN_Pos    (30U)                               
#define RCC_AHB1LPENR_OTGHSULPILPEN_Msk    (0x1U << RCC_AHB1LPENR_OTGHSULPILPEN_Pos) /*!< 0x40000000 */
#define RCC_AHB1LPENR_OTGHSULPILPEN        RCC_AHB1LPENR_OTGHSULPILPEN_Msk     

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define RCC_AHB2LPENR_DCMILPEN_Pos         (0U)                                
#define RCC_AHB2LPENR_DCMILPEN_Msk         (0x1U << RCC_AHB2LPENR_DCMILPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB2LPENR_DCMILPEN             RCC_AHB2LPENR_DCMILPEN_Msk          
#define RCC_AHB2LPENR_RNGLPEN_Pos          (6U)                                
#define RCC_AHB2LPENR_RNGLPEN_Msk          (0x1U << RCC_AHB2LPENR_RNGLPEN_Pos) /*!< 0x00000040 */
#define RCC_AHB2LPENR_RNGLPEN              RCC_AHB2LPENR_RNGLPEN_Msk           
#define RCC_AHB2LPENR_OTGFSLPEN_Pos        (7U)                                
#define RCC_AHB2LPENR_OTGFSLPEN_Msk        (0x1U << RCC_AHB2LPENR_OTGFSLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB2LPENR_OTGFSLPEN            RCC_AHB2LPENR_OTGFSLPEN_Msk         

/********************  Bit definition for RCC_AHB3LPENR register  *************/
#define RCC_AHB3LPENR_FMCLPEN_Pos          (0U)                                
#define RCC_AHB3LPENR_FMCLPEN_Msk          (0x1U << RCC_AHB3LPENR_FMCLPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB3LPENR_FMCLPEN              RCC_AHB3LPENR_FMCLPEN_Msk           
#define RCC_AHB3LPENR_QSPILPEN_Pos         (1U)                                
#define RCC_AHB3LPENR_QSPILPEN_Msk         (0x1U << RCC_AHB3LPENR_QSPILPEN_Pos) /*!< 0x00000002 */
#define RCC_AHB3LPENR_QSPILPEN             RCC_AHB3LPENR_QSPILPEN_Msk          
/********************  Bit definition for RCC_APB1LPENR register  *************/
#define RCC_APB1LPENR_TIM2LPEN_Pos         (0U)                                
#define RCC_APB1LPENR_TIM2LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM2LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB1LPENR_TIM2LPEN             RCC_APB1LPENR_TIM2LPEN_Msk          
#define RCC_APB1LPENR_TIM3LPEN_Pos         (1U)                                
#define RCC_APB1LPENR_TIM3LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM3LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB1LPENR_TIM3LPEN             RCC_APB1LPENR_TIM3LPEN_Msk          
#define RCC_APB1LPENR_TIM4LPEN_Pos         (2U)                                
#define RCC_APB1LPENR_TIM4LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM4LPEN_Pos) /*!< 0x00000004 */
#define RCC_APB1LPENR_TIM4LPEN             RCC_APB1LPENR_TIM4LPEN_Msk          
#define RCC_APB1LPENR_TIM5LPEN_Pos         (3U)                                
#define RCC_APB1LPENR_TIM5LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM5LPEN_Pos) /*!< 0x00000008 */
#define RCC_APB1LPENR_TIM5LPEN             RCC_APB1LPENR_TIM5LPEN_Msk          
#define RCC_APB1LPENR_TIM6LPEN_Pos         (4U)                                
#define RCC_APB1LPENR_TIM6LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM6LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB1LPENR_TIM6LPEN             RCC_APB1LPENR_TIM6LPEN_Msk          
#define RCC_APB1LPENR_TIM7LPEN_Pos         (5U)                                
#define RCC_APB1LPENR_TIM7LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM7LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB1LPENR_TIM7LPEN             RCC_APB1LPENR_TIM7LPEN_Msk          
#define RCC_APB1LPENR_TIM12LPEN_Pos        (6U)                                
#define RCC_APB1LPENR_TIM12LPEN_Msk        (0x1U << RCC_APB1LPENR_TIM12LPEN_Pos) /*!< 0x00000040 */
#define RCC_APB1LPENR_TIM12LPEN            RCC_APB1LPENR_TIM12LPEN_Msk         
#define RCC_APB1LPENR_TIM13LPEN_Pos        (7U)                                
#define RCC_APB1LPENR_TIM13LPEN_Msk        (0x1U << RCC_APB1LPENR_TIM13LPEN_Pos) /*!< 0x00000080 */
#define RCC_APB1LPENR_TIM13LPEN            RCC_APB1LPENR_TIM13LPEN_Msk         
#define RCC_APB1LPENR_TIM14LPEN_Pos        (8U)                                
#define RCC_APB1LPENR_TIM14LPEN_Msk        (0x1U << RCC_APB1LPENR_TIM14LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB1LPENR_TIM14LPEN            RCC_APB1LPENR_TIM14LPEN_Msk         
#define RCC_APB1LPENR_LPTIM1LPEN_Pos       (9U)                                
#define RCC_APB1LPENR_LPTIM1LPEN_Msk       (0x1U << RCC_APB1LPENR_LPTIM1LPEN_Pos) /*!< 0x00000200 */
#define RCC_APB1LPENR_LPTIM1LPEN           RCC_APB1LPENR_LPTIM1LPEN_Msk        
#define RCC_APB1LPENR_WWDGLPEN_Pos         (11U)                               
#define RCC_APB1LPENR_WWDGLPEN_Msk         (0x1U << RCC_APB1LPENR_WWDGLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB1LPENR_WWDGLPEN             RCC_APB1LPENR_WWDGLPEN_Msk          
#define RCC_APB1LPENR_SPI2LPEN_Pos         (14U)                               
#define RCC_APB1LPENR_SPI2LPEN_Msk         (0x1U << RCC_APB1LPENR_SPI2LPEN_Pos) /*!< 0x00004000 */
#define RCC_APB1LPENR_SPI2LPEN             RCC_APB1LPENR_SPI2LPEN_Msk          
#define RCC_APB1LPENR_SPI3LPEN_Pos         (15U)                               
#define RCC_APB1LPENR_SPI3LPEN_Msk         (0x1U << RCC_APB1LPENR_SPI3LPEN_Pos) /*!< 0x00008000 */
#define RCC_APB1LPENR_SPI3LPEN             RCC_APB1LPENR_SPI3LPEN_Msk          
#define RCC_APB1LPENR_SPDIFRXLPEN_Pos      (16U)                               
#define RCC_APB1LPENR_SPDIFRXLPEN_Msk      (0x1U << RCC_APB1LPENR_SPDIFRXLPEN_Pos) /*!< 0x00010000 */
#define RCC_APB1LPENR_SPDIFRXLPEN          RCC_APB1LPENR_SPDIFRXLPEN_Msk       
#define RCC_APB1LPENR_USART2LPEN_Pos       (17U)                               
#define RCC_APB1LPENR_USART2LPEN_Msk       (0x1U << RCC_APB1LPENR_USART2LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB1LPENR_USART2LPEN           RCC_APB1LPENR_USART2LPEN_Msk        
#define RCC_APB1LPENR_USART3LPEN_Pos       (18U)                               
#define RCC_APB1LPENR_USART3LPEN_Msk       (0x1U << RCC_APB1LPENR_USART3LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB1LPENR_USART3LPEN           RCC_APB1LPENR_USART3LPEN_Msk        
#define RCC_APB1LPENR_UART4LPEN_Pos        (19U)                               
#define RCC_APB1LPENR_UART4LPEN_Msk        (0x1U << RCC_APB1LPENR_UART4LPEN_Pos) /*!< 0x00080000 */
#define RCC_APB1LPENR_UART4LPEN            RCC_APB1LPENR_UART4LPEN_Msk         
#define RCC_APB1LPENR_UART5LPEN_Pos        (20U)                               
#define RCC_APB1LPENR_UART5LPEN_Msk        (0x1U << RCC_APB1LPENR_UART5LPEN_Pos) /*!< 0x00100000 */
#define RCC_APB1LPENR_UART5LPEN            RCC_APB1LPENR_UART5LPEN_Msk         
#define RCC_APB1LPENR_I2C1LPEN_Pos         (21U)                               
#define RCC_APB1LPENR_I2C1LPEN_Msk         (0x1U << RCC_APB1LPENR_I2C1LPEN_Pos) /*!< 0x00200000 */
#define RCC_APB1LPENR_I2C1LPEN             RCC_APB1LPENR_I2C1LPEN_Msk          
#define RCC_APB1LPENR_I2C2LPEN_Pos         (22U)                               
#define RCC_APB1LPENR_I2C2LPEN_Msk         (0x1U << RCC_APB1LPENR_I2C2LPEN_Pos) /*!< 0x00400000 */
#define RCC_APB1LPENR_I2C2LPEN             RCC_APB1LPENR_I2C2LPEN_Msk          
#define RCC_APB1LPENR_I2C3LPEN_Pos         (23U)                               
#define RCC_APB1LPENR_I2C3LPEN_Msk         (0x1U << RCC_APB1LPENR_I2C3LPEN_Pos) /*!< 0x00800000 */
#define RCC_APB1LPENR_I2C3LPEN             RCC_APB1LPENR_I2C3LPEN_Msk          
#define RCC_APB1LPENR_I2C4LPEN_Pos         (24U)                               
#define RCC_APB1LPENR_I2C4LPEN_Msk         (0x1U << RCC_APB1LPENR_I2C4LPEN_Pos) /*!< 0x01000000 */
#define RCC_APB1LPENR_I2C4LPEN             RCC_APB1LPENR_I2C4LPEN_Msk          
#define RCC_APB1LPENR_CAN1LPEN_Pos         (25U)                               
#define RCC_APB1LPENR_CAN1LPEN_Msk         (0x1U << RCC_APB1LPENR_CAN1LPEN_Pos) /*!< 0x02000000 */
#define RCC_APB1LPENR_CAN1LPEN             RCC_APB1LPENR_CAN1LPEN_Msk          
#define RCC_APB1LPENR_CAN2LPEN_Pos         (26U)                               
#define RCC_APB1LPENR_CAN2LPEN_Msk         (0x1U << RCC_APB1LPENR_CAN2LPEN_Pos) /*!< 0x04000000 */
#define RCC_APB1LPENR_CAN2LPEN             RCC_APB1LPENR_CAN2LPEN_Msk          
#define RCC_APB1LPENR_CECLPEN_Pos          (27U)                               
#define RCC_APB1LPENR_CECLPEN_Msk          (0x1U << RCC_APB1LPENR_CECLPEN_Pos) /*!< 0x08000000 */
#define RCC_APB1LPENR_CECLPEN              RCC_APB1LPENR_CECLPEN_Msk           
#define RCC_APB1LPENR_PWRLPEN_Pos          (28U)                               
#define RCC_APB1LPENR_PWRLPEN_Msk          (0x1U << RCC_APB1LPENR_PWRLPEN_Pos) /*!< 0x10000000 */
#define RCC_APB1LPENR_PWRLPEN              RCC_APB1LPENR_PWRLPEN_Msk           
#define RCC_APB1LPENR_DACLPEN_Pos          (29U)                               
#define RCC_APB1LPENR_DACLPEN_Msk          (0x1U << RCC_APB1LPENR_DACLPEN_Pos) /*!< 0x20000000 */
#define RCC_APB1LPENR_DACLPEN              RCC_APB1LPENR_DACLPEN_Msk           
#define RCC_APB1LPENR_UART7LPEN_Pos        (30U)                               
#define RCC_APB1LPENR_UART7LPEN_Msk        (0x1U << RCC_APB1LPENR_UART7LPEN_Pos) /*!< 0x40000000 */
#define RCC_APB1LPENR_UART7LPEN            RCC_APB1LPENR_UART7LPEN_Msk         
#define RCC_APB1LPENR_UART8LPEN_Pos        (31U)                               
#define RCC_APB1LPENR_UART8LPEN_Msk        (0x1U << RCC_APB1LPENR_UART8LPEN_Pos) /*!< 0x80000000 */
#define RCC_APB1LPENR_UART8LPEN            RCC_APB1LPENR_UART8LPEN_Msk         

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define RCC_APB2LPENR_TIM1LPEN_Pos         (0U)                                
#define RCC_APB2LPENR_TIM1LPEN_Msk         (0x1U << RCC_APB2LPENR_TIM1LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB2LPENR_TIM1LPEN             RCC_APB2LPENR_TIM1LPEN_Msk          
#define RCC_APB2LPENR_TIM8LPEN_Pos         (1U)                                
#define RCC_APB2LPENR_TIM8LPEN_Msk         (0x1U << RCC_APB2LPENR_TIM8LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB2LPENR_TIM8LPEN             RCC_APB2LPENR_TIM8LPEN_Msk          
#define RCC_APB2LPENR_USART1LPEN_Pos       (4U)                                
#define RCC_APB2LPENR_USART1LPEN_Msk       (0x1U << RCC_APB2LPENR_USART1LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB2LPENR_USART1LPEN           RCC_APB2LPENR_USART1LPEN_Msk        
#define RCC_APB2LPENR_USART6LPEN_Pos       (5U)                                
#define RCC_APB2LPENR_USART6LPEN_Msk       (0x1U << RCC_APB2LPENR_USART6LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB2LPENR_USART6LPEN           RCC_APB2LPENR_USART6LPEN_Msk        
#define RCC_APB2LPENR_ADC1LPEN_Pos         (8U)                                
#define RCC_APB2LPENR_ADC1LPEN_Msk         (0x1U << RCC_APB2LPENR_ADC1LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB2LPENR_ADC1LPEN             RCC_APB2LPENR_ADC1LPEN_Msk          
#define RCC_APB2LPENR_ADC2LPEN_Pos         (9U)                                
#define RCC_APB2LPENR_ADC2LPEN_Msk         (0x1U << RCC_APB2LPENR_ADC2LPEN_Pos) /*!< 0x00000200 */
#define RCC_APB2LPENR_ADC2LPEN             RCC_APB2LPENR_ADC2LPEN_Msk          
#define RCC_APB2LPENR_ADC3LPEN_Pos         (10U)                               
#define RCC_APB2LPENR_ADC3LPEN_Msk         (0x1U << RCC_APB2LPENR_ADC3LPEN_Pos) /*!< 0x00000400 */
#define RCC_APB2LPENR_ADC3LPEN             RCC_APB2LPENR_ADC3LPEN_Msk          
#define RCC_APB2LPENR_SDMMC1LPEN_Pos       (11U)                               
#define RCC_APB2LPENR_SDMMC1LPEN_Msk       (0x1U << RCC_APB2LPENR_SDMMC1LPEN_Pos) /*!< 0x00000800 */
#define RCC_APB2LPENR_SDMMC1LPEN           RCC_APB2LPENR_SDMMC1LPEN_Msk        
#define RCC_APB2LPENR_SPI1LPEN_Pos         (12U)                               
#define RCC_APB2LPENR_SPI1LPEN_Msk         (0x1U << RCC_APB2LPENR_SPI1LPEN_Pos) /*!< 0x00001000 */
#define RCC_APB2LPENR_SPI1LPEN             RCC_APB2LPENR_SPI1LPEN_Msk          
#define RCC_APB2LPENR_SPI4LPEN_Pos         (13U)                               
#define RCC_APB2LPENR_SPI4LPEN_Msk         (0x1U << RCC_APB2LPENR_SPI4LPEN_Pos) /*!< 0x00002000 */
#define RCC_APB2LPENR_SPI4LPEN             RCC_APB2LPENR_SPI4LPEN_Msk          
#define RCC_APB2LPENR_SYSCFGLPEN_Pos       (14U)                               
#define RCC_APB2LPENR_SYSCFGLPEN_Msk       (0x1U << RCC_APB2LPENR_SYSCFGLPEN_Pos) /*!< 0x00004000 */
#define RCC_APB2LPENR_SYSCFGLPEN           RCC_APB2LPENR_SYSCFGLPEN_Msk        
#define RCC_APB2LPENR_TIM9LPEN_Pos         (16U)                               
#define RCC_APB2LPENR_TIM9LPEN_Msk         (0x1U << RCC_APB2LPENR_TIM9LPEN_Pos) /*!< 0x00010000 */
#define RCC_APB2LPENR_TIM9LPEN             RCC_APB2LPENR_TIM9LPEN_Msk          
#define RCC_APB2LPENR_TIM10LPEN_Pos        (17U)                               
#define RCC_APB2LPENR_TIM10LPEN_Msk        (0x1U << RCC_APB2LPENR_TIM10LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB2LPENR_TIM10LPEN            RCC_APB2LPENR_TIM10LPEN_Msk         
#define RCC_APB2LPENR_TIM11LPEN_Pos        (18U)                               
#define RCC_APB2LPENR_TIM11LPEN_Msk        (0x1U << RCC_APB2LPENR_TIM11LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB2LPENR_TIM11LPEN            RCC_APB2LPENR_TIM11LPEN_Msk         
#define RCC_APB2LPENR_SPI5LPEN_Pos         (20U)                               
#define RCC_APB2LPENR_SPI5LPEN_Msk         (0x1U << RCC_APB2LPENR_SPI5LPEN_Pos) /*!< 0x00100000 */
#define RCC_APB2LPENR_SPI5LPEN             RCC_APB2LPENR_SPI5LPEN_Msk          
#define RCC_APB2LPENR_SPI6LPEN_Pos         (21U)                               
#define RCC_APB2LPENR_SPI6LPEN_Msk         (0x1U << RCC_APB2LPENR_SPI6LPEN_Pos) /*!< 0x00200000 */
#define RCC_APB2LPENR_SPI6LPEN             RCC_APB2LPENR_SPI6LPEN_Msk          
#define RCC_APB2LPENR_SAI1LPEN_Pos         (22U)                               
#define RCC_APB2LPENR_SAI1LPEN_Msk         (0x1U << RCC_APB2LPENR_SAI1LPEN_Pos) /*!< 0x00400000 */
#define RCC_APB2LPENR_SAI1LPEN             RCC_APB2LPENR_SAI1LPEN_Msk          
#define RCC_APB2LPENR_SAI2LPEN_Pos         (23U)                               
#define RCC_APB2LPENR_SAI2LPEN_Msk         (0x1U << RCC_APB2LPENR_SAI2LPEN_Pos) /*!< 0x00800000 */
#define RCC_APB2LPENR_SAI2LPEN             RCC_APB2LPENR_SAI2LPEN_Msk          
#define RCC_APB2LPENR_LTDCLPEN_Pos         (26U)                               
#define RCC_APB2LPENR_LTDCLPEN_Msk         (0x1U << RCC_APB2LPENR_LTDCLPEN_Pos) /*!< 0x04000000 */
#define RCC_APB2LPENR_LTDCLPEN             RCC_APB2LPENR_LTDCLPEN_Msk          

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSEON_Pos                 (0U)                                
#define RCC_BDCR_LSEON_Msk                 (0x1U << RCC_BDCR_LSEON_Pos)        /*!< 0x00000001 */
#define RCC_BDCR_LSEON                     RCC_BDCR_LSEON_Msk                  
#define RCC_BDCR_LSERDY_Pos                (1U)                                
#define RCC_BDCR_LSERDY_Msk                (0x1U << RCC_BDCR_LSERDY_Pos)       /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                    RCC_BDCR_LSERDY_Msk                 
#define RCC_BDCR_LSEBYP_Pos                (2U)                                
#define RCC_BDCR_LSEBYP_Msk                (0x1U << RCC_BDCR_LSEBYP_Pos)       /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP                    RCC_BDCR_LSEBYP_Msk                 
#define RCC_BDCR_LSEDRV_Pos                (3U)                                
#define RCC_BDCR_LSEDRV_Msk                (0x3U << RCC_BDCR_LSEDRV_Pos)       /*!< 0x00000018 */
#define RCC_BDCR_LSEDRV                    RCC_BDCR_LSEDRV_Msk                 
#define RCC_BDCR_LSEDRV_0                  (0x1U << RCC_BDCR_LSEDRV_Pos)       /*!< 0x00000008 */
#define RCC_BDCR_LSEDRV_1                  (0x2U << RCC_BDCR_LSEDRV_Pos)       /*!< 0x00000010 */
#define RCC_BDCR_RTCSEL_Pos                (8U)                                
#define RCC_BDCR_RTCSEL_Msk                (0x3U << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                    RCC_BDCR_RTCSEL_Msk                 
#define RCC_BDCR_RTCSEL_0                  (0x1U << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                  (0x2U << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000200 */
#define RCC_BDCR_RTCEN_Pos                 (15U)                               
#define RCC_BDCR_RTCEN_Msk                 (0x1U << RCC_BDCR_RTCEN_Pos)        /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                     RCC_BDCR_RTCEN_Msk                  
#define RCC_BDCR_BDRST_Pos                 (16U)                               
#define RCC_BDCR_BDRST_Msk                 (0x1U << RCC_BDCR_BDRST_Pos)        /*!< 0x00010000 */
#define RCC_BDCR_BDRST                     RCC_BDCR_BDRST_Msk                  

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                  (0U)                                
#define RCC_CSR_LSION_Msk                  (0x1U << RCC_CSR_LSION_Pos)         /*!< 0x00000001 */
#define RCC_CSR_LSION                      RCC_CSR_LSION_Msk                   
#define RCC_CSR_LSIRDY_Pos                 (1U)                                
#define RCC_CSR_LSIRDY_Msk                 (0x1U << RCC_CSR_LSIRDY_Pos)        /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                     RCC_CSR_LSIRDY_Msk                  
#define RCC_CSR_RMVF_Pos                   (24U)                               
#define RCC_CSR_RMVF_Msk                   (0x1U << RCC_CSR_RMVF_Pos)          /*!< 0x01000000 */
#define RCC_CSR_RMVF                       RCC_CSR_RMVF_Msk                    
#define RCC_CSR_BORRSTF_Pos                (25U)                               
#define RCC_CSR_BORRSTF_Msk                (0x1U << RCC_CSR_BORRSTF_Pos)       /*!< 0x02000000 */
#define RCC_CSR_BORRSTF                    RCC_CSR_BORRSTF_Msk                 
#define RCC_CSR_PINRSTF_Pos                (26U)                               
#define RCC_CSR_PINRSTF_Msk                (0x1U << RCC_CSR_PINRSTF_Pos)       /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                    RCC_CSR_PINRSTF_Msk                 
#define RCC_CSR_PORRSTF_Pos                (27U)                               
#define RCC_CSR_PORRSTF_Msk                (0x1U << RCC_CSR_PORRSTF_Pos)       /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                    RCC_CSR_PORRSTF_Msk                 
#define RCC_CSR_SFTRSTF_Pos                (28U)                               
#define RCC_CSR_SFTRSTF_Msk                (0x1U << RCC_CSR_SFTRSTF_Pos)       /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                    RCC_CSR_SFTRSTF_Msk                 
#define RCC_CSR_IWDGRSTF_Pos               (29U)                               
#define RCC_CSR_IWDGRSTF_Msk               (0x1U << RCC_CSR_IWDGRSTF_Pos)      /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                   RCC_CSR_IWDGRSTF_Msk                
#define RCC_CSR_WWDGRSTF_Pos               (30U)                               
#define RCC_CSR_WWDGRSTF_Msk               (0x1U << RCC_CSR_WWDGRSTF_Pos)      /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                   RCC_CSR_WWDGRSTF_Msk                
#define RCC_CSR_LPWRRSTF_Pos               (31U)                               
#define RCC_CSR_LPWRRSTF_Msk               (0x1U << RCC_CSR_LPWRRSTF_Pos)      /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF                   RCC_CSR_LPWRRSTF_Msk                

/********************  Bit definition for RCC_SSCGR register  *****************/
#define RCC_SSCGR_MODPER_Pos               (0U)                                
#define RCC_SSCGR_MODPER_Msk               (0x1FFFU << RCC_SSCGR_MODPER_Pos)   /*!< 0x00001FFF */
#define RCC_SSCGR_MODPER                   RCC_SSCGR_MODPER_Msk                
#define RCC_SSCGR_INCSTEP_Pos              (13U)                               
#define RCC_SSCGR_INCSTEP_Msk              (0x7FFFU << RCC_SSCGR_INCSTEP_Pos)  /*!< 0x0FFFE000 */
#define RCC_SSCGR_INCSTEP                  RCC_SSCGR_INCSTEP_Msk               
#define RCC_SSCGR_SPREADSEL_Pos            (30U)                               
#define RCC_SSCGR_SPREADSEL_Msk            (0x1U << RCC_SSCGR_SPREADSEL_Pos)   /*!< 0x40000000 */
#define RCC_SSCGR_SPREADSEL                RCC_SSCGR_SPREADSEL_Msk             
#define RCC_SSCGR_SSCGEN_Pos               (31U)                               
#define RCC_SSCGR_SSCGEN_Msk               (0x1U << RCC_SSCGR_SSCGEN_Pos)      /*!< 0x80000000 */
#define RCC_SSCGR_SSCGEN                   RCC_SSCGR_SSCGEN_Msk                

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define RCC_PLLI2SCFGR_PLLI2SN_Pos         (6U)                                
#define RCC_PLLI2SCFGR_PLLI2SN_Msk         (0x1FFU << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00007FC0 */
#define RCC_PLLI2SCFGR_PLLI2SN             RCC_PLLI2SCFGR_PLLI2SN_Msk          
#define RCC_PLLI2SCFGR_PLLI2SN_0           (0x001U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000040 */
#define RCC_PLLI2SCFGR_PLLI2SN_1           (0x002U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000080 */
#define RCC_PLLI2SCFGR_PLLI2SN_2           (0x004U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000100 */
#define RCC_PLLI2SCFGR_PLLI2SN_3           (0x008U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000200 */
#define RCC_PLLI2SCFGR_PLLI2SN_4           (0x010U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000400 */
#define RCC_PLLI2SCFGR_PLLI2SN_5           (0x020U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000800 */
#define RCC_PLLI2SCFGR_PLLI2SN_6           (0x040U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00001000 */
#define RCC_PLLI2SCFGR_PLLI2SN_7           (0x080U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00002000 */
#define RCC_PLLI2SCFGR_PLLI2SN_8           (0x100U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00004000 */
#define RCC_PLLI2SCFGR_PLLI2SP_Pos         (16U)                               
#define RCC_PLLI2SCFGR_PLLI2SP_Msk         (0x3U << RCC_PLLI2SCFGR_PLLI2SP_Pos) /*!< 0x00030000 */
#define RCC_PLLI2SCFGR_PLLI2SP             RCC_PLLI2SCFGR_PLLI2SP_Msk          
#define RCC_PLLI2SCFGR_PLLI2SP_0           (0x1U << RCC_PLLI2SCFGR_PLLI2SP_Pos) /*!< 0x00010000 */
#define RCC_PLLI2SCFGR_PLLI2SP_1           (0x2U << RCC_PLLI2SCFGR_PLLI2SP_Pos) /*!< 0x00020000 */
#define RCC_PLLI2SCFGR_PLLI2SQ_Pos         (24U)                               
#define RCC_PLLI2SCFGR_PLLI2SQ_Msk         (0xFU << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x0F000000 */
#define RCC_PLLI2SCFGR_PLLI2SQ             RCC_PLLI2SCFGR_PLLI2SQ_Msk          
#define RCC_PLLI2SCFGR_PLLI2SQ_0           (0x1U << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x01000000 */
#define RCC_PLLI2SCFGR_PLLI2SQ_1           (0x2U << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x02000000 */
#define RCC_PLLI2SCFGR_PLLI2SQ_2           (0x4U << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x04000000 */
#define RCC_PLLI2SCFGR_PLLI2SQ_3           (0x8U << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x08000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_Pos         (28U)                               
#define RCC_PLLI2SCFGR_PLLI2SR_Msk         (0x7U << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x70000000 */
#define RCC_PLLI2SCFGR_PLLI2SR             RCC_PLLI2SCFGR_PLLI2SR_Msk          
#define RCC_PLLI2SCFGR_PLLI2SR_0           (0x1U << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x10000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_1           (0x2U << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x20000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_2           (0x4U << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x40000000 */

/********************  Bit definition for RCC_PLLSAICFGR register  ************/
#define RCC_PLLSAICFGR_PLLSAIN_Pos         (6U)                                
#define RCC_PLLSAICFGR_PLLSAIN_Msk         (0x1FFU << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00007FC0 */
#define RCC_PLLSAICFGR_PLLSAIN             RCC_PLLSAICFGR_PLLSAIN_Msk          
#define RCC_PLLSAICFGR_PLLSAIN_0           (0x001U << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000040 */
#define RCC_PLLSAICFGR_PLLSAIN_1           (0x002U << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000080 */
#define RCC_PLLSAICFGR_PLLSAIN_2           (0x004U << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000100 */
#define RCC_PLLSAICFGR_PLLSAIN_3           (0x008U << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000200 */
#define RCC_PLLSAICFGR_PLLSAIN_4           (0x010U << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000400 */
#define RCC_PLLSAICFGR_PLLSAIN_5           (0x020U << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00000800 */
#define RCC_PLLSAICFGR_PLLSAIN_6           (0x040U << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00001000 */
#define RCC_PLLSAICFGR_PLLSAIN_7           (0x080U << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00002000 */
#define RCC_PLLSAICFGR_PLLSAIN_8           (0x100U << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00004000 */
#define RCC_PLLSAICFGR_PLLSAIP_Pos         (16U)                               
#define RCC_PLLSAICFGR_PLLSAIP_Msk         (0x3U << RCC_PLLSAICFGR_PLLSAIP_Pos) /*!< 0x00030000 */
#define RCC_PLLSAICFGR_PLLSAIP             RCC_PLLSAICFGR_PLLSAIP_Msk          
#define RCC_PLLSAICFGR_PLLSAIP_0           (0x1U << RCC_PLLSAICFGR_PLLSAIP_Pos) /*!< 0x00010000 */
#define RCC_PLLSAICFGR_PLLSAIP_1           (0x2U << RCC_PLLSAICFGR_PLLSAIP_Pos) /*!< 0x00020000 */
#define RCC_PLLSAICFGR_PLLSAIQ_Pos         (24U)                               
#define RCC_PLLSAICFGR_PLLSAIQ_Msk         (0xFU << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x0F000000 */
#define RCC_PLLSAICFGR_PLLSAIQ             RCC_PLLSAICFGR_PLLSAIQ_Msk          
#define RCC_PLLSAICFGR_PLLSAIQ_0           (0x1U << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x01000000 */
#define RCC_PLLSAICFGR_PLLSAIQ_1           (0x2U << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x02000000 */
#define RCC_PLLSAICFGR_PLLSAIQ_2           (0x4U << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x04000000 */
#define RCC_PLLSAICFGR_PLLSAIQ_3           (0x8U << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x08000000 */
#define RCC_PLLSAICFGR_PLLSAIR_Pos         (28U)                               
#define RCC_PLLSAICFGR_PLLSAIR_Msk         (0x7U << RCC_PLLSAICFGR_PLLSAIR_Pos) /*!< 0x70000000 */
#define RCC_PLLSAICFGR_PLLSAIR             RCC_PLLSAICFGR_PLLSAIR_Msk          
#define RCC_PLLSAICFGR_PLLSAIR_0           (0x1U << RCC_PLLSAICFGR_PLLSAIR_Pos) /*!< 0x10000000 */
#define RCC_PLLSAICFGR_PLLSAIR_1           (0x2U << RCC_PLLSAICFGR_PLLSAIR_Pos) /*!< 0x20000000 */
#define RCC_PLLSAICFGR_PLLSAIR_2           (0x4U << RCC_PLLSAICFGR_PLLSAIR_Pos) /*!< 0x40000000 */

/********************  Bit definition for RCC_DCKCFGR1 register  ***************/
#define RCC_DCKCFGR1_PLLI2SDIVQ_Pos        (0U)                                
#define RCC_DCKCFGR1_PLLI2SDIVQ_Msk        (0x1FU << RCC_DCKCFGR1_PLLI2SDIVQ_Pos) /*!< 0x0000001F */
#define RCC_DCKCFGR1_PLLI2SDIVQ            RCC_DCKCFGR1_PLLI2SDIVQ_Msk         
#define RCC_DCKCFGR1_PLLI2SDIVQ_0          (0x01U << RCC_DCKCFGR1_PLLI2SDIVQ_Pos) /*!< 0x00000001 */
#define RCC_DCKCFGR1_PLLI2SDIVQ_1          (0x02U << RCC_DCKCFGR1_PLLI2SDIVQ_Pos) /*!< 0x00000002 */
#define RCC_DCKCFGR1_PLLI2SDIVQ_2          (0x04U << RCC_DCKCFGR1_PLLI2SDIVQ_Pos) /*!< 0x00000004 */
#define RCC_DCKCFGR1_PLLI2SDIVQ_3          (0x08U << RCC_DCKCFGR1_PLLI2SDIVQ_Pos) /*!< 0x00000008 */
#define RCC_DCKCFGR1_PLLI2SDIVQ_4          (0x10U << RCC_DCKCFGR1_PLLI2SDIVQ_Pos) /*!< 0x00000010 */

#define RCC_DCKCFGR1_PLLSAIDIVQ_Pos        (8U)                                
#define RCC_DCKCFGR1_PLLSAIDIVQ_Msk        (0x1FU << RCC_DCKCFGR1_PLLSAIDIVQ_Pos) /*!< 0x00001F00 */
#define RCC_DCKCFGR1_PLLSAIDIVQ            RCC_DCKCFGR1_PLLSAIDIVQ_Msk         
#define RCC_DCKCFGR1_PLLSAIDIVQ_0          (0x01U << RCC_DCKCFGR1_PLLSAIDIVQ_Pos) /*!< 0x00000100 */
#define RCC_DCKCFGR1_PLLSAIDIVQ_1          (0x02U << RCC_DCKCFGR1_PLLSAIDIVQ_Pos) /*!< 0x00000200 */
#define RCC_DCKCFGR1_PLLSAIDIVQ_2          (0x04U << RCC_DCKCFGR1_PLLSAIDIVQ_Pos) /*!< 0x00000400 */
#define RCC_DCKCFGR1_PLLSAIDIVQ_3          (0x08U << RCC_DCKCFGR1_PLLSAIDIVQ_Pos) /*!< 0x00000800 */
#define RCC_DCKCFGR1_PLLSAIDIVQ_4          (0x10U << RCC_DCKCFGR1_PLLSAIDIVQ_Pos) /*!< 0x00001000 */

#define RCC_DCKCFGR1_PLLSAIDIVR_Pos        (16U)                               
#define RCC_DCKCFGR1_PLLSAIDIVR_Msk        (0x3U << RCC_DCKCFGR1_PLLSAIDIVR_Pos) /*!< 0x00030000 */
#define RCC_DCKCFGR1_PLLSAIDIVR            RCC_DCKCFGR1_PLLSAIDIVR_Msk         
#define RCC_DCKCFGR1_PLLSAIDIVR_0          (0x1U << RCC_DCKCFGR1_PLLSAIDIVR_Pos) /*!< 0x00010000 */
#define RCC_DCKCFGR1_PLLSAIDIVR_1          (0x2U << RCC_DCKCFGR1_PLLSAIDIVR_Pos) /*!< 0x00020000 */

#define RCC_DCKCFGR1_SAI1SEL_Pos           (20U)                               
#define RCC_DCKCFGR1_SAI1SEL_Msk           (0x3U << RCC_DCKCFGR1_SAI1SEL_Pos)  /*!< 0x00300000 */
#define RCC_DCKCFGR1_SAI1SEL               RCC_DCKCFGR1_SAI1SEL_Msk            
#define RCC_DCKCFGR1_SAI1SEL_0             (0x1U << RCC_DCKCFGR1_SAI1SEL_Pos)  /*!< 0x00100000 */
#define RCC_DCKCFGR1_SAI1SEL_1             (0x2U << RCC_DCKCFGR1_SAI1SEL_Pos)  /*!< 0x00200000 */

#define RCC_DCKCFGR1_SAI2SEL_Pos           (22U)                               
#define RCC_DCKCFGR1_SAI2SEL_Msk           (0x3U << RCC_DCKCFGR1_SAI2SEL_Pos)  /*!< 0x00C00000 */
#define RCC_DCKCFGR1_SAI2SEL               RCC_DCKCFGR1_SAI2SEL_Msk            
#define RCC_DCKCFGR1_SAI2SEL_0             (0x1U << RCC_DCKCFGR1_SAI2SEL_Pos)  /*!< 0x00400000 */
#define RCC_DCKCFGR1_SAI2SEL_1             (0x2U << RCC_DCKCFGR1_SAI2SEL_Pos)  /*!< 0x00800000 */

#define RCC_DCKCFGR1_TIMPRE_Pos            (24U)                               
#define RCC_DCKCFGR1_TIMPRE_Msk            (0x1U << RCC_DCKCFGR1_TIMPRE_Pos)   /*!< 0x01000000 */
#define RCC_DCKCFGR1_TIMPRE                RCC_DCKCFGR1_TIMPRE_Msk             

/********************  Bit definition for RCC_DCKCFGR2 register  ***************/
#define RCC_DCKCFGR2_USART1SEL_Pos         (0U)                                
#define RCC_DCKCFGR2_USART1SEL_Msk         (0x3U << RCC_DCKCFGR2_USART1SEL_Pos) /*!< 0x00000003 */
#define RCC_DCKCFGR2_USART1SEL             RCC_DCKCFGR2_USART1SEL_Msk          
#define RCC_DCKCFGR2_USART1SEL_0           (0x1U << RCC_DCKCFGR2_USART1SEL_Pos) /*!< 0x00000001 */
#define RCC_DCKCFGR2_USART1SEL_1           (0x2U << RCC_DCKCFGR2_USART1SEL_Pos) /*!< 0x00000002 */
#define RCC_DCKCFGR2_USART2SEL_Pos         (2U)                                
#define RCC_DCKCFGR2_USART2SEL_Msk         (0x3U << RCC_DCKCFGR2_USART2SEL_Pos) /*!< 0x0000000C */
#define RCC_DCKCFGR2_USART2SEL             RCC_DCKCFGR2_USART2SEL_Msk          
#define RCC_DCKCFGR2_USART2SEL_0           (0x1U << RCC_DCKCFGR2_USART2SEL_Pos) /*!< 0x00000004 */
#define RCC_DCKCFGR2_USART2SEL_1           (0x2U << RCC_DCKCFGR2_USART2SEL_Pos) /*!< 0x00000008 */
#define RCC_DCKCFGR2_USART3SEL_Pos         (4U)                                
#define RCC_DCKCFGR2_USART3SEL_Msk         (0x3U << RCC_DCKCFGR2_USART3SEL_Pos) /*!< 0x00000030 */
#define RCC_DCKCFGR2_USART3SEL             RCC_DCKCFGR2_USART3SEL_Msk          
#define RCC_DCKCFGR2_USART3SEL_0           (0x1U << RCC_DCKCFGR2_USART3SEL_Pos) /*!< 0x00000010 */
#define RCC_DCKCFGR2_USART3SEL_1           (0x2U << RCC_DCKCFGR2_USART3SEL_Pos) /*!< 0x00000020 */
#define RCC_DCKCFGR2_UART4SEL_Pos          (6U)                                
#define RCC_DCKCFGR2_UART4SEL_Msk          (0x3U << RCC_DCKCFGR2_UART4SEL_Pos) /*!< 0x000000C0 */
#define RCC_DCKCFGR2_UART4SEL              RCC_DCKCFGR2_UART4SEL_Msk           
#define RCC_DCKCFGR2_UART4SEL_0            (0x1U << RCC_DCKCFGR2_UART4SEL_Pos) /*!< 0x00000040 */
#define RCC_DCKCFGR2_UART4SEL_1            (0x2U << RCC_DCKCFGR2_UART4SEL_Pos) /*!< 0x00000080 */
#define RCC_DCKCFGR2_UART5SEL_Pos          (8U)                                
#define RCC_DCKCFGR2_UART5SEL_Msk          (0x3U << RCC_DCKCFGR2_UART5SEL_Pos) /*!< 0x00000300 */
#define RCC_DCKCFGR2_UART5SEL              RCC_DCKCFGR2_UART5SEL_Msk           
#define RCC_DCKCFGR2_UART5SEL_0            (0x1U << RCC_DCKCFGR2_UART5SEL_Pos) /*!< 0x00000100 */
#define RCC_DCKCFGR2_UART5SEL_1            (0x2U << RCC_DCKCFGR2_UART5SEL_Pos) /*!< 0x00000200 */
#define RCC_DCKCFGR2_USART6SEL_Pos         (10U)                               
#define RCC_DCKCFGR2_USART6SEL_Msk         (0x3U << RCC_DCKCFGR2_USART6SEL_Pos) /*!< 0x00000C00 */
#define RCC_DCKCFGR2_USART6SEL             RCC_DCKCFGR2_USART6SEL_Msk          
#define RCC_DCKCFGR2_USART6SEL_0           (0x1U << RCC_DCKCFGR2_USART6SEL_Pos) /*!< 0x00000400 */
#define RCC_DCKCFGR2_USART6SEL_1           (0x2U << RCC_DCKCFGR2_USART6SEL_Pos) /*!< 0x00000800 */
#define RCC_DCKCFGR2_UART7SEL_Pos          (12U)                               
#define RCC_DCKCFGR2_UART7SEL_Msk          (0x3U << RCC_DCKCFGR2_UART7SEL_Pos) /*!< 0x00003000 */
#define RCC_DCKCFGR2_UART7SEL              RCC_DCKCFGR2_UART7SEL_Msk           
#define RCC_DCKCFGR2_UART7SEL_0            (0x1U << RCC_DCKCFGR2_UART7SEL_Pos) /*!< 0x00001000 */
#define RCC_DCKCFGR2_UART7SEL_1            (0x2U << RCC_DCKCFGR2_UART7SEL_Pos) /*!< 0x00002000 */
#define RCC_DCKCFGR2_UART8SEL_Pos          (14U)                               
#define RCC_DCKCFGR2_UART8SEL_Msk          (0x3U << RCC_DCKCFGR2_UART8SEL_Pos) /*!< 0x0000C000 */
#define RCC_DCKCFGR2_UART8SEL              RCC_DCKCFGR2_UART8SEL_Msk           
#define RCC_DCKCFGR2_UART8SEL_0            (0x1U << RCC_DCKCFGR2_UART8SEL_Pos) /*!< 0x00004000 */
#define RCC_DCKCFGR2_UART8SEL_1            (0x2U << RCC_DCKCFGR2_UART8SEL_Pos) /*!< 0x00008000 */
#define RCC_DCKCFGR2_I2C1SEL_Pos           (16U)                               
#define RCC_DCKCFGR2_I2C1SEL_Msk           (0x3U << RCC_DCKCFGR2_I2C1SEL_Pos)  /*!< 0x00030000 */
#define RCC_DCKCFGR2_I2C1SEL               RCC_DCKCFGR2_I2C1SEL_Msk            
#define RCC_DCKCFGR2_I2C1SEL_0             (0x1U << RCC_DCKCFGR2_I2C1SEL_Pos)  /*!< 0x00010000 */
#define RCC_DCKCFGR2_I2C1SEL_1             (0x2U << RCC_DCKCFGR2_I2C1SEL_Pos)  /*!< 0x00020000 */
#define RCC_DCKCFGR2_I2C2SEL_Pos           (18U)                               
#define RCC_DCKCFGR2_I2C2SEL_Msk           (0x3U << RCC_DCKCFGR2_I2C2SEL_Pos)  /*!< 0x000C0000 */
#define RCC_DCKCFGR2_I2C2SEL               RCC_DCKCFGR2_I2C2SEL_Msk            
#define RCC_DCKCFGR2_I2C2SEL_0             (0x1U << RCC_DCKCFGR2_I2C2SEL_Pos)  /*!< 0x00040000 */
#define RCC_DCKCFGR2_I2C2SEL_1             (0x2U << RCC_DCKCFGR2_I2C2SEL_Pos)  /*!< 0x00080000 */
#define RCC_DCKCFGR2_I2C3SEL_Pos           (20U)                               
#define RCC_DCKCFGR2_I2C3SEL_Msk           (0x3U << RCC_DCKCFGR2_I2C3SEL_Pos)  /*!< 0x00300000 */
#define RCC_DCKCFGR2_I2C3SEL               RCC_DCKCFGR2_I2C3SEL_Msk            
#define RCC_DCKCFGR2_I2C3SEL_0             (0x1U << RCC_DCKCFGR2_I2C3SEL_Pos)  /*!< 0x00100000 */
#define RCC_DCKCFGR2_I2C3SEL_1             (0x2U << RCC_DCKCFGR2_I2C3SEL_Pos)  /*!< 0x00200000 */
#define RCC_DCKCFGR2_I2C4SEL_Pos           (22U)                               
#define RCC_DCKCFGR2_I2C4SEL_Msk           (0x3U << RCC_DCKCFGR2_I2C4SEL_Pos)  /*!< 0x00C00000 */
#define RCC_DCKCFGR2_I2C4SEL               RCC_DCKCFGR2_I2C4SEL_Msk            
#define RCC_DCKCFGR2_I2C4SEL_0             (0x1U << RCC_DCKCFGR2_I2C4SEL_Pos)  /*!< 0x00400000 */
#define RCC_DCKCFGR2_I2C4SEL_1             (0x2U << RCC_DCKCFGR2_I2C4SEL_Pos)  /*!< 0x00800000 */
#define RCC_DCKCFGR2_LPTIM1SEL_Pos         (24U)                               
#define RCC_DCKCFGR2_LPTIM1SEL_Msk         (0x3U << RCC_DCKCFGR2_LPTIM1SEL_Pos) /*!< 0x03000000 */
#define RCC_DCKCFGR2_LPTIM1SEL             RCC_DCKCFGR2_LPTIM1SEL_Msk          
#define RCC_DCKCFGR2_LPTIM1SEL_0           (0x1U << RCC_DCKCFGR2_LPTIM1SEL_Pos) /*!< 0x01000000 */
#define RCC_DCKCFGR2_LPTIM1SEL_1           (0x2U << RCC_DCKCFGR2_LPTIM1SEL_Pos) /*!< 0x02000000 */
#define RCC_DCKCFGR2_CECSEL_Pos            (26U)                               
#define RCC_DCKCFGR2_CECSEL_Msk            (0x1U << RCC_DCKCFGR2_CECSEL_Pos)   /*!< 0x04000000 */
#define RCC_DCKCFGR2_CECSEL                RCC_DCKCFGR2_CECSEL_Msk             
#define RCC_DCKCFGR2_CK48MSEL_Pos          (27U)                               
#define RCC_DCKCFGR2_CK48MSEL_Msk          (0x1U << RCC_DCKCFGR2_CK48MSEL_Pos) /*!< 0x08000000 */
#define RCC_DCKCFGR2_CK48MSEL              RCC_DCKCFGR2_CK48MSEL_Msk           
#define RCC_DCKCFGR2_SDMMC1SEL_Pos         (28U)                               
#define RCC_DCKCFGR2_SDMMC1SEL_Msk         (0x1U << RCC_DCKCFGR2_SDMMC1SEL_Pos) /*!< 0x10000000 */
#define RCC_DCKCFGR2_SDMMC1SEL             RCC_DCKCFGR2_SDMMC1SEL_Msk          

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*
* @brief FLASH Total Sectors Number
*/
#define FLASH_SECTOR_TOTAL  8

/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY_Pos         (0U)                                     
#define FLASH_ACR_LATENCY_Msk         (0xFU << FLASH_ACR_LATENCY_Pos)          /*!< 0x0000000F */
#define FLASH_ACR_LATENCY             FLASH_ACR_LATENCY_Msk                    
#define FLASH_ACR_LATENCY_0WS         0x00000000U                              
#define FLASH_ACR_LATENCY_1WS         0x00000001U                              
#define FLASH_ACR_LATENCY_2WS         0x00000002U                              
#define FLASH_ACR_LATENCY_3WS         0x00000003U                              
#define FLASH_ACR_LATENCY_4WS         0x00000004U                              
#define FLASH_ACR_LATENCY_5WS         0x00000005U                              
#define FLASH_ACR_LATENCY_6WS         0x00000006U                              
#define FLASH_ACR_LATENCY_7WS         0x00000007U                              
#define FLASH_ACR_LATENCY_8WS         0x00000008U                              
#define FLASH_ACR_LATENCY_9WS         0x00000009U                              
#define FLASH_ACR_LATENCY_10WS        0x0000000AU                              
#define FLASH_ACR_LATENCY_11WS        0x0000000BU                              
#define FLASH_ACR_LATENCY_12WS        0x0000000CU                              
#define FLASH_ACR_LATENCY_13WS        0x0000000DU                              
#define FLASH_ACR_LATENCY_14WS        0x0000000EU                              
#define FLASH_ACR_LATENCY_15WS        0x0000000FU                              
#define FLASH_ACR_PRFTEN_Pos          (8U)                                     
#define FLASH_ACR_PRFTEN_Msk          (0x1U << FLASH_ACR_PRFTEN_Pos)           /*!< 0x00000100 */
#define FLASH_ACR_PRFTEN              FLASH_ACR_PRFTEN_Msk                     
#define FLASH_ACR_ARTEN_Pos           (9U)                                     
#define FLASH_ACR_ARTEN_Msk           (0x1U << FLASH_ACR_ARTEN_Pos)            /*!< 0x00000200 */
#define FLASH_ACR_ARTEN               FLASH_ACR_ARTEN_Msk                      
#define FLASH_ACR_ARTRST_Pos          (11U)                                    
#define FLASH_ACR_ARTRST_Msk          (0x1U << FLASH_ACR_ARTRST_Pos)           /*!< 0x00000800 */
#define FLASH_ACR_ARTRST              FLASH_ACR_ARTRST_Msk   

/*--------------------------------------------------------
  Parameters
*------------------------------------------------------*/
#define ALL_PORTS_AHB_CLK_ENABLE (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | \
                                  RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN | RCC_AHB1ENR_GPIOJEN | \
                                  RCC_AHB1ENR_GPIOKEN)

#define ALL_PINS_AF_ENABLE (GPIO_MODER_MODER0_1  | GPIO_MODER_MODER1_1  | GPIO_MODER_MODER2_1  | GPIO_MODER_MODER3_1  | \
                            GPIO_MODER_MODER4_1  | GPIO_MODER_MODER5_1  | GPIO_MODER_MODER6_1  | GPIO_MODER_MODER7_1  | \
                            GPIO_MODER_MODER8_1  | GPIO_MODER_MODER9_1  | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 | \
                            GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1)


#define ALL_PINS_HIGH_SPEED (GPIO_OSPEEDER_OSPEEDR0_1  | GPIO_OSPEEDER_OSPEEDR1_1  | GPIO_OSPEEDER_OSPEEDR2_1  | GPIO_OSPEEDER_OSPEEDR3_1  | \
                             GPIO_OSPEEDER_OSPEEDR4_1  | GPIO_OSPEEDER_OSPEEDR5_1  | GPIO_OSPEEDER_OSPEEDR6_1  | GPIO_OSPEEDER_OSPEEDR7_1  | \
                             GPIO_OSPEEDER_OSPEEDR8_1  | GPIO_OSPEEDER_OSPEEDR9_1  | GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR11_1 | \
                             GPIO_OSPEEDER_OSPEEDR12_1 | GPIO_OSPEEDER_OSPEEDR13_1 | GPIO_OSPEEDER_OSPEEDR14_1 | GPIO_OSPEEDER_OSPEEDR15_1)

#define PI12_STANDBY_MODE_EN  (1<<12)

#define PK3_TURN_ON_BACKLIGHT (1<<3)


/*--------------------------------------------------------
  Prototypes
 *------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------------
  GPIO_init 
-----------------------------------------------------------------------------------------------------*/
void GPIO_init(void);


/*----------------------------------------------------------------------------------------------------
  system_clock_init 
------------------------------------------------------------------------------------------------------

   1. System clock configuration (SYSCLK) - (CPU clock HCLK)
      The system clock (SYSCLK) is the clock the other clocks in the processor are derived from,
      included but not limited to the core clock (HCLK) and peripheral clocks
      
      
      
      
                           |\              -------> HCLK ---> Core, AHB, DMA
         RCC-------->HSI-->| \             |
         XTL-------->HSE-->|  |--- Sysclk---->
               |     PLL-->| /
               ↓           |/|
            ------           |
            |PLLSAI|        SW
            ------

      Flash latency
      Represents the ratio of the CPU clock (HCLK) period to the Flash memory acces time
      As the CPU is running at full speed, then the number of wait sates need to be programmed.
      The CPU can run without any wait state at 24MHz or below

   2. Configure the required pixel clock as indicated in the panel datasheet to 9.5 MHz
      PLLSAI use the same input clock as PLL
      LTDC clock is derived from PLLSAI 
   
      HSE  = 25MHz
      PLLM = 25
   
      VCO_input_freq = HSI/PLLM
      
      VCO_out_freq = VCO_input_freq*PLLSAIN, (PLLSAIN = 192)
         
      f(PLLSAIR) = VCO_out_freq/PLLSAIR,  (PLLSAIR = 5)
       
      LCD_clk = f(PLLSAIR)/PLLSAIDIVR,  (PLLSAIDIVR = 4)
     
                          ______________PLLSAI_________
                         |                             |
      HSE--->| /PLLM |-->|--->|VCO|------->|/PLLSAIR|--|--> f(PLLSAIR) --->|/PLLSAIDIVR|-->|AND|-->LCD_clk          
                         | ↑         ↓                 |                                     ↑
                         | ↑---|xPLLSAIN|              |                                   LTDC_EN
                         |_____________________________|     

      
      
      These bits should written when the PLLSAI is disabled
   
      PLLM is set to 25, which generates a 25MHz/25= 1MHz input clock to the VCO_input_freq
      to avoid jitter             

*-----------------------------------------------------------------------------------------------------*/
void system_clock_init(void);



/*----------------------------------------------------------------------------------------------
  LTDC initialization 
------------------------------------------------------------------------------------------------
   1. Synchronous timings configuration: 
      VSYNC, HSYNC, vertical and horizontal back porch, active data area and the front porch 
      timings following the panel datasheet

      Custom resolution is 480x272 pixels of Active Area
      
      Hsync = 4  | HBP = 43 | HActive = 480 | HFP = 8 

      Vsync = 10 | VBP = 12 | VActive = 272 | VFP = 4         

      Hsync, Vsync, not-data-enable and pixel_clk wll be active low by default
      Dither disable by default      

   2. Layer Configuration

   3. Reload the shadow registers to active register
      Most of the layers’ configuration registers are shadowed
      so they must be reloaded after being configured and before
      enabling the LTDC

-----------------------------------------------------------------------------------------------*/
void LTDC_init(void);


/*----------------------------------------------------------------------------------------------
   system_init 
----------------------------------------------------------------------------------------------*/
int32_t system_init(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SYSTEM_INIT_H */
