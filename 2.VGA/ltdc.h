/*-----------------------------------------------------------------------------
   LCD-TFT Display Controller (LTDC)
-------------------------------------------------------------------------------
   Author: Fernando Ojeda L.                                          Jan-2020
-------------------------------------------------------------------------------
   Description:
-------------------------------------------------------------------------------
   The
   
   
   
   
  
-------------------------------------------------------------------------------*/

/*--------------------------------------------------------
  Include Files
 *------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>                
//#include "480x272.h"
#include "pirilika480x272.h"                


/*--------------------------------------------------------
  Macros
*------------------------------------------------------*/
#define     __IO    volatile             /*!< Defines 'read / write' permissions */


/* Error list */
#define LTDC_ERROR_SYNC 457
#define LTDC_ERROR_BUFF 458
#define LTDC_ERROR_AREA 459

/* Max Parameters */
#define HSYNCMAX 1
#define VSYNCMAX 10
#define HBPMAX 43
#define VBPMAX 12

#define ACTIVE_WMAX 480
#define ACTIVE_HMAX 272
#define ACTIVE_TOTAL_WMAX 531
#define ACTIVE_TOTAL_HMAX 288


/*--------------------------------------------------------
  LCD-TFT Display Controller Registers
*------------------------------------------------------*/
typedef struct
{
  uint32_t      RESERVED0[2];  /*!< Reserved, 0x00-0x04 */
  __IO uint32_t SSCR;          /*!< LTDC Synchronization Size Configuration Register,    Address offset: 0x08 */
  __IO uint32_t BPCR;          /*!< LTDC Back Porch Configuration Register,              Address offset: 0x0C */
  __IO uint32_t AWCR;          /*!< LTDC Active Width Configuration Register,            Address offset: 0x10 */
  __IO uint32_t TWCR;          /*!< LTDC Total Width Configuration Register,             Address offset: 0x14 */
  __IO uint32_t GCR;           /*!< LTDC Global Control Register,                        Address offset: 0x18 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x1C-0x20 */
  __IO uint32_t SRCR;          /*!< LTDC Shadow Reload Configuration Register,           Address offset: 0x24 */
  uint32_t      RESERVED2[1];  /*!< Reserved, 0x28 */
  __IO uint32_t BCCR;          /*!< LTDC Background Color Configuration Register,        Address offset: 0x2C */
  uint32_t      RESERVED3[1];  /*!< Reserved, 0x30 */
  __IO uint32_t IER;           /*!< LTDC Interrupt Enable Register,                      Address offset: 0x34 */
  __IO uint32_t ISR;           /*!< LTDC Interrupt Status Register,                      Address offset: 0x38 */
  __IO uint32_t ICR;           /*!< LTDC Interrupt Clear Register,                       Address offset: 0x3C */
  __IO uint32_t LIPCR;         /*!< LTDC Line Interrupt Position Configuration Register, Address offset: 0x40 */
  __IO uint32_t CPSR;          /*!< LTDC Current Position Status Register,               Address offset: 0x44 */
  __IO uint32_t CDSR;         /*!< LTDC Current Display Status Register,                 Address offset: 0x48 */
} LTDC_TypeDef;

/*--------------------------------------------------------
  LCD-TFT Display layer x Controllerr Registers
*------------------------------------------------------*/
typedef struct
{
  __IO uint32_t CR;            /*!< LTDC Layerx Control Register                                  Address offset: 0x84 */
  __IO uint32_t WHPCR;         /*!< LTDC Layerx Window Horizontal Position Configuration Register Address offset: 0x88 */
  __IO uint32_t WVPCR;         /*!< LTDC Layerx Window Vertical Position Configuration Register   Address offset: 0x8C */
  __IO uint32_t CKCR;          /*!< LTDC Layerx Color Keying Configuration Register               Address offset: 0x90 */
  __IO uint32_t PFCR;          /*!< LTDC Layerx Pixel Format Configuration Register               Address offset: 0x94 */
  __IO uint32_t CACR;          /*!< LTDC Layerx Constant Alpha Configuration Register             Address offset: 0x98 */
  __IO uint32_t DCCR;          /*!< LTDC Layerx Default Color Configuration Register              Address offset: 0x9C */
  __IO uint32_t BFCR;          /*!< LTDC Layerx Blending Factors Configuration Register           Address offset: 0xA0 */
  uint32_t      RESERVED0[2];  /*!< Reserved */
  __IO uint32_t CFBAR;         /*!< LTDC Layerx Color Frame Buffer Address Register               Address offset: 0xAC */
  __IO uint32_t CFBLR;         /*!< LTDC Layerx Color Frame Buffer Length Register                Address offset: 0xB0 */
  __IO uint32_t CFBLNR;        /*!< LTDC Layerx ColorFrame Buffer Line Number Register            Address offset: 0xB4 */
  uint32_t      RESERVED1[3];  /*!< Reserved */
  __IO uint32_t CLUTWR;        /*!< LTDC Layerx CLUT Write Register                               Address offset: 0x144 */

} LTDC_Layer_TypeDef;

/*--------------------------------------------------------
  Peripheral memory map
*--------------------------------------------------------*/
#define PERIPH_BASE            0x40000000U /* Base address of : AHB/ABP Peripherals */
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define LTDC_BASE             (APB2PERIPH_BASE + 0x6800U)
#define LTDC_Layer1_BASE      (LTDC_BASE + 0x84U)


/******************************************************************************/
/*                                                                            */
/*                      LCD-TFT Display Controller (LTDC)                     */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for LTDC_SSCR register  *****************/

#define LTDC_SSCR_VSH_Pos            (0U)                                      
#define LTDC_SSCR_VSH_Msk            (0x7FFU << LTDC_SSCR_VSH_Pos)             /*!< 0x000007FF */
#define LTDC_SSCR_VSH                LTDC_SSCR_VSH_Msk                         /*!< Vertical Synchronization Height  */
#define LTDC_SSCR_HSW_Pos            (16U)                                     
#define LTDC_SSCR_HSW_Msk            (0xFFFU << LTDC_SSCR_HSW_Pos)             /*!< 0x0FFF0000 */
#define LTDC_SSCR_HSW                LTDC_SSCR_HSW_Msk                         /*!< Horizontal Synchronization Width */

/********************  Bit definition for LTDC_BPCR register  *****************/

#define LTDC_BPCR_AVBP_Pos           (0U)                                      
#define LTDC_BPCR_AVBP_Msk           (0x7FFU << LTDC_BPCR_AVBP_Pos)            /*!< 0x000007FF */
#define LTDC_BPCR_AVBP               LTDC_BPCR_AVBP_Msk                        /*!< Accumulated Vertical Back Porch   */
#define LTDC_BPCR_AHBP_Pos           (16U)                                     
#define LTDC_BPCR_AHBP_Msk           (0xFFFU << LTDC_BPCR_AHBP_Pos)            /*!< 0x0FFF0000 */
#define LTDC_BPCR_AHBP               LTDC_BPCR_AHBP_Msk                        /*!< Accumulated Horizontal Back Porch */

/********************  Bit definition for LTDC_AWCR register  *****************/

#define LTDC_AWCR_AAH_Pos            (0U)                                      
#define LTDC_AWCR_AAH_Msk            (0x7FFU << LTDC_AWCR_AAH_Pos)             /*!< 0x000007FF */
#define LTDC_AWCR_AAH                LTDC_AWCR_AAH_Msk                         /*!< Accumulated Active heigh */
#define LTDC_AWCR_AAW_Pos            (16U)                                     
#define LTDC_AWCR_AAW_Msk            (0xFFFU << LTDC_AWCR_AAW_Pos)             /*!< 0x0FFF0000 */
#define LTDC_AWCR_AAW                LTDC_AWCR_AAW_Msk                         /*!< Accumulated Active Width */

/********************  Bit definition for LTDC_TWCR register  *****************/

#define LTDC_TWCR_TOTALH_Pos         (0U)                                      
#define LTDC_TWCR_TOTALH_Msk         (0x7FFU << LTDC_TWCR_TOTALH_Pos)          /*!< 0x000007FF */
#define LTDC_TWCR_TOTALH             LTDC_TWCR_TOTALH_Msk                      /*!< Total Heigh */
#define LTDC_TWCR_TOTALW_Pos         (16U)                                     
#define LTDC_TWCR_TOTALW_Msk         (0xFFFU << LTDC_TWCR_TOTALW_Pos)          /*!< 0x0FFF0000 */
#define LTDC_TWCR_TOTALW             LTDC_TWCR_TOTALW_Msk                      /*!< Total Width */

/********************  Bit definition for LTDC_GCR register  ******************/

#define LTDC_GCR_LTDCEN_Pos          (0U)                                      
#define LTDC_GCR_LTDCEN_Msk          (0x1U << LTDC_GCR_LTDCEN_Pos)             /*!< 0x00000001 */
#define LTDC_GCR_LTDCEN              LTDC_GCR_LTDCEN_Msk                       /*!< LCD-TFT controller enable bit       */
#define LTDC_GCR_DBW_Pos             (4U)                                      
#define LTDC_GCR_DBW_Msk             (0x7U << LTDC_GCR_DBW_Pos)                /*!< 0x00000070 */
#define LTDC_GCR_DBW                 LTDC_GCR_DBW_Msk                          /*!< Dither Blue Width                   */
#define LTDC_GCR_DGW_Pos             (8U)                                      
#define LTDC_GCR_DGW_Msk             (0x7U << LTDC_GCR_DGW_Pos)                /*!< 0x00000700 */
#define LTDC_GCR_DGW                 LTDC_GCR_DGW_Msk                          /*!< Dither Green Width                  */
#define LTDC_GCR_DRW_Pos             (12U)                                     
#define LTDC_GCR_DRW_Msk             (0x7U << LTDC_GCR_DRW_Pos)                /*!< 0x00007000 */
#define LTDC_GCR_DRW                 LTDC_GCR_DRW_Msk                          /*!< Dither Red Width                    */
#define LTDC_GCR_DEN_Pos             (16U)                                     
#define LTDC_GCR_DEN_Msk             (0x1U << LTDC_GCR_DEN_Pos)                /*!< 0x00010000 */
#define LTDC_GCR_DEN                 LTDC_GCR_DEN_Msk                          /*!< Dither Enable                       */
#define LTDC_GCR_PCPOL_Pos           (28U)                                     
#define LTDC_GCR_PCPOL_Msk           (0x1U << LTDC_GCR_PCPOL_Pos)              /*!< 0x10000000 */
#define LTDC_GCR_PCPOL               LTDC_GCR_PCPOL_Msk                        /*!< Pixel Clock Polarity                */
#define LTDC_GCR_DEPOL_Pos           (29U)                                     
#define LTDC_GCR_DEPOL_Msk           (0x1U << LTDC_GCR_DEPOL_Pos)              /*!< 0x20000000 */
#define LTDC_GCR_DEPOL               LTDC_GCR_DEPOL_Msk                        /*!< Data Enable Polarity                */
#define LTDC_GCR_VSPOL_Pos           (30U)                                     
#define LTDC_GCR_VSPOL_Msk           (0x1U << LTDC_GCR_VSPOL_Pos)              /*!< 0x40000000 */
#define LTDC_GCR_VSPOL               LTDC_GCR_VSPOL_Msk                        /*!< Vertical Synchronization Polarity   */
#define LTDC_GCR_HSPOL_Pos           (31U)                                     
#define LTDC_GCR_HSPOL_Msk           (0x1U << LTDC_GCR_HSPOL_Pos)              /*!< 0x80000000 */
#define LTDC_GCR_HSPOL               LTDC_GCR_HSPOL_Msk                        /*!< Horizontal Synchronization Polarity */

/* Legacy define */
#define LTDC_GCR_DTEN                       LTDC_GCR_DEN

/********************  Bit definition for LTDC_SRCR register  *****************/

#define LTDC_SRCR_IMR_Pos            (0U)                                      
#define LTDC_SRCR_IMR_Msk            (0x1U << LTDC_SRCR_IMR_Pos)               /*!< 0x00000001 */
#define LTDC_SRCR_IMR                LTDC_SRCR_IMR_Msk                         /*!< Immediate Reload         */
#define LTDC_SRCR_VBR_Pos            (1U)                                      
#define LTDC_SRCR_VBR_Msk            (0x1U << LTDC_SRCR_VBR_Pos)               /*!< 0x00000002 */
#define LTDC_SRCR_VBR                LTDC_SRCR_VBR_Msk                         /*!< Vertical Blanking Reload */

/********************  Bit definition for LTDC_BCCR register  *****************/

#define LTDC_BCCR_BCBLUE_Pos         (0U)                                      
#define LTDC_BCCR_BCBLUE_Msk         (0xFFU << LTDC_BCCR_BCBLUE_Pos)           /*!< 0x000000FF */
#define LTDC_BCCR_BCBLUE             LTDC_BCCR_BCBLUE_Msk                      /*!< Background Blue value  */
#define LTDC_BCCR_BCGREEN_Pos        (8U)                                      
#define LTDC_BCCR_BCGREEN_Msk        (0xFFU << LTDC_BCCR_BCGREEN_Pos)          /*!< 0x0000FF00 */
#define LTDC_BCCR_BCGREEN            LTDC_BCCR_BCGREEN_Msk                     /*!< Background Green value */
#define LTDC_BCCR_BCRED_Pos          (16U)                                     
#define LTDC_BCCR_BCRED_Msk          (0xFFU << LTDC_BCCR_BCRED_Pos)            /*!< 0x00FF0000 */
#define LTDC_BCCR_BCRED              LTDC_BCCR_BCRED_Msk                       /*!< Background Red value   */

/********************  Bit definition for LTDC_IER register  ******************/

#define LTDC_IER_LIE_Pos             (0U)                                      
#define LTDC_IER_LIE_Msk             (0x1U << LTDC_IER_LIE_Pos)                /*!< 0x00000001 */
#define LTDC_IER_LIE                 LTDC_IER_LIE_Msk                          /*!< Line Interrupt Enable            */
#define LTDC_IER_FUIE_Pos            (1U)                                      
#define LTDC_IER_FUIE_Msk            (0x1U << LTDC_IER_FUIE_Pos)               /*!< 0x00000002 */
#define LTDC_IER_FUIE                LTDC_IER_FUIE_Msk                         /*!< FIFO Underrun Interrupt Enable   */
#define LTDC_IER_TERRIE_Pos          (2U)                                      
#define LTDC_IER_TERRIE_Msk          (0x1U << LTDC_IER_TERRIE_Pos)             /*!< 0x00000004 */
#define LTDC_IER_TERRIE              LTDC_IER_TERRIE_Msk                       /*!< Transfer Error Interrupt Enable  */
#define LTDC_IER_RRIE_Pos            (3U)                                      
#define LTDC_IER_RRIE_Msk            (0x1U << LTDC_IER_RRIE_Pos)               /*!< 0x00000008 */
#define LTDC_IER_RRIE                LTDC_IER_RRIE_Msk                         /*!< Register Reload interrupt enable */

/********************  Bit definition for LTDC_ISR register  ******************/

#define LTDC_ISR_LIF_Pos             (0U)                                      
#define LTDC_ISR_LIF_Msk             (0x1U << LTDC_ISR_LIF_Pos)                /*!< 0x00000001 */
#define LTDC_ISR_LIF                 LTDC_ISR_LIF_Msk                          /*!< Line Interrupt Flag */
#define LTDC_ISR_FUIF_Pos            (1U)                                      
#define LTDC_ISR_FUIF_Msk            (0x1U << LTDC_ISR_FUIF_Pos)               /*!< 0x00000002 */
#define LTDC_ISR_FUIF                LTDC_ISR_FUIF_Msk                         /*!< FIFO Underrun Interrupt Flag */
#define LTDC_ISR_TERRIF_Pos          (2U)                                      
#define LTDC_ISR_TERRIF_Msk          (0x1U << LTDC_ISR_TERRIF_Pos)             /*!< 0x00000004 */
#define LTDC_ISR_TERRIF              LTDC_ISR_TERRIF_Msk                       /*!< Transfer Error Interrupt Flag */
#define LTDC_ISR_RRIF_Pos            (3U)                                      
#define LTDC_ISR_RRIF_Msk            (0x1U << LTDC_ISR_RRIF_Pos)               /*!< 0x00000008 */
#define LTDC_ISR_RRIF                LTDC_ISR_RRIF_Msk                         /*!< Register Reload interrupt Flag */

/********************  Bit definition for LTDC_ICR register  ******************/

#define LTDC_ICR_CLIF_Pos            (0U)                                      
#define LTDC_ICR_CLIF_Msk            (0x1U << LTDC_ICR_CLIF_Pos)               /*!< 0x00000001 */
#define LTDC_ICR_CLIF                LTDC_ICR_CLIF_Msk                         /*!< Clears the Line Interrupt Flag */
#define LTDC_ICR_CFUIF_Pos           (1U)                                      
#define LTDC_ICR_CFUIF_Msk           (0x1U << LTDC_ICR_CFUIF_Pos)              /*!< 0x00000002 */
#define LTDC_ICR_CFUIF               LTDC_ICR_CFUIF_Msk                        /*!< Clears the FIFO Underrun Interrupt Flag */
#define LTDC_ICR_CTERRIF_Pos         (2U)                                      
#define LTDC_ICR_CTERRIF_Msk         (0x1U << LTDC_ICR_CTERRIF_Pos)            /*!< 0x00000004 */
#define LTDC_ICR_CTERRIF             LTDC_ICR_CTERRIF_Msk                      /*!< Clears the Transfer Error Interrupt Flag */
#define LTDC_ICR_CRRIF_Pos           (3U)                                      
#define LTDC_ICR_CRRIF_Msk           (0x1U << LTDC_ICR_CRRIF_Pos)              /*!< 0x00000008 */
#define LTDC_ICR_CRRIF               LTDC_ICR_CRRIF_Msk                        /*!< Clears Register Reload interrupt Flag */

/********************  Bit definition for LTDC_LIPCR register  ****************/

#define LTDC_LIPCR_LIPOS_Pos         (0U)                                      
#define LTDC_LIPCR_LIPOS_Msk         (0x7FFU << LTDC_LIPCR_LIPOS_Pos)          /*!< 0x000007FF */
#define LTDC_LIPCR_LIPOS             LTDC_LIPCR_LIPOS_Msk                      /*!< Line Interrupt Position */

/********************  Bit definition for LTDC_CPSR register  *****************/

#define LTDC_CPSR_CYPOS_Pos          (0U)                                      
#define LTDC_CPSR_CYPOS_Msk          (0xFFFFU << LTDC_CPSR_CYPOS_Pos)          /*!< 0x0000FFFF */
#define LTDC_CPSR_CYPOS              LTDC_CPSR_CYPOS_Msk                       /*!< Current Y Position */
#define LTDC_CPSR_CXPOS_Pos          (16U)                                     
#define LTDC_CPSR_CXPOS_Msk          (0xFFFFU << LTDC_CPSR_CXPOS_Pos)          /*!< 0xFFFF0000 */
#define LTDC_CPSR_CXPOS              LTDC_CPSR_CXPOS_Msk                       /*!< Current X Position */

/********************  Bit definition for LTDC_CDSR register  *****************/

#define LTDC_CDSR_VDES_Pos           (0U)                                      
#define LTDC_CDSR_VDES_Msk           (0x1U << LTDC_CDSR_VDES_Pos)              /*!< 0x00000001 */
#define LTDC_CDSR_VDES               LTDC_CDSR_VDES_Msk                        /*!< Vertical Data Enable Status       */
#define LTDC_CDSR_HDES_Pos           (1U)                                      
#define LTDC_CDSR_HDES_Msk           (0x1U << LTDC_CDSR_HDES_Pos)              /*!< 0x00000002 */
#define LTDC_CDSR_HDES               LTDC_CDSR_HDES_Msk                        /*!< Horizontal Data Enable Status     */
#define LTDC_CDSR_VSYNCS_Pos         (2U)                                      
#define LTDC_CDSR_VSYNCS_Msk         (0x1U << LTDC_CDSR_VSYNCS_Pos)            /*!< 0x00000004 */
#define LTDC_CDSR_VSYNCS             LTDC_CDSR_VSYNCS_Msk                      /*!< Vertical Synchronization Status   */
#define LTDC_CDSR_HSYNCS_Pos         (3U)                                      
#define LTDC_CDSR_HSYNCS_Msk         (0x1U << LTDC_CDSR_HSYNCS_Pos)            /*!< 0x00000008 */
#define LTDC_CDSR_HSYNCS             LTDC_CDSR_HSYNCS_Msk                      /*!< Horizontal Synchronization Status */

/********************  Bit definition for LTDC_LxCR register  *****************/

#define LTDC_LxCR_LEN_Pos            (0U)                                      
#define LTDC_LxCR_LEN_Msk            (0x1U << LTDC_LxCR_LEN_Pos)               /*!< 0x00000001 */
#define LTDC_LxCR_LEN                LTDC_LxCR_LEN_Msk                         /*!< Layer Enable              */
#define LTDC_LxCR_COLKEN_Pos         (1U)                                      
#define LTDC_LxCR_COLKEN_Msk         (0x1U << LTDC_LxCR_COLKEN_Pos)            /*!< 0x00000002 */
#define LTDC_LxCR_COLKEN             LTDC_LxCR_COLKEN_Msk                      /*!< Color Keying Enable       */
#define LTDC_LxCR_CLUTEN_Pos         (4U)                                      
#define LTDC_LxCR_CLUTEN_Msk         (0x1U << LTDC_LxCR_CLUTEN_Pos)            /*!< 0x00000010 */
#define LTDC_LxCR_CLUTEN             LTDC_LxCR_CLUTEN_Msk                      /*!< Color Lockup Table Enable */

/********************  Bit definition for LTDC_LxWHPCR register  **************/

#define LTDC_LxWHPCR_WHSTPOS_Pos     (0U)                                      
#define LTDC_LxWHPCR_WHSTPOS_Msk     (0xFFFU << LTDC_LxWHPCR_WHSTPOS_Pos)      /*!< 0x00000FFF */
#define LTDC_LxWHPCR_WHSTPOS         LTDC_LxWHPCR_WHSTPOS_Msk                  /*!< Window Horizontal Start Position */
#define LTDC_LxWHPCR_WHSPPOS_Pos     (16U)                                     
#define LTDC_LxWHPCR_WHSPPOS_Msk     (0xFFFFU << LTDC_LxWHPCR_WHSPPOS_Pos)     /*!< 0xFFFF0000 */
#define LTDC_LxWHPCR_WHSPPOS         LTDC_LxWHPCR_WHSPPOS_Msk                  /*!< Window Horizontal Stop Position  */

/********************  Bit definition for LTDC_LxWVPCR register  **************/

#define LTDC_LxWVPCR_WVSTPOS_Pos     (0U)                                      
#define LTDC_LxWVPCR_WVSTPOS_Msk     (0xFFFU << LTDC_LxWVPCR_WVSTPOS_Pos)      /*!< 0x00000FFF */
#define LTDC_LxWVPCR_WVSTPOS         LTDC_LxWVPCR_WVSTPOS_Msk                  /*!< Window Vertical Start Position */
#define LTDC_LxWVPCR_WVSPPOS_Pos     (16U)                                     
#define LTDC_LxWVPCR_WVSPPOS_Msk     (0xFFFFU << LTDC_LxWVPCR_WVSPPOS_Pos)     /*!< 0xFFFF0000 */
#define LTDC_LxWVPCR_WVSPPOS         LTDC_LxWVPCR_WVSPPOS_Msk                  /*!< Window Vertical Stop Position  */

/********************  Bit definition for LTDC_LxCKCR register  ***************/

#define LTDC_LxCKCR_CKBLUE_Pos       (0U)                                      
#define LTDC_LxCKCR_CKBLUE_Msk       (0xFFU << LTDC_LxCKCR_CKBLUE_Pos)         /*!< 0x000000FF */
#define LTDC_LxCKCR_CKBLUE           LTDC_LxCKCR_CKBLUE_Msk                    /*!< Color Key Blue value  */
#define LTDC_LxCKCR_CKGREEN_Pos      (8U)                                      
#define LTDC_LxCKCR_CKGREEN_Msk      (0xFFU << LTDC_LxCKCR_CKGREEN_Pos)        /*!< 0x0000FF00 */
#define LTDC_LxCKCR_CKGREEN          LTDC_LxCKCR_CKGREEN_Msk                   /*!< Color Key Green value */
#define LTDC_LxCKCR_CKRED_Pos        (16U)                                     
#define LTDC_LxCKCR_CKRED_Msk        (0xFFU << LTDC_LxCKCR_CKRED_Pos)          /*!< 0x00FF0000 */
#define LTDC_LxCKCR_CKRED            LTDC_LxCKCR_CKRED_Msk                     /*!< Color Key Red value   */

/********************  Bit definition for LTDC_LxPFCR register  ***************/

#define LTDC_LxPFCR_PF_Pos           (0U)                                      
#define LTDC_LxPFCR_PF_Msk           (0x7U << LTDC_LxPFCR_PF_Pos)              /*!< 0x00000007 */
#define LTDC_LxPFCR_PF               LTDC_LxPFCR_PF_Msk                        /*!< Pixel Format */

/********************  Bit definition for LTDC_LxCACR register  ***************/

#define LTDC_LxCACR_CONSTA_Pos       (0U)                                      
#define LTDC_LxCACR_CONSTA_Msk       (0xFFU << LTDC_LxCACR_CONSTA_Pos)         /*!< 0x000000FF */
#define LTDC_LxCACR_CONSTA           LTDC_LxCACR_CONSTA_Msk                    /*!< Constant Alpha */

/********************  Bit definition for LTDC_LxDCCR register  ***************/

#define LTDC_LxDCCR_DCBLUE_Pos       (0U)                                      
#define LTDC_LxDCCR_DCBLUE_Msk       (0xFFU << LTDC_LxDCCR_DCBLUE_Pos)         /*!< 0x000000FF */
#define LTDC_LxDCCR_DCBLUE           LTDC_LxDCCR_DCBLUE_Msk                    /*!< Default Color Blue  */
#define LTDC_LxDCCR_DCGREEN_Pos      (8U)                                      
#define LTDC_LxDCCR_DCGREEN_Msk      (0xFFU << LTDC_LxDCCR_DCGREEN_Pos)        /*!< 0x0000FF00 */
#define LTDC_LxDCCR_DCGREEN          LTDC_LxDCCR_DCGREEN_Msk                   /*!< Default Color Green */
#define LTDC_LxDCCR_DCRED_Pos        (16U)                                     
#define LTDC_LxDCCR_DCRED_Msk        (0xFFU << LTDC_LxDCCR_DCRED_Pos)          /*!< 0x00FF0000 */
#define LTDC_LxDCCR_DCRED            LTDC_LxDCCR_DCRED_Msk                     /*!< Default Color Red   */
#define LTDC_LxDCCR_DCALPHA_Pos      (24U)                                     
#define LTDC_LxDCCR_DCALPHA_Msk      (0xFFU << LTDC_LxDCCR_DCALPHA_Pos)        /*!< 0xFF000000 */
#define LTDC_LxDCCR_DCALPHA          LTDC_LxDCCR_DCALPHA_Msk                   /*!< Default Color Alpha */

/********************  Bit definition for LTDC_LxBFCR register  ***************/

#define LTDC_LxBFCR_BF2_Pos          (0U)                                      
#define LTDC_LxBFCR_BF2_Msk          (0x7U << LTDC_LxBFCR_BF2_Pos)             /*!< 0x00000007 */
#define LTDC_LxBFCR_BF2              LTDC_LxBFCR_BF2_Msk                       /*!< Blending Factor 2 */
#define LTDC_LxBFCR_BF1_Pos          (8U)                                      
#define LTDC_LxBFCR_BF1_Msk          (0x7U << LTDC_LxBFCR_BF1_Pos)             /*!< 0x00000700 */
#define LTDC_LxBFCR_BF1              LTDC_LxBFCR_BF1_Msk                       /*!< Blending Factor 1 */

/********************  Bit definition for LTDC_LxCFBAR register  **************/

#define LTDC_LxCFBAR_CFBADD_Pos      (0U)                                      
#define LTDC_LxCFBAR_CFBADD_Msk      (0xFFFFFFFFU << LTDC_LxCFBAR_CFBADD_Pos)  /*!< 0xFFFFFFFF */
#define LTDC_LxCFBAR_CFBADD          LTDC_LxCFBAR_CFBADD_Msk                   /*!< Color Frame Buffer Start Address */

/********************  Bit definition for LTDC_LxCFBLR register  **************/

#define LTDC_LxCFBLR_CFBLL_Pos       (0U)                                      
#define LTDC_LxCFBLR_CFBLL_Msk       (0x1FFFU << LTDC_LxCFBLR_CFBLL_Pos)       /*!< 0x00001FFF */
#define LTDC_LxCFBLR_CFBLL           LTDC_LxCFBLR_CFBLL_Msk                    /*!< Color Frame Buffer Line Length    */
#define LTDC_LxCFBLR_CFBP_Pos        (16U)                                     
#define LTDC_LxCFBLR_CFBP_Msk        (0x1FFFU << LTDC_LxCFBLR_CFBP_Pos)        /*!< 0x1FFF0000 */
#define LTDC_LxCFBLR_CFBP            LTDC_LxCFBLR_CFBP_Msk                     /*!< Color Frame Buffer Pitch in bytes */

/********************  Bit definition for LTDC_LxCFBLNR register  *************/

#define LTDC_LxCFBLNR_CFBLNBR_Pos    (0U)                                      
#define LTDC_LxCFBLNR_CFBLNBR_Msk    (0x7FFU << LTDC_LxCFBLNR_CFBLNBR_Pos)     /*!< 0x000007FF */
#define LTDC_LxCFBLNR_CFBLNBR        LTDC_LxCFBLNR_CFBLNBR_Msk                 /*!< Frame Buffer Line Number */

/********************  Bit definition for LTDC_LxCLUTWR register  *************/

#define LTDC_LxCLUTWR_BLUE_Pos       (0U)                                      
#define LTDC_LxCLUTWR_BLUE_Msk       (0xFFU << LTDC_LxCLUTWR_BLUE_Pos)         /*!< 0x000000FF */
#define LTDC_LxCLUTWR_BLUE           LTDC_LxCLUTWR_BLUE_Msk                    /*!< Blue value   */
#define LTDC_LxCLUTWR_GREEN_Pos      (8U)                                      
#define LTDC_LxCLUTWR_GREEN_Msk      (0xFFU << LTDC_LxCLUTWR_GREEN_Pos)        /*!< 0x0000FF00 */
#define LTDC_LxCLUTWR_GREEN          LTDC_LxCLUTWR_GREEN_Msk                   /*!< Green value  */
#define LTDC_LxCLUTWR_RED_Pos        (16U)                                     
#define LTDC_LxCLUTWR_RED_Msk        (0xFFU << LTDC_LxCLUTWR_RED_Pos)          /*!< 0x00FF0000 */
#define LTDC_LxCLUTWR_RED            LTDC_LxCLUTWR_RED_Msk                     /*!< Red value    */
#define LTDC_LxCLUTWR_CLUTADD_Pos    (24U)                                     
#define LTDC_LxCLUTWR_CLUTADD_Msk    (0xFFU << LTDC_LxCLUTWR_CLUTADD_Pos)      /*!< 0xFF000000 */
#define LTDC_LxCLUTWR_CLUTADD        LTDC_LxCLUTWR_CLUTADD_Msk                 /*!< CLUT address */


/*--------------------------------------------------------
  LTDC Parameters
*------------------------------------------------------*/

/* Horizontal and Vertical timing synchronization */
#define HSYNC  1
#define VSYNC 10
#define HBP 43 
#define VBP 12
#define HACTIVE 480
#define VACTIVE 272
#define HFP 8 
#define VFP 4

#define HSYNC_REG (HSYNC - 1)
#define VSYNC_REG (VSYNC - 1)

#define HBP_REG (HSYNC + HBP - 1) 
#define VBP_REG (VSYNC + VBP - 1)

/* Active Zone */
#define HACTIVE_REG (HSYNC + HBP + HACTIVE - 1)
#define VACTIVE_REG (VSYNC + VBP + VACTIVE - 1)

/* Total Area */
#define HTOTAL_REG (HSYNC + HBP + HACTIVE + HFP - 1)
#define VTOTAL_REG (VSYNC + VBP + VACTIVE + VFP - 1)

#define BLACK_BACKGROUND 0x000000FF

/* Layer Config */
#define H_START_POSITION  (HBP_REG + 1)
#define H_STOP_POSITION    HACTIVE_REG

#define V_START_POSITION  (VBP_REG + 1)
#define V_STOP_POSITION    VACTIVE_REG

/* Pixel Format */
#define RGB565 0x02    // RGB565 to fit burst size

#define HPIXELS 480
#define BYTES_PER_PIXEL 2
#define BUFFER_LINE_LENGTH  (HPIXELS*BYTES_PER_PIXEL)
#define BUFFER_LINE_LENGTH_REG  (BUFFER_LINE_LENGTH + 3)

#define BUFFER_PITCH BUFFER_LINE_LENGTH

#define BUFFER_LINES 272

/*--------------------------------------------------------
  Objects
 *------------------------------------------------------*/



/*--------------------------------------------------------
  ltdcConfig

  Members:
  hStartPosition
  hStopPosition
  vStartPosition
  vStopPosition
  pixelFormat
  * ptrImageBuffer
  bufferLineLength
  bufferPitch
  bufferNumOfLines
 *------------------------------------------------------*/
struct ltdcConfig
{
   uint32_t hStartPosition;
   uint32_t hStopPosition;
   uint32_t vStartPosition;
   uint32_t vStopPosition;
   uint32_t pixelFormat;
   const uint32_t * ptrImageBuffer;
   uint32_t bufferLineLength;
   uint32_t bufferPitch;
   uint32_t bufferNumOfLines;
};

/*--------------------------------------------------------
  Prototypes
 *------------------------------------------------------*/

/*---------------------------------------------------------------- 
  Synchronous timings configuration
  ----------------------------------------------------------------
  Description:
  Configure the vertical and horizontal synchronous parameters

  Parameters:
  hSync       IN -
  vSync       IN -
  hBackPorch  IN -
  vBackPorch  IN - 
-----------------------------------------------------------------*/    
int32_t ltdcSyncConfig(uint32_t hSync, uint32_t vSync, uint32_t hBackPorch, uint32_t vBackPorch);


/*---------------------------------------------------------------- 
  Area configuration
  ----------------------------------------------------------------
  Description:
  Define the active zone area for the LTDC window 1

  Paramterers:
  activeZoneWidth  IN - 
  activeZoneHeight IN - 
  totalAreaWidth   IN - 
  totalAreaHeight  IN -
  backgroundColor  IN -
-----------------------------------------------------------------*/ 
int32_t ltdcAreaConfig(uint32_t activeZoneWidth, uint32_t activeZoneHeight, uint32_t totalAreaWidth, uint32_t totalAreaHeight, uint32_t backgroundColor);



/*---------------------------------------------------------------- 
  Layer Configuration
  Description:
  Define the window 1 position parameters, buffer configuration and
  select the image to be displayed  

  Parameters:
  ptrLtdcConfig IN - 
-----------------------------------------------------------------*/
int32_t ltdcLayerConfig(struct ltdcConfig * ptrLtdcConfig);

/*---------------------------------------------------------------- 
   Start the LTDC Controller
-----------------------------------------------------------------*/
void ltdcStart(void);

/*---------------------------------------------------------------- 
   Reload the shadow registers to active register
-----------------------------------------------------------------*/
void ltdcReload(void);

