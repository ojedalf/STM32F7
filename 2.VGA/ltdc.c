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
  Include files
*------------------------------------------------------*/      
#include "ltdc.h"
#include "errors.h"

/* LTDC Registers Base address */
LTDC_TypeDef * ltdcRegisters = (LTDC_TypeDef *)LTDC_BASE;
LTDC_Layer_TypeDef * ltdcLayer1Registers = (LTDC_Layer_TypeDef *)LTDC_Layer1_BASE;


/*---------------------------------------------------------------- 
   Synchronous timings configuration
-----------------------------------------------------------------*/    
int32_t ltdcSyncConfig(uint32_t hSync, uint32_t vSync, uint32_t hBackPorch, uint32_t vBackPorch)
{
   int32_t erc = ERROR;
   LTDC_TypeDef * ltdcDev = ltdcRegisters;

   /* Check Input Parameters */
   if ((hSync <= HSYNCMAX) | (vSync <= VSYNCMAX) | (hBackPorch <= HBPMAX) | (vBackPorch <= VBPMAX))
   {
      ltdcDev -> SSCR |= (hSync << LTDC_SSCR_HSW_Pos);  
      ltdcDev -> SSCR |= (vSync << LTDC_SSCR_VSH_Pos);

      ltdcDev -> BPCR |= (hBackPorch << LTDC_BPCR_AHBP_Pos);
      ltdcDev -> BPCR |= (vBackPorch << LTDC_BPCR_AVBP_Pos);

      erc = OK;
   }
   else
      return erc;
}


/*---------------------------------------------------------------- 
  Area configuration
-----------------------------------------------------------------*/ 
int32_t ltdcAreaConfig(uint32_t activeZoneWidth, uint32_t activeZoneHeight, uint32_t totalAreaWidth, uint32_t totalAreaHeight, uint32_t backgroundColor)
{
   int32_t erc = ERROR;
   LTDC_TypeDef * ltdcDev = ltdcRegisters;

   if( (activeZoneWidth <= ACTIVE_WMAX) | (activeZoneHeight <= ACTIVE_HMAX) | (totalAreaWidth <= ACTIVE_TOTAL_WMAX) | (totalAreaHeight <= ACTIVE_TOTAL_HMAX))
   {
      /* the Active Zone */
      ltdcDev -> AWCR |= (activeZoneWidth << LTDC_AWCR_AAW_Pos);          
      ltdcDev -> AWCR |= (activeZoneHeight << LTDC_AWCR_AAH_Pos);          

      /* Total Area */
      ltdcDev -> TWCR |= (totalAreaWidth << LTDC_TWCR_TOTALW_Pos);       
      ltdcDev -> TWCR |= (totalAreaHeight << LTDC_TWCR_TOTALH_Pos);       

      // Background color area
      ltdcDev -> BCCR = backgroundColor;

      // Configure the synchronous signals and clock polarity
      ltdcDev -> GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);

      erc = OK;
   }
   else
      return erc;
}


/*---------------------------------------------------------------- 
   Layer Configuration
-----------------------------------------------------------------*/
int32_t ltdcLayerConfig(struct ltdcConfig * ptrLtdcConfig)
{
   int32_t erc = ERROR;
   LTDC_Layer_TypeDef * ltdcLayer1Dev = ltdcLayer1Registers;

   if (ptrLtdcConfig -> ptrImageBuffer != NULL)
   {
      // Window Horizontal position   
      ltdcLayer1Dev -> WHPCR =  ((ptrLtdcConfig -> hStartPosition) << LTDC_LxWHPCR_WHSTPOS_Pos) | ((ptrLtdcConfig -> hStopPosition) << LTDC_LxWHPCR_WHSPPOS_Pos);
      
      // Window Vertical position 
      ltdcLayer1Dev -> WVPCR =  ((ptrLtdcConfig -> vStartPosition) << LTDC_LxWVPCR_WVSTPOS_Pos) | ((ptrLtdcConfig -> vStopPosition) << LTDC_LxWVPCR_WVSPPOS_Pos);    
      
      // pixel input format
      ltdcLayer1Dev -> PFCR = ptrLtdcConfig -> pixelFormat;                                                                         

      // Color frame buffer start address
      ltdcLayer1Dev -> CFBAR = (uint32_t)ptrLtdcConfig -> ptrImageBuffer;   // cast pointer to integer;

      // Color frame buffer line length | Color frame buffer pitch in bytes
      ltdcLayer1Dev -> CFBLR |= ((ptrLtdcConfig -> bufferLineLength) << LTDC_LxCFBLR_CFBLL_Pos) | ((ptrLtdcConfig -> bufferPitch) << LTDC_LxCFBLR_CFBP_Pos);                                  
      
      // Number of lines of the color frame buffer
      ltdcLayer1Dev -> CFBLNR |= ((ptrLtdcConfig -> bufferNumOfLines) << LTDC_LxCFBLNR_CFBLNBR_Pos);

      // Enable Layer 1
      ltdcLayer1Dev -> CR |= LTDC_LxCR_LEN;

      erc = OK;
   }
   else
      return erc;
}


/*---------------------------------------------------------------- 
   Start the LTDC Controller
-----------------------------------------------------------------*/
void ltdcStart(void)
{
   LTDC_TypeDef * ltdcDev = ltdcRegisters;

   ltdcDev -> SRCR |= LTDC_SRCR_IMR; 
   ltdcDev -> GCR |= LTDC_GCR_LTDCEN;   // Enable the LCD-TFT controller
   ltdcDev -> SRCR |= LTDC_SRCR_IMR; 
}


/*---------------------------------------------------------------- 
   Reload the shadow registers to active register
-----------------------------------------------------------------*/
void ltdcReload(void)
{
      LTDC_TypeDef * ltdcDev = ltdcRegisters;
      
      ltdcDev -> SRCR |= LTDC_SRCR_VBR;                      // reload shadow registers on vertical blanking period
      while ((ltdcDev->CDSR & LTDC_CDSR_VSYNCS) == 0);       // wait for next frame
      while ((ltdcDev->CDSR & LTDC_CDSR_VSYNCS) == 1);
}
