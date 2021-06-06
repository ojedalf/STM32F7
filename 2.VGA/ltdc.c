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
  Libraries
 *------------------------------------------------------*/      
#include "ltdc.h"


/*---------------------------------------------------------------- 
   Synchronous timings configuration
-----------------------------------------------------------------*/    
int32_t ltdcSyncConfig(uint32_t hSync, uint32_t vSync, uint32_t hBackPorch, uint32_t vBackPorch)
{
   /* Check Input Parameters */
   if ((hSync > HSYNCMAX) | (vSync > VSYNCMAX) | (hBackPorch > HBPMAX) | (vBackPorch > VBPMAX))
   {
      LTDC -> SSCR |= (hSync << LTDC_SSCR_HSW_Pos);  
      LTDC -> SSCR |= (vSync << LTDC_SSCR_VSH_Pos);

      LTDC -> BPCR |= (hBackPorch << LTDC_BPCR_AHBP_Pos);
      LTDC -> BPCR |= (vBackPorch << LTDC_BPCR_AVBP_Pos);
   }
   else
      return LTDC_ERROR_SYNC;
}


/*---------------------------------------------------------------- 
  Area configuration
-----------------------------------------------------------------*/ 
void ltdcAreaConfig(uint32_t activeZoneWidth, uint32_t activeZoneHeight, uint32_t totalAreaWidth, uint32_t totalAreaHeight, uint32_t backgroundColor)
{
   /* the Active Zone */
   LTDC -> AWCR |= (activeZoneWidth << LTDC_AWCR_AAW_Pos);          
   LTDC -> AWCR |= (activeZoneHeight << LTDC_AWCR_AAH_Pos);          

   /* Total Area */
   LTDC -> TWCR |= (totalAreaWidth << LTDC_TWCR_TOTALW_Pos);       
   LTDC -> TWCR |= (totalAreaHeight << LTDC_TWCR_TOTALH_Pos);       

   // Background color area
   LTDC -> BCCR = backgroundColor;

   // Configure the synchronous signals and clock polarity
   LTDC -> GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);
}


/*---------------------------------------------------------------- 
   Layer Configuration
-----------------------------------------------------------------*/
void ltdcLayerConfig(struct ltdcConfig * ptrLtdcConfig)
{

   if (ptrLtdcConfig -> ptrImageBuffer != NULL)
   {
      // Window Horizontal position   
      LTDC_Layer1 -> WHPCR =  ((ptrLtdcConfig -> hStartPosition) << LTDC_LxWHPCR_WHSTPOS_Pos) | ((ptrLtdcConfig -> hStopPosition) << LTDC_LxWHPCR_WHSPPOS_Pos);
      
      // Window Vertical position 
      LTDC_Layer1 -> WVPCR =  ((ptrLtdcConfig -> vStartPosition) << LTDC_LxWVPCR_WVSTPOS_Pos) | ((ptrLtdcConfig -> vStopPosition) << LTDC_LxWVPCR_WVSPPOS_Pos);    
      
      // pixel input format
      LTDC_Layer1 -> PFCR = ptrLtdcConfig -> pixelFormat;                                                                         

      // Color frame buffer start address
      LTDC_Layer1 -> CFBAR = (uint32_t)ptrLtdcConfig -> ptrImageBuffer;   // cast pointer to integer;

      // Color frame buffer line length | Color frame buffer pitch in bytes
      LTDC_Layer1 -> CFBLR |= ((ptrLtdcConfig -> bufferLineLength) << LTDC_LxCFBLR_CFBLL_Pos) | ((ptrLtdcConfig -> bufferPitch) << LTDC_LxCFBLR_CFBP_Pos);                                  
      
      // Number of lines of the color frame buffer
      LTDC_Layer1 -> CFBLNR |= ((ptrLtdcConfig -> bufferNumOfLines) << LTDC_LxCFBLNR_CFBLNBR_Pos);

      // Enable Layer 1
      LTDC_Layer1 -> CR |= LTDC_LxCR_LEN;
   }
   else
      LTDC_ERROR_BUFF;
}


/*---------------------------------------------------------------- 
   Reload the shadow registers to active register
-----------------------------------------------------------------*/
void ltdcReload(void)
{
   LTDC -> SRCR |= LTDC_SRCR_IMR; 
   LTDC -> GCR |= LTDC_GCR_LTDCEN;   // Enable the LCD-TFT controller
   LTDC -> SRCR |= LTDC_SRCR_IMR; 
}
