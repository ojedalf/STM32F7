/*-----------------------------------------------------------------------------
   Blinking Led
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
#include<stdlib.h>
#include<stdio.h>
#include<stdint.h>         // Integer datatype standards
#include "stm32f746xx.h"   // Register Definition file
#include "ltdc.h"          // ltdc init functions


/*--------------------------------------------------------
  SysTick IRQ Handler
 *------------------------------------------------------*/
#define SIZE                 110
#define DISPLAY_WIDTH        480
#define DISPLAY_HEIGHT       272
#define ACTIVE_AREA_START_X   47
#define ACTIVE_AREA_START_Y   22


/*--------------------------------------------------------
  Main
 *------------------------------------------------------*/
void main()
{     
      
   // Enable Interrupts   
   //__asm volatile("cpsie i"); /* enable interrupts */   
   

   
   /*-------------------------------------------------------
     LTCD programming procedure
   *------------------------------------------------------*/
      
      
   LTDC_GPIO_config();
   
   system_clock_config();
   
   LTDC_init();


   uint32_t x_offset = 10;
   uint32_t y_offset = 10;
   uint32_t vx = 1;
   uint32_t vy = 1;

   for(;;)
   {
      
      // Random layer moving
      // if ((x_offset <= 0) || ((x_offset + SIZE) >= DISPLAY_WIDTH))
         // vx = -vx;
      
      // if ((y_offset <= 0) || ((y_offset + SIZE) >= DISPLAY_HEIGHT))
         // vy = -vy;
      
      // x_offset += vx;
      // y_offset += vy;
      
      // LTDC_Layer1->WHPCR = ACTIVE_AREA_START_X + SIZE + (int)x_offset - 1 << LTDC_LxWHPCR_WHSPPOS_Pos | ACTIVE_AREA_START_X + (int)x_offset << LTDC_LxWHPCR_WHSTPOS_Pos;
      // LTDC_Layer1->WVPCR = ACTIVE_AREA_START_Y + SIZE + (int)y_offset - 1 << LTDC_LxWVPCR_WVSPPOS_Pos | ACTIVE_AREA_START_Y + (int)y_offset << LTDC_LxWVPCR_WVSTPOS_Pos;
      
      LTDC -> SRCR |= LTDC_SRCR_VBR;                      // reload shadow registers on vertical blanking period
      while ((LTDC->CDSR & LTDC_CDSR_VSYNCS) == 0);        // wait for next frame
      
      while ((LTDC->CDSR & LTDC_CDSR_VSYNCS) == 1);
  
   }
   
}







