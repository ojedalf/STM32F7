/*-----------------------------------------------------------------------------
   VGA Controller
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
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>        
#include "system_init.h"


/*--------------------------------------------------------
  Macro Definition
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
   uint32_t x_offset = 10;
   uint32_t y_offset = 10;
   uint32_t vx = 1;
   uint32_t vy = 1;

   // Enable Interrupts   
   //__asm volatile("cpsie i"); /* enable interrupts */   


   /* Initialize the system */
   system_init();

   /* Application */
   for(;;)
   {  
      LTDC -> SRCR |= LTDC_SRCR_VBR;                      // reload shadow registers on vertical blanking period
      while ((LTDC->CDSR & LTDC_CDSR_VSYNCS) == 0);       // wait for next frame
      
      while ((LTDC->CDSR & LTDC_CDSR_VSYNCS) == 1);
   }
   
}
