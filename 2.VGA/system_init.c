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
#include "system_init.h"
#include "errors.h"

/*-------------------------------------------------------
  Macro definition
*------------------------------------------------------*/
#define LTDC_Pin 0xE
#define AF_LTDC 0xE
#define AF_JTAG 0x9

/*-------------------------------------------------------
  Objects
*------------------------------------------------------*/
struct ltdcConfig * ptrLtdcConfig;


/*-------------------------------------------------------
  GPIO pin configuration
*------------------------------------------------------*/
void GPIO_init(void)
{   
   // Enable all ports clocks for AHB1
   RCC -> AHB1ENR = 0x00107FF;
   
   // Each port has 16 pins, each 4-bit AFRn corresponds to a pin
   // Set GPIO ports mode to Alternate function
   GPIOA -> MODER = 0xAAAAAAAA;
   GPIOB -> MODER = 0xAAAAAAAA;
   GPIOC -> MODER = 0xAAAAAAAA;
   GPIOD -> MODER = 0xAAAAAAAA;
   GPIOE -> MODER = 0xAAAAAAAA;
   GPIOF -> MODER = 0xAAAAAAAA;
   GPIOG -> MODER = 0xAAAAAAAA;
   GPIOH -> MODER = 0xAAAAAAAA;
   GPIOI -> MODER = 0xA9AAAAAA;                 // set PI12 as output
   GPIOJ -> MODER = 0xAAAAAAAA;
   GPIOK -> MODER = 0xAAAAAA6A;                 // set PK3 as output
      
   
   // LTCD alternate function pins configuration 
   // AF14 = 0xE corresponds to LCD pins
   GPIOA -> AFR[0] = 0x0EEEEEE0;                // low register   [AFR7-AFR0]
   GPIOA -> AFR[1] = 0x000EE00E;                // high register  [AFR15-AFR8], Here are located the debug pins
   
   GPIOB -> AFR[0] = 0x00000099;                // These are the debug pins
   GPIOB -> AFR[1] = 0x0000EEEE; 
   
   GPIOC -> AFR[0] = 0xEE00000E;                
   GPIOC -> AFR[1] = 0x00000E00; 
   
   GPIOD -> AFR[0] = 0x0E00E000;                
   GPIOD -> AFR[1] = 0x00000E00; 
   
   GPIOE -> AFR[0] = 0x0EEE0000;                
   GPIOE -> AFR[1] = 0xEEEEE000; 
   
   GPIOF -> AFR[0] = 0x00000000;                
   GPIOF -> AFR[1] = 0x00000E00; 
   
   GPIOG -> AFR[0] = 0xEE000000;                
   GPIOG -> AFR[1] = 0x0EEEEE00; 
   
   GPIOH -> AFR[0] = 0x0000EE00;              
   GPIOH -> AFR[1] = 0xEEEEEEEE; 
   
   GPIOI -> AFR[0] = 0xEEEE0EEE;                
   GPIOI -> AFR[1] = 0xEEE00EE0; 
   
   GPIOJ -> AFR[0] = 0xEEEEEEEE;               
   GPIOJ -> AFR[1] = 0xEEEEEEEE; 
   
   GPIOK -> AFR[0] = 0xEEEEEEEE;                
   GPIOK -> AFR[1] = 0x00000000; 
   
   
   // Configure ports as high-speed
   GPIOA -> OSPEEDR = 0xAAAAAAAA;
   GPIOB -> OSPEEDR = 0xAAAAAAAA;
   GPIOC -> OSPEEDR = 0xAAAAAAAA;
   GPIOD -> OSPEEDR = 0xAAAAAAAA;
   GPIOE -> OSPEEDR = 0xAAAAAAAA;
   GPIOF -> OSPEEDR = 0xAAAAAAAA;
   GPIOG -> OSPEEDR = 0xAAAAAAAA;
   GPIOH -> OSPEEDR = 0xAAAAAAAA;
   GPIOI -> OSPEEDR = 0xA8AAAAAA;   // PI12 as low speed
   GPIOJ -> OSPEEDR = 0xAAAAAAAA;
   GPIOK -> OSPEEDR = 0xAAAAAA2A;   // PK3 as low speed


   // Enable  backlight
   GPIOI -> ODR |= 0x00001000;      // PI12 (DISP) standby mode
   GPIOK -> ODR |= 0x00000008;      // PK3  (Turn on-off backlight)
}


/*----------------------------------------------------------------------------------------------------
  system_clock_init
*-----------------------------------------------------------------------------------------------------*/
void system_clock_init(void)
{
   /* 1. System clock configuration (SYSCLK) - (CPU clock HCLK) */
   
   // APB2 Prescaler to set 216MHz/2 = 108MHz maximum frequency
   RCC->CFGR |= RCC_CFGR_PPRE2_2; 
   
   // Enable the LTDC peripheral clock 
   RCC -> APB2ENR |= RCC_APB2ENR_LTDCEN;
   
   // Enable HSE   
   RCC->CR |= RCC_CR_HSEON;                
   while (!(RCC->CR & RCC_CR_HSERDY));                                                                  // Wait for the HSE ready flag
   
   // Flash Latency
   FLASH->ACR |= FLASH_ACR_LATENCY_5WS;                                                                 // Five wait states
   
   // Configure PLL 
   RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_0 | RCC_PLLCFGR_PLLM_3 | RCC_PLLCFGR_PLLM_4;                        // PLLM = 25
   RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLN_8;   // PLLN = 432
   RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN_6);
  
   // Set PLL = HSE
   RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;                                                                  

   // Turn on the PLL
   RCC->CR |= RCC_CR_PLLON; 
   while((RCC->CR & RCC_CR_PLLRDY) == 0){}   // wait for the PLL ready flag

   /* Modify CPU clock source (HCLK), SYSCLK = PLL
      which by default is HSI
      We assign the output of our PLL as the source of the system frequency*/
   RCC->CFGR |= RCC_CFGR_SW_PLL;                                                                       
   
   // wait for the ready flag:
   while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1) {}    
   
   
   /* 2. Configure the required pixel clock as indicated in the panel datasheet to 9.5 MHz */
   
   // Multiplication factor for VCO_out_freq
   RCC -> PLLSAICFGR &=  ~(RCC_PLLSAICFGR_PLLSAIN);                
   RCC -> PLLSAICFGR |=   (192 << RCC_PLLSAICFGR_PLLSAIN_Pos);   // (*192) = 192MHz, 
   
   // Division Factor for f(PLLSAIR)
   RCC -> PLLSAICFGR &= ~(RCC_PLLSAICFGR_PLLSAIR);      
   RCC -> PLLSAICFGR |=  (5 << RCC_PLLSAICFGR_PLLSAIR_Pos);      // (/5) = 38.4MHz, 
        
   // Division Factor for LCD_clk
   RCC -> DCKCFGR1   &= (RCC_DCKCFGR1_PLLSAIDIVR);      
   RCC -> DCKCFGR1   |= (1 << RCC_DCKCFGR1_PLLSAIDIVR_Pos);      // (/4) = 9.6MHz

   // Enable PLLSAI
   RCC -> CR |= RCC_CR_PLLSAION;
   
   // Wait for the PLLSAI ready flag
   while ((RCC->CR & RCC_CR_PLLSAIRDY) == 0) {} 
   
}


/*----------------------------------------------------------------------------------------------
  LTDC initialization 
-----------------------------------------------------------------------------------------------*/
void LTDC_init(void)
{
   ltdcSyncConfig(HSYNC_REG, VSYNC_REG, HBP_REG, VBP_REG);

   ltdcAreaConfig(HACTIVE_REG, VACTIVE_REG, HTOTAL_REG, VTOTAL_REG, BLACK_BACKGROUND);

   ptrLtdcConfig -> hStartPosition   = H_START_POSITION;
   ptrLtdcConfig -> hStopPosition    = H_STOP_POSITION;
   ptrLtdcConfig -> vStartPosition   = V_START_POSITION;
   ptrLtdcConfig -> vStopPosition    = V_STOP_POSITION;
   ptrLtdcConfig -> pixelFormat      = RGB565;
   ptrLtdcConfig -> ptrImageBuffer   = pirilika;
   ptrLtdcConfig -> bufferLineLength = BUFFER_LINE_LENGTH_REG;
   ptrLtdcConfig -> bufferPitch      = BUFFER_PITCH;
   ptrLtdcConfig -> bufferNumOfLines = BUFFER_LINES;
   ltdcLayerConfig(ptrLtdcConfig);
}


/*----------------------------------------------------------------------------------------------
   system_init 
----------------------------------------------------------------------------------------------*/
int32_t system_init(void)
{
  static uint32_t initialized = FALSE;

   if (initialized != TRUE)
   {
      /* 1.- PIN configuration */
      GPIO_init();

      /* 2. System Clock Configuration */
      system_clock_init();
      
      /* 3. LTDC Initialization */
      LTDC_init();

      initialized = TRUE;
   }
   else
      return ERROR;
}
