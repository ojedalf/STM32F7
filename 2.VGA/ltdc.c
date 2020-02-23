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




/*-------------------------------------------------------
  GPIO pin configuration
*------------------------------------------------------*/
void LTDC_GPIO_config(void)
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



/*-------------------------------------------------------
  System clock configuration 
*------------------------------------------------------*/
void system_clock_config(void)
{
   // APB2 Prescaler to set 216MHz/2 = 108MHz maximum frequency
      RCC->CFGR |= RCC_CFGR_PPRE2_2; 
   
   // 1. Enable the LTDC clock in the RCC register.
      RCC -> APB2ENR |= RCC_APB2ENR_LTDCEN;
      
   // System clock   
      RCC->CR |= RCC_CR_HSEON;                // Enable HSE
      while (!(RCC->CR & RCC_CR_HSERDY)); 

      FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

   // Configure PLL 
      RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_0 | RCC_PLLCFGR_PLLM_3 | RCC_PLLCFGR_PLLM_4;                       // PLLM = 25
      RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLN_8;  // PLLN = 432
      RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN_6);
     
      RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;                                                                 // Set PLL = HSE

   // Now turn on the PLL and wait for the ready flag:
      RCC->CR |= RCC_CR_PLLON; 
      while((RCC->CR & RCC_CR_PLLRDY) == 0){} 

   // We assign the output of our PLL as the source of the system frequency, SYSCLK = PLL
      RCC->CFGR |= RCC_CFGR_SW_PLL; 
   // wait for the ready flag:
      while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1) {}    
        
        
   /*----------------------------------------------------------------------------------------- 
   2. Configure the required pixel clock following the panel datasheet, 5-9-12 MHz
   
      HSE  = 25MHz
      PLLM = 25
   
      VCO_input_freq = HSI/PLLM
      
      VCO_out_freq = VCO_input_freq*PLLSAIN, (PLLSAIN = 192)
            
      f(PLLSAIR) = VCO_out_freq/PLLSAIR,  (PLLSAIR = 5)
       
      LCD_clk = f(PLLSAIR)/PLLSAIDIVR,  (PLLSAIDIVR = 4)
      
                          ______________PLLSAI_________
                         |                             |
      HSE--->| /PLLM |-->|--->|VCO|------->|/PLLSAIR|--|--> f(PLLSAIR) --->|/PLLSAIDIVR|-->LCD_clk          
                         | ↑         ↓                 |
                         | ↑---|xPLLSAIN|              | 
                         |_____________________________|     

                           
   ------------------------------------------------------------------------------------------*/
      
      // these bits should written when the PLLSAI is disabled
      
      // PLLM is set by default to 16, which generates a 16MHz/16= 1MHz input clock to the VCO_input_freq
      // to avoid jitter

      // Multiplication factor for VCO_out_freq
      RCC -> PLLSAICFGR &=  ~(RCC_PLLSAICFGR_PLLSAIN);                
      RCC -> PLLSAICFGR |=   (192 << RCC_PLLSAICFGR_PLLSAIN_Pos);   // *100 = 100MHz, 50 <= PLLSAIN <= 432
      
      // Division Factor for f(PLLSAIR)
      RCC -> PLLSAICFGR &= ~(RCC_PLLSAICFGR_PLLSAIR);      
      RCC -> PLLSAICFGR |=  (5 << RCC_PLLSAICFGR_PLLSAIR_Pos);      // /5 = 20MHz, 2 <= PLLSAIR <= 7
           
      // Division Factor for LCD_clk
      RCC -> DCKCFGR1   &= (RCC_DCKCFGR1_PLLSAIDIVR);      
      RCC -> DCKCFGR1   |= (1 << RCC_DCKCFGR1_PLLSAIDIVR_Pos);       // /2 = 10MHz, 0-/2, 1-/4, 2-/8, 3-/16
   
      // Enable PLLSAI
      RCC -> CR |= RCC_CR_PLLSAION;
      
      // Wait for the PLLSAI ready flag
      while ((RCC->CR & RCC_CR_PLLSAIRDY) == 0) {} 
}



/*-------------------------------------------------------
  LTDC initialization 
*------------------------------------------------------*/
void LTDC_init(void)
{
   /*------------------------------------------------------------------------------------------ 
   3. Configure the synchronous timings: 
   
      VSYNC, HSYNC, vertical and horizontal back porch, active data area and the front porch 
      timings following the panel datasheet

   
      Custom resolution is 480x272 pixels of Active Area
      
      Hsync = 4  | HBP = 43 | HActive = 480 | HFP = 8 
   
      Vsync = 10 | VBP = 12 | VActive = 272 | VFP = 4         

      Hsync, Vsync, not-data-enable and pixel_clk wll be active low by default
      Dither disable by default      
   -------------------------------------------------------------------------------------------*/
   
   
      /*-------------------------------------- 
         Define Hsync and Vsync width
         Hsync_reg = Hsync - 1
         Vsync_reg = Vsync - 1             
      --------------------------------------*/
      LTDC -> SSCR |= (0 << LTDC_SSCR_HSW_Pos);  
      LTDC -> SSCR |= (9 << LTDC_SSCR_VSH_Pos);
     
      /*-------------------------------------- 
         Define BP width
         HBP_reg = Hsync + HBP - 1
         VBP_reg = Vsync + VBP - 1        
      ---------------------------------------*/
      LTDC -> BPCR |= (43 << LTDC_BPCR_AHBP_Pos);
      LTDC -> BPCR |= (21 << LTDC_BPCR_AVBP_Pos);
      
      /*-------------------------------------- 
         Define the Active Zone
         HAct_reg = Hsync + HBP + HActive - 1
         VAct_reg = Vsync + VBP + VActive - 1  
      ---------------------------------------*/
      LTDC -> AWCR |= (523 << LTDC_AWCR_AAW_Pos);     // width
      LTDC -> AWCR |= (293 << LTDC_AWCR_AAH_Pos);     // Height 
      
      /*-----------------------------------------------
         Define the Total Area
         HTotal_reg = Hsync + HBP + HActive + HFP - 1
         Vtotal_reg = Vsync + VBP + VActive + VFP - 1  
      ------------------------------------------------*/
      LTDC -> TWCR |= (531 << LTDC_TWCR_TOTALW_Pos);       // width
      LTDC -> TWCR |= (297 << LTDC_TWCR_TOTALH_Pos);       // Height
   
      // Background color area
      LTDC -> BCCR = 0x000000FF;
      
      // Configure the synchronous signals and clock polarity
      LTDC -> GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);

   
   /*---------------------------------------------------------------- 
   5. Layer Configuration
   -----------------------------------------------------------------*/
   
      // Window Horizontal position   
      LTDC_Layer1 -> WHPCR =  (44 << LTDC_LxWHPCR_WHSTPOS_Pos) | (523 << LTDC_LxWHPCR_WHSPPOS_Pos);       // Start = LTDC_BPCR_AHBP + 1   |  Stop  = LTDC_AWCR_AAW
      
      // Window Vertical position 
      LTDC_Layer1 -> WVPCR =  (22 << LTDC_LxWVPCR_WVSTPOS_Pos) | (293 << LTDC_LxWVPCR_WVSPPOS_Pos);       // Start = LTDC_BPCR_AVBP + 1   | Stop  = LTDC_AWCR_AAH
      
      // pixel input format
      LTDC_Layer1 -> PFCR = 0x02;                                                                         // RGB565 to fit burst size

      // Color frame buffer start address
      LTDC_Layer1 -> CFBAR = (uint32_t)&image_data_480x270;
      
      // Color frame buffer line length | Color frame buffer pitch in bytes                               // Horizontal_pixelsx(Bytes_per_pixel) + 3 
      LTDC_Layer1 -> CFBLR |= (963 << LTDC_LxCFBLR_CFBLL_Pos) | (960 << LTDC_LxCFBLR_CFBP_Pos);           // 480x(2bytes) + 3 | pitch = line length                       
      
      // Number of lines of the color frame buffer
      LTDC_Layer1 -> CFBLNR |= (272 << LTDC_LxCFBLNR_CFBLNBR_Pos);
   
   // 6. Enable Layer 1
      LTDC_Layer1 -> CR |= LTDC_LxCR_LEN;
      //LTDC_Layer2 -> CR |= LTDC_LxCR_LEN;
   
   /*---------------------------------------------------------------- 
   7. Reload the shadow registers to active register
   
      Most of the layers’ configuration registers are shadowed
      so they must be reloaded after being configured and before
      enabling the LTDC
   -----------------------------------------------------------------*/
      LTDC -> SRCR |= LTDC_SRCR_IMR; 
   
   // 8. Enable the LCD-TFT controller
      LTDC -> GCR |= LTDC_GCR_LTDCEN;

      LTDC -> SRCR |= LTDC_SRCR_IMR; 
}

