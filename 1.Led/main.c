/*-----------------------------------------------------------------------------
   Blinking Led
-------------------------------------------------------------------------------
   Author: Fernando Ojeda L.                                          Dic-2019
-------------------------------------------------------------------------------
   Description:
-------------------------------------------------------------------------------
   This is a very simple blinking led that employs a systick timer interrupt
   in order to blink the led every 5 seconds.
   
 -------------------------------------------------------------------------------
   Core registers:
-------------------------------------------------------------------------------     
   These registers are not memory mapped and can only be configured through
   ASM instructions detailed in ARM Cortex-M7 documentation.
   
   R0-R12       General purpose registers
   MSP          Main Stack Pointer 
   PSP          Processor Stack Pointer
   LR           Link Register
   PC           Program Counter
   PSR          Program Status Register
   ASPR         Application Program Status Register
   IPSR         Interrupt Program Status Register
   EPSR         Execution Program Status Register
   PRIMASK      Priority Mask Register 
   FAULTMASK    Fault Mask Register 
   BASEPRI      Base Priority Mask Register 
   CONTROL      Control Register
  
-------------------------------------------------------------------------------
   Cortex-M7 processor core peripherals:
-------------------------------------------------------------------------------   
   Unlike CPU registers, the CPU peripherals are memory mapped and can be 
   accessed through Memory Access Macros. ST documentation does not provide
   register details, to find detailed information consult ARM Cortex-M7 
   documentation.
   
   1. Nested Vectored Interrupt Controller (NVIC)
   2. System Control Block (SCB)
   3. System timer
   4. Integrated instruction and data caches (optional)
   5. Memory Protection Unit (optional)
   6. Floating-point unit (optional)
   
   
-------------------------------------------------------------------------------
   Cortex-M7 processor diagram:
-------------------------------------------------------------------------------   
   ___________________________
  |  ______    ____________   |    
  | | Core |  | Peripherals|  |
  |       ________            |
  |      |Memories|           |
  |___________________________|

  
-------------------------------------------------------------------------------
   SysTick Counter:
-------------------------------------------------------------------------------    
  It is a countdown timer that runs on the Processor clock

  HSI is used as the system clock at reset by default.
  HCLK is the processor clock and equals to HSI/AHB_PRESCALER.

                             ------------------HCLK--------------------
                             ↑                                        ↓
  HSI   --->|AHB PRESCALER|-----|/8|-----------External Clock---->|SysTick|
  16MHz      /Not divided
  
-------------------------------------------------------------------------------*/


/*--------------------------------------------------------
  Libraries
 *------------------------------------------------------*/
#include<stdlib.h>
#include<stdio.h>
#include<stdint.h>   // Integer datatype standards


/*--------------------------------------------------------
   Memory Access Macros
--------------------------------------------------------*/
// RCC registers
#define RCC_CR         *((volatile int *) 0x40023800)
#define RCC_AHB1ENR    *((volatile int *) 0x40023830)
// PORTI registers
#define GPIOI_MODER     *((volatile int *) 0x40022000)
#define GPIOI_OTYPER    *((volatile int *) 0x40022004)
#define GPIOI_OSPEEDR   *((volatile int *) 0x40022008)
#define GPIOI_PUPDR     *((volatile int *) 0x4002200C)
#define GPIOI_IDR       *((volatile int *) 0x40022010)
#define GPIOI_ODR       *((volatile int *) 0x40022014)
// Systick registers
#define SYST_CSR        *((volatile int *) 0xE000E010)
#define SYST_RVR        *((volatile int *) 0xE000E014)
#define SYST_CVR        *((volatile int *) 0xE000E018)
#define SYST_CALIB      *((volatile int *) 0xE000E01C)



/*--------------------------------------------------------
  SysTick IRQ Handler (ISR)
  --------------------------------------------------------
  The interrupt is generated when the down-counter reaches
  zero. In this case every second.
  
  Some exceptions are permanently enabled; these include
  the reset and NMI interrupts, but also the Systick Timer.   
 *------------------------------------------------------*/
void SysTick_Handler (void) 
{                               
   static uint32_t ticks;
  
   switch (ticks++) 
   {
      case  0: GPIOI_ODR = 0x00000002;; break;  // Turn on LED
      case  5: GPIOI_ODR = 0x00000000;; break;  // Turn off LED
      case  9: ticks  = 0; break;
      default:
         if (ticks > 10) 
         {
            ticks = 0;
         }
  }
}


/*--------------------------------------------------------
  Main
 *------------------------------------------------------*/
void main()
{
   // Enable PortI AHB1 clock
   RCC_AHB1ENR   = 0x00000100;
  
   // Configure GPIO registers for port I
   GPIOI_MODER   = 0x00000004;  // Pin1 as output
   GPIOI_OTYPER  = 0x00000000;  // 0-Push pull, 2-Open Drain
   GPIOI_OSPEEDR = 0x00000000;  // 0-Low Speed, 4-medium, 8-high, C-very high
   GPIOI_PUPDR   = 0x00000000;  // 0-No, 4-PU, 8-PD, C-reserved
 
   // Configure Systick Timer
   SYST_RVR = 0x00F42400;       // Reload Value, 16000000*(1/HCLK) = 1 seg
   SYST_CSR = 0x00000007;       // | Processor clock | Enable Interrupt | Enable systick counter
   
   while(1);  
}