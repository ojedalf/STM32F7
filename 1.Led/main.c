/*-----------------------------------------------------------------------------
   Blinking Led
-------------------------------------------------------------------------------
   Author: Fernando Ojeda L.                                          Dic-2019
-------------------------------------------------------------------------------
   Description:
-------------------------------------------------------------------------------
   The FFT is implemented in a serial processing form using a radix-2 algorithm.
   The input buffers need first to be filled with 64 samples from the real and 
   imaginary input signals before processing their frequency response.
   
   For 64 points, 6 stages are required, each with N/2 butterflies. Therefore,
   6(N/2) = 192 cycles are required to compute the FFT for 64 input samples.
   
   To guarantee an output signal value of one in magnitud and avoid overflow, 
   the input must be limited to 1/sqrt(2). After every butterfly, the magnitude
   is doubled, thus an scaling of 1/2 is required.
   
   Rounding to the nearest value is applied after every multiplier when passing
   from 32 to 16 bits in order to avoid DC bias.
   
   
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
  |___________________________|   
  
  
-------------------------------------------------------------------------------
   SysTick Counter:
-------------------------------------------------------------------------------    
  It is a countdown timer that runs on the Processor clock

  HSI is used as the system clock at reset by default

  HSI   --->|AHB PRESCALER|---|/8|---2MHz---->|SysTick|
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
  SysTick IRQ Handler
 *------------------------------------------------------*/
void SysTick_Handler (void) {                               
   static uint32_t ticks;
  
   switch (ticks++) {
      case  0: GPIOI_ODR = 0x00000002;; break;
      case  5: GPIOI_ODR = 0x00000000;; break;
      case  9: ticks  = 0; break;
      default:
         if (ticks > 10) {
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
      
   // Set interrupt priority   
      
      
      
   // Enable Interrupts   
   __asm volatile("cpsie i"); /* enable interrupts */   
      
 
   SYST_RVR = 0x00FFFFFF;    // Reload Value, 16777215*(1/2MHz) = 8.38 seg
   SYST_CSR = 0x00000007;    // Enable systick counter
   
   while(1);
   
}







