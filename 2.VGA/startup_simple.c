/*----------------------------------------------------------------------------------
   Startup Code
------------------------------------------------------------------------------------
   Author: Fernando Ojeda L.                                                Dic-2019
------------------------------------------------------------------------------------
   Description:
------------------------------------------------------------------------------------
   By default, the startup file only defines the handler for reset, that will be 
   executed after power up or a warm reset for system initialization. And all of the
   other interrupts handlers are exported as “weak”, and the default handlers of the
   interrupts are relocated to “Default_Handler” which implemented as an endless loop
   
   If peripheral interrupts are enabled, but the ISRs are not implemented correctly.
   When an IRQ is received, the weakly defined handlers “Default_Handler” in the
   startup file will get used instead, which is an endless loop

   Reference:

   Cortex™ -M4 Devices Generic User Guide
   Understand the GNU assembler startup file of cortex M4: wwww.silabs.com
   
------------------------------------------------------------------------------------
   The interrupt vector table is a simple array of pointers to void. All interrupt
   handlers are placed in the array in their corresponding order as documented in 
   the Cortex-M7 Documentation.
   
   The Vector Table Offset Register (VTOR) which defines the interrupt vector table
   location is programmed by default to the Flash 0x00 location.
   
   
   
----------------------------------------------------------------------------------*/

/*--------------------------------------------------------
  Pre Processor Directives
 *------------------------------------------------------*/
#define STACK_TOP 0x2004BFFF                                   // Stack pointer address




/*--------------------------------------------------------
  Linker generated Symbols
 *------------------------------------------------------*/
// Define symbols used in the linker script
extern unsigned int _DATA_ROM_START;
extern unsigned int _DATA_RAM_START;
extern unsigned int _DATA_RAM_END;
extern unsigned int _BSS_START;
extern unsigned int _BSS_END;


/*--------------------------------------------------------
  External references
 *------------------------------------------------------*/
 void main();
 
/*--------------------------------------------------------
  Internal references
 *------------------------------------------------------*/
void Default_Handler(void);
void Reset_Handler();


/*--------------------------------------------------------
  Exception / Interrupt Handler
 *-------------------------------------------------------*/
/* Cortex-M7 Processor Exceptions */
void NMI_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler   (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler   (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler    (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler  (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));

/* ARMCM7 Specific Interrupts */
void WDT_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM0_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void MCIA_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void MCIB_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void AACI_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void CLCD_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void ENET_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void USBDC_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void USBHC_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void CHLCD_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void FLEXRAY_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void LIN_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void CPU_CLCD_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void UART3_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));



/*--------------------------------------------------------
   Exception / Interrupt Vector table
--------------------------------------------------------*/
void * myvectors[] __attribute__ ((section(".isr_vector"))) =      // Define the vector table and put it in the vectors section
{
   /* Cortex-M7 Exceptions Handler */
   (unsigned int *)    STACK_TOP,            // Initial Stack pointer value
   Reset_Handler,                            /*      Reset Handler             */ 
   NMI_Handler,                              /*      NMI Handler               */ 
   HardFault_Handler,                        /*      Hard Fault Handler        */ 
   MemManage_Handler,                        /*      MPU Fault Handler         */ 
   BusFault_Handler,                         /*      Bus Fault Handler         */ 
   UsageFault_Handler,                       /*      Usage Fault Handler       */ 
   0,                                        /*      Reserved                  */ 
   0,                                        /*      Reserved                  */ 
   0,                                        /*      Reserved                  */ 
   0,                                        /*      Reserved                  */ 
   SVC_Handler,                              /*      SVCall Handler            */ 
   DebugMon_Handler,                         /*      Debug Monitor Handler     */ 
   0,                                        /*      Reserved                  */ 
   PendSV_Handler,                           /*      PendSV Handler            */ 
   SysTick_Handler,                          /*      SysTick Handler           */ 
    
   /* External interrupts */ 
   WDT_IRQHandler,                           /*  0:  Watchdog Timer            */ 
   RTC_IRQHandler,                           /*  1:  Real Time Clock           */ 
   TIM0_IRQHandler,                          /*  2:  Timer0 / Timer1           */ 
   TIM2_IRQHandler,                          /*  3:  Timer2 / Timer3           */ 
   MCIA_IRQHandler,                          /*  4:  MCIa                      */ 
   MCIB_IRQHandler,                          /*  5:  MCIb                      */ 
   UART0_IRQHandler,                         /*  6:  UART0 - DUT FPGA          */ 
   UART1_IRQHandler,                         /*  7:  UART1 - DUT FPGA          */ 
   UART2_IRQHandler,                         /*  8:  UART2 - DUT FPGA          */ 
   UART4_IRQHandler,                         /*  9:  UART4 - not connected     */ 
   AACI_IRQHandler,                          /* 10: AACI / AC97                */ 
   CLCD_IRQHandler,                          /* 11: CLCD Combined Interrupt    */ 
   ENET_IRQHandler,                          /* 12: Ethernet                   */ 
   USBDC_IRQHandler,                         /* 13: USB Device                 */ 
   USBHC_IRQHandler,                         /* 14: USB Host Controller        */ 
   CHLCD_IRQHandler,                         /* 15: Character LCD              */ 
   FLEXRAY_IRQHandler,                       /* 16: Flexray                    */ 
   CAN_IRQHandler,                           /* 17: CAN                        */ 
   LIN_IRQHandler,                           /* 18: LIN                        */ 
   I2C_IRQHandler,                           /* 19: I2C ADC/DAC                */ 
   0,                                        /* 20: Reserved                   */ 
   0,                                        /* 21: Reserved                   */ 
   0,                                        /* 22: Reserved                   */ 
   0,                                        /* 23: Reserved                   */ 
   0,                                        /* 24: Reserved                   */ 
   0,                                        /* 25: Reserved                   */ 
   0,                                        /* 26: Reserved                   */ 
   0,                                        /* 27: Reserved                   */ 
   CPU_CLCD_IRQHandler,                      /* 28: Reserved - CPU FPGA CLCD   */ 
   0,                                        /* 29: Reserved - CPU FPGA        */ 
   UART3_IRQHandler,                         /* 30: UART3    - CPU FPGA        */ 
   SPI_IRQHandler                            /* 31: SPI Touchscreen - CPU FPGA */     
};


/*--------------------------------------------------------
  Reset Handler called on controller reset
 *------------------------------------------------------*/
void Reset_Handler()
{
    /* Copy data belonging to the `.data` section from its
     * load time position on flash (ROM) to its run time position
     * in SRAM. */
    volatile unsigned int * data_rom_start_p = &_DATA_ROM_START;
    volatile unsigned int * data_ram_start_p = &_DATA_RAM_START;
    volatile unsigned int * data_ram_end_p   = &_DATA_RAM_END;

    while(data_ram_start_p != data_ram_end_p)
    {
        *data_ram_start_p = *data_rom_start_p;
        data_ram_start_p++;
        data_rom_start_p++;
    }

    // Initialize data in the `.bss` section to zeros.
    volatile unsigned int * bss_start_p = &_BSS_START; 
    volatile unsigned int * bss_end_p = &_BSS_END;

    while(bss_start_p != bss_end_p)
    {
        *bss_start_p = 0;
        bss_start_p++;
    }

    // Call the `main()` function defined in `test_program.c`.
    main();
}


/*--------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *------------------------------------------------------*/
void Default_Handler(void) {

	while(1);
}
