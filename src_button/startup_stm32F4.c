#include <stdint.h>
#include "hardware_init.h"

/*
    Define where the stack starts - End of SRAM
    This will be the first entry in the vector table
    SRAM_START  0x2000_0000U
    SRAM_SIZE   0x0002_0000U = (128U * 1024U)
    --------------------------------------
    SRAM_END    (SRAM_START + SRAM_SIZE)
*/
#define STACK_START 0x20020000U

/*
    Symbols defined in the linker script
*/
extern uint32_t _text_end;
extern uint32_t _si_data;
extern uint32_t _data_start;
extern uint32_t _data_end;
extern uint32_t _bss_start;
extern uint32_t _bss_end;
extern uint32_t _end;

// Initialize LED # control vairable
volatile uint32_t led_to_toggle = 11;

/*
    Prototypes
*/
extern int main(void);
void __libc_init_array(void);

/*
    STM32F407xxx exception & interrupt handler function prototypes
    The first 16 are system exceptions
    The remaining 82 are device specific interrupts
    These can be found in table 6.1 in the reference manual
*/
void Reset_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

// Device-Specific Interrupts
void WWDG_IRQHandler(void);
void PVD_IRQHandler(void);
void TAMP_STAMP_IRQHandler(void);
void RTC_WKUP_IRQHandler(void);
void FLASH_IRQHandler(void);
void RCC_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void ADC_IRQHandler(void);
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void CAN1_RX1_IRQHandler(void);
void CAN1_SCE_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM1_BRK_TIM9_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void);
void TIM1_TRG_COM_TIM11_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void RTC_Alarm_IRQHandler(void);
void OTG_FS_WKUP_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);
void FSMC_IRQHandler(void);
void SDIO_IRQHandler(void);
void TIM5_IRQHandler(void);
void SPI3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void ETH_IRQHandler(void);
void ETH_WKUP_IRQHandler(void);
void CAN2_TX_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);
void CAN2_RX1_IRQHandler(void);
void CAN2_SCE_IRQHandler(void);
void OTG_FS_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
void USART6_IRQHandler(void);
void I2C3_EV_IRQHandler(void);
void I2C3_ER_IRQHandler(void);
void OTG_HS_EP1_OUT_IRQHandler(void);
void OTG_HS_EP1_IN_IRQHandler(void);
void OTG_HS_WKUP_IRQHandler(void);
void OTG_HS_IRQHandler(void);
void DCMI_IRQHandler(void);
void CRYP_IRQHandler(void);
void HASH_RNG_IRQHandler(void);
void FPU_IRQHandler(void);

/*
    Vector Table Initialization - See table 6.1 in the reference manual
*/
uint32_t *vector_table[] __attribute__((section(".isr_vector"))) = {
    // System Exceptions
    (uint32_t*) STACK_START,          // Stack pointer
    (uint32_t*) Reset_Handler,        // Reset handler
    (uint32_t*) NMI_Handler,          // NMI handler
    (uint32_t*) HardFault_Handler,    // Hard Fault handler
    (uint32_t*) MemManage_Handler,    // MPU Fault handler
    (uint32_t*) BusFault_Handler,     // Bus Fault handler
    (uint32_t*) UsageFault_Handler,   // Usage Fault handler
    0, 0, 0, 0,                       // Reserved
    (uint32_t*) SVC_Handler,          // SVCall handler
    (uint32_t*) DebugMon_Handler,     // Debug Monitor handler
    0,                                // Reserved
    (uint32_t*) PendSV_Handler,       // PendSV handler
    (uint32_t*) SysTick_Handler,      // SysTick handler
    
    // STM32F4xx Specific Interrupts 
    (uint32_t*) WWDG_IRQHandler,      // Window Watchdog
    (uint32_t*) PVD_IRQHandler,       // PVD through EXTI Line detect
    (uint32_t*) TAMP_STAMP_IRQHandler,// Tamper and Time Stamp
    (uint32_t*) RTC_WKUP_IRQHandler,  // RTC Wakeup
    (uint32_t*) FLASH_IRQHandler,     // FLASH
    (uint32_t*) RCC_IRQHandler,       // RCC
    (uint32_t*) EXTI0_IRQHandler,     // EXTI Line0
    (uint32_t*) EXTI1_IRQHandler,     // EXTI Line1
    (uint32_t*) EXTI2_IRQHandler,     // EXTI Line2
    (uint32_t*) EXTI3_IRQHandler,     // EXTI Line3
    (uint32_t*) EXTI4_IRQHandler,     // EXTI Line4
    (uint32_t*) DMA1_Stream0_IRQHandler,  // DMA1 Stream 0
    (uint32_t*) DMA1_Stream1_IRQHandler,  // DMA1 Stream 1
    (uint32_t*) DMA1_Stream2_IRQHandler,  // DMA1 Stream 2
    (uint32_t*) DMA1_Stream3_IRQHandler,  // DMA1 Stream 3
    (uint32_t*) DMA1_Stream4_IRQHandler,  // DMA1 Stream 4
    (uint32_t*) DMA1_Stream5_IRQHandler,  // DMA1 Stream 5
    (uint32_t*) DMA1_Stream6_IRQHandler,  // DMA1 Stream 6
    (uint32_t*) ADC_IRQHandler,       // ADC1, ADC2 and ADC3
    (uint32_t*) CAN1_TX_IRQHandler,   // CAN1 TX
    (uint32_t*) CAN1_RX0_IRQHandler,  // CAN1 RX0
    (uint32_t*) CAN1_RX1_IRQHandler,  // CAN1 RX1
    (uint32_t*) CAN1_SCE_IRQHandler,  // CAN1 SCE
    (uint32_t*) EXTI9_5_IRQHandler,   // EXTI Line9..5
    (uint32_t*) TIM1_BRK_TIM9_IRQHandler, // TIM1 Break and TIM9
    (uint32_t*) TIM1_UP_TIM10_IRQHandler, // TIM1 Update and TIM10
    (uint32_t*) TIM1_TRG_COM_TIM11_IRQHandler, // TIM1 Trigger and Commutation and TIM11
    (uint32_t*) TIM1_CC_IRQHandler,   // TIM1 Capture Compare
    (uint32_t*) TIM2_IRQHandler,      // TIM2
    (uint32_t*) TIM3_IRQHandler,      // TIM3
    (uint32_t*) TIM4_IRQHandler,      // TIM4
    (uint32_t*) I2C1_EV_IRQHandler,   // I2C1 Event
    (uint32_t*) I2C1_ER_IRQHandler,   // I2C1 Error
    (uint32_t*) I2C2_EV_IRQHandler,   // I2C2 Event
    (uint32_t*) I2C2_ER_IRQHandler,   // I2C2 Error
    (uint32_t*) SPI1_IRQHandler,      // SPI1
    (uint32_t*) SPI2_IRQHandler,      // SPI2
    (uint32_t*) USART1_IRQHandler,    // USART1
    (uint32_t*) USART2_IRQHandler,    // USART2
    (uint32_t*) USART3_IRQHandler,    // USART3
    (uint32_t*) EXTI15_10_IRQHandler, // EXTI Line15..10
    (uint32_t*) RTC_Alarm_IRQHandler, // RTC Alarm (A and B) through EXTI Line
    (uint32_t*) OTG_FS_WKUP_IRQHandler, // USB OTG FS Wakeup through EXTI line
    (uint32_t*) TIM8_BRK_TIM12_IRQHandler, // TIM8 Break and TIM12
    (uint32_t*) TIM8_UP_TIM13_IRQHandler,  // TIM8 Update and TIM13
    (uint32_t*) TIM8_TRG_COM_TIM14_IRQHandler, // TIM8 Trigger and Commutation and TIM14
    (uint32_t*) TIM8_CC_IRQHandler,   // TIM8 Capture Compare
    (uint32_t*) DMA1_Stream7_IRQHandler, // DMA1 Stream7
    (uint32_t*) FSMC_IRQHandler,      // FSMC
    (uint32_t*) SDIO_IRQHandler,      // SDIO
    (uint32_t*) TIM5_IRQHandler,      // TIM5
    (uint32_t*) SPI3_IRQHandler,      // SPI3
    (uint32_t*) UART4_IRQHandler,     // UART4
    (uint32_t*) UART5_IRQHandler,     // UART5
    (uint32_t*) TIM6_DAC_IRQHandler,  // TIM6 and DAC1&2 underrun errors
    (uint32_t*) TIM7_IRQHandler,      // TIM7
    (uint32_t*) DMA2_Stream0_IRQHandler, // DMA2 Stream 0
    (uint32_t*) DMA2_Stream1_IRQHandler, // DMA2 Stream 1
    (uint32_t*) DMA2_Stream2_IRQHandler, // DMA2 Stream 2
    (uint32_t*) DMA2_Stream3_IRQHandler, // DMA2 Stream 3
    (uint32_t*) DMA2_Stream4_IRQHandler, // DMA2 Stream 4
    (uint32_t*) ETH_IRQHandler,       // Ethernet
    (uint32_t*) ETH_WKUP_IRQHandler,  // Ethernet Wakeup through EXTI line
    (uint32_t*) CAN2_TX_IRQHandler,   // CAN2 TX
    (uint32_t*) CAN2_RX0_IRQHandler,  // CAN2 RX0
    (uint32_t*) CAN2_RX1_IRQHandler,  // CAN2 RX1
    (uint32_t*) CAN2_SCE_IRQHandler,  // CAN2 SCE
    (uint32_t*) OTG_FS_IRQHandler,    // USB OTG FS
    (uint32_t*) DMA2_Stream5_IRQHandler, // DMA2 Stream 5
    (uint32_t*) DMA2_Stream6_IRQHandler, // DMA2 Stream 6
    (uint32_t*) DMA2_Stream7_IRQHandler, // DMA2 Stream 7
    (uint32_t*) USART6_IRQHandler,    // USART6
    (uint32_t*) I2C3_EV_IRQHandler,   // I2C3 event
    (uint32_t*) I2C3_ER_IRQHandler,   // I2C3 error
    (uint32_t*) OTG_HS_EP1_OUT_IRQHandler, // USB OTG HS End Point 1 Out
    (uint32_t*) OTG_HS_EP1_IN_IRQHandler, // USB OTG HS End Point 1 In
    (uint32_t*) OTG_HS_WKUP_IRQHandler, // USB OTG HS Wakeup through EXTI
    (uint32_t*) OTG_HS_IRQHandler,    // USB OTG HS
    (uint32_t*) DCMI_IRQHandler,      // DCMI
    0,                                // Reserved
    (uint32_t*) HASH_RNG_IRQHandler,  // Hash and Rng
    (uint32_t*) FPU_IRQHandler,       // FPU
};

/*
    Definition of handlers
    At a minimum for simple programs you at least need the Reset Handler
        * Initialize
        * Call main()
    The remainder of the unused interrupts can be handled in at least 2 ways
        1. Create a default handler to capture events
        2. Define every handler (since I plan to build in complexity I selected this option)
*/

// Reset handler: Called on microcontroller reset
void Reset_Handler(void) {
    // Initialize .data section - copy .data section to SRAM
    uint32_t size = (uint32_t)&_data_end - (uint32_t)&_data_start;
    uint8_t *pDst = (uint8_t*)&_data_start; //SRAM 
    uint8_t *pSrc = (uint8_t*)&_si_data; //FLASH
    uint32_t i = 0;

    //
    for(i=0; i < size; i++)
    {
        *pDst++ = *pSrc++;
    }

    // Initalize .bss section - set to zero
    size = (uint32_t)&_bss_end - (uint32_t)&_bss_start;
    pDst = (uint8_t*)&_bss_start;

    for(i=0; i < size; i++)
    {
        *pDst++ = 0;
    }

     __libc_init_array();

    //Call the main() function
    main();
}

// Non-maskable interrupt (NMI) handler: Cannot be stopped or preempted
void NMI_Handler(void) {
    // TODO: Handle Non Maskable Int, such as unexpected events like clock failure
    while(1);
}

// Hard fault handler: Handles most system errors, such as memory access violation
void HardFault_Handler(void) {
    // TODO: Handle Hard Fault interrupt
    while(1);
}

// Memory management fault handler: Handles memory protection faults
void MemManage_Handler(void) {
    // TODO: Handle Memory Manage interrupt
    while(1);
}

// Bus fault handler: Handles faults on bus access
void BusFault_Handler(void) {
    // TODO: Handle Bus Fault interrupt
    while(1);
}

// Usage fault handler: Handles undefined instruction, division by zero etc.
void UsageFault_Handler(void) {
    // TODO: Handle Usage Fault interrupt
    while(1);
}

// SVCall handler: Handles system service call via SWI instruction
void SVC_Handler(void) {
    // TODO: Handle SVCall interrupt
    while(1);
}

// Debug monitor handler: Handles debug events
void DebugMon_Handler(void) {
    // TODO: Handle Debug Monitor interrupt
    while(1);
}

// Pendable request for system service handler: Handles requests for system level
void PendSV_Handler(void) {
    // TODO: Handle PendSV interrupt, maybe context switch in an OS environment
    while(1);
}

// System tick timer handler: Handles a system tick timer counter
void SysTick_Handler(void) {
    // TODO: Handle SysTick interrupt, usually used for OS task scheduling, time/delay functions
    while(1);
}

// Window Watchdog interrupt handler
void WWDG_IRQHandler(void) {
    // TODO: Handle Window WatchDog interrupt
    while(1);
}

// PVD through EXTI Line detection interrupt handler
void PVD_IRQHandler(void) {
    // TODO: Handle PVD through EXTI Line Detection interrupt
    while(1);
}

// Tamper and TimeStamp through EXTI line interrupt handler
void TAMP_STAMP_IRQHandler(void) {
    // TODO: Handle Tamper and TimeStamp interrupts through the EXTI line
    while(1);
}

// RTC Wakeup interrupt through EXTI line interrupt handler
void RTC_WKUP_IRQHandler(void) {
    // TODO: Handle RTC Wakeup interrupt through the EXTI line
    while(1);
}

// FLASH global interrupt handler
void FLASH_IRQHandler(void) {
    // TODO: Handle Flash global interrupt
    while(1);
}

// RCC global interrupt handler
void RCC_IRQHandler(void) {
    // TODO: Handle RCC global interrupt
    while(1);
}

// EXTI Line0 interrupt handler
void EXTI0_IRQHandler(void) {

    /*
    Setup Pending register
    12.3.3 in the reference manual -
        Bits 22:0 PRx: Pending bit
            * 0: No trigger request occurred
            * 1: selected trigger request occurred
            * This bit is set when the selected edge event arrives on the external interrupt line. This bit is cleared by programming it to ‘1’.
   */
  uint32_t* pEXTI_PR = (uint32_t*)(EXTI_BASE +  0x14);

  if (*pEXTI_PR & (1 << 0))
  {
    /*
        Control which LED # is to be toggled
        If the LED # is > or = to 15 reset to 12 else increment the # by 1
    */
    led_to_toggle = led_to_toggle >= 15 ? 12 : led_to_toggle + 1;

    *pEXTI_PR |= (1 << 0); // Clear interrupt bit

  }
}

// EXTI Line1 interrupt handler
void EXTI1_IRQHandler(void) {
    // TODO: Handle EXTI Line1 interrupt
    while(1);
}

// EXTI Line2 interrupt handler
void EXTI2_IRQHandler(void) {
    // TODO: Handle EXTI Line2 interrupt
    while(1);
}

// EXTI Line3 interrupt handler
void EXTI3_IRQHandler(void) {
    // TODO: Handle EXTI Line3 interrupt
    while(1);
}

// EXTI Line4 interrupt handler
void EXTI4_IRQHandler(void) {
    // TODO: Handle EXTI Line4 interrupt
    while(1);
}

// DMA1 Stream0 global interrupt handler
void DMA1_Stream0_IRQHandler(void) {
    // TODO: Handle DMA1 Stream0 global interrupt
    while(1);
}

// DMA1 Stream1 global interrupt handler
void DMA1_Stream1_IRQHandler(void) {
    // TODO: Handle DMA1 Stream1 global interrupt
    while(1);
}

// DMA1 Stream2 global interrupt handler
void DMA1_Stream2_IRQHandler(void) {
    // TODO: Handle DMA1 Stream2 global interrupt
    while(1);
}

// DMA1 Stream3 global interrupt handler
void DMA1_Stream3_IRQHandler(void) {
    // TODO: Handle DMA1 Stream3 global interrupt
    while(1);
}

// DMA1 Stream4 global interrupt handler
void DMA1_Stream4_IRQHandler(void) {
    // TODO: Handle DMA1 Stream4 global interrupt
    while(1);
}

// DMA1 Stream5 global interrupt handler
void DMA1_Stream5_IRQHandler(void) {
    // TODO: Handle DMA1 Stream5 global interrupt
    while(1);
}

// DMA1 Stream6 global interrupt handler
void DMA1_Stream6_IRQHandler(void) {
    // TODO: Handle DMA1 Stream6 global interrupt
    while(1);
}

// ADC1, ADC2 and ADC3 global interrupts handler
void ADC_IRQHandler(void) {
    // TODO: Handle ADC1, ADC2 and ADC3 global interrupts
    while(1);
}

// CAN1 TX interrupt handler
void CAN1_TX_IRQHandler(void) {
    // TODO: Handle CAN1 TX interrupt
    while(1);
}

// CAN1 RX0 interrupt handler
void CAN1_RX0_IRQHandler(void) {
    // TODO: Handle CAN1 RX0 interrupt
    while(1);
}

// CAN1 RX1 interrupt handler
void CAN1_RX1_IRQHandler(void) {
    // TODO: Handle CAN1 RX1 interrupt
    while(1);
}

// CAN1 SCE interrupt handler
void CAN1_SCE_IRQHandler(void) {
    // TODO: Handle CAN1 SCE interrupt
    while(1);
}

// EXTI Line[9:5] interrupts handler
void EXTI9_5_IRQHandler(void) {
    // TODO: Handle EXTI Line[9:5] interrupts
    while(1);
}

// TIM1 Break interrupt and TIM9 global interrupt handler
void TIM1_BRK_TIM9_IRQHandler(void) {
    // TODO: Handle TIM1 Break interrupt and TIM9 global interrupt
    while(1);
}

// TIM1 Update interrupt and TIM10 global interrupt handler
void TIM1_UP_TIM10_IRQHandler(void) {
    // TODO: Handle TIM1 Update interrupt and TIM10 global interrupt
    while(1);
}

// TIM1 Trigger and Commutation interrupt and TIM11 global interrupt handler
void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    // TODO: Handle TIM1 Trigger and Commutation interrupt and TIM11 global interrupt
    while(1);
}

// TIM1 Capture Compare interrupt handler
void TIM1_CC_IRQHandler(void) {
    // TODO: Handle TIM1 Capture Compare interrupt
    while(1);
}

// TIM2 global interrupt handler
void TIM2_IRQHandler(void) {
    // TODO: Handle TIM2 global interrupt
    while(1);
}

// TIM3 global interrupt handler
void TIM3_IRQHandler(void) {
    // TODO: Handle TIM3 global interrupt
    while(1);
}

// TIM4 global interrupt handler
void TIM4_IRQHandler(void) {
    // TODO: Handle TIM4 global interrupt
    while(1);
}

// I2C1 Event interrupt handler
void I2C1_EV_IRQHandler(void) {
    // TODO: Handle I2C1 Event interrupt
    while(1);
}

// I2C1 Error interrupt handler
void I2C1_ER_IRQHandler(void) {
    // TODO: Handle I2C1 Error interrupt
    while(1);
}

// I2C2 Event interrupt handler
void I2C2_EV_IRQHandler(void) {
    // TODO: Handle I2C2 Event interrupt
    while(1);
}

// I2C2 Error interrupt handler
void I2C2_ER_IRQHandler(void) {
    // TODO: Handle I2C2 Error interrupt
    while(1);
}

// SPI1 global interrupt handler
void SPI1_IRQHandler(void) {
    // TODO: Handle SPI1 global interrupt
    while(1);
}

// SPI2 global interrupt handler
void SPI2_IRQHandler(void) {
    // TODO: Handle SPI2 global interrupt
    while(1);
}

// USART1 global interrupt handler
void USART1_IRQHandler(void) {
    // TODO: Handle USART1 global interrupt
    while(1);
}

// USART2 global interrupt handler
void USART2_IRQHandler(void) {
    // TODO: Handle USART2 global interrupt
    while(1);
}

// USART3 global interrupt handler
void USART3_IRQHandler(void) {
    // TODO: Handle USART3 global interrupt
    while(1);
}

// EXTI Line[15:10] interrupts handler
void EXTI15_10_IRQHandler(void) {
    // TODO: Handle EXTI Line[15:10] interrupts
    while(1);
}

// RTC Alarms (A and B) through EXTI Line interrupt handler
void RTC_Alarm_IRQHandler(void) {
    // TODO: Handle RTC Alarms (A and B) through EXTI Line interrupt
    while(1);
}

// USB OTG FS Wakeup through EXTI line interrupt handler
void OTG_FS_WKUP_IRQHandler(void) {
    // TODO: Handle USB OTG FS Wakeup through EXTI line interrupt
    while(1);
}

// TIM8 Break interrupt and TIM12 global interrupt handler
void TIM8_BRK_TIM12_IRQHandler(void) {
    // TODO: Handle TIM8 Break interrupt and TIM12 global interrupt
    while(1);
}

// TIM8 Update interrupt and TIM13 global interrupt handler
void TIM8_UP_TIM13_IRQHandler(void) {
    // TODO: Handle TIM8 Update interrupt and TIM13 global interrupt
    while(1);
}

// TIM8 Trigger and Commutation interrupt and TIM14 global interrupt handler
void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    // TODO: Handle TIM8 Trigger and Commutation interrupt and TIM14 global interrupt
    while(1);
}

// TIM8 Capture Compare interrupt handler
void TIM8_CC_IRQHandler(void) {
    // TODO: Handle TIM8 Capture Compare interrupt
    while(1);
}

// DMA1 Stream7 global interrupt handler
void DMA1_Stream7_IRQHandler(void) {
    // TODO: Handle DMA1 Stream7 global interrupt
    while(1);
}

// FSMC global interrupt handler
void FSMC_IRQHandler(void) {
    // TODO: Handle FSMC global interrupt
    while(1);
}

// SDIO global interrupt handler
void SDIO_IRQHandler(void) {
    // TODO: Handle SDIO global interrupt
    while(1);
}

// TIM5 global interrupt handler
void TIM5_IRQHandler(void) {
    // TODO: Handle TIM5 global interrupt
    while(1);
}

// SPI3 global interrupt handler
void SPI3_IRQHandler(void) {
    // TODO: Handle SPI3 global interrupt
    while(1);
}

// UART4 global interrupt handler
void UART4_IRQHandler(void) {
    // TODO: Handle UART4 global interrupt
    while(1);
}

// UART5 global interrupt handler
void UART5_IRQHandler(void) {
    // TODO: Handle UART5 global interrupt
    while(1);
}

// TIM6 global and DAC1&2 underrun error  interrupts handler
void TIM6_DAC_IRQHandler(void) {
    // TODO: Handle TIM6 global and DAC1&2 underrun error interrupts
    while(1);
}

// TIM7 global interrupt handler
void TIM7_IRQHandler(void) {
    // TODO: Handle TIM7 global interrupt
    while(1);
}

// DMA2 Stream0 global interrupt handler
void DMA2_Stream0_IRQHandler(void) {
    // TODO: Handle DMA2 Stream0 global interrupt
    while(1);
}

// DMA2 Stream1 global interrupt handler
void DMA2_Stream1_IRQHandler(void) {
    // TODO: Handle DMA2 Stream1 global interrupt
    while(1);
}

// DMA2 Stream2 global interrupt handler
void DMA2_Stream2_IRQHandler(void) {
    // TODO: Handle DMA2 Stream2 global interrupt
    while(1);
}

// DMA2 Stream3 global interrupt handler
void DMA2_Stream3_IRQHandler(void) {
    // TODO: Handle DMA2 Stream3 global interrupt
    while(1);
}

// DMA2 Stream4 global interrupt handler
void DMA2_Stream4_IRQHandler(void) {
    // TODO: Handle DMA2 Stream4 global interrupt
    while(1);
}

// Ethernet global interrupt handler
void ETH_IRQHandler(void) {
    // TODO: Handle Ethernet global interrupt
    while(1);
}

// Ethernet Wakeup through EXTI line interrupt handler
void ETH_WKUP_IRQHandler(void) {
    // TODO: Handle Ethernet Wakeup through EXTI line interrupt
    while(1);
}

// CAN2 TX interrupt handler
void CAN2_TX_IRQHandler(void) {
    // TODO: Handle CAN2 TX interrupt
    while(1);
}

// CAN2 RX0 interrupt handler
void CAN2_RX0_IRQHandler(void) {
    // TODO: Handle CAN2 RX0 interrupt
    while(1);
}

// CAN2 RX1 interrupt handler
void CAN2_RX1_IRQHandler(void) {
    // TODO: Handle CAN2 RX1 interrupt
    while(1);
}

// CAN2 SCE interrupt handler
void CAN2_SCE_IRQHandler(void) {
    // TODO: Handle CAN2 SCE interrupt
    while(1);
}

// USB OTG FS global interrupt handler
void OTG_FS_IRQHandler(void) {
    // TODO: Handle USB OTG FS global interrupt
    while(1);
}

// DMA2 Stream5 global interrupt handler
void DMA2_Stream5_IRQHandler(void) {
    // TODO: Handle DMA2 Stream5 global interrupt
    while(1);
}

// DMA2 Stream6 global interrupt handler
void DMA2_Stream6_IRQHandler(void) {
    // TODO: Handle DMA2 Stream6 global interrupt
    while(1);
}

// DMA2 Stream7 global interrupt handler
void DMA2_Stream7_IRQHandler(void) {
    // TODO: Handle DMA2 Stream7 global interrupt
    while(1);
}

// USART6 global interrupt handler
void USART6_IRQHandler(void) {
    // TODO: Handle USART6 global interrupt
    while(1);
}

// I2C3 event interrupt handler
void I2C3_EV_IRQHandler(void) {
    // TODO: Handle I2C3 event interrupt
    while(1);
}

// I2C3 error interrupt handler
void I2C3_ER_IRQHandler(void) {
    // TODO: Handle I2C3 error interrupt
    while(1);
}

// USB OTG HS End Point 1 Out global interrupt handler
void OTG_HS_EP1_OUT_IRQHandler(void) {
    // TODO: Handle USB OTG HS End Point 1 Out global interrupt
    while(1);
}

// USB OTG HS End Point 1 In global interrupt handler
void OTG_HS_EP1_IN_IRQHandler(void) {
    // TODO: Handle USB OTG HS End Point 1 In global interrupt
    while(1);
}

// USB OTG HS Wakeup through EXTI interrupt handler
void OTG_HS_WKUP_IRQHandler(void) {
    // TODO: Handle USB OTG HS Wakeup through EXTI interrupt
    while(1);
}

// USB OTG HS global interrupt handler
void OTG_HS_IRQHandler(void) {
    // TODO: Handle USB OTG HS global interrupt
    while(1);
}

// DCMI global interrupt handler
void DCMI_IRQHandler(void) {
    // TODO: Handle DCMI global interrupt
    while(1);
}

// CRYP crypto global interrupt handler
void CRYP_IRQHandler(void) {
    // TODO: Handle CRYP crypto global interrupt
    while(1);
}

// Hash and Rng global interrupt handler
void HASH_RNG_IRQHandler(void) {
    // TODO: Handle Hash and Rng global interrupt
    while(1);
}

// FPU global interrupt handler
void FPU_IRQHandler(void) {
    // TODO: Handle FPU global interrupt
    while(1);
}

