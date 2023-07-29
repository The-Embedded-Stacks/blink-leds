/*
    Enables and initializes registers that can be used for multiple functions.
        * AHB1 - Clock, Port A, and Port D
        * EXTI IRQ Handler
        * TIM2 IRQ Handler
*/
#include <stdint.h>
#include "hardware_init.h"

/*

*/
void init_hardware(void)
{
    /* 
        Enable clock for peripheral - the AHB1 RCC base address is 0x40023800
        Go to the reference manual RCC section and find the enable offset 0x30
        0x4002 3800 + 0x30 = 0x40023830 
    */
    uint32_t* pRCC_ANB1ENR = (uint32_t*)(RCC_BASE + 0x30);

    /*
        Dereference & perform bitwise operation to enable clocks
    */
    *pRCC_ANB1ENR |= (1U << 0); // Bit 0 GPIOAEN: IO port A clock enable - found in reference manual
    *pRCC_ANB1ENR |= (1U << 3); // Bit 3 GPIODEN: IO port D clock enable - found in reference manual

    /* 
        Enable APB2 peripheral reset register - enable the SYSCFG clock
        Bit 14 SYSCFGRST: System configuration controller reset
            Set and cleared by software.
            * 0: does not reset the System configuration controller
            * 1: resets the System configuration controller
    */
    uint32_t* pRCC_APB2RSTR = (uint32_t*)(RCC_BASE + 0x24);
    *pRCC_APB2RSTR |= (1U << 14);

    /*
        Enable System Configuration - Register 1 Line 0
        This must be done to divert the input signal from PA0 to the EXTI Controller to invoke the interrput
        9.2.3 in the reference manual - These bits are written by software to select the source input for the EXTIx external interrupt.
            * 0000: PA[x] pin
            * 0000: PA[0] pin
    */
   uint32_t* pSYSCFG_EXTICR1 = (uint32_t*)(SYSCFG_BASE + 0x08);
   *pSYSCFG_EXTICR1 &= ~(0x000F); // ~0b1111 --- need to set the first 4 bits to 0

   /*
        Setup Interrupt Mask Register
        12.3.1 in the reference manual -
            Bits 22:0 MRx: Interrupt mask on line x
                * 0: Interrupt request from line x is masked
                * 1: Interrupt request from line x is not masked
   */
  uint32_t* pEXTI_IMR = (uint32_t*)(EXTI_BASE);
  *pEXTI_IMR |= (1U << 0);

    /*
        Setup Rising trigger selection register
        12.3.3 in the reference manual -
            Bits 22:0 TRx: Rising trigger event configuration bit of line x
                * 0: Rising trigger disabled (for Event and Interrupt) for input line
                * 1: Rising trigger enabled (for Event and Interrupt) for input line
   */
  uint32_t* pEXTI_RTSR = (uint32_t*)(EXTI_BASE + 0x08);
  *pEXTI_RTSR |= (1U << 0); // Set bit 0 to 1 for a rising edge (pull-down configuration)

  /*
    Set Interupt Set-Enable Register (ISER) - Cortex-M4 specific
    Table 61 in the reference manual shows us that EXTI0 is in position 6 (IRQ6) - offset address 0x0000 0058
    Under 6.2.1 in the Cortex-M4 processor technical reference manual we see:
        * NVIC_ISER0 base address is 0xE000 E100
    ISER0 is a 32-bit wide register and therefore handles interrupts 0-31
  */
 uint32_t* NVIC_ISER0 = (uint32_t*)ISER0_BASE;
 *NVIC_ISER0 |= (1U << EXTI0_IRQ); // Enable EXTI0
 *NVIC_ISER0 |= (1U << TIM2_IRQ); // Enable TIM2

 /*
    Set Interrupt Priority Register (IPR) - Cortex-M4 specific
    Under 6.2.1 in the Cortex-M4 processor technical reference manual we see:
        * NVIC_IPR0 base address is 0xE000 E400
    IPR0 is a 32-bit wide register and therefore handles interrupts 0 - 3
        * Each pritory register is 8-bits wide with the priority begin set in the higher 4
        * 2^4 = 16 possible priority levels with 0 being the highest and 15 being the lowest
    We are intrested in IRQ6 so:
        * the priority is 4-bits wide
        * IPR offset = IRQ# / 4 --- 6/4 = 1 --- therefore the offset = 0x04 (IPR1)
        * Each IPR contains 4 priority fields so 6 mod 4 = 2
        * Therefore we have an offset of 0x04 and need to access the 2nd 8-bit index --- bits 8 - 15 
 */
uint32_t* NVIC_IPR1 = (uint32_t*)(IPR1_BASE + ((EXTI0_IRQ / 4) * 4));
*NVIC_IPR1 &= ~(0xFF << ((EXTI0_IRQ % 4) * 8)); // Clear second index in IPR1 before setting priority level
*NVIC_IPR1 |= ((15 << 4) << ((EXTI0_IRQ % 4) * 8)); // Shift lowest priority level to priority range than shift to correct index position

/*
    Enable Timer 2 - General purpose timer for debounce
    Table 10 in the datasheet manual:
        * TIM 2 is on the APB1 Bus
        * Base address of: 0x4000 0000
    Reference manual:
        * 16/32-bit counter - the counter can count up and can be divided by a prescalar ( 1 - 65536 )
        * The time-base unit includes:
            • Counter register (TIMx_CNT)
            • Prescaler register (TIMx_PSC):
            • Auto-Reload register (TIMx_ARR)

    Bit 0 TIM2EN: TIM2 clock enable
        Set and cleared by software
        0: TIM2 clock disabled
        1: TIM2 clock enabled
*/ 
uint32_t* pRCC_APB1ENR = (uint32_t*)(RCC_BASE + 0x40);
*pRCC_APB1ENR |= (1U << 0); // Bit 0 TIM2EN: TIM2 clock enable - found in reference manual

/*
    Configure the TIM2 as general debouncer for user button
        * Enable counter
        * Set prescaler
        * Set auto-reload value
    From the TIM2 section in the reference manual:
        Bit 0 UIE: Update interrupt enable
            0: Update interrupt disabled
            1: Update interrupt enabled
        Bits 15:0 CNT[15:0]: Counter value :: uint32_t* pTIM2_CNT = (uint32_t*)(TIM2_BASE + 0x24);
        Bits 15:0 PSC[15:0]: Prescaler value
            The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
        Bits 15:0 ARR[15:0]: Auto-reload value
            ARR is the value to be loaded in the actual auto-reload register.
            Refer to the Section 18.3.1: Time-base unit for more details about ARR update and
            behavior.
            The counter is blocked while the auto-reload value is null
*/
uint32_t* pTIM2_CR1 = (uint32_t*)TIM2_BASE;
uint32_t* pTIM2_DIER = (uint32_t*)(TIM2_BASE + 0x0C);
uint32_t* pTIM2_PSC = (uint32_t*)(TIM2_BASE + 0x28);
uint32_t* pTIM2_ARR = (uint32_t*)(TIM2_BASE + 0x2C);

*pTIM2_CR1 |= (1U << 0); // Counter enabled
*pTIM2_DIER |= (1U << 0); // Update interrupt enabled

/*
    We will target a small prescaler and large AAR
        * this will increase power consumption but will remain flexiable for other use cases
        * the alternative is a large prescaler and small AAR
    prescaler = (SYSCLK SPD / resolution) - 1
        where in this case
            * resolution = 10kHz
            * SYSCLK SPD = 16MHz
*/
*pTIM2_PSC = 1599; // Set prescaler value to 1599

/*
    We want extend out that fine resolution of 100 ms to generate an interrupt every second
    100 ms * 10000 = 1 sec (1000 ms)
*/
*pTIM2_ARR = 10000;

}