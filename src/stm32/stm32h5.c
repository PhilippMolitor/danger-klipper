// Code to setup clocks on stm32h5
//
// Copyright (C) 2024 Philipp Molitor <phil.x64 at gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h" // VectorTable
#include "board/armcm_reset.h" // try_request_canboot
#include "board/irq.h" // irq_disable
#include "board/misc.h" // bootloader_request
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // get_pclock_frequency
#include "sched.h" // sched_main


/****************************************************************
 * Clock setup
 ****************************************************************/

#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / 4)

// Map a peripheral address to its enable bits
struct cline
lookup_clock_line(uint32_t periph_base)
{
#error "TODO"
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
#error "TODO"
}

// TODO: check what pins apply to stm32h5
/*
#if !CONFIG_STM32_CLOCK_REF_INTERNAL
DECL_CONSTANT_STR("RESERVE_PINS_crystal", "PH0,PH1");
#endif
*/

// Main clock and power setup called at chip startup
static void
clock_setup(void)
{
#error "TODO"
}


/****************************************************************
 * Bootloader
 ****************************************************************/

// Handle reboot requests
void
bootloader_request(void)
{
    try_request_canboot();
    dfu_reboot();
}


/****************************************************************
 * Startup
 ****************************************************************/

// Main entry point - called from armcm_boot.c:ResetHandler()
void
armcm_main(void)
{
    // TODO: stm32g0 (which seems to be closely related) has a different
    //       routine for this. Check if it applies to stm32h5 as well.
    
    // Run SystemInit() and then restore VTOR
    SystemInit();
    
    // TODO: more registers to reset?
    RCC->APB1LENR = 0x00000000;
    RCC->APB1HENR = 0x00000000;
    RCC->APB2ENR = 0x00000000;
    RCC->APB3ENR = 0x00000000;

    SCB->VTOR = (uint32_t)VectorTable;

    dfu_reboot_check();

    clock_setup();

    sched_main();
}
