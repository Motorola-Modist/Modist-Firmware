/************************************************************************************
 * arch/arm/src/stm32/chip/stm32l4x6xx_vectors.h
 *
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/

/* This file is included by stm32_vectors.S.  It provides the macro VECTOR that
 * supplies each STM32L4x6xx vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/stm32/stm32l4x6xx_irq.h.
 * stm32_vectors.S will define the VECTOR macro in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve interrupt table entries for I/O interrupts. */
#  define ARMV7M_PERIPHERAL_INTERRUPTS 82

#else

VECTOR(stm32_wwdg, STM32_IRQ_WWDG)               /* Vector 16+0:  Window Watchdog interrupt */
VECTOR(stm32_pvd, STM32_IRQ_PVD)                 /* Vector 16+1:  PVD through EXTI Line detection interrupt */
VECTOR(stm32_tamper, STM32_IRQ_TAMPER)           /* Vector 16+2:  Tamper and time stamp interrupts */
VECTOR(stm32_rtc_wkup, STM32_IRQ_RTC_WKUP)       /* Vector 16+3:  RTC global interrupt */
VECTOR(stm32_flash, STM32_IRQ_FLASH)             /* Vector 16+4:  Flash global interrupt */
VECTOR(stm32_rcc, STM32_IRQ_RCC)                 /* Vector 16+5:  RCC global interrupt */
VECTOR(stm32_exti0, STM32_IRQ_EXTI0)             /* Vector 16+6:  EXTI Line 0 interrupt */
VECTOR(stm32_exti1, STM32_IRQ_EXTI1)             /* Vector 16+7:  EXTI Line 1 interrupt */
VECTOR(stm32_exti2, STM32_IRQ_EXTI2)             /* Vector 16+8:  EXTI Line 2 interrupt */
VECTOR(stm32_exti3, STM32_IRQ_EXTI3)             /* Vector 16+9:  EXTI Line 3 interrupt */
VECTOR(stm32_exti4, STM32_IRQ_EXTI4)             /* Vector 16+10: EXTI Line 4 interrupt */
VECTOR(stm32_dma1ch1, STM32_IRQ_DMA1CH1)         /* Vector 16+11: DMA1 Channel 1 global interrupt */
VECTOR(stm32_dma1ch2, STM32_IRQ_DMA1CH2)         /* Vector 16+12: DMA1 Channel 2 global interrupt */
VECTOR(stm32_dma1ch3, STM32_IRQ_DMA1CH3)         /* Vector 16+13: DMA1 Channel 3 global interrupt */
VECTOR(stm32_dma1ch4, STM32_IRQ_DMA1CH4)         /* Vector 16+14: DMA1 Channel 4 global interrupt */
VECTOR(stm32_dma1ch5, STM32_IRQ_DMA1CH5)         /* Vector 16+15: DMA1 Channel 5 global interrupt */
VECTOR(stm32_dma1ch6, STM32_IRQ_DMA1CH6)         /* Vector 16+16: DMA1 Channel 6 global interrupt */
VECTOR(stm32_dma1ch7, STM32_IRQ_DMA1CH7)         /* Vector 16+17: DMA1 Channel 7 global interrupt */
VECTOR(stm32_adc12, STM32_IRQ_ADC12)             /* Vector 16+18: ADC1 and ADC2 global interrupt */
VECTOR(stm32_can1tx, STM32_IRQ_CAN1TX)           /* Vector 16+19: CAN1 TX interrupts */
VECTOR(stm32_can1rx0, STM32_IRQ_CAN1RX0)         /* Vector 16+20: CAN1 RX0 interrupts */
VECTOR(stm32_can1rx1, STM32_IRQ_CAN1RX1)         /* Vector 16+21: CAN1 RX1 interrupt */
VECTOR(stm32_can1sce, STM32_IRQ_CAN1SCE)         /* Vector 16+22: CAN1 SCE interrupt */
VECTOR(stm32_exti95, STM32_IRQ_EXTI95)           /* Vector 16+23: EXTI Line[9:5] interrupts */
VECTOR(stm32_tim1brk, STM32_IRQ_TIM1BRK)         /* Vector 16+24: TIM1 Break interrupt/TIM15 global interrupt */
VECTOR(stm32_tim1up, STM32_IRQ_TIM1UP)           /* Vector 16+25: TIM1 Update interrupt/TIM16 global interrupt */
VECTOR(stm32_tim1trgcom, STM32_IRQ_TIM1TRGCOM)   /* Vector 16+26: TIM1 Trigger and Commutation interrupts/TIM17 global interrupt */
VECTOR(stm32_tim1cc, STM32_IRQ_TIM1CC)           /* Vector 16+27: TIM1 Capture Compare interrupt */
VECTOR(stm32_tim2, STM32_IRQ_TIM2)               /* Vector 16+28: TIM2 global interrupt */
VECTOR(stm32_tim3, STM32_IRQ_TIM3)               /* Vector 16+29: TIM3 global interrupt */
VECTOR(stm32_tim4, STM32_IRQ_TIM4)               /* Vector 16+30: TIM4 global interrupt */
VECTOR(stm32_i2c1ev, STM32_IRQ_I2C1EV)           /* Vector 16+31: I2C1 event interrupt */
VECTOR(stm32_i2c1er, STM32_IRQ_I2C1ER)           /* Vector 16+32: I2C1 error interrupt */
VECTOR(stm32_i2c2ev, STM32_IRQ_I2C2EV)           /* Vector 16+33: I2C2 event interrupt */
VECTOR(stm32_i2c2er, STM32_IRQ_I2C2ER)           /* Vector 16+34: I2C2 error interrupt */
VECTOR(stm32_spi1, STM32_IRQ_SPI1)               /* Vector 16+35: SPI1 global interrupt */
VECTOR(stm32_spi2, STM32_IRQ_SPI2)               /* Vector 16+36: SPI2 global interrupt */
VECTOR(stm32_usart1, STM32_IRQ_USART1)           /* Vector 16+37: USART1 global interrupt */
VECTOR(stm32_usart2, STM32_IRQ_USART2)           /* Vector 16+38: USART2 global interrupt */
VECTOR(stm32_usart3, STM32_IRQ_USART3)           /* Vector 16+39: USART3 global interrupt */
VECTOR(stm32_exti1510, STM32_IRQ_EXTI1510)       /* Vector 16+40: EXTI Line[15:10] interrupts */
VECTOR(stm32_rtcalrm, STM32_IRQ_RTCALRM)         /* Vector 16+41: RTC alarm through EXTI line interrupt */
VECTOR(stm32_otgfswkup, STM32_IRQ_DFSDM4)        /* Vector 16+42: DFSDM4 global interrupt */
VECTOR(stm32_tim8brk, STM32_IRQ_TIM8BRK)         /* Vector 16+43: TIM8 Break interrupt/TIM12 global interrupt */
VECTOR(stm32_tim8up, STM32_IRQ_TIM8UP)           /* Vector 16+44: TIM8 Update interrup/TIM13 global interrupt */
VECTOR(stm32_tim8trgcom, STM32_IRQ_TIM8TRGCOM)   /* Vector 16+45: TIM8 Trigger and Commutation interrupts/TIM14 global interrupt */
VECTOR(stm32_tim8cc, STM32_IRQ_TIM8CC)           /* Vector 16+46: TIM8 Capture Compare interrupt */
VECTOR(stm32_adc3, STM32_IRQ_ADC3)               /* Vector 16+47: ADC3 global interrupt */
VECTOR(stm32_fmc, STM32_IRQ_FMC)                 /* Vector 16+48: FMC global interrupt */
VECTOR(stm32_sdmmc, STM32_IRQ_SDMMC)             /* Vector 16+49: SDMMC global interrupt */
VECTOR(stm32_tim5, STM32_IRQ_TIM5)               /* Vector 16+50: TIM5 global interrupt */
VECTOR(stm32_spi3, STM32_IRQ_SPI3)               /* Vector 16+51: SPI3 global interrupt */
VECTOR(stm32_uart4, STM32_IRQ_UART4)             /* Vector 16+52: UART4 global interrupt */
VECTOR(stm32_uart5, STM32_IRQ_UART5)             /* Vector 16+53: UART5 global interrupt */
VECTOR(stm32_tim6, STM32_IRQ_TIM6)               /* Vector 16+54: TIM6 global interrupt/DAC1 and DAC2 underrun error interrupts */
VECTOR(stm32_tim7, STM32_IRQ_TIM7)               /* Vector 16+55: TIM7 global interrupt */
VECTOR(stm32_dma2ch1, STM32_IRQ_DMA2CH1)         /* Vector 16+56: DMA2 Channel 1 global interrupt */
VECTOR(stm32_dma2ch2, STM32_IRQ_DMA2CH2)         /* Vector 16+57: DMA2 Channel 2 global interrupt */
VECTOR(stm32_dma2ch3, STM32_IRQ_DMA2CH3)         /* Vector 16+58: DMA2 Channel 3 global interrupt */
VECTOR(stm32_dma2ch4, STM32_IRQ_DMA2CH4)         /* Vector 16+59: DMA2 Channel 4 global interrupt */
VECTOR(stm32_dma2ch5, STM32_IRQ_DMA2CH5)         /* Vector 16+60: DMA2 Channel 5 global interrupt */
VECTOR(stm32_dfsdm1, STM32_IRQ_DFSDM1)           /* Vector 16+61: DFSDM1 global interrupt */
VECTOR(stm32_dfsdm2, STM32_IRQ_DFSDM2)           /* Vector 16+62: DFSDM2 global interrupt */
VECTOR(stm32_dfsdm3, STM32_IRQ_DFSDM3)           /* Vector 16+63: DFSDM3 global interrupt */
VECTOR(stm32_comp, STM32_IRQ_COMP)               /* Vector 16+64: COMP1/COMP2 through EXTI lines 21/22 interrupts */
VECTOR(stm32_lptim1, STM32_IRQ_LPTIM1)           /* Vector 16+65: LPTIM1 global interrupt */
VECTOR(stm32_lptim2, STM32_IRQ_LPTIM2)           /* Vector 16+66: LPTIM2 global interrupt */
VECTOR(stm32_otgfs, STM32_IRQ_OTGFS)             /* Vector 16+67: USB On The Go FS global interrupt */
VECTOR(stm32_dma2ch6, STM32_IRQ_DMA2CH6)         /* Vector 16+68: DMA2 Channel 6 global interrupt */
VECTOR(stm32_dma2ch7, STM32_IRQ_DMA2CH7)         /* Vector 16+69: DMA2 Channel 7 global interrupt */
VECTOR(stm32_lpuart1, STM32_IRQ_LPUART1)         /* Vector 16+70: LPUART1 global interrupt */
VECTOR(stm32_quadspi, STM32_IRQ_QUADSPI)         /* Vector 16+71: QUADSPI global interrupt */
VECTOR(stm32_i2c3ev, STM32_IRQ_I2C3EV)           /* Vector 16+72: I2C3 event interrupt */
VECTOR(stm32_i2c3er, STM32_IRQ_I2C3ER)           /* Vector 16+73: I2C3 error interrupt */
VECTOR(stm32_sai1, STM32_IRQ_SAI1)               /* Vector 16+74: SAI1 global interrupt */
VECTOR(stm32_sai2, STM32_IRQ_SAI2)               /* Vector 16+75: SAI2 global interrupt */
VECTOR(stm32_swpmi1, STM32_IRQ_SWPMI1)           /* Vector 16+76: SWPMI1 global interrupt */
VECTOR(stm32_tsc, STM32_IRQ_TSC)                 /* Vector 16+77: TSC global interrupt */
VECTOR(stm32_lcd, STM32_IRQ_LCD)                 /* Vector 16+78: LCD global interrupt */
VECTOR(stm32_aes, STM32_IRQ_AES)                 /* Vector 16+79: AES global interrupt */
VECTOR(stm32_rng, STM32_IRQ_RNG)                 /* Vector 16+80: RNG global interrupt */
VECTOR(stm32_fpu, STM32_IRQ_FPU)                 /* Vector 16+81: FPU global interrupt */

#endif /* CONFIG_ARMV7M_CMNVECTOR */
