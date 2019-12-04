/* ------------------------------------------------------ */
/* File Description */
/* ------------------------------------------------------ */
/*
  file stm8as1_wrap.h
  brief definition of stm8L-151-C8 and its device peripherals.
        not an exhaustive list of all peripheral registers.
  only works for cosmic compiler
*/
/* ------------------------------------------------------ */
/* End of File Description */
/* ------------------------------------------------------ */

#ifndef STM8L151C8_WRAP_H_
#define STM8L151C8_WRAP_H_

#include "STM8L151C8.h"
#include "type_def.h"

/* ------------------------------------------------------ */
/* Compiler Selection */
/* ------------------------------------------------------ */

/* cosmic c compiler */
#if defined(__CSMC__)
/* syntax for variables at absolute addresses */
#define reg(addr, type, name) extern volatile type name @addr
/* single line inline assembler */
#define ASM(mnem) _asm(mnem)
/* start multi-line inline assembler */
#define ASM_START _Pragma("asm")
/* end multi-line inline assembler */
#define ASM_END _Pragma("endasm")
/* memory location decoration */
#define FAR @far
#define NEAR @near
#define TINY @tiny
#define EEPROM @eeprom
/* inline as compiler directive */
#define INLINE @ inline
//#define CONST  			 const
#else
#error "Only Cosmic C compiler supported"
#endif

/*whether enable functions for correction
        to disable, comment the follwoing line*/
#define ENABLE_CHECK
/* ------------------------------------------------------ */
/* End of Compiler Selection */
/* ------------------------------------------------------ */

/* ------------------------------------------------------ */
/* Feature Selection */
/* ------------------------------------------------------ */
/* comment out peripheral not needed*/
/*Port*/
#define HAS_PORTA
#define HAS_PORTB
#define HAS_PORTC
#define HAS_PORTD
#define HAS_PORTE
#define HAS_PORTF
#define HAS_PORTG
#define HAS_PORTH
#define HAS_PORTI
/*flash control */
#define HAS_FLASH
/* direct memory access*/
#define HAS_DMA
/* system config */
#define HAS_SYSCFG
/* external interrupt control */
#define HAS_ITC_EXT
/* reset control */
#define HAS_RST
/*power control*/
#define HAS_PWR
/* clock related */
#define HAS_CLK
#define HAS_WWDG
#define HAS_IWDG
#define HAS_BEEP
/*real time clock*/
#define HAS_RTC
/* clock security system */
#define HAS_CSS
#define HAS_SPI1
#define HAS_SPI2
#define HAS_I2C
#define HAS_USART1
#define HAS_USART2
#define HAS_USART3
#define HAS_ADC1
#define HAS_ADC2
#define HAS_DAC
#define HAS_TIM1
#define HAS_TIM2
#define HAS_TIM3
#define HAS_TIM4
#define HAS_TIM5
#define HAS_IRTIM
#define HAS_LCD
#define HAS_RI
#define HAS_COMP1_2
#define HAS_CPU_REG
#define HAS_ITC_SPR
#define HAS_SWIM
#define HAS_DM

/* ------------------------------------------------------ */
/* End of Feature Selection */
/* ------------------------------------------------------ */

/* ------------------------------------------------------ */
/* Memory  Info*/
/* ------------------------------------------------------ */
#define PFLASH_SIZE 1024 * 64 /*64kB*/
#define RAM_SIZE 1024 * 4     /*4KB*/
#define EEPROM_SIZE 1024 * 2  /*2KB*/

/*memory addresses  //Table 10 of the datasheet */
/*  memory address and address range <=64kB (->16b pointer)
        or >=64kB (->32b pointer)*/
#define RAM_START 0x0000 /*including 513 bytes stack*/
#define RAM_END (RAM_START + RAM_SIZE - 1)
#define EEPROM_START 0x4000 /*just behind RAM*/
#define EEPROM_END (EEPROM_START + EEPROM_SIZE - 1)
#define PFLASH_START 0x8000
#define PFLASH_END (PFLASH_START + PFLASH_SIZE - 1)

/* address space width */
#if (PFLASH_END <= 0xFFFF) /*if flash memory <= 64K*/
#define ADDR_WIDTH 16
#define MEM_POINTER_T uint16_t
#else /*if flash memory >64k*/
#define ADDR_WIDTH 32
#define MEM_POINTER_T uint32_t
#endif

/* ------------------------------------------------------ */
/* End of Memory  Info*/
/* ------------------------------------------------------ */

/* ------------------------------------------------------ */
/* Register Mapping */
/* ------------------------------------------------------ */

/* From data sheet for STM8L151x6/8 STML152x6/8
        named "stm8l152r8.pdf" */
/* set peripherals base addresses. */

/*read out protection, Table 12 of datasheet	*/
#define OPT_BaseAddress 0x4800
/* internal reference voltage factory conversion
        temperature sensor output voltage */
#define FACTORY_BaseAddress 0x4910
/* port a-i, Table 8 of datasheet*/

#if defined(HAS_PORTA)
#define PORTA_BaseAddress 0x5000
#endif

#if defined(HAS_PORTB)
#define PORTB_BaseAddress 0x5005
#endif

#if defined(HAS_PORTC)
#define PORTC_BaseAddress 0x500A
#endif

#if defined(HAS_PORTD)
#define PORTD_BaseAddress 0x500F
#endif

#if defined(HAS_PORTE)
#define PORTE_BaseAddress 0x5014
#endif

#if defined(HAS_PORTF)
#define PORTF_BaseAddress 0x5019
#endif

#if defined(HAS_PORTG)
#define PORTG_BaseAddress 0x501E
#endif

#if defined(HAS_PORTH)
#define PORTH_BaseAddress 0x5023
#endif

#if defined(HAS_PORTI)
#define PORTI_BaseAddress 0x5028
#endif

/* flash control register, Table 9 of datasheet */
#if defined(HAS_FLASH)
#define FLASH_BaseAddress 0x5050
#endif

#if defined(HAS_DMA)
#define DMA_BaseAddress 0x5070
#endif

/* for remapping register */
#if defined(HAS_SYSCFG)
#define SYSCFG_BaseAddress 0x509D
#endif

/* external interrupt control, Table 9, p46
        from 0x50A0 to 0x50AB, including ITC-EXT1
        and WFE */
#if defined(HAS_ITC_EXT)
#define ITC_EXT_BaseAddress 0x50A0
#endif

/* reset control, Table 9, p46. */
#if defined(HAS_RST)
#define RST_BaseAddress 0x50B0
#endif

#if defined(HAS_PWR)
#define PWR_BaseAddress 0x50B2 /* power control */
#endif

#if defined(HAS_CLK)
#define CLK_BaseAddress 0x50C0 /* clock */
#endif

#if defined(HAS_WWDG)
#define WWDG_BaseAddress 0x50D3
#endif

#if defined(HAS_IWDG)
#define IWDG_BaseAddress 0x50E0
#endif

#if defined(HAS_BEEP)
#define BEEP_BaseAddress 0x50F0
#endif

/* realtime clock registers */
#if defined(HAS_RTC)
#define RTC_BaseAddress 0x5140
#endif

#if defined(HAS_CSS)
#define CSS_BaseAddress 0x5190
#endif

#if defined(HAS_SPI1)
#define SPI1_BaseAddress 0x5200
#endif

#if defined(HAS_I2C)
#define I2C_BaseAddress 0x5210
#endif

#if defined(HAS_USART1)
#define USART1_BaseAddress 0x5230
#endif

#if defined(HAS_TIM2)
#define TIM2_BaseAddress 0x5250
#endif

#if defined(HAS_TIM3)
#define TIM3_BaseAddress 0x5380
#endif

#if defined(HAS_TIM1)
#define TIM1_BaseAddress 0x52B0
#endif

#if defined(HAS_TIM4)
#define TIM4_BaseAddress 0x52E0
#endif

#if defined(HAS_IRTIM)
#define IRTIM 0x52FF
#endif

#if defined(HAS_TIM5)
#define TIM5_BaseAddress 0x5300
#endif

#if defined(HAS_ADC1)
#define ADC1_BaseAddress 0x5340
#endif

#if defined(HAS_DAC)
#define DAC_BaseAddress 0x5380
#endif

#if defined(HAS_SPI2)
#define SPI2_BaseAddress 0x53C0
#endif

#if defined(HAS_USART2)
#define USART2_BaseAddress 0x53E0
#endif

#if defined(HAS_USART3)
#define USART3_BaseAddress 0x53F0
#endif

#if defined(HAS_LCD)
#define LCD_BaseAddress 0x5400
#endif

#if defined(HAS_RI)
#define RI_BaseAddress 0x5430
#endif

#if defined(HAS_COMP1_2)
#define COMP1_2_BaseAddress 0x5440
#endif

#if defined(HAS_CPU_REG)
#define CPU_REG_BaseAddress 0x7F00
#endif

#if defined(HAS_ITC_SPR)
#define ITC_SPR_BaseAddress 0x7F70
#endif

#if defined(HAS_SWIM)
#define SWIM_BaseAddress 0x7F80
#endif

#if defined(HAS_DM)
#define DM_BaseAddress 0x7F90
#endif

/* ------------------------------------------------------ */
/* End of Register Mapping */
/* ------------------------------------------------------ */

#endif /* STM8L151C8_WRAP_H_ */
