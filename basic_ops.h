/* file basic_ops.h */

#ifndef BASIC_OPS_H_
#define BASIC_OPS_H_

#include "stm8l151c8_wrap.h"
#include "type_def.h"

/* skip warnings for unused parameters,
see
http://stackoverflow.com/questions/3599160/unused-parameter-warnings-in-c-code
*/
#define unused(x) (void)(x)

/* bits manipulation macros */
/*  get bit value 2^n */
#define bit(n) (0x01 << n)
/* clear single bit in data to '0' */
#define bit_clear(b, n) ((b) &= ~(0x01 << n))
/* read single bit position in byte */
#define bit_read(b, n) (((b) & (0x01 << n)) >> n)
/* set single bit in data to '1' */
#define bit_set(b, n) ((b) |= (0x01 << n))
/* toggle single bit state in byte */
#define bit_toggle(b, n) ((b) ^= (0x01 << n))
/* set single bit value in byte to value */
#define bit_write(b, n, value)                                                 \
  ((value) ? ((b) |= (0x01 << (n))) : ((b) &= ~(0x01 << (n))))
/* get low (=rightmost) byte from x */
#define low_byte(x) ((uint8_t)x)
/* get high (=2nd) byte from a word */
#define high_byte(x) ((uint8_t)(((uint16_t)x) >> 8))
/* concat 2 bytes to a 16bit word */
#define concat_bytes(hb, lb) ((((uint16_t)hb) << 8) | ((uint16_t)lb))

/* MCU related macros */
/* common assembler instructions */
/* perform a nop() operation (=minimum delay) */
#define nop() ASM("nop")
/* disable interrupt handling */
#define no_interrupts() ASM("sim")
/* enable interrupt handling */
#define interrupts() ASM("rim")
/* trigger a trap (=soft interrupt) e.g. for EMC robustness (see AN1015)*/
#define trap() ASM("trap")
/* stop code execution and wait for interrupt */
#define wait_for_interrupt() ASM("wfi")
/* return from interrupt */
#define return_from_interrupt() ASM("iret")
/* wait for event */
#define wait_for_event() ASM("wfe")
/* put controller to HALT mode */
#define halt() ASM("halt")
/* generic */
/* start critical section (see
 * https://tenbaht.github.io/sduino/developer/sdcc/#notes-on-sdcc) */
#define critical_section_start() ASM("sim")
/* end critical section (see
 * https://tenbaht.github.io/sduino/developer/sdcc/#notes-on-sdcc) */
#define critical_section_end() ASM("rim")
/* reset controller via WWGD module */
#define reset() WWDG_CR = 0xBF

/* Error check macro*/
#define check_error(e)                                                         \
  do {                                                                         \
    if ((e))                                                                   \
      return ((e));                                                            \
  } while (0)

/* Basic math */
#define max(x, y) ((x) >= (y)) ? (x) : (y)
#define min(x, y) ((x) <= (y)) ? (x) : (y)
#define abs(x) ((x) >= 0) ? (x) : (-(x))
#define square(x) ((x) * (x))
#define cube(x) ((x) * (x) * (x))

#endif /* BASIC_OPS_H_ */
