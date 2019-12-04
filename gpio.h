/*
  file gpio.h
  declaration of functions/macros for port pin control
  Optional functionality via #define:
    - USE_PORT_ISR: allow attaching user function to port interrupts (EXTIn)
    - USE_TLI_ISR:  allow attaching user function to interrupt on pin PD7 (TLI)
*/

#ifndef GPIO_H_
#define GPIO_H_

#include "basic_ops.h"
#include "stm8l151c8_wrap.h"

/*help macro for IDE completion*/

#if (1) /* dummy for filtering out in editor */

/** structure for controlling/monitoring pins in PORT mode (PORT_t) */
typedef struct {
  /* rw: in output mode, control pin logicl level */
  byte_t ODR; /* Port x output data register (Px_ODR) */
              /* readonly, 0: low logic level, 1: high logic level */
  byte_t IDR; /* Port x pin input register (Px_IDR) */
              /* rw. 0: input mode. 1 output mode */
  byte_t DDR; /* Port x data direction register (Px_DDR) */
              /* rw, 0: floating input /open drain output
                     1: pull_up input / push-pull output */
  byte_t CR1; /* Port x control register 1 (Px_CR1) */
              /* rw, 0: interupt disabled input / slow output
                                       1: interrupt enabled input / high-speed output */
  byte_t CR2; /* Port x control register 2 (Px_CR2) */
} PORT_t;

/* port A..F implemented on all devices */
#if defined(HAS_PORTA)
reg(PORTA_BaseAddress, PORT_t, PORT_A); /* registers for port A access */
#endif
#if defined(HAS_PORTB)
reg(PORTB_BaseAddress, PORT_t, PORT_B); /* registers for port B access */
#endif
#if defined(HAS_PORTC)
reg(PORTC_BaseAddress, PORT_t, PORT_C); /* registers for port C access */
#endif
#if defined(HAS_PORTD)
reg(PORTD_BaseAddress, PORT_t, PORT_D); /* registers for port D access */
#endif
#if defined(HAS_PORTE)
reg(PORTE_BaseAddress, PORT_t, PORT_E); /* registers for port E access */
#endif
#if defined(HAS_PORTF)
reg(PORTF_BaseAddress, PORT_t, PORT_F); /* registers for port F access */
#endif
#if defined(HAS_PORTG)
reg(PORTG_BaseAddress, PORT_t, PORT_G); /* registers for port G access */
#endif
#if defined(HAS_PORTH)
reg(PORTH_BaseAddress, PORT_t, PORT_H); /* registers for port H access */
#endif
#if defined(HAS_PORTI)
reg(PORTI_BaseAddress, PORT_t, PORT_I); /* registers for port I access */
#endif

/* PORT Module Reset Values (all ports) */
#define PORT_ODR_RESET_VALUE ((uint8_t)0x00)
#define PORT_DDR_RESET_VALUE ((uint8_t)0x00)
/* except PortA_CR1*/
#define PORT_CR1_RESET_VALUE ((uint8_t)0x00)
#define PORT_CR2_RESET_VALUE ((uint8_t)0x00)

/* Notice here */
#define PORTA_CR1_RESET_VALUE ((uint8_t)0x01)

#endif

/* port/pin control macros. */

typedef enum pin_mode_t {
  input_float_itr,
  input_float_no_iter,
  input_pullup_itr,
  input_pullup_no_itr,
  output_opendrain_2m,
  output_opendrain_16m,
  output_pushpull_2m,
  output_pushpull_16m
} pin_mode_t;

void pin_mode_set(PORT_t *port_ptr, uint8_t pin, pin_mode_t mode);

// pin configurations for below pinMode(). For details see 'stm8as.h'
#define INPUT 0x00 /* configure pin as: input, float, no port interrupt */
#define INPUT_INTERRUPT                                                        \
  0x01 /* configure pin as: input, float, with port interrupt */
#define INPUT_PULLUP                                                           \
  0x02 /* configure pin as: input, pull-up, no port interrupt */
#define INPUT_PULLUP_INTERRUPT                                                 \
  0x03 /* configure pin as: input, pull-up, with port interrupt */
#define OUTPUT_OPENDRAIN_SLOW                                                  \
  0x04 /* configure pin as: output, open-drain, slow (2MHz) */
#define OUTPUT_OPENDRAIN                                                       \
  0x05 /* configure pin as: output, open-drain, fast (10MHz) */
#define OUTPUT_SLOW 0x06 /* configure pin as: output, push-pull, slow (2MHz)   \
                          */
#define OUTPUT 0x07      /* configure pin as: output, push-pull, fast (10MHz) */

// edge sensitivities for port/EXINT and pin/TLI interrupts
#define LOW 0     ///< EXINT on low level (EXINT). Warning: may stall device!
#define CHANGE 1  ///< EXINT on both edges (EXINT)
#define RISING 2  ///< EXINT on rising edge (EXINT & TLI)
#define FALLING 3 ///< EXINT on falling edge (EXINT & TLI)

// direct pin read/write via bitwise access (port + "pin0"..."pin7")
#define pin_output_reg(pPort, pin)                                             \
  ((pPort)->ODR.bit.pin) ///< pin output register (1 pin)
#define pin_input_reg(pPort, pin)                                              \
  ((pPort)->IDR.bit.pin) ///< pin input register (1 pin)

// direct port read/write via bytewise access (port)
#define port_output_reg(pPort)                                                 \
  ((pPort)->ODR.byte) ///< port output register (8 pins)
#define port_input_reg(pPort)                                                  \
  ((pPort)->IDR.byte) ///< port input register (8 pins)

#define pin_output_high(pPort, pin)                                            \
  ((pPort)->ODR.byte |= (0x01 << pin)) ///< set pin output state high
#define pin_output_low(pPort, pin)                                             \
  ((pPort)->ODR.byte &= ~(0x01 << pin)) ///< set pin output state high
#define pin_output_toggle(pPort, pin)                                          \
  ((pPort)->ODR.byte ^= (0x01 << pin)) ///< toggle pin output state
#define pin_output_set(pPort, pin, state)                                      \
  (state ? pin_output_high(pPort, pin)                                         \
         : pin_output_low(pPort, pin)) ///< set pin output state
#define pin_input_read(pPort, pin)                                             \
  (((pPort)->IDR.byte & (0x01 << pin)) >> pin) ///< read pin input state

/*-----------------------------------------------------------------------------
    DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/// set I/O pin mode
void pin_set(PORT_t *port_ptr, uint8_t pin, uint8_t mode);

/// set I/O port modes
void port_mode(PORT_t *port_ptr, uint8_t dir, uint8_t cr1, uint8_t cr2);

// for external port interrupts
#if defined(USE_PORTA_ISR) || defined(USE_PORTB_ISR) ||                        \
    defined(USE_PORTC_ISR) || defined(USE_PORTD_ISR) ||                        \
    defined(USE_PORTE_ISR) || defined(USE_PORTF_ISR) || defined(USE_PORT_ISR)
/// configure edge sensitivity for EXINT
void config_edge_external_interrupt(PORT_t *port_ptr, uint8_t edge);

#endif

// for top level interrupt (=pin interrupt)
#if defined(USE_TLI_ISR)
/// configure edge sensitivity for TLI
void config_edge_toplevel_interrupt(uint8_t edge);

#endif

#endif // GPIO_H_
