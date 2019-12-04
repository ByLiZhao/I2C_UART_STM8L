/**
  \file gpio.c
  implementation of functions/macros for port pin control
  Optional functionality via #define:
    - USE_PORT_ISR: allow attaching user function to port interrupts (EXTIn)
    - USE_TLI_ISR:  allow attaching user function to interrupt on pin PD7 (TLI)
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "gpio.h"
#include "basic_ops.h"
#include "type_def.h"

/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

void pin_mode_set(PORT_t *port_ptr, uint8_t pin, pin_mode_t mode) {
  switch (mode) {
  case input_float_itr:
    bit_clear(port_ptr->DDR.byte, pin);
    bit_clear(port_ptr->CR1.byte, pin);
    bit_set(port_ptr->CR2.byte, pin);
    break;
  case input_float_no_iter:
    bit_clear(port_ptr->DDR.byte, pin);
    bit_clear(port_ptr->CR1.byte, pin);
    bit_clear(port_ptr->CR2.byte, pin);
    break;
  case input_pullup_itr:
    bit_clear(port_ptr->DDR.byte, pin);
    bit_set(port_ptr->CR1.byte, pin);
    bit_set(port_ptr->CR2.byte, pin);
    break;
  case input_pullup_no_itr:
    bit_clear(port_ptr->DDR.byte, pin);
    bit_set(port_ptr->CR1.byte, pin);
    bit_clear(port_ptr->CR2.byte, pin);
    break;
  case output_opendrain_2m:
    bit_set(port_ptr->DDR.byte, pin);
    bit_clear(port_ptr->CR1.byte, pin);
    bit_clear(port_ptr->CR2.byte, pin);
    break;
  case output_opendrain_16m:
    bit_set(port_ptr->DDR.byte, pin);
    bit_clear(port_ptr->CR1.byte, pin);
    bit_set(port_ptr->CR2.byte, pin);
    break;
  case output_pushpull_2m:
    bit_set(port_ptr->DDR.byte, pin);
    bit_set(port_ptr->CR1.byte, pin);
    bit_clear(port_ptr->CR2.byte, pin);
    break;
  case output_pushpull_16m:
    bit_set(port_ptr->DDR.byte, pin);
    bit_set(port_ptr->CR1.byte, pin);
    bit_set(port_ptr->CR2.byte, pin);
    break;
  default:
    break;
  }
  return;
}

/**
  \fn void pin_set(PORT_t * port_ptr, uint8_t pin, uint8_t mode)

  \brief configure pin mode

  \param[in]  port_ptr  pointer to port, e.g. &PORT_A
  \param[in]  pin    		pin to configure (0..7)
  \param[in]  mode   		new mode, e.g. INPUT_PULLUP (see pin_set.h)

  Configure pin as input or output mode with various options, e.g. pull-up or
  open-drain. See gpio.h for supported modes.
*/
void pin_set(PORT_t *port_ptr, uint8_t pin, uint8_t mode) {

  /* set pin mode */
  /* input(=0) or output(=1) */
  bit_write(port_ptr->DDR.byte, pin, (uint8_t)(mode & 0x04));
  /* input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull */
  bit_write(port_ptr->CR1.byte, pin, (uint8_t)(mode & 0x02));
  /* input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope	*/
  bit_write(port_ptr->CR2.byte, pin, (uint8_t)(mode & 0x01));
} // pinMode

/**
  \fn void port_mode(PORT_t *pPort, uint8_t dir, uint8_t cr1, uint8_t cr2)

  \brief configure pin mode

  \param[in]  port_ptr  pointer to port, e.g. &PORT_A
  \param[in]  dir    		bitmask for pin directions: input(=0) or
  output(=1)
  \param[in]  cr1    		bitmask for pin options 1: input: 0=float,
  1=pull-up; output: 0=open-drain, 1=push-pull
  \param[in]  cr2    		bitmask for pin options 2: input: 0=no exint,
  1=exint; output: 0=2MHz slope, 1=10MHz slope

  Configure port pins as input or output mode with various options, e.g. pull-up
  or open-drain. See gpio.h for supported modes.
*/
void port_mode(PORT_t *port_ptr, uint8_t dir, uint8_t cr1, uint8_t cr2) {

  // set port pin modes
  port_ptr->DDR.byte = dir; /* input(=0) or output(=1) */
  port_ptr->CR1.byte =
      cr1; /* input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull */
  port_ptr->CR2.byte =
      cr2; /* input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope */

} // portMode

#if defined(USE_PORTA_ISR) || defined(USE_PORTB_ISR) ||                        \
    defined(USE_PORTC_ISR) || defined(USE_PORTD_ISR) ||                        \
    defined(USE_PORTE_ISR) || defined(USE_PORTF_ISR) || defined(USE_PORT_ISR)

/**
  \fn void config_edge_external_interrupt(PORT_t *pPort, uint8_t edge)

  \brief configure edge sensitivity for EXINT

  \param[in]  pPort  pointer to port to configure, e.g. &PORT_A (see stm8as.h)
  \param[in]  edge   port edge sensitivity. Supported sensitivities are
    - LOW         interrupt on low level. Warning: may stall device!
    - CHANGE      interrupt on both edges
    - RISING      interrupt on rising edge
    - FALLING     interrupt on falling edge
    - NOCHANGE    keep current sensitivity

  configure edge sensitivity for port interrupt service routine.
  For available ports see datasheet of device.
  \note
    pin interrupts are not en-/disabled by this routine. Use pinMode() in module
  gpio instead
*/
void config_edge_external_interrupt(PORT_t *port_ptr, uint8_t edge) {

  uint8_t tmp;

  // disable port interrupts for input pins while modifying
  // for output pins slope is modified temporarily, which is uncritical
  tmp = port_ptr->CR2.byte;
  port_ptr->CR2.byte = port_ptr->DDR.byte;

  // configure port A ISR
  if (port_ptr == &PORT_A) {

    // set new edge sensitivity
    if (edge == LOW)
      EXTI.CR1.reg.PAIS = 0;
    else if (edge == RISING)
      EXTI.CR1.reg.PAIS = 1;
    else if (edge == FALLING)
      EXTI.CR1.reg.PAIS = 2;
    else if (edge == CHANGE)
      EXTI.CR1.reg.PAIS = 3;

  } // port A

  // configure port B ISR
  else if (port_ptr == &PORT_B) {

    // set new edge sensitivity
    if (edge == LOW)
      EXTI.CR1.reg.PBIS = 0;
    else if (edge == RISING)
      EXTI.CR1.reg.PBIS = 1;
    else if (edge == FALLING)
      EXTI.CR1.reg.PBIS = 2;
    else if (edge == CHANGE)
      EXTI.CR1.reg.PBIS = 3;

  } // port B

  // configure port C ISR
  else if (port_ptr == &PORT_C) {

    // set new edge sensitivity
    if (edge == LOW)
      EXTI.CR1.reg.PCIS = 0;
    else if (edge == RISING)
      EXTI.CR1.reg.PCIS = 1;
    else if (edge == FALLING)
      EXTI.CR1.reg.PCIS = 2;
    else if (edge == CHANGE)
      EXTI.CR1.reg.PCIS = 3;

  } // port C

  // configure port D ISR
  else if (port_ptr == &PORT_D) {

    // set new edge sensitivity
    if (edge == LOW)
      EXTI.CR1.reg.PDIS = 0;
    else if (edge == RISING)
      EXTI.CR1.reg.PDIS = 1;
    else if (edge == FALLING)
      EXTI.CR1.reg.PDIS = 2;
    else if (edge == CHANGE)
      EXTI.CR1.reg.PDIS = 3;

  } // port D

  // configure port E ISR
  else if (port_ptr == &PORT_E) {

    // set new edge sensitivity
    if (edge == LOW)
      EXTI.CR2.reg.PEIS = 0;
    else if (edge == RISING)
      EXTI.CR2.reg.PEIS = 1;
    else if (edge == FALLING)
      EXTI.CR2.reg.PEIS = 2;
    else if (edge == CHANGE)
      EXTI.CR2.reg.PEIS = 3;

  } // port E

  // configure port F ISR
  else if (port_ptr == &PORT_F) {

    // set new edge sensitivity
    if (edge == LOW)
      EXTI.CR2.reg.PFIS = 0;
    else if (edge == RISING)
      EXTI.CR2.reg.PFIS = 1;
    else if (edge == FALLING)
      EXTI.CR2.reg.PFIS = 2;
    else if (edge == CHANGE)
      EXTI.CR2.reg.PFIS = 3;
  } // port F

  // configure port G ISR
  else if (port_ptr == &PORT_G) {

    // set new edge sensitivity
    if (edge == LOW)
      EXTI.CR2.reg.PFIS = 0;
    else if (edge == RISING)
      EXTI.CR2.reg.PFIS = 1;
    else if (edge == FALLING)
      EXTI.CR2.reg.PFIS = 2;
    else if (edge == CHANGE)
      EXTI.CR2.reg.PFIS = 3;

  } // port G

  // configure port H ISR
  else if (port_ptr == &PORT_H) {

    // set new edge sensitivity
    if (edge == LOW)
      EXTI.CR2.reg.PFIS = 0;
    else if (edge == RISING)
      EXTI.CR2.reg.PFIS = 1;
    else if (edge == FALLING)
      EXTI.CR2.reg.PFIS = 2;
    else if (edge == CHANGE)
      EXTI.CR2.reg.PFIS = 3;

  } // port H

  // configure port I ISR
  else if (port_ptr == &PORT_I) {

    // set new edge sensitivity
    if (edge == LOW)
      EXTI.CR2.reg.PFIS = 0;
    else if (edge == RISING)
      EXTI.CR2.reg.PFIS = 1;
    else if (edge == FALLING)
      EXTI.CR2.reg.PFIS = 2;
    else if (edge == CHANGE)
      EXTI.CR2.reg.PFIS = 3;

  } // port I
  // restore original port interrupt setting
  port_ptr->CR2.byte = tmp;

} // configEdgeExint

#endif // USE_PORT_ISR

#if defined(USE_TLI_ISR)

/**
  \fn void config_edge_toplevel_interrupt(uint8_t edge)

  \brief configure edge sensitivity for TLI

  \param[in]  edge      edge sensitivity. Supported sensitivities are
    - RISING      interrupt on rising edge
    - FALLING     interrupt on falling edge

  configure edge sensitivity for pin interrupt (TLI) service routine.
  For available pins see datasheet of device.
  \note
    pin interrupts are not en-/disabled by this routine. Use pinMode() in module
  gpio instead
*/
void config_edge_toplevel_interrupt(uint8_t edge) {

  uint8_t tmp;

  // disable port D interrupts (TLI=PD7) while modifying
  // for output pins slope is modified temporarily, which is uncritical
  tmp = PORT_D.CR2.byte;

  // set TLI edge sensitivity (only rising and falling)
  if (edge == RISING)
    EXTI.CR2.reg.TLIS = 1;
  else
    EXTI.CR2.reg.TLIS = 0;

  // restore original port interrupt setting
  PORT_D.CR2.byte = tmp;

} // config_edge_toplevel_interrupt

#endif // USE_TLI_ISR
