/* ------------------------------------------------------ */
/* File Description */
/* ------------------------------------------------------ */
/*
  file usart.h
  declaration of functions for USART communication
*/
/* ------------------------------------------------------ */
/* File Description */
/* ------------------------------------------------------ */

#ifndef USART_H_
#define USART_H_

#include "basic_ops.h"
#include "stm8l151c8_wrap.h"
#include "type_def.h"

/* ------------------------------------------------------ */
/* Macro Defitnion */
/* ------------------------------------------------------ */
#if (1)
typedef struct {
  union {
    uint8_t byte;
    struct {
      /* r: cleared by reading this register followed by reading DR
                      1 means parity error */
      uint8_t PE : 1;
      /* r: cleared by reading this register followed by reading DR
      1 means frame error (de-synchronization, excessive noise
      or a break character)*/
      uint8_t FE : 1;
      /* r: cleared by reading this register followed by reading DR
      1 means noise is dected */
      uint8_t NF : 1;
      /* r: cleared by reading this register followed by reading DR
      1 means shift register will be overwritten
      an interrupt will occur when CR2.RIEN is set */
      uint8_t OR : 1;
      /* r: cleared by reading this register followed by reading DR
      1 means idle line is detected
      an interrupt will occur when CR2.ILIEN is set */
      uint8_t IDLE : 1;
      /* r: cleared by by reading DR
      1 means DR is updated thus ready to be read
      an interrupt will occur when CR2.RIEN is set */
      uint8_t RXNE : 1;
      /*rc_w0: cleared by reading this register followed by writing DR
      1 means transmission has completed
      an interrupt will occur when CR2.TCIEN is set */
      uint8_t TC : 1;
      /*r: cleared by writing DR
      1 means content of DR has been moved to shift register
      an interrupt will occur when CR2.TIEN is set */
      uint8_t TXE : 1;
    } reg;
  } SR;

  /*when receive, acts as RDR
    when transmit, acts as TDR */
  byte_t DR;

  /* USART_DIV[11: 4] */
  byte_t BRR1;

  /* Bits 7:4 USART_DIV[15: 12]
          Bit 3:0 USART_DIV[3: 0] */
  byte_t BRR2;

  union {
    uint8_t byte;
    struct {
      /*rw: 0: parity interrupt disable 1: enabled */
      uint8_t PIEN : 1;
      /*rw: 0: even parity, 1:odd parity*/
      uint8_t PS : 1;
      /*rw: 0: parity disable, 1: parity enable */
      uint8_t PCEN : 1;
      /*rw: 0: wakeup on idle line, 1: wakeup on address mark */
      uint8_t WAKE : 1;
      /*rw: 0: 8-bit format, 1: 9-bit format */
      uint8_t M : 1;
      /*0: USART enable, 1:USART prescaler and outputs disable */
      uint8_t USARTD : 1;
      /* 9th bit of transmitted data*/
      uint8_t T8 : 1;
      /* 9th bit of received data */
      uint8_t R8 : 1;
    } reg;
  } CR1;

  union {
    uint8_t byte;
    struct {
      /* rw: 1 means break character will be transmitted */
      uint8_t SBK : 1;
      /* rw: 1 means receive in mute mode */
      uint8_t RWU : 1;
      /* rw: 1 means receive eanble */
      uint8_t REN : 1;
      /* rw: 1 means transmitter enable */
      uint8_t TEN : 1;
      /* rw: means IDLE line interrupt enable */
      uint8_t ILIEN : 1;
      /* rw: 1 means Receive interupt enable */
      uint8_t RIEN : 1;
      /* rw: 1 means transmission complete interrupt enable */
      uint8_t TCIEN : 1;
      /* rw: 1 means content of TDR moved to shift register */
      uint8_t TIEN : 1;
    } reg;
  } CR2;

  union {
    uint8_t byte;
    /* CPOL, CPHA, LBCL should not be written
            while the transmitter is enabled */
    struct {
      /* rw: 1 means the clock pulse of the last bit is output to USART_CK pin*/
      uint8_t LBCL : 1;
      /*rw: 0: the first clock transition is the first data capture edge
1: the sencond clock transition is the first data capture edge */
      uint8_t CPHA : 1;
      /* rw: 0: USART_CK to 0 when idle
1: USART_CK to 1 when dile */
      uint8_t CPOL : 1;
      /* rw: 0: USART_CK pin disabled
      1: USART_CK pin enabled */
      uint8_t CLKEN : 1;
      /* rw: 00: 1 stop bits,
01: reserved,
10: 2 stop bits
11: 1.5 stop bits */
      uint8_t STOP : 2;
      uint8_t Reserved : 1;
    } reg;
  } CR3;

  union {
    uint8_t byte;
    struct {
      /* address of the USART node */
      /* This is used in multiprocessor communication during mute mode,
               for wakeup with address mark detection */
      uint8_t ADD : 4;
      uint8_t Reserved : 1;
    } reg;
  } CR4;

  union {
    uint8_t byte;
    struct {
      /* rw: 1 means error interrupt enabled
      an interrupt is generated when CR5.DMAR=1
      and SR.FE=1 or SR.OR=1 or SR.NF=1 */
      uint8_t EIE : 1;
      /* rw: 1 means IrDA enabled */
      uint8_t IREN : 1;
      /* rw: 1 means low power mode IrDA mode */
      uint8_t IRLP : 1;
      /* rw: 1 means half duplex mode is selected */
      uint8_t HDSEL : 1;
      /* rw: NACK transmission during parity error is enabled */
      uint8_t NACK : 1;
      /* rw: 1 means smartcard mode enabled */
      uint8_t SCEN : 1;
      /* rw: 1 means DMA mode is enabled for reception */
      uint8_t DMAR : 1;
      /* rw: 1 means DMA mode is enabled for transmission */
      uint8_t DMAT : 1;
    } reg;
  } CR5;

  /* rw: Guard time in terms of baud clocks
used in smartcard mode,
The Transmission complete flag is set after this value */
  byte_t GTR;

  /*rw: PSC[7:0] is prescaler in IrDA low power mode
PSC[4:0] is prescaler in smartcard mode */
  byte_t PSC;
} USART_t;

#define USART_SR_RESET_VALUE ((uint8_t)0xC0)
#define USART_BRR1_RESET_VALUE ((uint8_t)0x00)
#define USART_BRR2_RESET_VALUE ((uint8_t)0x00)
#define USART_CR1_RESET_VALUE ((uint8_t)0x00)
#define USART_CR2_RESET_VALUE ((uint8_t)0x00)
#define USART_CR3_RESET_VALUE ((uint8_t)0x00)
#define USART_CR4_RESET_VALUE ((uint8_t)0x00)
#define USART_CR5_RESET_VALUE ((uint8_t)0x00)
#define USART_GTR_RESET_VALUE ((uint8_t)0x00)
#define USART_PSC_RESET_VALUE ((uint8_t)0x00)

#endif /* 1 */

#if defined(HAS_USART1)

reg(USART1_BaseAddress, USART_t, USART1);

#endif /* HAS_USART1 */

#if defined(HAS_USART2)

reg(USART2_BaseAddress, USART_t, USART2);

#endif /* HAS_USART2 */

#if defined(HAS_USART3)

reg(USART3_BaseAddress, USART_t, USART3);

#endif /* HAS_USART3 */

/* ------------------------------------------------------ */
/* End of Macro Defitnion */
/* ------------------------------------------------------ */

/* ------------------------------------------------------ */
/* Function Declaration */
/* ------------------------------------------------------ */

/* initialize USART1
 return 0 means success
 called before any other functions are called
 8-bit mode, 1 STOP, no parity check, with interrupt TXE */
int8_t usart1_init(void);

/* close usart
 return 0 means success
 called after any other functions are called.*/
int8_t usart1_close(void);

/* send buffer content via USART1
   return 0 means success
         block before finished */
int8_t usart1_write(int8_t *bufTx, uint8_t numTx);

/* send buffer conternt via USART, activated on TXE interrupt
   return 1 means contents have been put in a buffer to be sent,
   does not block	*/
int8_t usart1_write_itr(int8_t *bufTx, uint8_t numTx);

@far @interrupt void on_USART1_TXE_interrupt(void);

/* test functions */
void usart1_test(void); /* test usart1_write();*/
void usart1_itr_test(void);

/* ------------------------------------------------------ */
/* End of Function Declaration */
/* ------------------------------------------------------ */

#endif /* USART_H_ */
