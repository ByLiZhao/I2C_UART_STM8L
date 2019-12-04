/**
  \file i2c.h
  declaration of functions for I2C bus communication
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef I2C_H_
#define I2C_H_

#include "stm8l151c8_wrap.h"
#include "type_def.h"

/* ------------------------
 I2C Bus Interface (all devices)
------------------------ */
#if defined(HAS_I2C) /* if need I2C*/

/** struct containing I2C registers */
typedef struct {
  /** I2C Control register 1 (I2C_CR1) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct {                 /* bitwise access to register */
      uint8_t PE : 1;        /* Peripheral enable, 0: disable, 1: enable */
      uint8_t SMBUS : 1;     /* 0: I2C mode, 1: SMBUS mode. */
      uint8_t res : 1;       /* Reserved, forced by hardware to 0. */
      uint8_t SMBTYPE : 1;   /* 0: SMBUS device, 1: SMBUS host. */
      uint8_t ENARP : 1;     /* address recognized, 0: disabled, 1: enabled */
      uint8_t ENPEC : 1;     /* 0: PEC calculation disabled, 1: enabled. */
      uint8_t ENGC : 1;      /* General call enable, 0: disabled, 1: enabled. */
      uint8_t NOSTRETCH : 1; /* Clock stretching disable (Slave mode). 0:
                                enabled, 1: disabled. */
    } reg;
  } CR1;

  /** I2C Control register 2 (I2C_CR2) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct {             /* bitwise access to register */
      uint8_t START : 1; /* Start generation. master 0: start gen. 1: continous
                            start gen. */
      /* slave 0: no start generation, 1: start gen when bus is free */
      uint8_t STOP : 1; /* Stop generation. 0: no stop generation. master 1:
                           stop gen. salve 1 :release lines */
      uint8_t ACK : 1;  /* Acknowledge enable. 0: no ack, 1: ack returned when
                           byte received. */
      uint8_t POS : 1;  /* Acknowledge position (for data reception). 0: current
                           byte, 1: next byte */
      uint8_t PEC : 1;  /* packet error checking. 0: no PEC transfer, 1: PEC
                           transfer. */
      uint8_t ALERT : 1; /* SMBus alert. 0: Releases SMBAlert pin high. 1:
                            Drives SMBAlert pin low */
      uint8_t res : 1;   /* Reserved */
      uint8_t SWRST : 1; /* Software reset 0: Peripheral not at reset state */
    } reg;
  } CR2;

  /** I2C Frequency register (I2C_FREQR) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct {            /* bitwise access to register */
      uint8_t FREQ : 6; /* Peripheral clock frequency 1M Hz - 16M Hz */
      uint8_t res : 2;  /* Reserved */
    } reg;
  } FREQR;

  /** I2C Own address register LSB (I2C_OARL) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct {            /* bitwise access to register */
      uint8_t ADD0 : 1; /* Interface address [0] (in 10-bit address mode only)
                           7-bit addressing mode don't care */
      uint8_t ADD : 7;  /* Interface address [7:1] */
    } reg;
  } OARL;

  /** I2C own address high (I2C_OARH) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct {           /* bitwise access to register */
      uint8_t res : 1; /* Reserved */
      uint8_t
          ADD : 2; /* Interface address [9:8] (in 10-bit address mode only) */
      uint8_t res2 : 3;    /* Reserved */
      uint8_t ADDCONF : 1; /* Address mode configuration, must always be 1 */
      uint8_t ADDMODE : 1; /* 7-/10-bit addressing mode (Slave mode) 0: 7bit
                              slave 1: 10 bit slave */
    } reg;
  } OARH;

  /**I2C register for dul mode*/
  union {
    uint8_t byte;
    struct {
      uint8_t ENDUAL : 1; /* Dual addressing mode enable 0: use only UAR1, 1:
                             use both OAR1 and OAR2. */
      uint8_t ADD2 : 7;   /* address 2. */
    } reg;
  } OAR2;

  /** I2C data register (I2C_DR) */
  union {
    uint8_t byte;
    byte_t bits;
  } DR;

  /** I2C Status register 1 (I2C_SR1) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct {          /* bitwise access to register */
      uint8_t SB : 1; /* Start bit (Mastermode) 0: no start condition, 1: start
                         condition generated */
      /* Cleared by software by reading the SR1 register followed by writing the
       * DR register, */
      uint8_t ADDR : 1; /* Address sent (master mode) / matched (slave mode) */
      /* master 0: no end of address transmission. 1: end of address
       * transmission */
      /* slave 0: address mismatched or not received. 1: received address
       * matched. */
      uint8_t BTF : 1;   /* Byte transfer finished. 0: not done, 1: finished. */
      uint8_t ADD10 : 1; /* 10-bit header sent (Master mode) */
      uint8_t STOPF : 1; /* Stop detection (Slave mode) 0: no stop condition
                            detected. 1: stop condition detected */
      uint8_t res : 1;   /* Reserved */
      uint8_t RXNE : 1;  /* Data register not empty (receivers) 0: data register
                            empty, 1: data register not empty */
      uint8_t TXE : 1;   /* Data register empty (transmitters) 0: data register
                            NOT empty, 1: empty */
    } reg;
  } SR1;

  /** I2C Status register 2 (I2C_SR2) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct {            /* bitwise access to register */
      uint8_t BERR : 1; /* Bus error. 0: no error, 1: bus error */
      uint8_t ARLO : 1; /* Arbitration lost (master mode) 0: no arbitration lost
                           1: arbitration lost detected */
      uint8_t AF : 1;   /* Acknowledge failure */
      uint8_t OVR : 1;  /* Overrun/underrun */
      uint8_t PECERR : 1;   /* PEC Error in reception */
      uint8_t WUFH : 1;     /* Wakeup from Halt */
      uint8_t TIMEOUT : 1;  /* SCL time out, remain low more than 25ms. */
      uint8_t SMBALERT : 1; /* SMBus alert */
    } reg;
  } SR2;

  /** I2C Status register 3 (I2C_SR3) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct {           /* bitwise access to register */
      uint8_t MSL : 1; /* Master/Slave 0: slave, 1: master, set by hardware */
      /* Cleared by hardware when detect a stop condition, or lose arbitration.
       */
      uint8_t BUSY : 1; /* Bus busy. 0: dile, 1: busy */
      uint8_t TRA : 1;  /* Transmitter/Receiver 0: received, 1: transmitted. */
      uint8_t res : 1;  /* Reserved */
      uint8_t GENCALL : 1; /* General call header (Slavemode) */
      uint8_t res2 : 2;    /* Reserved */
      uint8_t DUALF : 1;   /* Duall flag (slave mode). */
    } reg;
  } SR3;

  /** I2C Interrupt registerstm8 i2c (I2C_ITR) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct { /* bitwise access to register */
      uint8_t
          ITERREN : 1; /* Error interrupt enable 0: disable. 1: enable. rw */
      uint8_t ITEVTEN : 1; /* Event interrupt enable */
      uint8_t ITBUFEN : 1; /* Buffer interrupt enable */
      uint8_t DMAEN : 1;   /* DMA requests enable, 0: disable, 1: enable */
      uint8_t LAST : 1;    /* DMA last transfer 0: Next DMA EOT is not the last
                              transfer */
      uint8_t res : 3;     /* Reserved */
    } reg;
  } ITR;

  /** I2C Clock control register low (I2C_CCRL) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct {           /* bitwise access to register */
      uint8_t CCR : 8; /* Clock control register (Master mode) */
    } reg;
  } CCRL;

  /** I2C Clock control register high (I2C_CCRH) */
  union { /// bytewise access to register */
    uint8_t byte;
    struct {            /// bitwise access to register */
      uint8_t CCR : 4;  /* Clock control register in Fast/Standard mode (Master
                           mode) */
      uint8_t res : 2;  /* Reserved */
      uint8_t DUTY : 1; /* Fast mode duty cycle. 0 t_low/t_high =2. 1:
                           t_low/t_high = 16/9 */
      uint8_t FS : 1;   /* I2C master mode selection 0: standard mode. 1: fast
                           mode. */
    } reg;
  } CCRH;

  /** I2C TRISE register (I2C_TRISER) */
  union { /* bytewise access to register */
    uint8_t byte;
    struct { /* bitwise access to register */
      uint8_t
          TRISE : 6; /* Maximum rise time in Fast/Standard mode (Master mode) */
      uint8_t res : 2; /* Reserved */
    } reg;
  } TRISER;

  /** Packet error checking register*/
  union {
    uint8_t byte;
    struct {
      uint8_t PEC : 8;
    } reg;
  } PECR;
} I2C_t;

/* pointer to all I2C registers (all devices) */
reg(I2C_BaseAddress, I2C_t, I2C);
/* I2C Module Reset Values */
#define I2C_CR1_RESET_VALUE ((uint8_t)0x00)
#define I2C_CR2_RESET_VALUE ((uint8_t)0x00)
#define I2C_FREQR_RESET_VALUE ((uint8_t)0x00)
#define I2C_OARL_RESET_VALUE ((uint8_t)0x00)
#define I2C_OARH_RESET_VALUE ((uint8_t)0x00)
#define I2C_OAR2_RESET_VALUE ((uint8_t)0x00)
#define I2C_DR_RESET_VALUE ((uint8_t)0x00)
#define I2C_SR1_RESET_VALUE ((uint8_t)0x00)
#define I2C_SR2_RESET_VALUE ((uint8_t)0x00)
#define I2C_SR3_RESET_VALUE ((uint8_t)0x00)
#define I2C_ITR_RESET_VALUE ((uint8_t)0x00)
#define I2C_CCRL_RESET_VALUE ((uint8_t)0x00)
#define I2C_CCRH_RESET_VALUE ((uint8_t)0x00)
#define I2C_TRISER_RESET_VALUE ((uint8_t)0x02)

/*I2C functions */
/* configure I2C bus as master in standard mode */
void i2c_init(void);

/* wait until bus is free */
uint8_t i2c_wait_til_free(void);

/* generate I2C start condition */
uint8_t i2c_gen_start(void);

/* generate I2C stop condition */
uint8_t i2c_gen_stop(void);

/* write data via I2C */
uint8_t i2c_write(uint8_t addr, uint8_t offset, uint8_t numTx, uint8_t *Tx);

/* request data via I2C as master */
uint8_t i2c_read(uint8_t addr, uint8_t offset, uint8_t numRx, uint8_t *Rx);

#endif /* HAS_I2C */
#endif /* I2C_H_ */
