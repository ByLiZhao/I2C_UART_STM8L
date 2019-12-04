/**
  \file i2c.c
  implementation of functions for I2C bus communication
*/

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "type_def.h"
#include "stm8l151c8_wrap.h"
#include "i2c.h"
#include "gpio.h"


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void i2c_init(void)
   
  \brief configure I2C bus
  configure I2C bus as master in standard mode, BR=100kHz, 7bit address, 250ns rise time
*/
void i2c_init() {//init mcu as master
	uint16_t count = 10000;
	uint16_t i =0;
	
	/* set clock */
	#define CLK_CKDIVR_HSIDIV    ((uint8_t)0x18) /*!< High speed internal clock prescaler */
	#define HSIPrescaler				 ((uint8_t)0x00) /*system clock source /1 */
	#define CLOCK_I2C						 ((uint8_t)0x08) /*bit mask to enable I2C clock */
	/* Clear High speed internal clock prescaler */
	//CLK_CKDIVR &= (~CLK_CKDIVR_HSIDIV);
	//CLK_CKDIVR |= HSIPrescaler;
	
	CLK_PCKENR1|= CLOCK_I2C; /*enable I2C clock */
	
	/*pin PD6 is SD, PD7 is INTB */
	//pin_set(&PORT_D, 6, OUTPUT_OPENDRAIN_SLOW);
  //pin_set(&PORT_D, 7, INPUT_INTERRUPT);// push-pull output in sensor's side
	//PORT_D.CR2.byte = 0x80; //for buggy compiler
	
	pin_mode_set(&PORT_D, 6, output_opendrain_2m);
	pin_mode_set(&PORT_D, 7, input_float_itr);

	
  /* configure I2C pins PC1(=SCL) and PC0(=SDA) */
  //pin_set(&PORT_C, 1, OUTPUT_OPENDRAIN);
  //pin_set(&PORT_C, 0, OUTPUT_OPENDRAIN);
	//PORT_C.CR2.byte = 0x03; // for buggy compiler
	
	pin_mode_set(&PORT_C, 1, output_opendrain_16m);
	pin_mode_set(&PORT_C, 0, output_opendrain_16m);
	
	/* pull down the SD pin to start the sensor */
	PORT_D.ODR.bit.b6 = 0; 
	
	
	/*reset i2c bus*/
	I2C.CR1.byte = I2C_CR1_RESET_VALUE; /* Peripheral disabled, general call diaabled, no clock stretching */
	I2C.CR2.byte = I2C_CR2_RESET_VALUE; /* reset I2C bus */
	I2C.FREQR.byte = I2C_FREQR_RESET_VALUE;
	I2C.OARL.byte = I2C_OARL_RESET_VALUE;
	I2C.OARH.byte = I2C_OARH_RESET_VALUE;
	I2C.DR.byte = I2C_DR_RESET_VALUE;
	I2C.SR1.byte = I2C_SR1_RESET_VALUE;
	I2C.SR2.byte = I2C_SR2_RESET_VALUE;
  I2C.SR3.byte = 	I2C_SR3_RESET_VALUE;
	I2C.ITR.byte =  I2C_ITR_RESET_VALUE;
	I2C.CCRL.byte = I2C_CCRL_RESET_VALUE;
	I2C.CCRH.byte = I2C_CCRH_RESET_VALUE;
	I2C.TRISER.byte = I2C_TRISER_RESET_VALUE;
	
	
  /* init I2C bus */   
  I2C.CR2.reg.ACK       = 1;   //set ACK, but no way to suceed, don't know why
  I2C.FREQR.reg.FREQ    = 16;       /* peripheral clock = 16MHz (f=val*1MHz) */
  I2C.OARH.reg.ADDCONF  = 1;        /* This must be set to 1 always. */
	I2C.OARH.reg.ADDMODE	= 0;				/* 7-bit mode if as slave */
	
  I2C.OARL.reg.ADD      = 0x04;     /* set own 7b addr to 0x04 */
  I2C.SR2.byte          = 0x00;			/* start with no error */
  I2C.CCRL.reg.CCR      = 0x50;     /* 80, SCL frequency = 100KHz, 80 = 16M/(100K *2) */
  I2C.CCRH.byte         = 0x00;     /* I2C standard mode */
  I2C.TRISER.reg.TRISE  = 0x11;     /* 17, t_master = 1000/16=62.5ns. (1000/62.5)+1. 1000ns is maximum allowed rising time */
	I2C.CR1.reg.ENGC			= 0;				/* general call disabled. */
	I2C.CR1.reg.NOSTRETCH = 1;				/* clock strench disabled, sensor does not use strench */
	for(i=0; i<4; ++i){
		count = 10000;
	while(--count);}
	I2C.CR1.reg.PE        = 1;        /* enable Peripheral */
} /* i2c_init */



/**
  \fn uint8_t i2c_waitFree(void)
   
  \brief wait until bus is free
  \return error code (0=ok; 1=timeout)
  wait until bus is free with timeout
*/
uint8_t i2c_wait_til_free() {

  uint16_t  countTimeout;     /* use counter for timeout to minimize dependencies */
  
  /* wait until bus free with timeout */
  countTimeout = 10000;                           /* ~1.1us/inc -> ~10ms */
  while ((I2C.SR3.reg.BUSY) && (--countTimeout));
  /* on I2C timeout return error */
  if (I2C.SR3.reg.BUSY) {
    return(1);
  }
  
  /* return success */
  return(0);

} /* i2c_wait_til_free */



/**
  \fn uint8_t i2c_start(void)
   
  \brief generate I2C start condition
  \return error code (0=ok; 1=timeout)
  generate I2C start condition with timeout
*/
uint8_t i2c_gen_start() {
  uint16_t  countTimeout;     /* use counter for timeout to minimize dependencies */
	countTimeout = 10000; 
	
  /* generate start condition with timeout */
	I2C.CR2.reg.START = 1;
	//                      
  while ((!I2C.SR1.reg.SB) && (--countTimeout));
	
  /* on I2C timeout return error */
  if (!I2C.SR1.reg.SB) {
    return(1);
  }
  /* return success */
  return(0);

} /* i2c_gen_start */



/**
  \fn uint8_t i2c_stop(void)
   
  \brief generate I2C stop condition
  \return error code (0=ok; 1=timeout)
  generate I2C stop condition with timeout 
*/
uint8_t i2c_gen_stop() {

  uint16_t  countTimeout;     /* use counter for timeout to minimize dependencies */

  /* generate stop condition with timeout */
  I2C.CR2.reg.STOP = 1;
  countTimeout = 10000;                           /* ~1.1us/inc -> ~10ms */
  while ((I2C.SR3.reg.MSL) && (--countTimeout));
 
  /* on I2C timeout set error flag */
  if (I2C.SR3.reg.MSL) {
    return(1);
  }
  
  /* return success */
  return(0);

} /* i2c_gen_stop */



/**
  \fn uint8_t i2c_write(uint8_t addr, uint8_t offset, uint8_t numTx, uint8_t *bufTx)
	\ only support 7-bit address mode
  \brief write data via I2C
  \param[in]  addr        7b address [6:0] of I2C slave
	\param[in]  offset			offest or register address of the slave device.
  \param[in]  numTx       number of bytes to send
  \param[in]  bufTx       send buffer
  \return error code (0=ok; 1=timeout)
  write data to I2C slave with frame timeout. Note that no start or 
  stop condition is generated.
*/
uint8_t i2c_write(uint8_t addr, uint8_t offset, uint8_t numTx, uint8_t *bufTx) {
	uint8_t   i = 0;
	uint16_t  countTimeout = 10000;     /* use counter for timeout to minimize dependencies */
	uint8_t temp = 0;
	
	I2C.CR2.reg.ACK = 0;																/*disable ack*/
	I2C.CR1.reg.NOSTRETCH = 1;				/* clock strench disabled */
	
	while(i2c_wait_til_free()){	/* pull busy bit */
		return 1;
		/* wait until the i2c bus is free */
	}
	
	if(i2c_gen_start()){	/*generate START condition. */
		return (2);	/* return ERROR if START condition cannot be generated. */
	}

  /* send 7b slave adress [7:1] + write flag (LSB=0) */
	I2C.DR.byte = (uint8_t) ((addr << 1) & (~0x01));
	while ((!I2C.SR1.reg.ADDR) && (--countTimeout));     		/* wait until address sent or timeout */
	if(countTimeout==0) 
		return(3);
	temp = I2C.SR3.byte; //clear ADDR by reading SR1 then SR3
	
	/*send offset, the sensor register address to be written to */
	countTimeout = 10000;
	while((!I2C.SR1.reg.TXE) && (--countTimeout));					/* TXE==0 means not empty */
	if(!I2C.SR1.reg.TXE) 
		return(5);
	I2C.DR.byte = offset;	/* send offset; */
	
	/* send data load */
	if(numTx!=0 && bufTx != NULL){/* data needs to be sent */
		for (i=0; i<numTx; i++) {
			/*check offset has been sent*/
			countTimeout = 10000;                             // ~1.1us/inc -> ~1ms */
			while((!I2C.SR1.reg.TXE) && (--countTimeout));
				if(!I2C.SR1.reg.TXE) /* previous byte has not been sent */
					return (8);
			I2C.DR.byte = bufTx[i];
			/* while check if byte transfer succeed */
		}/* for */
		
		/* check last byte sent*/
		countTimeout = 10000;                             // ~1.1us/inc -> ~1ms */
		while((!I2C.SR1.reg.TXE) && (--countTimeout));
				if(!I2C.SR1.reg.TXE) /* last byte has not been sent */
					return(9);
		/*all data sent, gen stop condition*/
		if(i2c_gen_stop()){	/*generate STOP condition. */
			return (10);	/*return ERROR if STOP condition cannot be generated. */
		}/*if gen_stop */
	}
  return(0);
} /* i2c_write */

/**
  \fn uint8_t i2c_read(uint8_t slaveAddr, uint8_t offest, uint8_t numRx, uint8_t *bufRx)
   
  \brief request data via I2C as master
  \param[in]  slaveAddr   7b address [6:0] of I2C slave
	\param[in]  offset			offset or register address of slave device.
  \param[in]  numRx       number of bytes to receive
  \param[out] bufRx       receive buffer
  \return error code (0=ok; 1=timeout)
  request data from I2C slave with frame timeout. Note that no start or 
  stop condition is generated.
*/
uint8_t i2c_read(uint8_t addr, uint8_t offset, uint8_t numRx, uint8_t *bufRx) {

  uint8_t   i = 0;
  uint16_t  countTimeout = 10000;     /* use counter for timeout to minimize dependencies */
	uint8_t num_to_read = numRx;
	uint8_t temp = 0;
	
	I2C.CR2.reg.ACK = 1;							/*enable ack*/
	I2C.CR1.reg.NOSTRETCH = 0;				/* clock strench enable*/
  /* init receive buffer */
	if(numRx == 0 || bufRx == NULL){
		return (1);
	}
	
	if(i2c_gen_start()){	/*generate START condition. */
		return (2);	/*return ERROR if START condition cannot be generated. */
	}

  /* send 7b slave adress [7:1] + write flag (LSB=0) */
  I2C.DR.byte = (uint8_t) ((addr << 1) & ~0x01);       /* shift left and set LSB (write=0, read=1) */
  countTimeout = 10000;                                /* ~1.1us/inc -> ~10ms */
	while (!I2C.SR1.reg.ADDR && --countTimeout);     											/* wait until address sent or timeout */
		if(!I2C.SR1.reg.ADDR)
			return(3);
	temp = I2C.SR3.byte; //clear ADDR by reading SR1 then SR3
	
	
	/* data register should be empty by now */
	countTimeout = 10000;
	while((!I2C.SR1.reg.TXE) && --countTimeout);				/* TXE==0 means not empty */
		if(!I2C.SR1.reg.TXE)
			return(4);
	I2C.DR.byte = offset;	/*send offset; */
	countTimeout = 10000;
	while((!I2C.SR1.reg.TXE) && --countTimeout);
		if(!I2C.SR1.reg.TXE)
			return(5);
	/*check if byte transfer succeed */
		
	if(i2c_gen_start()){	/*generate repeated START condition. */
		return (6);	/*return ERROR if START condition cannot be generated. */
	}
	
  /* send 7b slave adress [7:1] + read flag (LSB=0) */
  I2C.DR.byte = (uint8_t) ((addr << 1) | 0x01);       /* repeated start with LSB=0. */
	
  countTimeout = 10000;                               /* ~1.1us/inc -> ~10ms */
	while ((!I2C.SR1.reg.ADDR) && --countTimeout);     											/* wait until address sent or timeout */
		if(!I2C.SR1.reg.ADDR)
			return(7);
	
	
	/* starting read data from slave */
	
	if(numRx == 1){
		I2C.CR2.reg.ACK = 0;						/* clear ack */
		no_interrupts();									/* disable interrupt */
		temp = I2C.SR3.byte;						/* by reading SR3, ADDR is cleared */
		I2C.CR2.reg.STOP = 1;						/* generate stop after current byte is read */
		interrupts();										/* enable interrupts */
		
		countTimeout = 10000;
		while((!I2C.SR1.reg.RXNE) && --countTimeout);
			if(!I2C.SR1.reg.RXNE)
				return (9);
		/* read byte */
		bufRx[0] = I2C.DR.byte;
	}
	else if(numRx == 2){
		I2C.CR2.reg.POS = 1;						/*set pos, next byte received in shift register will be NACKed */
		
		no_interrupts();									/*disable interrupt */
		temp = I2C.SR3.byte;						/*by reading SR3, ADDR is cleared */
		I2C.CR2.reg.ACK = 0;						/*clear ack */
		interrupts();										/*enable interrupts */
		
		countTimeout = 1000;            /* ~1.1us/inc -> ~1ms */
		while((!I2C.SR1.reg.BTF) && --countTimeout);	/*because clock strench enabled.
		BTF is set when new byte is received and DR is full, meaning 2 bytes are ready */
			if(!I2C.SR1.reg.BTF)
					return (11);
		
		no_interrupts();									/* disable interrupt */
		I2C.CR2.reg.STOP = 1;						/* generate stop after current byte is read */
		bufRx[0] = I2C.DR.byte;	  			/* read first byte */
		interrupts();										/* enable interrupts */
		
		bufRx[1] = I2C.DR.byte;					/*read second byte */
	}
	else if (numRx == 3){
		countTimeout = 1000;                             /* ~1.1us/inc -> ~1ms */
		while((!I2C.SR1.reg.BTF) && --countTimeout);				/*pull BTF, BTF==0 means byte transfer not succeed */
			if(!I2C.SR1.reg.BTF)
					return (12);
		/*first two bytes received*/			
		I2C.CR2.reg.ACK = 0;						/*clear ack */
		no_interrupts();									/*disable interrupt */
		bufRx[0] = I2C.DR.byte;					/*read first byte */
		I2C.CR2.reg.STOP = 1;						/*generate stop after current byte is read */
		bufRx[1] = I2C.DR.byte;					/*read second byte */
		interrupts();										/*enable interrupts */
		/*wait for last byte*/
		countTimeout = 10000;
		while(!I2C.SR1.reg.RXNE && --countTimeout);
			if(!I2C.SR1.reg.RXNE)
			return (13);
		bufRx[2] = I2C.DR.byte;					/* read third byte */
	}
	else{/*numRx >3 */
		/*read first numRx-3 bytes*/
		while(num_to_read >3){
			countTimeout = 1000;                             /* ~1.1us/inc -> ~1ms */
			while(!I2C.SR1.reg.BTF && --countTimeout){			/* pull BTF, BTF==0 means byte transfer not succeed */
				if(countTimeout-- == 0)
					return (14);
			}/*while BTF*/
			bufRx[i++] = I2C.DR.byte;
			--num_to_read;
		}/*while >2*/
		
		/* read last 3 bytes */
		countTimeout = 1000;                             /* ~1.1us/inc -> ~1ms */
		while((!I2C.SR1.reg.BTF) && --countTimeout);				/*pull BTF, BTF==0 means byte transfer not succeed */
			if(!I2C.SR1.reg.BTF)
					return (15);
		/*first two bytes received*/			
		I2C.CR2.reg.ACK = 0;						/*clear ack */
		no_interrupts();									/*disable interrupt */
		bufRx[i++] = I2C.DR.byte;					/*read first byte */
		I2C.CR2.reg.STOP = 1;						/*generate stop after current byte is read */
		bufRx[i++] = I2C.DR.byte;					/*read second byte */
		interrupts();										/*enable interrupts */
		/*wait for last byte*/
		countTimeout = 10000;
		while(!I2C.SR1.reg.RXNE && --countTimeout);
			if(!I2C.SR1.reg.RXNE)
			return (16);
		bufRx[i++] = I2C.DR.byte;					/* read third byte */
	}
	/*all data sent, gen stop condition*/
		if(i2c_gen_stop()){	/*generate STOP condition. */
			return (10);	/*return ERROR if STOP condition cannot be generated. */
		}/*if gen_stop */
		
	}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
