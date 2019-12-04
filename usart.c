/**
  \file usart.c
  declaration of functions for USART communication
*/

// Source code under CC0 1.0
#include "type_def.h"
#include "stm8l151c8_wrap.h"
#include "gpio.h"
#include "usart.h"

/* internal buffer for usart1 */
#define USART1_BUF_SIZE 10
int8_t usart1_buf[USART1_BUF_SIZE];
int8_t usart1_buf_idx = USART1_BUF_SIZE;

int8_t usart1_init(){
	#define CLOCK_USART_ENABLE 0x20
	
	/* No prescaler, Set the frequency to 16 MHz */
	CLK_CKDIVR = 0x00;
  /* Enable peripherals */
  CLK_PCKENR1 |= CLOCK_USART_ENABLE;

	pin_mode_set(&PORT_C, 3, output_pushpull_16m);

  /*reset CR1 and CR3, which means 8-bit word, no parity, 1 stop*/
	USART1.CR1.byte = USART_CR1_RESET_VALUE;
	USART1.CR3.byte = USART_CR3_RESET_VALUE;
	
  /* BRR2 = 0x03; BRR1 = 0x68;  9600 baud */
  /* BRR2 = 0x01; BRR1 = 0x34;19200 baud */
  USART1.BRR2.byte = 0x01;
	USART1.BRR1.byte = 0x34;
	
  /* Transmitter is enabled */
	USART1.CR2.reg.TEN = 1;

}

int8_t usart1_close(){
	/*disable interupt if all data have been sent */
	USART1.CR2.reg.TIEN = 0;
	/* transmitter disable */
	USART1.CR2.reg.TEN = 0;
}

static int8_t usart1_putchar(int8_t c)
{
	/*wait till content of DR moved to shift register */
	while(!USART1.SR.reg.TXE);
	USART1.DR.byte = c;
	return(c);
}

int8_t usart1_write(int8_t *bufTx, uint8_t numTx){
 int8_t idx = 0;
 for(idx=0; idx < numTx; ++idx){
	usart1_putchar(bufTx[idx]);
 }
}

int8_t usart1_write_itr(int8_t * bufTx, uint8_t numTx){
	int8_t i =0;
	
	if(numTx > USART1_BUF_SIZE)
		return(1);
	/* if previous sending hasn't finished.
	 The new write fails. */
	if(usart1_buf_idx != USART1_BUF_SIZE)
		return(2);
	
	/*move data to USART1 buffer*/
	for(i=0; i< numTx ; ++i){
		usart1_buf[i] = bufTx[i];
	}
	
	usart1_buf_idx = 0;
	/*enable interupt if all data have been sent */
	USART1.CR2.reg.TIEN = 1;
	
	return(0);
}

@far @interrupt void on_USART1_TXE_interrupt (void){
	if(usart1_buf_idx >= USART1_BUF_SIZE){
		USART1.CR2.reg.TIEN = 0;
		return_from_interrupt();
		
	}
	else{
		USART1.DR.byte = 	usart1_buf[usart1_buf_idx++];
		return_from_interrupt();
	}
}


/* test functions */
void usart1_test(void)
{
	unsigned long i = 0;
	int8_t str[] ="hello\n";
	usart1_init();
	for(;;)
	{
		usart1_write(str, sizeof(str));
		for(i = 0; i < 100000; i++); 
	}
}

void usart1_itr_test(void){
	unsigned long i = 0;
	int8_t str[] ="12345678";
	usart1_init();
	for(;;)
	{
		usart1_write_itr(str, sizeof(str));
		wait_for_interrupt();
	}
}