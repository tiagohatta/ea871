/*!
 * @file exp3.c
 * @brief Pisca-pisca de um led vermelho em tempo pre determinado
 * @mainpage Projeto de Pisca-Pisca com tempo controlado
 * @authors Tiago Hatta
 * @date 01/04/2016
 */

/*! Habilita as portas do GPIO (Reg. SIM_SCGC5) */
#define SIM_SCGC5   (*(unsigned int volatile *) 0x40048038) 

/*! MUX de PTB18 (Reg. PORTB_PCR18) */
#define PORTB_PCR18 (*(unsigned int volatile *) 0x4004A048) 
/*!  Data direction do PORTB (Reg. GPIOB_PDDR) */
#define GPIOB_PDDR  (*(unsigned int volatile *) 0x400FF054)
/*! Set bit register do PORTB (Reg. GPIOB_PSOR) */
#define GPIOB_PSOR  (*(unsigned int volatile *) 0x400FF044)
/*! Clear bit register do PORTB (Reg. GPIOB_PCOR) */
#define GPIOB_PCOR  (*(unsigned int volatile *) 0x400FF048) 


/*! MUX de PTE23 (Reg. PORTE_PCR23) */
#define PORTE_PCR23 (*(unsigned int volatile *) 0x4004D05C) 
/*!  Data direction do PORT E (Reg. GPIOE_PDDR) */
#define GPIOE_PDDR  (*(unsigned int volatile *) 0x400FF114)
/*! Toggle bit register do PORT E (Reg. GPIOE_PTOR) */
#define GPIOE_PTOR  (*(unsigned int volatile *) 0x400FF10C)

/**
 * @brief gera um atraso correspondente a t iteracoes
 * @param t numero de iteracoes
 */

void delay10us(unsigned int t) 
{
	/*! Deve-se utilizar a instrucao NOP 21 vezes
	 *! para assim ter 21 ciclos de relogio e consequentemente 10us de delay
	 *! Usa-se __asm__ para conseguir programar assembly dentro deste codigo em C
	 *! */
	while(t!=0){
		__asm__(
			"mov r0, #21 \n\t"
			"loop: \n\t"
			"nop \n\t"
			"sub r0, r0, #1 \n\t"
			"cmp r0, #0 \n\t"
			"bne loop \n\t"
		);
		t--;
	}
}

 /*!
  * @brief Led piscante
  * @return 1 somente para satisfazer a sintaxe C
  */
int main(void)
{
	SIM_SCGC5 = SIM_SCGC5 | (1<<10);    /*!  Habilita clock GPIO do PORTB */
	SIM_SCGC5 = SIM_SCGC5 | (1<<13);    /*!  Habilita clock GPIO do PORTE */
	
	PORTB_PCR18 = PORTB_PCR18 & 0xFFFFF8FF; /*! Zera bits 10, 9 e 8 (MUX) de PTB18 */
	PORTB_PCR18 = PORTB_PCR18 | 0x00000100; /*! Seta bit 8 do MUX de PTB18, assim os 3 bits de MUX serao 001 */
	
	PORTE_PCR23 = PORTE_PCR23 & 0xFFFFF8FF; /*! Zera bits 10, 9 e 8 (MUX) de PTE23 */
	PORTE_PCR23 = PORTE_PCR23 | 0x00000100; /*! Seta bit 8 do MUX de PTE23, assim os 3 bits de MUX serao 001 */
	
	GPIOB_PDDR  = GPIOB_PDDR  | (1<<18);    /*! Seta pino 18 do PORTB como saida */
	GPIOE_PDDR  = GPIOE_PDDR  | (1<<23);    /*! Seta pino 23 do PORTE como saida */
	
	for(;;) 
	{
		
		GPIOB_PSOR = (1<<18); /*! Set bit 18, LED vermelho em PTB18 (apaga) */
		GPIOE_PTOR = (1<<23); /*! Toggle bit 23, LED vermelho em PTE23 (inverte -> apaga) */
		delay10us(200000);        /*! Espera um tempo */
		
		GPIOB_PCOR = (1<<18); /*! Clear bit 18, LED vermelho em PTB18 (acende) */
		GPIOE_PTOR = (1<<23); /*! Toggle bit 23, LED vermelho em PTE23 (inverte -> acende) */
		delay10us(200000);        /*! Espera um tempo */
		
	}
	return 0;
}
