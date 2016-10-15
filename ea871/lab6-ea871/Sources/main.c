/*!
 * \file main.c
 * \brief Comunica&ccedil;&atilde;o Serial Ass&iacute;ncrono UART.
 * \details Este c&oacute;digo ilustra o uso do m&oacute;dulo UARTx dispon&iacute;vel no MKZ25Z128. 
 *          &Eacute; demonstrado o uso do buffer circular para transmitir dados entre um produtor e 
 *          um consumidor de diferetnes velocidades.       
 * \mainpage Projeto de envio de uma sequ&ecirc;ncia grande de dados ao Terminal.
 * \author Wu Shin-Ting
 * \date 24/04/2016
 * \note Documenta&ccedil;&atilde;o do c&oacute;digo no formato de Doxygen:
 *       http://mcuoneclipse.com/2014/09/01/automatic-documentation-generation-doxygen-with-processor-expert/
 */
#include "derivative.h" ///< include peripheral declarations 
#include "stdlib.h" ///< include peripheral declarations 
#include "string.h" ///< include memory operation functions

#define GPIO_PIN(x)  ((1)<<(x)) ///< obtem o bit do pino x
#define TAM_MAX 1024            ///< tamanho do buffer circular

char *buffer_circular;          ///< endere&ccedi;o do buffer circular
unsigned short head, tail;      ///< &iacute;ndices do buffer circular
uint8_t flag;                  ///< flag de controle da regi&atilde;o cr&iacute;tica

/*!
 * \fn delay10us (unsigned int t) 
 * \brief Gera um atraso de em torno de t*10us.
 * \param[in] t m&uacute;ltiplos de 10us
*/
void delay10us(unsigned int t) {
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
 * \fn initUART (void)
 * \brief Inicializa os pinos conectados a UART0 e ao OpenSDA.
 */
void initUART(void) {
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(0x1);	///< configura a fonte de relógio
	SIM_SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK;	 
	                                         ///< no reset, o valor é zero (FLL)
	                                         ///< mas não deixa de ser uma boa
	                                         ///< prática de inicializar explicitamente
	
	PORTA_PCR1 |= PORT_PCR_MUX(0x2);         ///< UART0_RX
	PORTA_PCR2 |= PORT_PCR_MUX(0x2);         ///< UART0_TX
		
	UART0_C1 &= ~(UART0_C1_PE_MASK |
			UART0_C1_ILT_MASK |
			UART0_C1_WAKE_MASK |
			UART0_C1_M_MASK | 
			UART0_C1_DOZEEN_MASK |
			UART0_C1_LOOPS_MASK );
	UART0_C2 &= ~(UART_C2_RE_MASK |
				UART_C2_TE_MASK);
	UART0_BDH &= ~(UART_BDH_SBNS_MASK | 
				UART_BDH_RXEDGIE_MASK | 
				UART_BDH_LBKDIE_MASK);
	UART0_C4 &= ~(UART0_C4_OSR(0x0C));                             ///< taxa de superamostragem: 4
	/*!
	 * baud rate 19200 (Se&ccedil;&atilde;o 8.3.2 em 
	 * ftp://ftp.dca.fee.unicamp.br/pub/docs/ea871/ARM/KLQRUG.pdf)
	 */
	UART0_BDH |= UART_BDH_SBR(0x4);                             
	UART0_BDL |= UART_BDL_SBR(0x44);
	
	UART0_C5 |= UART0_C5_BOTHEDGE_SHIFT;
	
	UART0_S1 |= (UART0_S1_PF_MASK |              ///< Registradores de estado: w1c
				UART0_S1_FE_MASK |
				UART0_S1_NF_MASK |
				UART0_S1_OR_MASK |
				UART0_S1_IDLE_MASK);
	
	UART0_C2 |= (UART_C2_RIE_MASK |      ///< ativa interrup&ccedil;&atilde;o de recep&ccedil&atilde;o
				UART_C2_RE_MASK |        ///< habilita o canal receptor 
				UART_C2_TE_MASK);	     ///< habilita o canal transmissor    

}

/*!
 * \fn enableNVIC (void)
 * \brief Habilita interrupções via NVIC.
 */
void enableNVIC(void) {
	/* 
	 * "Non-core interrupt source count" (IRQ = Vector Number (46) - 16, 
	 * Table 3-7 de KL-25 Sub-Family Reference Manual) &eacute; habilitado
	 * ao escrever 1 no bit correpondente do registrador NVIC_ISER 
	 * (B3.4.3 de ARMv6-M Architecture Reference Manual) e a solicita&ccedil;&atilde;o de 
	 * interrup&ccedil;&atilde;o &eacute; limpa ao escrever 1 no bit correspondente do 
	 * registrador NVIC_ICPR (B3.4.6 de ARMv6-M Architecture Reference Manual) 
	 */
	/*! UART0: Vetor 28, IRQ12, prioridade em NVICIPR3[7:0]
	 * Se&ccedil;&atilde;o 3.3.2.3 em 
	 * <a href="ftp://ftp.dca.fee.unicamp.br/pub/docs/ea871/ARM/KL25P80M48SF0RM.pdf" target=_manual>ftp://ftp.dca.fee.unicamp.br/pub/docs/ea871/ARM/KL25P80M48SF0RM.pdf</a>
	 */
    NVIC_ISER |= 1 << (28-16);            ///< Habilita interrup&ccedil;&atilde;o UART
    NVIC_ICPR |= 1 << (28-16);
    NVIC_IPR3 |= NVIC_IP_PRI_12(0x40);    ///< N&iacute;vel de prioridade = 1
}

/*! 
 * \fn insere_item (char item)
 * \brief Insere um item no buffer circular
 * \param[in] item caracter a ser inserido 
 */
void insere_item (char item) {
	unsigned short tmp;
	
	if ((head==TAM_MAX-1 && tail == 0) || head == tail-1) {
		/*! N&atilde;o h&aacute; espa&ccedil;o. Uma estrat&eacut;gia: aguardar 5 casas. */
	    tmp = (tail+5)%TAM_MAX;
	    while (tail != tmp);
	}
	buffer_circular[head++] = item;
	head %= TAM_MAX;
	if (!(UART0_C2 & UART_C2_TIE_MASK)) UART0_C2 |= UART_C2_TIE_MASK;	///< ativa a máscara de interrupção 
}

/*!
 * \fn remove_item (void)
 * \brief Remova um itemdo buffer circular
 * \return the item pointed by tail.
 */
char remove_item () {
	char d;
	
	d = buffer_circular[tail++];
	tail %= TAM_MAX;
	if (head == tail) {
		/*! Buffer vazio */
		UART0_C2 &= ~UART_C2_TIE_MASK;	///< desativa a máscara de interrupção		
	}
	return d;
}

/*!
 * \fn UART0_IRQHandler (void)
 * \brief Rotina de serviço para UART0
 */
void UART0_IRQHandler(void) {
	char d;
	if (UART0_S1 & UART0_S1_RDRF_MASK)
	{
		d = UART0_D;
		if (flag) insere_item(d);  ///< insere item
	} else if (UART0_S1 & UART0_S1_TDRE_MASK) {
		UART0_D = remove_item();   ///< remove item
	}	
}

/*!
 * \fn initPerif (void) 
 * \brief Configura os módulos de interface com os periféricos
 */
void initPerif(void) {
	initUART();           ///< Inicializa comunicação UART
	enableNVIC();         ///< Habilitar o controle de interrupção para UART
}

int main(void)
{
	uint8_t j;
	char i;
	
	/*! Inicialize os pinos como GPIO, utilize-os para inicializar LCD e inicialize o controlador NVIC */
	initPerif();
	
	/*! Inicializa o buffer circular */
	buffer_circular = (char *) malloc(sizeof(TAM_MAX));
	head = tail = 0;
	for(;;) {
		/*! Regi&atilde;o cr&iacute;tica para entrada por usu&aacute;rio */
		flag = 0;
		for (j=0; j< 100; j++) {
			for (i=33; i<127;i++) insere_item(i);
			insere_item ('\n');
			insere_item ('\r');
		}
		/*! Regi&atilde;o liberada para entrada por usu&aacute;rio */
		flag = 1;
		delay10us(500000);
	}
}
