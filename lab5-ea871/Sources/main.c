/*!
 * \file main.c
 * \brief Captura dos eventos de push-button por mecanismo de interrup&ccedil;&atilde;o.
 * \details Este c&oacute;digo ilustra o mecanismo de interrup&ccedil;&atilde;o dispon&iacute;vel no MKZ25Z128 
 *          para captura de um evento externo ass &iacute;ncrono;.
 * \mainpage Projeto de controle da cor dos leds RGB por intervalos de tempo.
 * \author Wu Shin-Ting
 * \date 29/03/2016
 * \note Documenta&ccedil;&atilde;o do c&oacute;digo no formato de Doxygen:
 *       http://mcuoneclipse.com/2014/09/01/automatic-documentation-generation-doxygen-with-processor-expert/
 */

#include "derivative.h" ///< include peripheral declarations 
#include "stdlib.h" ///< include peripheral declarations 
#include "string.h" ///< include memory operation functions

#define GPIO_PIN(x)  ((1)<<(x)) ///< obtem o bit do pino x

uint32_t tempo;             ///< m&uacute;ltiplo de 0.25s
int indice;             ///< cor selecionada
int estado_pta12;       ///< pta12 pressionado (1) ou solto (0)

int estado_pta5;
int nivel;

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
 * \fn ConvUIparaDec(unsigned int j, char * s)
 * \brief Converte em ASCII o inteiro sem sinal j.
 * \param[in]  j valor inteiro sem sinal
 * \param[out] s represnetação em ASCII
 */
void ConvUIpraDec (unsigned int j, char * s) {
    short k = 0;
    char aux[11];
    /*!
	 * Passo 1: extrai os algarismos decimais
	 */
    if (j == 0){ ///< Caso especial
   	 aux[k] = 0;
   	 k++;
    }
    while (j) {
   	 aux[k] = j % 10;
   	 j = j / 10;
   	 k++;
    }
    /*!
	 * Passo 2: representa em ASCII os digitos decimais
	 */
    while (k > 0) {
   	 *s = (aux[k-1] + 0x30);
   	 s++;           		 ///< incrementa o endereco por unidade do tipo de dados
   	 k--;
    }
    *s = 0;
}
/*!
 * \fn ConvFpraStr(float j, char s[5)
 * \brief Converte valor float para codigo ASCII.
 * \param[in]  j valor float
 * \param[out] s representacao em ASCII
 */
void ConvFpraStr(float j, char s[5]){
    /*! Separa o float em parte inteira e decimal */
    unsigned int inteiro = (int) j;
    float decimal = j - (float)inteiro;
    unsigned int decInt = (int)(decimal*10);
    char *aux;
    
    aux = malloc(3*sizeof(char));
    
    /*! Converte a parte inteira para ASCII */
    ConvUIpraDec(inteiro, aux);
    
    /*! Verifica se o float tem uma ou duas casas antes da virgula */
    if(inteiro < 9){
   	 s[0] = aux[0];
   	 s[1] = '.';
   	 ConvUIpraDec(decInt, aux);
   	 s[2] = aux[0];
   	 s[3] = '\0';
   	 s[4] = '\0';
    }
    else{
   	 s[0] = aux[0];
   	 s[1] = aux[1];
   	 s[2] = '.';
   	 ConvUIpraDec(decInt, aux);
   	 s[3] = aux[0];
   	 s[4] = '\0';
    }
    /*! Libera a memoria */
    free(aux);
    
}
/*!
 * \brief Inicializa os pinos conectados aos periféricos (8 leds, LCD, push-buttons e led RGB).
 */
void inicGPIO(void) {

	SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK |
		SIM_SCGC5_PORTB_MASK |
		SIM_SCGC5_PORTC_MASK |
	        SIM_SCGC5_PORTD_MASK ) ;

	/*! Configura os pinos PORTC[10:0] como GPIO de saida */
	
	PORTC_PCR0 = PORT_PCR_MUX(0x1);  	 ///< D0-D7 dos dados (GPIO)
	PORTC_PCR1 = PORT_PCR_MUX(0x1);
	PORTC_PCR2 = PORT_PCR_MUX(0x1);
	PORTC_PCR3 = PORT_PCR_MUX(0x1);
	PORTC_PCR4 = PORT_PCR_MUX(0x1);
	PORTC_PCR5 = PORT_PCR_MUX(0x1);
	PORTC_PCR6 = PORT_PCR_MUX(0x1);
	PORTC_PCR7 = PORT_PCR_MUX(0x1);
	PORTC_PCR8 = PORT_PCR_MUX(0x1);   	 ///< RS do LCD
	PORTC_PCR9 = PORT_PCR_MUX(0x1);   	 ///< E do LCD
	PORTC_PCR10 = PORT_PCR_MUX(0x1); 	 ///< /LE do latch

	GPIOC_PDDR |= 0x000007FF;   	 ///< Configura como sa&iacute;das
	GPIOC_PDOR &= 0xFFFFF800;   	 ///< Inicializa os pinos com 0

	/*!
	 * Configura os pinos PORTA[12,5:4] como GPIO de entrada e inicialize-os com mecanismo de interrupcao
	 */
	PORTA_PCR4 |= PORT_PCR_MUX(0x7);
	PORTA_PCR5 |= PORT_PCR_ISF_MASK | 
				      PORT_PCR_IRQC(0xB) |            ///< "Interrupt on either edge"
				      PORT_PCR_MUX(0x1);
	PORTA_PCR12 |= PORT_PCR_ISF_MASK | 
		      	  PORT_PCR_IRQC(0xB) |                ///< "Interrupt on either edge"
		      	  PORT_PCR_MUX(0x1);

	GPIOA_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(4) | GPIO_PIN(5) | GPIO_PIN(12)); 
	PORTA_ISFR &= ~PORT_ISFR_ISF(GPIO_PIN(4) | GPIO_PIN(5) | GPIO_PIN(12));

	/*! Configura outros pinos */


	/*! Configura a direcao do pino 5 e 12  da PORTA como entrada */
	GPIOA_PDDR &= 0xFFFFEFDF;
	

	/*! Configura pino B18 como GPIO (MUX = 001) */
	PORTB_PCR18 &= 0xFFFFF8FF;
	PORTB_PCR18 |=  0x00000100;

	/*! Configura pino B19 como GPIO (MUX = 001) */
	PORTB_PCR19 &= 0xFFFFF8FF;
	PORTB_PCR19 |=  0x00000100;

	/*! Configura a direcao dos pinos 18 e 19 da PORTB como sa&iacute;da */
	GPIOB_PDDR |= 0x000c0000;

	/*! Inicializa os pinos 18 e 19 */
	GPIOB_PSOR |= 0x000c0000;

	/*! Configura pino D1 como GPIO (MUX = 001) */
	PORTD_PCR1 = PORTD_PCR1 & 0xFFFFF8FF;
	PORTD_PCR1 = PORTD_PCR1 | 0x00000100;

	/*! Configura a diire&ccedil;&atilde;o dos pino 1 da PORTD como sa&iacute;da */
	GPIOD_PDDR = GPIOD_PDDR | 0x00000002;
	GPIOD_PDOR |= 0x00000002;
}

/*!
 * \fn InitSysTick (void)
 * \brief Inicializa o temporizador SysTick.
 */
void InitSysTick (void) {
	SYST_RVR = SysTick_RVR_RELOAD(5242880);   ///< 0.25s
	SYST_CVR = SysTick_CVR_CURRENT(0);                
	SYST_CSR |= (SysTick_CSR_CLKSOURCE_MASK |     ///< core clock: 20.971520MHz
			SysTick_CSR_TICKINT_MASK);            
	SYST_CSR &= ~SysTick_CSR_ENABLE_MASK;       ///< desabilita o clock do SysTick     
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
	/*! PORTA: Vetor 46, IRQ30, prioridade em NVICIPR7[23:16]
	 * Se&ccedil;&atilde;o 3.3.2.3 em 
	 * <a href="ftp://ftp.dca.fee.unicamp.br/pub/docs/ea871/ARM/KL25P80M48SF0RM.pdf" target=_manual>ftp://ftp.dca.fee.unicamp.br/pub/docs/ea871/ARM/KL25P80M48SF0RM.pdf</a>
	 */
    NVIC_ISER |= 1 << (46-16);            ///< Habilita interrup&ccedil;&atilde;o PORTA
    NVIC_ICPR |= 1 << (46-16);
    NVIC_IPR7 |= NVIC_IP_PRI_30(0xc0);    ///< N&iacute;vel de prioridade = 3
}

/*!
 * \fn PORTA_IRQHandler (void)
 * \brief Rotina de serviço para GPIOA (Vetor 46, IRQ30, prioridade em NVICIPR7[31:30])
 */
void PORTA_IRQHandler(void) {
	if (PORTA_PCR5 & PORT_PCR_ISF_MASK) {
		if(GPIOA_PDIR & GPIO_PIN(5)){
			estado_pta5 = 0;
			nivel = (tempo > 0)?(tempo-1):0;
			SYST_CSR &= ~SysTick_CSR_ENABLE_MASK;    ///< desabilita o clock do SysTick
		}
		else{
			estado_pta5 = 1;
			tempo = 0;         ///< inicia a contagem de tempo
			SYST_CSR |= SysTick_CSR_ENABLE_MASK;     ///< habilita o clock do SysTick       
		}
		PORTA_PCR5 |= PORT_PCR_ISF_MASK;            ///< abaixa a bandeira de interrup&ccedil;&atilde;o
	} else if (PORTA_PCR12 & PORT_PCR_ISF_MASK) {   
		if (GPIOA_PDIR & GPIO_PIN(12)) {
			/*! Soltou */
			estado_pta12 = 0;
			indice = (tempo > 0)?(tempo-1):0;   ///< computa tempo transcorrido em múltiplo de 0.25s
			SYST_CSR &= ~SysTick_CSR_ENABLE_MASK;    ///< desabilita o clock do SysTick
		} else {
			/*! Apertou */
			estado_pta12 = 1;
			tempo = 0;         ///< inicia a contagem de tempo
			SYST_CSR |= SysTick_CSR_ENABLE_MASK;     ///< habilita o clock do SysTick       
		}
		PORTA_PCR12 |= PORT_PCR_ISF_MASK;            ///< abaixa a bandeira de interrup&ccedil;&atilde;o
	} 
}

/*!
 * \fn NMI_IRQHandler (void)
 * \brief Rotina de serviço para Non-Maskable Interrupt (Vetor 2)
 */
void NMI_Handler(void){
}

/*!
 * \fn SysTick_Handler (void)
 * \brief Rotina de serviço para exce&ccedil&atilde;o SysTick (Vetor 15)
 */
void SysTick_Handler(void) {
	tempo++;	
}

/*!
 * \fn pulso (char p)
 * \brief Gera um pulso "Enable" de t*1us.
 * \param[in] p para LCD (p=0) e para Leds (p=1).
 */
void pulso(char p) {
    GPIOC_PSOR |= 1 << (9 + p);     ///< Seta 1 no PORTC[9+p]
    delay10us(200);        		 ///< mant&eacute;m aprox. 2ms em 1
    GPIOC_PCOR |= 1 << (9 + p);     ///< Limpa em 0 o PORTC[9+p]
}

/*!
 * \fn RS (char l)
 * \brief Envia ao LCD o sinal RS pelo pino PORTC[8].
 * \param[in] l valor do RS (0, byte de instru&ccedil;&atilde;o e 1, byte de dados).
 */
void RS(char l) {
    if(l) {             		 ///< (l != 0)
   	 GPIOC_PSOR = 0x00000100; ///< Seta o LCD no modo de dados
    } else {   
   	 GPIOC_PCOR = 0x00000100; ///< Seta o LCD no modo de instru&ccedil;&atilde;o
    }
}


/*!
 * \fn enviaLedLCD (char c)
 * \brief Envia ao LCD um byte pelos pinos PORTC[7:0]
 * \param[in] c caracter em ASCII.
 */
void enviaLedLCD(char c) {
    GPIOC_PCOR |= 0x000000FF; 	 ///< limpa em 0 PORTC[7:0]
    GPIOC_PSOR |= (unsigned int)c; ///< Seta os bits que devem ser setados
    pulso(0);             		 ///< dispara o pulso "Enable" do LCD
    pulso(1);             		 ///< dispara o pulso "Enable" do Latch 74573
}


/*!
 *  \fn limpaLCD (void)
 * \brief Envia a instrucao "Clear Display" (0x01).
 */
void limpaLCD(void) {
    RS(0);                		 ///< Seta o LCD no modo de instru&ccedil;&atilde;o
    enviaLedLCD(1);
}

/*!
 * \fn trocaLinhaLCD (void)
 * \brief Muda a posicao do "adress counter" para a primeira posicao da segunda linha do lcd.
 */
void trocaLinhaLCD(void){
    RS(0);
    enviaLedLCD(0xC0);
}

/*!
 * \fn inicLCD (void) 
 * \brief Inicializa o LCD com a sequência de instruções recomendada pelo fabricante
 */
void inicLCD(void) {
    char d[] = {0x38,0x0C, 0x06, 0x01, 0x00}; ///< sequ&ecirc;ncia de comandos
    int k;
    for(k = 0; k < 34; k++) { ///< Espera mais de 100ms
   	 delay10us(10000);
    }      		 
    RS(0);          		 ///< Seta o LCD no modo de instru&ccedil;&atilde;o
    k = 0;
    while(d[k] != 0) {
   	 enviaLedLCD(d[k]);
   	 k++;
    }
}

/*!
 * \fn mandaString (char *s)
 * \brief Envia uma string de caracteres.
 * \param[in] s endereço inicial da string.
 */

void mandaString(char * s) {
    RS(1);                 		 ///< Seta o LCD no modo de dados
    while (*s) {           		 ///< enquanto o conte&uacute;do do endere&ccedil;o != 0
   	 enviaLedLCD(*s);   		 ///< envia o byte
   	 s++;               		 ///< incrementa o endere&ccedil;o
    }
}

/*!
 * \fn criaCaractere(unsigned int position, unsigned char *ptr)
 * \brief Cria um determinado caractere e armazena em CGRAM
 * \param[in] position posicao da memoria a ser colocado na memoria CGRAM
 * \param[in] *ptr vetor com os dados a serem gravados
 */
void criaCaractere(unsigned int position, unsigned char *ptr){
    unsigned char i;
    
    if(position<8){
   	 
   	 /*! ativa o envio de instrucoes */
   	 RS(0);
  		 enviaLedLCD(0x40+(position*8));
  		 /*! ativa o envio de dados */
  		 RS(1);
   	 for(i=0;i<8;i++)
   		 enviaLedLCD(ptr[i]);
   }
}
/*!
 * \fn initPerif (void) 
 * \brief Configura os módulos de interface com os periféricos
 */
void initPerif(void) {
	inicGPIO();           ///< Pinos digitais de propósito geral
	inicLCD();            ///< Inicializa o LCD
	enableNVIC();         ///< Habilitar o controle de interrupção para GPIOA
}

/**
* @brief Seta o estado do led
* @param[in] tipo vermelho (0),verde (1) e azul (2)
* @param[in] aceso (1) ou apagado (0)
*/
void led (char tipo, char aceso) {
    	if (aceso) {
            	switch(tipo)
            	{
                    	case 0:
                            	GPIOB_PCOR |= (1<<18); /*! Clear bit 18, LED vermelho em PTB18 (acende) */
                            	break;

                    	case 1:
                            	GPIOB_PCOR |= (1<<19); /*! Clear bit 19, LED verde em PTB19 (acende) */
                            	break;

                    	case 2:
                            	GPIOD_PCOR |= (1<<1); /*! Clear bit 1, LED azul em PTD1 (acende) */
                            	break;
            	}
    	}
    	else
    	{
            	switch (tipo)
            	{
                    	case 0:
                            	GPIOB_PSOR |= (1<<18); /*! Set bit 18, LED vermelho em PTB18 (apaga) */
                            	break;

                    	case 1:
                            	GPIOB_PSOR |= (1<<19); /*! Set bit 19, LED verde em PTB19 (apaga) */
                            	break;

                    	case 2:
                        	GPIOD_PSOR |= (1<<1); /*! Set bit 1, LED azul em PTD1 (apaga) */
                        	break;
        	}
	}
}

int main(void)
{
	uint8_t cores_estaticas[] = {1,0,0,1,1,1,1,1,0,0,1,1,1,0,1,0,0,0};
	    uint8_t *cores_dinamica;
	    indice = 5;
		estado_pta12 = 0; ///< estado inicial (pta12 solto)	
	    
	    char *str0 = "VERMELHO";
	    char *str1 = "BRANCO";
	    char *str2 = "AMARELO";
	    char *str3 = "CIANO";
	    char *str4 = "MAGENTA";
	    char *str5 = "PRETO";
	    char *str;
	    
	    estado_pta5 = 0;
	    nivel = 0;
	    
	    short i;
	    float time;
	    
	    float freq_estaticas[] = {0.5, 1.0, 2.0, 10.0, 30.0, 60.0};
	    float *freq_dinamica;
	    char *freq;
	    
	    /*! vetor com os dados para se criar o caractere e gravar na cgram */
	    unsigned char caractere[8] = {0xe, 0x11, 0xe, 0x11, 0x1f, 0x10, 0xe, 0x0};
	   	 
	    cores_dinamica = (uint8_t *)malloc(18*sizeof(uint8_t));  
	    freq_dinamica = (float *)malloc(6*sizeof(float));
	    
	    freq = malloc(sizeof(char)*5);

		for(i = 0; i < 18; i++) cores_dinamica[i] = cores_estaticas[i];
		for(i = 0; i < 6; i++) freq_dinamica[i] = freq_estaticas[i];
	    
		/*! Inicialize os pinos como GPIO e utilize-os para inicializar LCD */
	    inicGPIO();
	    inicLCD();
		/*! Inicialize os pinos como GPIO, utilize-os para inicializar LCD e inicialize o controlador NVIC */
		initPerif();
		/*! Inicialize o Timer do Processador */
		InitSysTick();
	    
	    /*! Criar o caractere "e com acento" */
	    criaCaractere(1, caractere);
	    
	    time = (1.0/(*(freq_dinamica+(nivel))));
	    
	    for(;;) {
	    	if(!estado_pta12 && !estado_pta5){
				if(indice < 6){
				 /*! Selecione a mensagem apropriada */
					 switch (indice){
						 case 0:
							 str = str0;
							 break;
						 case 1:
							 str = &str1[0];
							 break;
						 case 2:
							 str = &str2[0];
							 break;
						 case 3:
							 str = str3;
							 break;
						 case 4:
							 str = str4;
							 break;
						 case 5:
							 str = str5;
							 break;
					 }
				}
				else{ 
					if(indice > 5){
						/*! passou o intervalo maximo permitido */
						indice = 5;
					}
				}
				/*! Envie mensagem para LCD e leds */
				 //limpaLCD();
				 RS(0);
				 enviaLedLCD(0x80);
				 
				 mandaString("Cor: ");
				 mandaString(str);
				 mandaString("    ");
			
			
				if(nivel > 5)
					nivel = 5;
				
				if(nivel < 6)
					time = (1.0/(*(freq_dinamica+(nivel))));
				
				/*! Converte a frequencia float para ASCII */
				ConvFpraStr(freq_estaticas[nivel], freq);
				 /*! passa a escrever na segunda linha do lcd */
				 trocaLinhaLCD();
				 mandaString("Frequ");
				 /*! escreve o caractere "e" com acento */
				 enviaLedLCD(1);      		 
				 mandaString("ncia: ");
				 mandaString(freq);
			
							 
				 led (0, *(cores_dinamica+(indice*3)));
				 led (1, *(cores_dinamica+(indice*3+1)));
				 led (2, *(cores_dinamica+(indice*3+2)));
						 
				
				delay10us((int)time*100000);
					 
				if (indice != 5)
				{
				 /*! Alterna os leds  */
				 led (0, !(cores_estaticas[indice*3]));
				 led (1, !cores_estaticas[indice*3+1]);
				 led (2, !cores_estaticas[indice*3+2]);
							 
				}
				
				delay10us((int)time*100000);
	    	}

		}    
	    
	    /*! Desalocar a memoria */
	    free((char*)freq);
		free((char*)cores_dinamica);
		free((float*)freq_dinamica);

}
