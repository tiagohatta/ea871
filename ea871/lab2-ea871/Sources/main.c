/*!
@brief Sequencia de Cores atraves de leds vermelho, verde e azul
@mainpage Pisca Pisca de diversas cores
EXPERIMENTO 2 Ferramentas de Desenvolvimento EA871
@authors Tiago Hatta
@date 17/03/2016
*/

#define SIM_SCGC5   (*(unsigned int volatile *) 0x40048038) /*! Habilita as Portas do GPIO (Reg. SIM_SCGC5) */
#define PORTB_PCR18 (*(unsigned int volatile *) 0x4004A048) /*!  MUX de PTB18 (Reg. PORTB_PCR18) */
#define PORTB_PCR19 (*(unsigned int volatile *) 0x4004A04C) /*!  MUX de PTB19 (Reg. PORTB_PCR19) */
#define PORTD_PCR1  (*(unsigned int volatile *) 0x4004C004) /*!  MUX de PTB19 (Reg. PORTB_PCR) */
#define GPIOB_PDDR  (*(unsigned int volatile *) 0x400FF054) /*! Data direction do PORTB (Reg. GPIOB_PDDR) */
#define GPIOB_PSOR  (*(unsigned int volatile *) 0x400FF044) /*! Set bit register do PORTB (Reg. GPIOB_PSOR) */
#define GPIOB_PCOR  (*(unsigned int volatile *) 0x400FF048) /*! Clear bit register do PORTB (Reg. GPIOB_PCOR) */
#define GPIOD_PDDR  (*(unsigned int volatile *) 0x400FF0D4) /*! Data direction do PORTD (Reg. GPIOD_PDDR) */
#define GPIOD_PSOR  (*(unsigned int volatile *) 0x400FF0C4) /*! Set bit register do PORTD (Reg. GPIOD_PSOR) */
#define GPIOD_PCOR  (*(unsigned int volatile *) 0x400FF0C8) /*! Clear bit register do PORTD (Reg. GPIOD_PCOR) */

/*!
* @brief gera um atraso correspondente a I iteracoes
* @param [in] i numero de iteracoes
*/

void delay( unsigned int i) 
{
	while (i) i--;
}

/*!
* @brief acende leds
* @param [in] vermelho char tal que v acende, f nao acende
* @param [in] verde char tal que v acende, f nao acende
* @param [in] azul char tal que v acende, f nao acende
*/
void acendeLeds(char vermelho, char verde, char azul){
	if(vermelho == 'v' && verde == 'v'){
		GPIOB_PCOR = (11<<18); /*! Clear bit 18 e 19, LED vermelho em PTB18 e verde em PTB19 (acende) */
	}else{
		if(vermelho == 'v')
			GPIOB_PCOR = (1<<18); /*! Clear bit 18, LED vermelho em PTB18 (acende) */
		if(verde == 'v')
			GPIOB_PCOR = (1<<19); /*! Clear bit 19, LED vermelho em PTB19 (acende) */

	}

	if(azul == 'v')
				GPIOD_PCOR = 10; /*! Clear bit 2, LED azul em PTD1 (acende) */
	delay(500000);        /*! Espera um tempo */

	return;
}

/*!
* @brief apaga leds
* @param [in] vermelho char tal que v apaga
* @param [in] verde char tal que v apaga
* @param [in] azul char tal que v apaga
*/
void apagaLeds(char vermelho, char verde, char azul){
	if(vermelho == 'v' && verde == 'v'){
			GPIOB_PSOR = (11<<18); /*! Clear bit 18 e 19, LED vermelho em PTB18 e verde em PTB19 (apaga) */
	}else{
		if(vermelho == 'v')
			GPIOB_PSOR = (1<<18); /*! Clear bit 18, LED vermelho em PTB18 (apaga) */
		if(verde == 'v')
			GPIOB_PSOR = (1<<19); /*! Clear bit 19, LED vermelho em PTB19 (apaga) */

	}
	if(azul == 'v')
				GPIOD_PSOR = 10; /*! Clear bit 2, LED azul em PTD1 (apaga) */

	delay(500000);		  /*! Espera um tempo */
}

/*!
* @brief Sequencia de cores das luzes dos leds
*/
int main( void)
{
	SIM_SCGC5 = SIM_SCGC5 | (101<<10);    /*!  Habilita clock GPIO do PORTB e PORTD */

	PORTB_PCR18 = PORTB_PCR18 & 0xFFFFF8FF; /*! Zera bits 10, 9 e 8 (MUX) de PTB18 */
	PORTB_PCR18 = PORTB_PCR18 | 0x00000100; /*! Seta bit 8 do MUX de PTB18, assim os 3 bits de MUX serao 001 */
	
	PORTB_PCR19 = PORTB_PCR19 & 0xFFFFF8FF; /*! Zera bits 10, 9 e 8 (MUX) de PTB19 */
	PORTB_PCR19 = PORTB_PCR19 | 0x00000100; /*! Seta bit 8 do MUX de PTB19, assim os 3 bits de MUX serao 001 */
	
	PORTD_PCR1 = PORTD_PCR1 & 0xFFFFF8FF; /*! Zera bits 10, 9 e 8 (MUX) de PTD1 */
	PORTD_PCR1 = PORTD_PCR1 | 0x00000100; /*! Seta bit 8 do MUX de PTD1, assim os 3 bits de MUX serao 001 */
	
	GPIOB_PDDR  = GPIOB_PDDR  | (11<<18);    /*! Seta pinos 18 e 19 do PORTB como saida */
	GPIOD_PDDR  = GPIOD_PDDR  | 10;    /*! Seta pino 1 do PORTD como saida */

	GPIOB_PSOR = (11<<18); /*! Set bit 18, LED vermelho em PTB18 (apaga) */
	GPIOD_PSOR = (10); /*! Set bit 18, LED vermelho em PTB18 (apaga) */
	
	int i = 0;
	for(;;) 
	{
		for(i = 0; i < 5; i++){
			/*! cor vermelha */
			acendeLeds('v', 'f', 'f');		/*! Liga apenas led vermelho */
			apagaLeds('v', 'f', 'f');		/*! Apaga o led vermelho */
		}

		for(i = 0; i < 5; i++){
			/*! branco */
			acendeLeds('v', 'v', 'v');		/*! Acende todos os leds para cor branca */
			apagaLeds('v', 'v', 'v');		/*! Apaga todos os leds */
		}

		for(i = 0; i < 5; i++){
			/*! amarelo */
			acendeLeds('v', 'v', 'f');		/*! Acende vermelho e verde que formam cor amarela */
			apagaLeds('v', 'v', 'f');		/*! Apaga os leds ligados */
		}
		
		for(i = 0; i < 5; i++){
			/*! ciano */
			acendeLeds('f', 'v', 'v');		/*! Acende leds verde e azul para formar a cor ciano */
			apagaLeds('f', 'v', 'v');		/*! Apaga os leds */
		}
		
		for(i = 0; i < 5; i++){
			/*! magenta */
			acendeLeds('v', 'f', 'v');		/*! Acende os leds vermelho e azul para o magenta */
			apagaLeds('v', 'f', 'v');		/*! Apaga os leds vermelho e azul */
		}
		
		for(i = 0; i < 5; i++){
			/*! preto */
			delay(500000);        /*! Como o preto sao todos leds desligados, chama a funcao delay */
		}
		
	}
}
