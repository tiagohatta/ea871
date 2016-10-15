/*!
 /*! @file main.c
 * @brief Pisca o led em cores diferentes selecionadas de acordo com o intervalo de tempo de aperto 
 *        do push button conectado &agrave; PTA12
 * @mainpage Projeto de pisca-pisca com cores e frequencia selecionaveis
 * @author Tiago Hatta
 * @date 06/04/2016
 */

#include <stdlib.h>
#include <derivative.h>

/**
* @brief Gera um atraso de t*10us
* @param[in] t multiplo de 10us
*/
void delay10us (int t)
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

/**
* @brief Retorna o valor do pino 12 da porta A
* @return estado do botao (0 apertado; 1 normal)
*/
short le_pta12 () {
        volatile unsigned int a;
        a = GPIOA_PDIR;       /*! Faz a leitura na PORTA (32 bits) */
        return ((short) (a & (1<<12)));
}

/**
* @brief Retorna o valor do pino 5 da porta A
* @return estado do botao (0 apertado; 1 normal)
*/
short le_pta5 () {
        volatile unsigned int a;
        a = GPIOA_PDIR;       /*! Faz a leitura na PORTA (32 bits) */
        return ((short) (a & (1<<5)));
}

/**
* @brief Analisa quanto tempo determinado pushbutton esta pressionado
* @param[in] pushbutton a ser analisado pta5 ou pta12
* @return indice que indica a cor ou frequencia a ser selecionade de acordo com o parametro
*/
int tempo_espera (int pushbutton) {
        short pta;
        uint8_t i;
        uint8_t indice=0;
        
        if(pushbutton == 12){
        	for (i=0; i < 5; i++) {
        		delay10us(50000);   /*! tempo estimado para o usu&aacute;rio soltar o bot&atilde;o */
        	
        		pta = le_pta12();
        		
        		if (pta) return indice;
        		indice++;
        	}
        }
        else{
        	for (i = 0; i < 6; i++){
        		delay10us(50000);
        		
        		pta = le_pta5();
        	
        		if (pta) return indice;
        		indice++;
        	}
        }
        
        return indice; /*! atingiu o m&aacute;ximo tempo permitido */ 
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

/**
* @brief Inicializa como GPIO A5, A12, B18, B19 e D1 
*/
void inicGPIO(void) {
    SIM_SCGC5 |= 0x00000200;  /*!  Habilita clock da PORTA */
    SIM_SCGC5 |= 0x00000400;  /*!  Habilita clock da PORTB */
    SIM_SCGC5 |= 0x00001000;  /*!  Habilita clock da PORTD */

    /*! Configura pino A12 como GPIO (MUX = 001) */
    PORTA_PCR12 &= 0xFFFFF8FF;
    PORTA_PCR12 |=  0x00000100;

    /*! Configura a direcao do pino 5 e 12  da PORTA como entrada */
    GPIOA_PDDR &= 0xFFFFEFDF;
    
    /*! Configura pino A5 como GPIO (MUX = 001) */
    PORTA_PCR5 &= 0xFFFFF8FF;
    PORTA_PCR5  |= 0X00000100;

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
* @brief Executa a funcao de inicializacao dos pinos relativos ao GPIO e  
*     de acordo com o botao apertado, controla a cor dos leds piscantes conforme ou a frequencia em que piscam 
*     proporcional ao intervalo ds tempo que o botao ficou apertado.
* @return 1
*/
int main(void){
    short pta12, pta5;
    uint8_t cores_estaticas[] = {1,0,0,1,1,1,1,1,0,0,1,1,1,0,1,0,0,0};
    uint8_t *cores_dinamica;
    int indice = 5;
    
    float freq_estaticas[] = {0.5, 1.0, 2.0, 10.0, 30.0, 60.0}; 
    float *freq_dinamica;
    int nivel = 0;
    
    int time;
    
    inicGPIO();

    cores_dinamica = (uint8_t *)malloc(18*sizeof(uint8_t));
    
    freq_dinamica = (float *)malloc(6*sizeof(float));
    
    int i = 0;

    for(i = 0; i < 18; i++) cores_dinamica[i] = cores_estaticas[i];

    for(i = 0; i < 6; i++) freq_dinamica[i] = freq_estaticas[i];
    
    for(;;){
            pta12 = le_pta12();
            pta5 = le_pta5();
            
            if (!pta12)
            {
            	/*! Botao de pta12 apertado: verifique o intervalo de tempo  */
            	indice = tempo_espera(12);
            }

            if(!pta5)
            {
            	/*! Botao de pta5 apertado: verifique o intervalo de tempo  */	
            	nivel = tempo_espera(5);        	
			}
            
            led (0, *(cores_dinamica+(indice*3)));
        	led (1, *(cores_dinamica+(indice*3+1)));
        	led (2, *(cores_dinamica+(indice*3+2)));
	         
            /*! seleciona a frequencia de pisca pisca do led  */
            time = (int)(1.0/(*(freq_dinamica+(nivel))));
            
            delay10us(time*100000);
            
            if (indice != 5)
		    {
				/*! Alterna os leds  */
            	led (0, !(cores_estaticas[indice*3]));
				led (1, !cores_estaticas[indice*3+1]);
				led (2, !cores_estaticas[indice*3+2]);
                            
			}
            
            delay10us(time*100000);
    }
 	
    /*! libera memoria alocada dinamicamente  */
    free((char*)cores_dinamica);
    free((float*)freq_dinamica);
    
}     
