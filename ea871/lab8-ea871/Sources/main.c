/*!
 * \file exp9.c
 * \brief Amostragem de sinais analogicos (Sensor de temperatura e potenciometro).
 * \details Este codigo ilustra o uso do modulo ADC disponivel no MKZ25Z128. 
 * \mainpage Projeto de amostragem de sinais analogicos de um potenciometro para controlar
 *           frequencias de piscadas de leds.
 * \author Tiago Eidi Hatta
 * \date 20/06/2016
 * \note Documentacao do codigo no formato de Doxygen:
 *       http://mcuoneclipse.com/2014/09/01/automatic-documentation-generation-doxygen-with-processor-expert/
 */

#include "derivative.h"
#include <stdlib.h>
#include "string.h" ///< include memory operation functions

#define GPIO_PIN(x)  ((1)<<(x)) ///< obtem o bit do pino x
// TPM
#define COUNTER_OVF 655         ///< valor maximo do TPMx_CNT
#define TIMEOUT 3300            ///< timeout da botoeira em múltiplos de overflow do TPMx
// BUFFER
#define TAM_MAX 1024            ///< tamanho do buffer circular
// Conversor ADC
#define MAX 0xffff              ///< maximum conversion extent (12 bits)
#define VREFH 2921              ///< tensao de referencia alta (mV)
#define VREFL 0.0               ///< tensao de referencia baixa (mV)
#define M_POT 0.0205

char    *buffer_circular_t,
        *buffer_circular_r;         ///< endereço do buffer circular

unsigned short     head_t, tail_t,
			   	   head_r, tail_r;     ///< índices do buffer circular

unsigned short amostra;   

/*! instantes do aperto, da soltura da botoeira e o intervalo em multiplos de overflow (1ms) */
uint16_t multiplos, iteracao;  
uint8_t ApertouPta12; ///< flags de estado de reconhecimento do aperto do PTA12 */
uint8_t ApertouPta5; ///< flags de estado de reconhecimento do aperto do PTA5 */
uint8_t estado_terminal, times_passed;
uint8_t indice, nivel;
uint8_t ind;                ///< quantidade de elementos no buffer
uint8_t estado_pot = 0;

/*!
 * \fn InitSysTick (void)
 * \brief Inicializa o temporizador SysTick.
 */
void InitSysTick (void) {
	SYST_RVR = SysTick_RVR_RELOAD(5242880);   ///< 0.75s
	SYST_CVR = SysTick_CVR_CURRENT(0);                
	SYST_CSR |= (SysTick_CSR_CLKSOURCE_MASK |     ///< core clock: 20.971520MHz
			SysTick_CSR_TICKINT_MASK);            
	SYST_CSR |= SysTick_CSR_ENABLE_MASK;       ///< habilita o clock do SysTick     
}

/*!
 * \fn SysTick_Handler (void)
 * \brief Rotina de serviço para excecao SysTick (Vetor 15)
 */
void SysTick_Handler(void) {
	estado_pot = 1;	
}

/*!
 * \fn calibraADC (void)
 * \brief Calibra o modulo ADC
 * "Prior to calibration, the user must configure the ADC's clock source and 
 * frequency, low power configuration, voltage reference selection, sample time, 
 * and high speed configuration according to the application's clock source 
 * availability and needs." (Secao 28.4.6 de KL-25 Sub-Family 
 * Reference Manual)
 */
void calibraADC (void)
{
	uint16_t tmp=0;
	
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;      ///< habilita clock do modulo ADC0                             
	/*! Configuracao do relogio
	 * Os bits de ADC0_CFG1 sao resetados em 0: 
	 * ADLSMP=0 (short sample time) 
	 */ 
	ADC0_CFG1 &= (uint32_t)~(uint32_t)(ADC_CFG1_ADLPC_MASK); ///< operacao normal
	ADC0_CFG1 |= ADC_CFG1_ADIV(0x03) |             ///< ADCK = (source clock)/8
	            ADC_CFG1_ADLSMP_MASK |            ///< intervalo de amostragem longo
	            ADC_CFG1_MODE(0x03) |             ///< conversao em 16 bits
	            ADC_CFG1_ADICLK(0x01);            ///< (source clock) = (bus clock)/2
	/*! Configuracao dos ciclos adicionais 
	 * Os bits de ADC0_CFG2 sao resetados em 0: MUXSEL = 0 (canais ADxxa)  
	 */
	ADC0_CFG2 &= (uint32_t)~(uint32_t)(
	             ADC_CFG2_ADACKEN_MASK |     ///< desabilita a saida do sinal de clock      
	             ADC_CFG2_ADLSTS(0x03));     ///< 24 (20 extra+4) ADCK ciclos por amostra
	ADC0_CFG2 |=  ADC_CFG2_ADHSC_MASK;       ///< sequencia de conversao em alta velocidade com 2 ciclos adicionais
	/*! Os bits de ADC0_SC2 sao resetados em 0: ACFE=0 (desabilitada funcao de comparacao) */
	ADC0_SC2 &= ADC_SC2_REFSEL(0x00);    ///< Tensoes de referencia VREFH e VREFL                                
	  
	/*! Os bits de ADC0_SC3 sao resetados em 0 */
	ADC0_SC3 |= ADC_SC3_AVGE_MASK |  ///< hardware average habilitado
			    ADC_SC3_AVGS(0x03);  ///< numero de amostras por media = 32                
	ADC0_SC3 |= (uint32_t)(ADC_SC3_CAL_MASK);  ///< inicializa a calibracao	

	while (ADC0_SC3 & (uint32_t)(ADC_SC3_CAL_MASK))  {}  ///< aguarda a calibracao

	/*! Gere as correcoes de erros de ganhos, conforme o procedimento
	 *  na Secao 28.4.6 
	 */
	tmp = ADC0_CLP0 + ADC0_CLP1 + ADC0_CLP2 + ADC0_CLP3 + ADC0_CLP4 + ADC0_CLPS; ///< soma PS
	tmp = tmp/2;                            ///< divide por 2
	tmp |= (uint16_t)(0x8000);              ///< set o bit mais significativo
	ADC0_PG = tmp;                          ///< plus-side gain
	/*! Somente util para modos diferenciais */
	tmp = ADC0_CLM0 + ADC0_CLM1 + ADC0_CLM2 + ADC0_CLM3 + ADC0_CLM4 + ADC0_CLMS; ///< soma PS
	tmp = tmp/2;                            ///< divide por 2
	tmp |= (uint16_t)(0x8000);              ///< set o bit mais significativo
	ADC0_MG = tmp;	                        ///< minus side gain
}

/*!
 * \fn configuraADC (void)
 * \brief Configura o modo de operacao do módulo ADC
 */
void configuraADC(void) {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;     ///< habilita clock do modulo PORTB
	/*! Pino que serve o canal ADC0 */
	PORTB_PCR1 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK |        ///< baixa bandeira
			         PORT_PCR_MUX(0x07)));                            ///< funcao ADC0       
	/*! 
	 * Configuracao do relogio: conversao em 16 bits,
	 * frequencia ADCK = (bus clock)/2*8
	 */ 
	ADC0_CFG1 &= (uint32_t)~(uint32_t)(
	          ADC_CFG1_ADLSMP_MASK ) ;   ///< intervalo de amostragem curto
	ADC0_CFG1 |= ADC_CFG1_ADLPC_MASK;    ///< operacao low-power
	/*! 
	 * Configuracao da velocidade: MUXSEL=0 (canais ADxxa), ADACKEN=0 (desabilitada
	 * saida assincrona, ADLSTS=00 (24 ciclos)
	 */
	ADC0_CFG2 &= (uint32_t)~(uint32_t)(
			  ADC_CFG2_ADHSC_MASK);      ///< sequencia de conversao normal      
	/*! 
	 * Configura o modo de conversao: ADCO=0 (modo não continuo), AVGE=1 
	 * (habilita média por hardware)
	 */
	ADC0_SC3 &= (uint32_t)~(uint32_t)(
				ADC_SC3_AVGS(0x03));    ///<  numero de amostras por media = 4     
	ADC0_SC3 |= ADC_SC3_CALF_MASK;      ///< baixa a bandeira de "calibration failed"
      
	/*! ADC0_SC1A: COCO=0, DIFF=0 (medida unipolar) e ADCH=0b11111 (canais desabilitados) no reset */
	ADC0_SC1A |= ADC_SC1_AIEN_MASK;           ///< habilita interrupcao
}

/*!
 * \fn initADC (void) 
 * \brief Configura os módulos de interface com os periféricos
 */
void initADC(void) {
	calibraADC();         ///< calibra ADC
	configuraADC();       ///< configura modo de operacao do ADC

}



/*! 
 * \fn ADC0_IRQHandler (void)
 * \brief Rotina de servico para atender ADC0 
 */
void ADC0_IRQHandler (void) {
	if(( ADC0_SC1A & ADC_SC1_COCO_MASK ) == ADC_SC1_COCO_MASK)
	{
		amostra = ADC0_RA;              ///< ler o registrador do conversor (limpa COCO)
		ind = 1;
	}
}

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
 * \fn InitPort (void)
 * \brief Initialize the PORTs to serve other modules
 */
void InitPort (void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK|      ///< PORTB=SIM_SCGC5[9]=1 (Clock gate de PORTA habilitado)     
        SIM_SCGC5_PORTB_MASK|       ///< PORTB=SIM_SCGC5[10]=1 (Clock gate de PORTB habilitado)     
        SIM_SCGC5_PORTC_MASK|       ///< PORTB=SIM_SCGC5[11]=1 (Clock gate de PORTB habilitado)     
        SIM_SCGC5_PORTD_MASK;       ///< PORTD=SIM_SCGC5[12]=1  (Clock gate de PORTD habilitado)
    
    /*! Configura os pinos PORTC[10,7:2,0] como GPIO de saida para controlar o latch dos leds vermelhos */
    PORTC_PCR0 = PORT_PCR_MUX(0x1);       ///< D0, D2-D7 dos dados (GPIO)
    PORTC_PCR1 = PORT_PCR_MUX(0x1);
    PORTC_PCR2 = PORT_PCR_MUX(0x1);
    PORTC_PCR3 = PORT_PCR_MUX(0x1);
    PORTC_PCR4 = PORT_PCR_MUX(0x1);
    PORTC_PCR5 = PORT_PCR_MUX(0x1);
    PORTC_PCR6 = PORT_PCR_MUX(0x1);
    PORTC_PCR7 = PORT_PCR_MUX(0x1);
    PORTC_PCR8 = PORT_PCR_MUX(0x1);        ///< RS do LCD, bit que determina se o LCD está recebendo instrução (0)ou dado (1)
    PORTC_PCR9 = PORT_PCR_MUX(0x1);        ///< E do LCD  (p=0) sinal de enable
    PORTC_PCR10 = PORT_PCR_MUX(0x1);      ///< LE do latch

    /*! Configura como saídas*/
    GPIOC_PDDR |= GPIO_PDDR_PDD(GPIO_PIN(0) |
                GPIO_PIN(1) |
                GPIO_PIN(2) |
                GPIO_PIN(3) |
                GPIO_PIN(4) |
                GPIO_PIN(5) |
                GPIO_PIN(6) |
                GPIO_PIN(7) |
                GPIO_PIN(8) |
                GPIO_PIN(9) |
                GPIO_PIN(10));
    
    //GPIOC_PDOR &= 0xFFFFF800;       
    GPIOC_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(0) |
                GPIO_PIN(1) |
                GPIO_PIN(2) |
                GPIO_PIN(3) |
                GPIO_PIN(4) |
                GPIO_PIN(5) |
                GPIO_PIN(6) |
                GPIO_PIN(7) |
                GPIO_PIN(8) |
                GPIO_PIN(9) |
                GPIO_PIN(10));
    
}

/*!
 * \fn pulso (char p, int t)
 * \brief Gera um pulso "Enable" de t*10us.
 * \param[in] p para LCD (p=0) e para Leds (p=1).
 */
void pulso(char p, unsigned int t) {
    GPIOC_PSOR |= 1 << (9 + p);     ///< Seta 1 no PORTC[9+p]
    delay10us(10);                 ///< mantêm aprox. t*10us em 1
    GPIOC_PCOR |= 1 << (9 + p);     ///< Limpa em 0 o PORTC[9+p]
    delay10us(t);                 ///< mantêm aprox. t*10us em 1
}

/*!
 * \fn enviaLedLCD (char c)
 * \brief Envia ao LCD um byte pelos pinos PORTC[7:0]
 * \param[in] c caracter em ASCII.
 */
void enviaLedLCD(char c, int time) {
    GPIOC_PCOR |= 0x000000FF;      ///< limpa em 0 PORTC[7:0]
    GPIOC_PSOR |= (unsigned int)c; ///< Seta os bits que devem ser setados
    pulso(0, time);                      ///< dispara o pulso "Enable" do LCD
    pulso(1, time);                      ///< dispara o pulso "Enable" do Latch 74573
}

/*!
 * \fn RS (char l)
 * \brief Envia ao LCD o sinal RS pelo pino PORTC[8].
 * \param[in] l valor do RS (0, byte de instrucao e 1, byte de dados).
 */
void RS(char l) {
    
    if(l) {                      ///< (l != 0)
        GPIOC_PSOR = 0x00000100; ///< Seta o LCD no modo de dados
    } else {
        GPIOC_PCOR = 0x00000100; ///< Seta o LCD no modo de instrução
    }    

}

/*!
 * \fn inicLCD (void)
 * \brief Inicializa o LCD com a sequência de instruções recomendada pelo fabricante
 */
void inicLCD(void) {
    char d[] = {0x38,0x0C, 0x06, 0x01, 0x00}; ///< sequencia de comandos
    int k;
    for(k = 0; k < 34; k++) { ///< Espera mais de 100ms
        delay10us(10000);
    }              
    RS(0);                   ///< Seta o LCD no modo de instrucao
    k = 0;
    while(d[k] != 0) {
    	while(d[k] != 0) {
    		if (d[k] == 1)
    			enviaLedLCD(d[k],154);
    		else
    			enviaLedLCD(d[k],4);
    	k++;
    	} 
    }
}

/*!
 * \fn trocaLinhaLCD (void)
 * \brief Muda a posicao do "adress counter" para a primeira posicao da segunda linha do lcd.
 */
void trocaLinhaLCD(void){
    RS(0);
    enviaLedLCD(0xC0, 5);
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
           enviaLedLCD(0x40+(position*8), 5);
           /*! ativa o envio de dados */
           RS(1);
        for(i=0;i<8;i++)
            enviaLedLCD(ptr[i], 5);
   }
}

/*!
 * \fn mandaString (char *s)
 * \brief Envia uma string de caracteres.
 * \param[in] s endereço inicial da string.
 */
void mandaString(char * s) {
    RS(1);                          ///< Seta o LCD no modo de dados
    while (*s) {                    ///< enquanto o conteúdo do endereço != 0
        enviaLedLCD(*s, 5);          ///< envia o byte
        s++;                        ///< incrementa o endereço
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
        s++;                    ///< incrementa o endereco por unidade do tipo de dados
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
 * \fn mostra_lcd(char *opt, char *msg)
 * \brief Imprime no lcd cor e frequencia selecionados pelo usuario
 * \param[in] opcao cor ou frequencia a ser mostrada na tela
 * \param[i] nome da cor ou frequencia em string a ser mostrada no lcd
 */
void mostra_lcd(char *opt, char *msg){
    if(strcmp(opt, "cor") == 0){
        RS(0);
        enviaLedLCD(0x80, 5);
        mandaString("Cor: ");
        mandaString(msg);
        mandaString("    ");
    }
    
    if(strcmp(opt, "freq") == 0){
        /*! passa a escrever na segunda linha do lcd */
        trocaLinhaLCD();
        mandaString("Frequ");
        /*! escreve o caractere "e" com acento */
        enviaLedLCD(1, 5);              
         mandaString("ncia: ");
         mandaString(msg);

    }
}

/*!
 * \fn initUART (void)
 * \brief Inicializa os pinos conectados a UART0 e ao OpenSDA.
 */
void initUART(void) {
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
    SIM_SOPT2 |= SIM_SOPT2_UART0SRC(0x1);    ///< configura a fonte de relógio
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

    UART0_C4 &= ~(UART0_C4_OSR(0x0C));                           ///< taxa de superamostragem: 3 + 1 = 4
    
    UART0_C5 |= UART0_C5_BOTHEDGE_MASK;
    
    /*!
     * baud rate 4800 (Seção 8.3.2 em
     * ftp://ftp.dca.fee.unicamp.br/pub/docs/ea871/ARM/KLQRUG.pdf)
     */
    UART0_BDH |= UART_BDH_SBR(0x4);                             
    UART0_BDL |= UART_BDL_SBR(0x44);

    
    UART0_S1 |= (UART0_S1_PF_MASK |              ///< Registradores de estado: w1c
                UART0_S1_FE_MASK |
                UART0_S1_NF_MASK |
                UART0_S1_OR_MASK |
                UART0_S1_IDLE_MASK);
    
    UART0_C2 |= (UART_C2_RIE_MASK |      ///< ativa interrupção de recepção
                UART_C2_RE_MASK |        ///< habilita o canal receptor
                UART_C2_TE_MASK);         ///< habilita o canal transmissor               
}

/*!
 * \fn insere_item_t (char item)
 * \brief Insere um item no buffer circular
 * \param[in] item caracter a ser inserido
 */
void insere_item_t (char item) {
    unsigned short tmp;
    
    if ((head_t==TAM_MAX-1 && tail_t == 0) || head_t == tail_t-1) {            /*! head muito rápido ou tail muito rápido*/
        /*! Não há espaço. Uma estratégia: aguardar 5 casas. */
        tmp = (tail_t+5)%TAM_MAX;
        while (tail_t != tmp);
    }
    buffer_circular_t[head_t++] = item;
    head_t %= TAM_MAX;
    
    if (!(UART0_C2 & UART_C2_TIE_MASK)) UART0_C2 |= UART_C2_TIE_MASK;    ///< ativa a máscara de interrupção
}

/*!
 * \fn remove_item_t (void)
 * \brief Remova um itemdo buffer circular
 * \return the item pointed by tail.
 */
char remove_item_t () {
    char d;
    
    d = buffer_circular_t[tail_t++];
    tail_t %= TAM_MAX;
    if (head_t == tail_t) {
        /*! Buffer vazio */
        UART0_C2 &= ~UART_C2_TIE_MASK;    ///< desativa a máscara de interrupção        
    }
    return d;
}

/*!
 * \fn insere_item_r (char item)
 * \brief Insere um item no buffer circular
 * \param[in] item caracter a ser inserido
 */
void insere_item_r (char item) {
    unsigned short tmp;
    
    if ((head_r==TAM_MAX-1 && tail_r == 0) || head_r == tail_r-1) {            /*! head muito rápido ou tail muito rápido*/
        /*! Não há espaço. Uma estratégia: aguardar 5 casas. */
        tmp = (tail_r+5)%TAM_MAX;
        while (tail_r != tmp);
    }
    buffer_circular_r[head_r++] = item;
    head_r %= TAM_MAX;

}

/*!
 * \fn remove_item_r (void)
 * \brief Remova um item do buffer circular
 * \return the item pointed by tail.
 */
char remove_item_r () {
    char d;
    
    d = buffer_circular_r[tail_r++];
    tail_r %= TAM_MAX;
    
    return d;
}

/*!
 * \fn terminalHandler()
 * \brief Função que trata a palavra recebida do terminal
 * */
void terminalHandler(){
    char word[10];                 
    int tamanho;                
    int i=0;
    
    tamanho = (head_r - tail_r) + 1;
    
    /*! Passa a palavra recebida para o vetor word*/
    while(tail_r != head_r) {
        word[i] = remove_item_r();
        i++;
    }
 
    if(tamanho > 3){  
        if(strncmp(word, "vermelho", 8) == 0) {
            indice = 0;
        } else if(strncmp(word, "azul", 4) == 0) {
            indice = 1;
        } else if(strncmp(word, "amarelo", 6) == 0){
            indice = 2;
        } else if(strncmp(word, "laranja", 7) == 0){
            indice = 3;
        } else if(strncmp(word, "verde", 5) == 0){
            indice = 4;
        } else if(strncmp(word, "branco", 6) == 0){
            indice = 5;
        } else if(strncmp(word, "preto", 6) == 0){
            indice = 6;
            
        }     
    } else  {
        if(strncmp(word, "0", 1) == 0){
            nivel = 0;
        } else if(strncmp(word, "1", 1) == 0){
            nivel = 1;
        } else if(strncmp(word, "2", 1) == 0){
            nivel = 2;
        } else if(strncmp(word, "3", 1) == 0){
            nivel = 3;
        } else if(strncmp(word, "4", 1) == 0){
            nivel = 4;
        }  else if(strncmp(word, "5", 1) == 0){
            nivel = 5;
        } else if(strncmp(word, "6", 1) == 0){
            nivel = 6;
        } else if(strncmp(word, "7", 1) == 0){
            nivel = 7;
        } else if(strncmp(word, "8", 1) == 0){
            nivel = 8;
        } else if(strncmp(word, "8", 1) == 0){
            nivel = 9;

        }
    }
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
        
        if(d == '\r' || d == '\n'){
            insere_item_r('\0');      ///< insere \0 ao final da string
            insere_item_t('\r');    ///< volta para o início da linha no terminal    
            insere_item_t('\n');    ///< pula linha no terminal
            estado_terminal = 1;
        } else {
            insere_item_r(d);  ///< insere item                    ///< põe caracter no buffer de recepção
            insere_item_t(buffer_circular_r[head_r - 1]);        ///< imprime o caracter no terminal
        }
     }  else if (UART0_S1 & UART0_S1_TDRE_MASK) {
        UART0_D = remove_item_t();   ///< remove item
    }    
}

/*!
 *
 * \fn mandaStringUART()
 * \brief Envia string para o terminal
 *
 * */
void mandaStringUART(char *s){
    while (*s) {                    ///< enquanto o conteúdo do endereço != 0
        insere_item_t(*s);          ///< envia o byte
        s++;                        ///< incrementa o endereço
    }
    
}

/*!
 *
 * \fn string_terminal(char *cor, char* freq)
 * \brief Envia string para o terminal
 * \param[in] string da cor atual do led
 * \param[in] string que indica a frequencia atual de piscada do led
 * 
 * */
void string_terminal(char* cor, char* freq){

    mandaStringUART("Cor: ");    ///< Envia a cor para o terminal
    mandaStringUART(cor);
    insere_item_t ('\n');
    insere_item_t('\r');
    mandaStringUART("Frequência: ");    ///< Envia a cor para o terminal
    mandaStringUART(freq);
    insere_item_t ('\n');
    insere_item_t('\r');
}

/*!
 * \fn InitTPM (void)
 *
 * \brief Configura os módulos TPM0, TPM1, TPM2 de forma que o período do seu clock seja 1ms.
 *
 */
void InitTPM(void) {
    SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK |       ///< TPM0=SIM_SCGC6[24]=1 (Clock gate de TPM0 habilitado)
                 SIM_SCGC6_TPM1_MASK |       ///< TPM0=SIM_SCGC6[25]=1 (Clock gate de TPM1 habilitado)
                 SIM_SCGC6_TPM2_MASK;        ///< TPM2=SIM_SCGC6[26]=1 (Clock gate de TPM2 habilitado)
    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(0b01);     ///< TPMSRC=SIM_SOPT2[25:24]=01 (MCGFLLCLK|MCGPLLCLK/2)
    SIM_SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK;  ///< PLLFLLSEL=SIM_SOPT2[16]=0 (MCGFLLCLK)        
    
    /*!
     * Veja a relacao dos campos dos registradores na Figura 31-1 de KL-25 Sub-Family Reference Manual
     * Configuracao dos modulos TPM0, TPM1 e TPM2
     * Periodo_{UP} = (MOD+1) * (PS/(frequencia de source clock))      (CPWMS=0)
     * Periodo_{UP-DOWN} = (MOD) * (PS/(frequencia de source clock)) * (1/(1+CPWMS)) (CPWMS=1)
     * Fórmula geral:
     * Periodo_{TPMS} ~ (MOD) * (PS/(frequencia de source clock)) * (1/(1+CPWMS))
     * frequência de MCGFLLCLK =  20.971520MHz
     *
     * Vale observar que para programar um duty cycle 100%, MOD < 0xFFFF (Seção 31.4.6)
     */
    /*! (1) TPM0 */
    TPM0_SC |= (TPM_SC_TOF_MASK |            ///< TOF=TPM0_TOF[7]: w1c (limpa a pendenciaa do TPM0)
                TPM_SC_CMOD(0x1) |           ///< CMOD=TPM0_SC[4:3]=0b01 (incrementa a cada pulso do LPTPM)
                TPM_SC_PS(0b101) );            ///< PS=TPM0_SC[2:0]=0b000 (Fator Pre-escala=32)
    TPM0_SC &= ~(TPM_SC_DMA_MASK |           ///< DMA=TPM0_SC[8]=0 (desabilita DMA)
                TPM_SC_TOIE_MASK |           ///< TOIE=TPM0_SC[6]=0 (habilita interrup&ccedil;&atilde;o "overflow")
                TPM_SC_CPWMS_MASK);          ///< CPWMS=TPM0_SC[5]=0 (modo de contagem crescente)
    
    TPM0_MOD &= TPM_MOD_MOD(COUNTER_OVF);    ///< MOD=TPM0_MOD[15:0]=COUNTER_OVF atualizado conforme CMOD e CPWMS
                                             ///<  (Se&ccedil;&atilde;o 31.4.8.1 de KL25 Sub-Family Reference Manual)
    TPM0_CNT |= TPM_CNT_COUNT(0x0)  ;        ///< COUNT=TPM0_CNT[15:0]: w (qq acesso de escrita reseta LPTPM)
    
    TPM0_CONF &= ~(TPM_CONF_CROT_MASK |      ///< CROT=TPM0_CONF[18]=0 (n&atilde;o recarrega o valor de LPTPM no gatilho)
            TPM_CONF_CSOO_MASK |             ///< CSOO=TPM0_CONF[17]=0 (continua contagem em overflow)
            TPM_CONF_CSOT_MASK |             ///< CSOT=TPM0_CONF[16]=0 (inicia contagem quando habilitado)
            TPM_CONF_GTBEEN_MASK |             ///< GTBEEN=TPM0_CONF[9]=0 (LPTPM contador como base de tempo)
            TPM_CONF_DOZEEN_MASK);           ///< DOZEEN=TPM0_CONF[5]=0 (continua contagem em hiberna&ccedil;&atilde;o)
    TPM0_CONF |= TPM_CONF_DBGMODE(0b11);     ///< DBGMODE=TPM0_CONF[7:6]=0b11 (continua contagem em Debug)
    
    /*! (2) TPM1 */
    TPM1_SC |= (TPM_SC_TOF_MASK |            ///< TOF=TPM1_TOF[7]: w1c (limpa a pendenciaa do TPM0)
                TPM_SC_CMOD(0x1) |           ///< CMOD=TPM1_SC[4:3]=0b01 (incrementa a cada pulso do LPTPM)
                TPM_SC_PS(0b101) );            ///< PS=TPM1_SC[2:0]=0b000 (Fator Pre-escala=32)
    TPM1_SC &= ~(TPM_SC_DMA_MASK |           ///< DMA=TPM1_SC[8]=0 (desabilita DMA)
                TPM_SC_TOIE_MASK |           ///< TOIE=TPM1_SC[6]=0 (habilita interrupcao "overflow")
                TPM_SC_CPWMS_MASK);          ///< CPWMS=TPM1_SC[5]=0 (modo de contagem crescente)
    
    TPM1_MOD &= TPM_MOD_MOD(COUNTER_OVF);    ///< MOD=TPM1_MOD[15:0]=COUNTER_OVF atualizado conforme CMOD e CPWMS
                                             ///<  (Se&ccedil;&atilde;o 31.4.8.1 de KL25 Sub-Family Reference Manual)
    TPM1_CNT |= TPM_CNT_COUNT(0x0)  ;        ///< COUNT=TPM1_CNT[15:0]: w (qq acesso de escrita reseta LPTPM)
    
    TPM1_CONF &= ~(TPM_CONF_CROT_MASK |      ///< CROT=TPM1_CONF[18]=0 (n&atilde;o recarrega o valor de LPTPM no gatilho)
            TPM_CONF_CSOO_MASK |             ///< CSOO=TPM1_CONF[17]=0 (continua contagem em overflow)
            TPM_CONF_CSOT_MASK |             ///< CSOT=TPM1_CONF[16]=0 (inicia contagem quando habilitado)
            TPM_CONF_GTBEEN_MASK |             ///< GTBEEN=TPM1_CONF[9]=0 (LPTPM contador como base de tempo)
            TPM_CONF_DOZEEN_MASK);           ///< DOZEEN=TPM1_CONF[5]=0 (continua contagem em hibernacao)
    TPM1_CONF |= TPM_CONF_DBGMODE(0b11);     ///< DBGMODE=TPM1_CONF[7:6]=0b11 (continua contagem em Debug)
    
    /*! (3) TPM2 */
    TPM2_SC |= (TPM_SC_TOF_MASK |            ///< TOF=TPM2_TOF[7]: w1c (limpa a pendencia do TPM2)
                TPM_SC_CMOD(0x1) |           ///< CMOD=TPM2_SC[4:3]=0b01 (incrementa a cada pulso do LPTPM)
                TPM_SC_PS(0b101) );            ///< PS=TPM2_SC[2:0]=0b000 (Fator Pre-escala=32)
    TPM2_SC &= ~(TPM_SC_DMA_MASK |           ///< DMA=TPM2_SC[8]=0 (desabilita DMA)
                TPM_SC_TOIE_MASK |           ///< TOIE=TPM2_SC[6]=0 (desabilita interrupcao "overflow")
                TPM_SC_CPWMS_MASK);          ///< CPWMS=TPM2_SC[5]=0 (modo de contagem crescente)
    
    TPM2_MOD &= TPM_MOD_MOD(COUNTER_OVF);    ///< MOD=TPM2_MOD[15:0]=COUNTER_OVF atualizado conforme CMOD e CPWMS
                                             ///<  (Se&ccedil;&atilde;o 31.4.8.1 de KL25 Sub-Family Reference Manual)
    TPM2_CNT |= TPM_CNT_COUNT(0x0)  ;        ///< COUNT=TPM2_CNT[15:0]: w (qq acesso de escrita reseta LPTPM)
    
    TPM2_CONF &= ~(TPM_CONF_CROT_MASK |      ///< CROT=TPM2_CONF[18]=0 (n&atilde;o recarrega o valor de LPTPM no gatilho)
            TPM_CONF_CSOO_MASK |             ///< CSOO=TPM2_CONF[17]=0 (continua contagem em overflow)
            TPM_CONF_CSOT_MASK |             ///< CSOT=TPM2_CONF[16]=0 (inicia contagem quando habilitado)
            TPM_CONF_GTBEEN_MASK |             ///< GTBEEN=TPM2_CONF[9]=0 (LPTPM contador como base de tempo)
            TPM_CONF_DOZEEN_MASK);           ///< DOZEEN=TPM2_CONF[5]=0 (continua contagem em hibernacao)
    TPM2_CONF |= TPM_CONF_DBGMODE(0b11);     ///< DBGMODE=TPM2_CONF[7:6]=0b11 (continua contagem em Debug)
}

/*!
 * \fn InitTPMPWM (void)
 *
 * \brief Configura o canal 0 e o canal 1 do módulo TPM2, e o canal 1 do módulo TPM0 com a função "PWM".
 *
 */
void InitTPMPWM(void) {
    PORTB_PCR18 |= (PORT_PCR_ISF_MASK |       ///< ISF=PORTB_PCR18[14]: w1c (limpa a pendência)
                  PORT_PCR_MUX(0x3) );       ///< MUX=PORTB_PCR18[10:8]=0b011 (TPM2_CH0)
    PORTB_PCR19 |= (PORT_PCR_ISF_MASK |       ///< ISF=PORTB_PCR19[14]: w1c (limpa a pendência)
                  PORT_PCR_MUX(0x3) );       ///< MUX=PORTB_PCR19[10:8]=0b011 (TPM2_CH1)
    PORTD_PCR1 |= (PORT_PCR_ISF_MASK |       ///< ISF=PORTB_PCR19[14]: w1c (limpa a pendência)
                  PORT_PCR_MUX(0x4) );       ///< MUX=PORTB_PCR19[10:8]=0b011 (TPM0_CH1)
        
    /*!
     * Configuracao do canal 0 (led R): CPWMS(0), MSnB:MSnA(10), ELSnB:ELSnA(11) - edge-aligned PWM
     */
    TPM2_C0SC |= (TPM_CnSC_CHF_MASK);       ///< CHF=TPM2_C0SC[7]: w1c (limpa a pend&ecirc;ncia do canal)
    TPM2_C0SC &= ~(TPM_CnSC_CHIE_MASK |     ///< CHIE=TPM2_C0SC[6]=0 (desabilita interrup&ccedil;&atilde;o)
                TPM_CnSC_DMA_MASK);         ///< DMA=TPM2_C0SC[0]=0 (desabilita DMA)
    TPM2_C0V = TPM_CnV_VAL((short)(1.1*COUNTER_OVF)); ///< VAL=TPM2_C0V[15:0]=1.1*COUNTER_OVF
                                                      ///< COUNTER_OVF para 100% duty cycle
                                                      ///< conforme recomenda&ccedil;&atilde;o da Se&ccedil;&atilde;o 31.4.6
    /*! Selecao de modo, transicao enivel (desabilita o canal) */
    TPM2_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM2_C0SC[5]=0
                TPM_CnSC_MSA_MASK |         ///< MSA=TPM2_C0SC[4]=0
                TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C0SC[3]=0
                TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C0SC[2]=0

    /*!
     * Configuracao do canal 1 (led G): CPWMS(0), MSnB:MSnA(10), ELSnB:ELSnA(11) - edge-aligned PWM
     */
    TPM2_C1SC |= (TPM_CnSC_CHF_MASK);      ///< CHF=TPM2_C1SC[7]: w1c (limpa a pend&ecirc;ncia do canal)
    TPM2_C1SC &= ~(TPM_CnSC_CHIE_MASK |     ///< CHIE=TPM2_C1SC[6]=0 (desabilita interrup&ccedil;&atilde;o)
                TPM_CnSC_DMA_MASK);         ///< DMA=TPM2_C1SC[0]=0 (desabilita DMA)
    TPM2_C1V = TPM_CnV_VAL((short)(1.1*COUNTER_OVF)); ///< VAL=TPM2_C1V[15:0]=1.1*COUNTER_OVF
                                                      ///< COUNTER_OVF para 100% duty cycle
                                                      ///< conforme recomenda&ccedil;&atilde;o da Se&ccedil;&atilde;o 31.4.6
    /*! Selecao de modo, transicao e nivel (desabilita o canal) */
    TPM2_C1SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM2_C1SC[5]=0
                TPM_CnSC_MSA_MASK |         ///< MSA=TPM2_C1SC[4]=0
                TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C1SC[3]=0
                TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C1SC[2]=0

    /*!
     * Configuracao do canal 0 (led B): CPWMS(0), MSnB:MSnA(10), ELSnB:ELSnA(11) - edge-aligned PWM
     */
    TPM0_C1SC |= (TPM_CnSC_CHF_MASK);       ///< CHF=TPM0_C1SC[7]: w1c (limpa a pend&ecirc;ncia do canal)
    TPM0_C1SC &= ~(TPM_CnSC_CHIE_MASK |     ///< CHIE=TPM0_C1SC[6]=0 (desabilita interrup&ccedil;&atilde;o)
                TPM_CnSC_DMA_MASK);         ///< DMA=TPM0_C1SC[0]=0 (desabilita DMA)
    TPM0_C1V = TPM_CnV_VAL((short)(1.1*COUNTER_OVF)); ///< VAL=TPM2_C1V[15:0]=1.1*COUNTER_OVF
                                                      ///< COUNTER_OVF para 100% duty cycle
                                                      ///< conforme recomenda&ccedil;&atilde;o da Se&ccedil;&atilde;o 31.4.6
    /*! Selecao de modo, transicao e nivel (desabilita o canal) */
    TPM0_C1SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM0_C1SC[5]=0
                TPM_CnSC_MSA_MASK |         ///< MSA=TPM0_C1SC[4]=0
                TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM0_C1SC[3]=0
                TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM0_C1SC[2]=0
}

/**
 * \fn enableNVIC (void)
 * \brief Habilita interrupção do módulo TPM0 (vetor de interrupção = 33) e
 *        do módulo TPM1 (vetor de interrupção = 34) via NVIC.
 *
 */
void enableNVIC(void) {
    NVIC_ISER |= (1 << (33-16) | ///< NVIC_ISER[17]=1 (habilita IRQ17)
            1 << (34-16) );      ///< NVIC_ISER[18]=1 (habilita IRQ18)
    NVIC_ICPR |= (1 << (33-16) | ///< NVIC_ICPR[17]=1 (limpa as pendencias)
            1 << (34-16) );      ///< NVIC_ICPR[18]=1 (limpa as pendencias)
    NVIC_IPR4 |= NVIC_IP_PRI_17(0x80); ///< seta prioridade 2
    NVIC_IPR4 |= NVIC_IP_PRI_18(0x80); ///< seta prioridade 2         
    
    NVIC_ISER |= 1 << (28-16);            ///< Habilita interrupção UART
    NVIC_ICPR |= 1 << (28-16);
    NVIC_IPR3 |= NVIC_IP_PRI_12(0x40);    ///< Nível de prioridade = 1
    
	NVIC_IPR3 = (uint32_t)((NVIC_IPR3 & (uint32_t)~(uint32_t)( ///< NVIC_IPR3: PRI_15=0x80
	               NVIC_IP_PRI_15(0x7F)
	              )) | (uint32_t)(
	               NVIC_IP_PRI_15(0x80)
	              ));                                  
	NVIC_ISER |= NVIC_ISER_SETENA(GPIO_PIN(31-16));   ///< Habilita interrupcao ADC0                               
	NVIC_ICPR |= NVIC_ICPR_CLRPEND(GPIO_PIN(31-16));  ///< Limpa as possiveis pendencias do ADC0    
}

/*!
 * \fn FTM0_IRQHandler (void)
 * \brief Rotina de serviço para TPM0 (Vetor 33, IRQ17, prioridade em NVICIPR4[15:14])
 *        Captar ambas bordas e registra os instantes em que ocorreram
 */
void FTM0_IRQHandler(void) {
    static uint32_t tof0_vezes;
    if ((TPM0_STATUS & TPM_STATUS_TOF_MASK) && (TPM0_SC & TPM_SC_TOIE_MASK)) {
        tof0_vezes++;
        if (tof0_vezes >= TIMEOUT) {
            tof0_vezes = TIMEOUT;
            /*! Disable channel */
            TPM0_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM2_C0SC[5]=0
                        TPM_CnSC_MSA_MASK |         ///< MSA=TPM2_C0SC[4]=0
                        TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C0SC[3]=0
                        TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C0SC[2]=0
            
            /*!
             * Configura o canal 0 (LedR1): CPWMS(0), MSnB:MSnA(01), ELSnB:ELSnA(11) - set output on match
             */         
            TPM0_C0SC &= ~(TPM_CnSC_MSB_MASK);      ///< MSB=TPM0_C2SC[5]=0
            TPM0_C0SC |= (TPM_CnSC_MSA_MASK |       ///< MSA=TPM0_C2SC[3]=1
                          TPM_CnSC_ELSA_MASK |      ///< ELSA=TPM0_C2SC[4]=1
                          TPM_CnSC_ELSB_MASK );     ///< ELSB=TPM0_C2SC[2]=1
                          
            TPM0_C0V = TPM0_CNT+0x5;                ///< espera por 5 ticks do TPM0
            TPM0_C0SC |= TPM_CnSC_CHIE_MASK;        ///< habilita a interrupcao                 
            TPM0_SC &= ~TPM_SC_TOIE_MASK;           ///< desabilita timer overflow flag    
            
            times_passed = 1;
            SYST_CSR &= ~SysTick_CSR_ENABLE_MASK;       ///< desabilita o clock do SysTick   

        }
        TPM0_STATUS |= TPM_STATUS_TOF_MASK;     ///< TOF=TPM1_STATUS[8]: w1c (limpa a pendencia do canal)
    }
    
    if (TPM0_STATUS & TPM_STATUS_CH0F_MASK){
        if(TPM0_C0SC == (TPM0_C0SC | TPM_CnSC_CHIE_MASK)){
            TPM0_C0SC |= TPM_CnSC_CHF_MASK;       ///< CHF=TPM0_C0SC[7]: w1c (limpa a pendencia do canal)
            TPM0_C0SC &= ~TPM_CnSC_CHIE_MASK;     ///< desabilita a interrupcao
        }else{
            if (GPIOA_PDIR & GPIO_PIN(5)) { ///< soltou PTA5                      
                    iteracao = tof0_vezes;                 ///< m&uacute;ltiplos de overflow
                    /*! Disable channel */
                    TPM0_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM2_C0SC[5]=0
                                TPM_CnSC_MSA_MASK |         ///< MSA=TPM2_C0SC[4]=0
                                TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C0SC[3]=0
                                TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C0SC[2]=0
                    /*!
                     * Configuracao do canal 0 (LedR1): CPWMS(0), MSnB:MSnA(01), ELSnB:ELSnA(10) - clear output on match
                     */
                    TPM0_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM0_C2SC[5]=0
                            TPM_CnSC_ELSA_MASK);            ///< ELSA=TPM0_C2SC[4]=0
                    TPM0_C0SC |= (TPM_CnSC_MSA_MASK |       ///< MSA=TPM0_C2SC[3]=1
                                  TPM_CnSC_ELSB_MASK );     ///< ELSB=TPM0_C2SC[2]=1
                                  
                    TPM0_C0V = TPM0_CNT+0x5;                ///< espera por 5 ticks do TPM0
                    TPM0_C0SC |= TPM_CnSC_CHIE_MASK;        ///< habilita a interrup&ccedil;&atilde;o                 

                    TPM0_SC &= ~TPM_SC_TOIE_MASK;           ///< desabilita timer overflow flag        
                    NVIC_ICPR |= (1 << (33-16) );           ///< NVIC_ICPR[17]=1 (limpa as pend&ecirc;ncias do TPM0)
                    ApertouPta5 = 1;                       ///< seta a flag indicadora de "multiplos" ser v&aacute;lido
                    times_passed = 0;
                    
                    SYST_CSR |= SysTick_CSR_ENABLE_MASK;       ///< habilita o clock do SysTick   
                    
            } else {     ///< apertou PTA5
                tof0_vezes=0;   
                TPM0_SC |= TPM_SC_TOIE_MASK;   ///< habilita timer overflow flag
            }
            TPM0_C2SC |= TPM_CnSC_CHF_MASK;     ///< CHF=TPM1_C1SC[7]: w1c (limpa a pend&ecirc;ncia do canal)
                                
        }
    }
    
}

/*!
 * \fn FTM1_IRQHandler (void)
 * \brief Rotina de serviço para TPM1 (Vetor 34, IRQ18, prioridade em NVICIPR4[23:22])
 *        Captar ambas bordas e registra os instantes (valor do contador) em que ocorreram.
 */
void FTM1_IRQHandler(void) {
    static uint32_t tof1_vezes;
    
    if ((TPM1_STATUS & TPM_STATUS_TOF_MASK) && (TPM1_SC & TPM_SC_TOIE_MASK)) {
        tof1_vezes++;
        if (tof1_vezes >= TIMEOUT) {
           tof1_vezes = TIMEOUT;
            /*! Disable channel */
           TPM0_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM2_C0SC[5]=0
                        TPM_CnSC_MSA_MASK |         ///< MSA=TPM2_C0SC[4]=0
                        TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C0SC[3]=0
                        TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C0SC[2]=0
            /*!
             * Configuracao do canal 0 (LedR1): CPWMS(0), MSnB:MSnA(01), ELSnB:ELSnA(11) - set output on match
             */
           TPM0_C0SC &= ~(TPM_CnSC_MSB_MASK);      ///< MSB=TPM0_C2SC[5]=0
           TPM0_C0SC |= (TPM_CnSC_MSA_MASK |       ///< MSA=TPM0_C2SC[3]=1
                         TPM_CnSC_ELSA_MASK |      ///< ELSA=TPM0_C2SC[4]=1
                         TPM_CnSC_ELSB_MASK );     ///< ELSB=TPM0_C2SC[2]=1
                          
           TPM0_C0V = TPM0_CNT+0x5;                ///< espera por 5 ticks do TPM0
           TPM0_C0SC |= TPM_CnSC_CHIE_MASK;        ///< habilita a interrup&ccedil;&atilde;o                 
           TPM1_SC &= ~TPM_SC_TOIE_MASK;           ///< desabilita timer overflow flag  
            
           times_passed = 1;
           
           SYST_CSR &= ~SysTick_CSR_ENABLE_MASK;       ///< desabilita o clock do SysTick   
        }
        TPM1_STATUS |= TPM_STATUS_TOF_MASK;     ///< TOF=TPM1_STATUS[8]: w1c (limpa a pend&ecirc;ncia do canal)
    } else if (TPM1_STATUS & TPM_STATUS_CH0F_MASK) {
        if (GPIOA_PDIR & GPIO_PIN(12)) { ///< soltou PTA12

            multiplos = tof1_vezes;                 ///< m&uacute;ltiplos de overflow
            /*! Disable channel */
            TPM0_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM2_C0SC[5]=0
                        TPM_CnSC_MSA_MASK |         ///< MSA=TPM2_C0SC[4]=0
                        TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C0SC[3]=0
                        TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C0SC[2]=0
            /*!
             * Configuracao do canal 0 (LedR1): CPWMS(0), MSnB:MSnA(01), ELSnB:ELSnA(10) - clear output on match
             */
            TPM0_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM0_C2SC[5]=0
                    TPM_CnSC_ELSA_MASK);            ///< ELSA=TPM0_C2SC[4]=0
            TPM0_C0SC |= (TPM_CnSC_MSA_MASK |       ///< MSA=TPM0_C2SC[3]=1
                          TPM_CnSC_ELSB_MASK );     ///< ELSB=TPM0_C2SC[2]=1
                          
            TPM0_C0V = TPM0_CNT+0x5;                ///< espera por 5 ticks do TPM0
            TPM0_C0SC |= TPM_CnSC_CHIE_MASK;        ///< habilita a interrup&ccedil;&atilde;o                 

            TPM1_SC &= ~TPM_SC_TOIE_MASK;           ///< desabilita timer overflow flag        
            NVIC_ICPR |= (1 << (33-16) );           ///< NVIC_ICPR[17]=1 (limpa as pend&ecirc;ncias do TPM0)
            ApertouPta12 = 1;                       ///< seta a flag indicadora de "multiplos" ser v&aacute;lido
            times_passed = 0;
            SYST_CSR |= SysTick_CSR_ENABLE_MASK;       ///< habilita o clock do SysTick   
            
        } else {     ///< apertou PTA12
            tof1_vezes=0;   
            TPM1_SC |= TPM_SC_TOIE_MASK;   ///< habilita timer overflow flag
        }
        TPM1_C0SC |= TPM_CnSC_CHF_MASK;     ///< CHF=TPM1_C1SC[7]: w1c (limpa a pend&ecirc;ncia do canal)
    }
                
}

/*!
 * \fn InitTPMIC (void)
 * \brief Configura o canal 0 do módulo TPM1 com a função "Input Capture",
 *
 */
void InitTPMIC(void) {
    PORTA_PCR4 &= ~PORT_PCR_MUX(0x7);       ///< Estrat&eacute;gia para alterar a fun&ccedil;&atilde;o default: 0x7 (NMI)


    PORTA_PCR5 |= (PORT_PCR_ISF_MASK |      ///< ISF=PORTA_PCR5[14]: w1c (limpa a pendência)
                  PORT_PCR_MUX(0x3) );       ///< MUX=PORTA_PCR5[10:8]=0b011 (TPM0_CH2)
    PORTA_PCR12 |= (PORT_PCR_ISF_MASK |      ///< ISF=PORTA_PCR12[14]: w1c (limpa a pendência)
                  PORT_PCR_MUX(0x3) );       ///< MUX=PORTA_PCR12[10:8]=0b011 (TPM1_CH0)
    
        
    /*!
     * Configuracao do canal 0 (PTA12): CPWMS(0), MSnB:MSnA(10), ELSnB:ELSnA(11) - capture on either edges
     */
    TPM1_C0SC |= (TPM_CnSC_CHF_MASK |       ///< CHF=TPM1_C0SC[7]: w1c (limpa a pend&ecirc;ncia do canal)
                  TPM_CnSC_CHIE_MASK);      ///< CHIE=TPM1_C0SC[6]=1 (habilita interrup&ccedil;&atilde;o)  
    TPM1_C0SC &= ~(TPM_CnSC_DMA_MASK);      ///< DMA=TPM1_C0SC[0]=0 (desabilita DMA)
    TPM1_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM1_C0SC[5]=0
                TPM_CnSC_MSA_MASK);         ///< MSA=TPM1_C0SC[4]=0
    TPM1_C0SC |= (TPM_CnSC_ELSB_MASK |      ///< ELSB=TPM1_C0SC[3]=1
                TPM_CnSC_ELSA_MASK);        ///< ELSA=TPM1_C0SC[2]=1
    
    /*!
     * Configuracao do canal 2 (PTA5): CPWMS(0), MSnB:MSnA(10), ELSnB:ELSnA(11) - capture on either edges
     */
    TPM0_C2SC |= (TPM_CnSC_CHF_MASK |       ///< CHF=TPM1_C0SC[7]: w1c (limpa a pend&ecirc;ncia do canal)
                  TPM_CnSC_CHIE_MASK);      ///< CHIE=TPM1_C0SC[6]=1 (habilita interrup&ccedil;&atilde;o)  
    TPM0_C2SC &= ~(TPM_CnSC_DMA_MASK);      ///< DMA=TPM1_C0SC[0]=0 (desabilita DMA)
    TPM0_C2SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM1_C0SC[5]=0
                TPM_CnSC_MSA_MASK);         ///< MSA=TPM1_C0SC[4]=0
    TPM0_C2SC |= (TPM_CnSC_ELSB_MASK |      ///< ELSB=TPM1_C0SC[3]=1
                TPM_CnSC_ELSA_MASK);        ///< ELSA=TPM1_C0SC[2]=1
}

/*!
 * \fn InitTPMOC (void)
 * \brief Configura o canal 3 do módulo TPM0 com a função "Output Compare" e enable do latch que armazena o estado do led.
 *
 */
void InitTPMOC(void) {                              
    /*!
     * Limpe flag e dsabilite interrupcoes
     */
    TPM0_C0SC |= (TPM_CnSC_CHF_MASK);       ///< CHF=TPM0_C2SC[7]: w1c (limpa a pendencia do canal)
    TPM0_C0SC &= ~(TPM_CnSC_DMA_MASK |      ///< DMA=TPM0_C2SC[0]=0 (desabilita DMA)
                   TPM_CnSC_CHIE_MASK);     ///< CHIE=TPM0_C2SC[6]=1 (desabilita interrupcao)  

    /*!
     * Configuracao do canal 0 (LedR1): CPWMS(0), MSnB:MSnA(01), ELSnB:ELSnA(10) - clear output on match
     */
    TPM0_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM0_C2SC[5]=0
            TPM_CnSC_ELSA_MASK);            ///< ELSA=TPM0_C2SC[4]=0
    TPM0_C0SC |= (TPM_CnSC_MSA_MASK |       ///< MSA=TPM0_C2SC[3]=1
                  TPM_CnSC_ELSB_MASK );     ///< ELSB=TPM0_C2SC[2]=1
                  
    TPM0_C0V = TPM0_CNT+0x5;                ///< espera por 5 ticks do TPM0
    TPM0_C0SC |= TPM_CnSC_CHIE_MASK;        ///< habilita a interrupcao          
    

}

/*!
 * \fn selecionaCor(int indice)
 * \brief seleciona o nome da cor de acordo com o indice selecionado pelo usuario
 * \param[in] indice que indica a cor piscada do led
 */
char* selecionaCor(int indice){
    char *str0 = "VERMELHO";
    char *str1 = "AZUL";
    char *str2 = "AMARELO";
    char *str3 = "LARANJA";
    char *str4 = "VERDE";
    char *str5 = "BRANCO";
    char *str6 = "PRETO";

    char *str;
            
    /*! Selecione a mensagem apropriada */
     switch (indice){
         case 0:
             str = str0;
             break;
         case 1:
             str = str1;
             break;
         case 2:
             str = str2;
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
         case 6:
             str = str6;
             break;

     }
    
     return str;
}

/*!
 * \fn freq_potenciometro()
 * \brief retorna o parametro de frequenciae piscadas dos leds
 * \param[in] indice que indica a cor piscada do leds
 */
uint8_t freq_potenciometro(){
	float valor, freq_pot;
	uint8_t nivel_aux;
	
	 ind = 0;
	 ADC0_SC1A	&= (uint32_t)~(uint32_t)(ADC_SC1_ADCH(0x1f)); ///< zera o campo ADCH
	 ADC0_SC1A	|= ADC_SC1_ADCH(0x09);            ///< AD9 selecionado (PTB1)
	 while (ind == 0) {}                         ///< aguarda a amostragem dos 2 canais
	
	 /*!
	 * Conversao para tensao (teste)
	 * (Secao 28.6.1.3 em KL-25 Sub-Family Reference Manual)
	 */
	 valor = ((float)(amostra) * (VREFH-VREFL))/MAX;
	 ind = 0;
	 freq_pot = valor*M_POT;
	
	 if(freq_pot <= 0.5) nivel_aux = 0;
	 else if(freq_pot <= 1.0) nivel_aux = 1;
	 else if(freq_pot <= 2.0) nivel_aux = 2;
	 else if(freq_pot <= 5.0) nivel_aux = 3;
	 else if(freq_pot <= 10.0) nivel_aux = 4;
	 else if(freq_pot <= 20.0) nivel_aux = 5;
	 else if(freq_pot <= 30.0) nivel_aux = 6;
	 else if(freq_pot <= 40.0) nivel_aux = 7;
	 else if(freq_pot <= 50.0) nivel_aux = 8;
	 else nivel_aux = 9;
	 
	 return nivel_aux;
}

/*!
 * \fn main(void)
 */
int main(void){    
    float freq_estaticas[] = {0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
    float time;
	uint8_t nivel_pot_aux, nivel_pot;
    
    /*! vetor com os dados para se criar o caractere e gravar na cgram */
    unsigned char caractere[8] = {0xe, 0x11, 0xe, 0x11, 0x1f, 0x10, 0xe, 0x0};
    
    char *freq, *str;
    
    float cores_estaticas[21];
    
/*! Cores definidas conforme a tabela disponivel em http://www.rapidtables.com/web/color/RGB_Color.htm */
    cores_estaticas[0] = 1.0; cores_estaticas[1] = 0.; cores_estaticas[2] = 0.;    ///< vermelho
    cores_estaticas[3] = 0.; cores_estaticas[4] = 0.6; cores_estaticas[5] = 0.6;    ///< azul berilo    
    cores_estaticas[6] = 1.1; cores_estaticas[7] = 1.1; cores_estaticas[8] = 0.2;  ///< amarelo
    cores_estaticas[9] = 1.1; cores_estaticas[10] = 0.5; cores_estaticas[11] = 0.0;  ///< laranja
    cores_estaticas[12] = 0.; cores_estaticas[13] = 0.4; cores_estaticas[14] = 0.0;  ///< verde escuro
    cores_estaticas[15] = 1.0; cores_estaticas[16] = 1.0; cores_estaticas[17] = 1.0;  ///< branco
    cores_estaticas[18] = 0.; cores_estaticas[19] = 0.; cores_estaticas[20] = 0.;  ///< preto
    
    str = (char*)malloc(sizeof(char)*8);
    freq = (char*)malloc(sizeof(char)*5);
    
    buffer_circular_t = (char *) malloc(sizeof(char)*TAM_MAX);
    buffer_circular_r = (char *) malloc(sizeof(char)*TAM_MAX);
    head_t = tail_t = 0;
    head_r = tail_r = 0;
    estado_terminal = 0;
    times_passed = 0;
    
    estado_pot = 0;
    
    /*! inicializa gpio e uart */
    InitPort();
    initUART();
    /*! inicializa modulo tpm */
    InitTPM();
    InitTPMPWM();
    InitTPMIC();
    InitTPMOC();
    /*! inicializa lcd */
    inicLCD();            
    
	/*! Inicialize o conversor ADC com mecanismo de interrupcao (NVIC) e selecione os canais */
	initADC();
    enableNVIC();
    
    /*! Inicializa timer SysTick */
	InitSysTick();
	
    /*! Criar o caractere "e com acento" */
    criaCaractere(1, caractere);
    
    ApertouPta12 = 1;
    nivel = 2;
    
    time = (1.0/(freq_estaticas[nivel]));
    ConvFpraStr(freq_estaticas[nivel], freq);
    
    nivel_pot_aux = freq_potenciometro();
    
    for(;;) {
    	
     /*! Se ocorreu TimeOut, mostra no terminal e no lcd */
     if(times_passed == 1){
         mandaStringUART("TIMEOUT");
         insere_item_t ('\n');
         insere_item_t('\r');
          
         RS(0);
         enviaLedLCD(0x80, 5);
         mandaString("TIMEOUT        ");
         trocaLinhaLCD();
         mandaString("                ");
         
     }
     
     if ((ApertouPta12 == 1)||(ApertouPta5 == 1)
             ||(estado_terminal == 1)||(estado_pot == 1)) {
         
		if(estado_pot == 1){
			 nivel_pot = freq_potenciometro();
			 if(nivel_pot != nivel_pot_aux){
				 nivel = nivel_pot;
				 nivel_pot_aux = nivel_pot;
			 }
			 estado_pot = 0;
		 }
    		
         if(ApertouPta12){
             if (multiplos < 501) indice = 0;
             else if (multiplos < 1001) indice = 1;
             else if (multiplos < 1501) indice = 2;
             else if (multiplos < 2001) indice = 3;
             else if (multiplos < 2501) indice = 4;
             else if (multiplos < 3000) indice = 5;
             else indice = 6;
             
             ApertouPta12 = 0;     ///< resetar preparando-a para a proxima entrada
         }
         if(ApertouPta5){
             if (iteracao < 334) nivel = 0;
             else if (iteracao < 667) nivel = 1;
             else if (iteracao < 1000) nivel = 2;
             else if (iteracao < 1334) nivel = 3;
             else if (iteracao < 1667) nivel = 4;
             else if (iteracao < 2000) nivel = 5;
             else if (iteracao < 2334) nivel = 6;
             else if (iteracao < 2667) nivel = 7;
             else if (iteracao < 3000) nivel = 8;
             else nivel = 9;
             
             ApertouPta5 = 0;     ///< resetar preparando-a para a proxima entrada
         }
         
         if(estado_terminal == 1){
        	 terminalHandler();
        	 estado_terminal = 0;
         }
         
         str = selecionaCor(indice);
         time = (1.0/(freq_estaticas[nivel]));
                     
         /*! Converte a frequencia float para ASCII */
         ConvFpraStr(freq_estaticas[nivel], freq);
         
         /*! manda mensagem no lcd */
         mostra_lcd("cor", str);
         mostra_lcd("freq", freq);
         
         /*! manda mensagem para o terminal */
         string_terminal(str, freq);
    
     }
     
     if(!ApertouPta5 && !ApertouPta12 && !estado_terminal && !estado_pot){
    	 /*!  Recarga dos contadores dos canais PWM */
    	    TPM2_C0V = TPM_CnV_VAL((short)(((cores_estaticas[indice*3]))*COUNTER_OVF));     ///< % R
    	    TPM2_C1V = TPM_CnV_VAL((short)(((cores_estaticas[indice*3+1]))*COUNTER_OVF));   ///< % G
    	    TPM0_C1V = TPM_CnV_VAL((short)(((cores_estaticas[indice*3+2]))*COUNTER_OVF));   ///< % B
    	    
    	    /*! Selecao de modo, transicao e nivel (Modo: PWM alinhado a borda) */
    	    /*! Led R */
    	    TPM2_C0SC |= (TPM_CnSC_MSB_MASK |         ///< MSB=TPM2_C1SC[5]=1
    	                TPM_CnSC_ELSB_MASK |          ///< ELSB=TPM2_C1SC[3]=1
    	                TPM_CnSC_ELSA_MASK );         ///< ELSA=TPM2_C1SC[2]=1
    	    TPM2_C0SC &= ~TPM_CnSC_MSA_MASK;          ///< MSA=TPM2_C1SC[4]=0
    	    /*! Led G */
    	    TPM2_C1SC |= (TPM_CnSC_MSB_MASK |       ///< MSB=TPM2_C1SC[5]=1
    	                TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C1SC[3]=1
    	                TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C1SC[2]=1
    	    TPM2_C1SC &= ~TPM_CnSC_MSA_MASK;        ///< MSA=TPM2_C1SC[4]=0
    	    /*! Led B */
    	    TPM0_C1SC |= (TPM_CnSC_MSB_MASK |         ///< MSB=TPM2_C1SC[5]=1
    	                TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C1SC[3]=1
    	                TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C1SC[2]=1
    	    TPM0_C1SC &= ~TPM_CnSC_MSA_MASK;         ///< MSA=TPM2_C1SC[4]=0
    	    
    	    /*!
    	     *  Antes do delay, verifica se usuario esta mudando cor ou frequencia dos leds
    	     */
    	    if(!ApertouPta5 && !ApertouPta12 && !estado_terminal && !estado_pot)
    	    	delay10us((int)(time*100000));
    	    
    	    /*! Selecao de modo, transicao enivel (desabilita os 3 canais) */
    	    TPM2_C0SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM2_C0SC[5]=0
    	                TPM_CnSC_MSA_MASK |         ///< MSA=TPM2_C0SC[4]=0
    	                TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C0SC[3]=0
    	                TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C0SC[2]=0
    	    TPM2_C1SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM2_C1SC[5]=0
    	                TPM_CnSC_MSA_MASK |         ///< MSA=TPM2_C1SC[4]=0
    	                TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM2_C1SC[3]=0
    	                TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM2_C1SC[2]=0
    	    TPM0_C1SC &= ~(TPM_CnSC_MSB_MASK |      ///< MSB=TPM0_C1SC[5]=0
    	                TPM_CnSC_MSA_MASK |         ///< MSA=TPM0_C1SC[4]=0
    	                TPM_CnSC_ELSB_MASK |        ///< ELSB=TPM0_C1SC[3]=0
    	                TPM_CnSC_ELSA_MASK );       ///< ELSA=TPM0_C1SC[2]=0
    	    
    	    /*!
    	     *  Antes do delay, verifica se usuario esta mudando cor ou frequencia dos leds
    	     */
    	    if(!ApertouPta5 && !ApertouPta12 && !estado_terminal) 	    
    	    	delay10us((int)(time*100000));
     	 }
    }
       
    /*! Desalocar a memoria */
    free((char*)freq);
    free((char*)str);
    free((char*)buffer_circular_r);
    free((char*)buffer_circular_t);  

}
