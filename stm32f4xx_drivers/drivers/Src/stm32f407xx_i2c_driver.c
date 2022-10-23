/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 4 de set de 2022
 *      Author: Matteus
 */

/* ##############################################################################################################################################################################
 *
 *                  Problemas comuns na comunicação I2C
 *
 *                  Case 1: SDA and SCL line not held HIGH Voltage after I2C pin initialization

				################################################################################################
               	#  	Reason-1: Not activating the pullup resistors if you are using the internal                #
				#	pull up resistor of an I/O line															   #
                #   Debug Tip:																				   #
				#	worth checking the configuration register of an I/O line to see                            #
				#	whether the pullups are really activated or not, best way is to                            #
				#	dump the register contents.                                                                #
				################################################################################################
 *
 *                  Case 2: ACK failure
 *
 *              ################################################################################################
				#	Reason-1: Generating the address phase with wrong slave address                            #
				#	Debug Tip:                                                                                 #
				#	verify the slave address appearing on the SDA line by using                                #
				#	logic analyser.                                                                            #
                ################################################################################################

                ################################################################################################
                #   Reason-2 :																				   #
				#	Not enabling the ACKing feature in the I2C control register                                #
				#	Debug Tip:                                                                                 #
				#	Cross check the I2C Control register ACK enable field                                      #
                ################################################################################################

                    Case 3: Master is not producing the clock

                ################################################################################################
				#	Debug Tip-1 : First Check whether I2C peripheral clock is enabled and set to               #
				#	at least 2MHz to produce standard mode i2c serial clock frequency                          #
				#	Debug Tip - 2: Check whether GPIOs which you used for SCL and SDA                          #
                #   functionality are configured properly for the alternate                                    #
				#	functionality                                                                              #
                ################################################################################################
*/

#include <stdint.h>
#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB_PreScaler[4] = {2, 4, 8, 16};

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t nomeDaFlag);
uint32_t RCC_GetPLLOutputClock();
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t discAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t discAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START); //simples assim, precisamos apenas levantar o bit 8 do CR1
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t nomeDaFlag){
	if(pI2Cx->SR1 & nomeDaFlag){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

uint32_t RCC_GetPLLOutputClock(){ //essa função ainda não está implementada porque não utilizaremos pll por enquanto
	uint32_t var = 0;
	return var;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t discAddr){
	discAddr = (discAddr << 1);                                                        //precisamos deixar o primeiro bit para o read/write bit, por isso esse shift para esquerda
	discAddr &= ~(1 << 0);															   //limpando o bit zero (primeiro bit)
	pI2Cx->DR = discAddr;                                                              //discAddr = discipleAddress + r/w Bit
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = (SlaveAddr << 1);                 //fazemos esse shift para esquerda para adicionarmos o bit de read na mensagem também
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode - Lembrando que estamos criando esta função para um master apenas, não para um disciple
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);                                                 //simples assim, precisamos apenas levantar o bit 9 do CR
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi) //precisamos desta função para habilitar os enables das interrupções (lembra que temos ANDs em que uma das entradas são cada uma dessas flags e a outra a interrupção em si)
{
	 if(EnorDi == ENABLE)
	 {
			pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
			pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
			pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	 }else
	 {
			pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
			pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
			pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	 }
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, system_clock;

	//para sabermos que tipo de clock source estamos lidando temos que ler o RCC_CFGR e ver os bits 2 e 3
	uint8_t temp, ahbp /*AHB1 preescaler*/, apb1p /*APB1 preescaler*/, clk_src; //primeiro puxamos os bits 2 e 3 para os bits 0 e 1, depois zeramos todos menos os dois primeiros
	clk_src = (RCC->CFGR >> 2) & 0x3;

	/*
	* Bits 3:2 SWS: System clock switch status
	Set and cleared by hardware to indicate which clock source is used as the system clock.
	00: HSI oscillator used as the system clock
	01: HSE oscillator used as the system clock
	10: PLL used as the system clock
	11: not applicable
	 */

	if(clk_src == 0){
		system_clock = 16000000; //se a leitura for 0 então a clock é 16MHz (oscilador interno)
	}
	else if(clk_src == 1){
		system_clock = 8000000;  //se a leitura for 1 então a clock é 8MHz (cristal externo)
	}
	else if(clk_src == 2){
			system_clock = RCC_GetPLLOutputClock();  //se a leitura for 1 então a clock é 8MHz (cristal externo)
	}

	//Como vimos no clock tree do reference manual, as fontes de clock passam por 2 divisores de clock antes de irem fornecer
	//clock pro I2C, então devemos descobrir como estão setados os preescalers desses divisores para saber qual clock está indo
	//para o I2C

	//AHB
	temp = (RCC->CFGR >> 4) & 0xF;     //achando o preescaler do AHB1
	if(temp < 8){
		ahbp = 1;                      //sempre que os bits formarem um número menor que 8 temos um divisor de modulo 1
	}
	else{
		ahbp = AHB_PreScaler[temp-8]; //caso temp seja maior que 8 queremos uma certa posição no ahbp_PreScaler, subtrai-se 8 pra pegar a posição certa
									   //exemplo: se temp = 8 queremos a posição 0, ou seja, temp-8 e assim por diante
	}

	//APB
	temp = (RCC->CFGR >> 10) & 0x7;     //achando o preescaler do APB1, agora ox7 é suficiente para o masking
	if(temp < 4){
		apb1p = 1;                      //sempre que os bits formarem um número menor que 8 temos um divisor de modulo 1
	}
	else{
		apb1p = APB_PreScaler[temp-4]; //caso temp seja maior que 4 queremos uma certa posição no ahbp_PreScaler, subtrai-se 4 pra pegar a posição certa
									   //exemplo: se temp = 4 queremos a posição 0, ou seja, temp-4 e assim por diante
	}

	pclk1 = system_clock/ahbp/apb1p;  //finalmente calculamos que clock está indo para o I2C, depois de passar por 2 divisores de clock

	return pclk1;
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLOCK_EN();
		}
		else if (pI2Cx == I2C2){
			I2C2_PCLOCK_EN();
		}
		else if (pI2Cx == I2C3){
			I2C3_PCLOCK_EN();
		}
	}

	else{
		if(pI2Cx == I2C1){
			I2C1_PCLOCK_DI();
		}
		else if (pI2Cx == I2C2){
			I2C2_PCLOCK_DI();
		}
		else if (pI2Cx == I2C3){
			I2C3_PCLOCK_DI();
		}
	}
}

void I2C_deInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1){
		I2C1_REG_RST();
	}
	else if (pI2Cx == I2C2){
		I2C2_REG_RST();
	}
	else if (pI2Cx == I2C3){
		I2C3_REG_RST();
	}
}

/*
 * Lembrando que as implementações das funções de interrupt config e priority config para todos os periféricos são iguais pois
 * o que essas funções fazem é apenas para questão de configuração de uma interrupção externa qualquer que seja,
 * mas a diferenciação feita em cada driver é só pra questão de organizar qual periférico é que está fazendo a configuração
 * do NVIC para interrupção externa
 */
void I2C_IRQInterruptConfig(uint8_t IRQnumber, uint8_t  IRQEn)
{
	//A função GPIO_Init irá configurar o SYSCFG (que seta o PORT (A,B...) que deverá gerar a interrupção
	//E o EXTIx que seta o modo de detecção dessa interrupção (Falling ou Rising Trigger)
	//Agora esta função GPIO_IRQConfig irá efetivamente gerar a interrupção do processador,
	//Usando os IRQ set-enable/Clear-enable Registers (ISR e ICR)

	if(IRQEn == ENABLE){ //enable
		if(IRQnumber <= 31){
			//programa o ISR0 (Interrupt Set-Enable Register 0) que habilita as interrupções de número 0 até 31.
			//habilitar a interrupção de número IRQnumber significa setar 1 no espaço de memória de índice IRQnumber
			//do registrador  NVIC_ISER0
			*NVIC_ISER0 |= (1 << IRQnumber);
		}
		else if(IRQnumber > 31 && IRQnumber < 64){ //entre 32 e 63
			//c.c programar o ISR1 (Interrupt Set-Enable Register 1) que habilita as interrupções de número 32 até 63
			uint8_t temp1 = IRQnumber % 32;       //precisamos disso pois os registradores vão até 32 e os IRQnumbers superam esse valor
			*NVIC_ISER1 |= (1 << temp1);
		}
		else if(IRQnumber >= 64 && IRQnumber < 96 ){
			uint8_t temp1 = IRQnumber % 64;
			//c.c programar o ISR2 (Interrupt Set-Enable Register 2) que habilita as interrupções de número 64 até 96
			*NVIC_ISER2 |= (1 << temp1);
		}
	}
	else{ //Disable
		if(IRQnumber <= 31){
			//programar o ICR0 (Interrupt Clear-Enable Register 0) que desabilita as interrupções de número 0 até 31
			*NVIC_ICER0 |= (1 << IRQnumber);
		}
		else if(IRQnumber > 31 && IRQnumber < 64){ //entre 32 e 63
			//programar o ICR1 (Interrupt Clear-Enable Register 1) que desabilita as interrupções de número 32 até 63
			*NVIC_ICER1 |= (1 << IRQnumber%32); //aqui não usarei a variável auxiliar temp1 para ser mais rápido
		}
		else if(IRQnumber >= 64 && IRQnumber < 96 ){
			//programar o ICR2 (Interrupt Clear-Enable Register 2) que desabilita as interrupções de número 64 até 93
			*NVIC_ICER2 |= (1 << IRQnumber%64);
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;                                              //para descobrirmos qual registrador IPR entre 0 - 59 cada um com 4 seções de 8 bits (Se IQRNumber == 23 então IQRNumber/4 == 5 então deveremos mexr no IPR5)
	uint8_t iprx_section = IRQNumber % 4;                                      //para descobrirmos qual a seção
	uint8_t shift_amount = 8*iprx_section + (8 - NO_PR_BITS_IMPLEMENTED);		 //Esta linha primeiramente faz o shift maior colocando os 8 bits na seção certa, depois faz mais um shift para à esquerda de 4 bits pois os primeiros 4 bits de cada seção não são utilizados
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);               //para colocarmos efetivamente o IRQPriority na seção de 8 bits

	/*Como NVIC_PR_BASE_ADDR foi declarado como uint32_t cada +1 que somamos no seu endereço pula 32bits (4 bytes), ou seja
	  iprx pode ser diretamente somando ao NVIC_PR_BASE_ADDR que já pulará 4bytes, fazendo com que seja desnecessário o
	  uso da multiplicação por 4 nesse cenário. Caso o NVIC_PR_BASE_ADDR fosse declarado como uint8_t então o iprx teria que ser multiplicado por 4 mesmo*/
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);  //SEMPRE QUE PE ESTIVER EM HIGH AQUELE PERIFÉRICO ESTARÁ HABILITADO, CASO CONTRARIO ESTARÁ DESABILITADO
	}
	else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*
 * A partir daqui faremos as funções de init com configuração, lembramos que usaremos os registradores CR2 e CCR
 * Primeiramente configuramos o CR2 com a frequencia do APB1 bus que está conectado no I2Cx (todos os I2C estão conectados no APB1), depois configurremos o CCR
 */

/*
 * @I2C_FMDutyCycle - Qualquer coisa ver o reference manual
 * Bits 11:0 CCR[11:0]: Clock control register in Fm/Sm mode (Master mode)
Controls the SCL clock in master mode.
		Sm mode or SMBus:
		Thigh = CCR * TPCLK1
		Tlow = CCR * TPCLK1
		Fm mode:
		If DUTY = 0:
		Thigh = CCR * TPCLK1
		Tlow = 2 * CCR * TPCLK1
		If DUTY = 1:
		Thigh = 9 * CCR * TPCLK1
		Tlow = 16 * CCR * TPCLK1
		For instance: in Sm mode, to generate a 100 kHz SCL frequency:
		If FREQ = 08, TPCLK1 = 125 ns so CCR must be programmed with 0x28
		(0x28 <=> 40d x 125 ns = 5000 ns.)
		Note: The minimum allowed value is 0x04, except in FAST DUTY mode where the minimum
		allowed value is 0x01
		thigh = tr(SCL) + tw(SCLH). See device datasheet for the definitions of parameters.
		tlow = tf(SCL) + tw(SCLL). See device datasheet for the definitions of parameters.
		I2C communication speed, fSCL ~ 1/(thigh + tlow). The real frequency may differ due to
		the analog noise filter input delay.
		The CCR register must be configured only when the I2C is disabled (PE = 0).

COM AS INFORMAÇÕES ACIMA PODEMOS INFERIR QUE NO MODO SLOW (SM) o duty cicle sempre é 50%, já no Fast Mode o duty cicle pode ser 1/3 ou
19/9. Isso é especificidade do I2C, não podemos fazer nada, só olhar no reference manual e programar os registradores disponíveis
 */

/*
 * Example :
		In SM Mode , generate a 100 kHz SCL frequency
		APB1 Clock (PCLK1) = 16MHz
		1) Configure the mode in CCR register (15th bit )
		2) Program FREQ field of CR2 with the value of PCLK1
		3) Calculate and Program CCR value in CCR field of CCR register

		Thigh(scl) = CCR * TPCLK1
		Tlow(scl) = CCR * TPCLK1

		t_total = thigh + tlow = 2 * CCR * TPCLK1

		clock de 100kHz => thigh = 5us e tlow = 5us

		5us = CCR * (1/16MHz) => CCR = 80 => então devemos programar 0x50 no clock control register (CCR)
 */

/*
 * Example :
		In FM Mode , generate a 200 kHz SCL frequency
		APB1 Clock (PCLK1) = 16MHz
		1) Configure the mode in CCR register (15th bit )
		2) Select the duty cycle of Fast mode SCL in CCR register (14th bit)
		3) Program FREQ field of CR2 with the value of PCLK1
		4) Calculate and Program CCR value in CCR field of CCR register

		If DUTY = 0:
			Thigh = CCR * TPCLK1
			Tlow = 2 * CCR * TPCLK1

			thigh + tlow = 3 * CCR * TPCLK1 => 5us = 3 * CCR * 62.5ns => CCR = 26 (colocar esse numero no CCR register)

		If DUTY = 1: (to reach 400 kHz)
			Thigh = 9 * CCR * TPCLK1
			Tlow = 16 * CCR * TPCLK1

			thigh + tlow = 25 * CCR * TPCLK1 => 5us = 25 * CCR * 62.5ns
 */

void I2C_Init(I2C_Handle_t *pI2CHandle){
	{
		uint32_t tempreg = 0 ;

		//enable the clock for the i2cx peripheral
		I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

		//ack control bit
		tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10; //configurando o bit 10 do CR1
		pI2CHandle->pI2Cx->CR1 = tempreg;                       //aqui atribuimos mesmo o tempreg no CR1 para habilitarmos o ACK automatico

		//configure the FREQ field of CR2
		tempreg = 0;
		tempreg |= RCC_GetPCLK1Value() /1000000U ;             //dividimos por 1MHZ porque queremos apenas os digitos mais significativos. Por exemplo se o getValue retornar 16MHz então queremos apenas o numero 16
		pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);            //fazemos o masking com o 0x3F porque queremos apenas os 6 primeiros bits vão para o CR2, os outros devem estar zerados

	   //program the device own address - apenas util se o dispositivo for um disciple
		tempreg = 0;
		tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;  //Aqui o usuário digitou o endereço que quer atribuir a este MCU no modo disciple, OAR = Own Adress Register
		tempreg |= ( 1 << 14);                                     //shiftamos em 1 porque não queremos o primeiro bit, ele não faz parte do adress
		pI2CHandle->pI2Cx->OAR1 = tempreg;                         //Também temos que manter o bit 14 em high segundo o reference manual (ele não explica o motivo)

		//CCR calculations - Fazer todo o tramite de dado o modo (slow mode ou fast mode) calcular o CCR value para o output ser a frequencia que queremos
		//após estes calculos então estará produzido o serial clock do I2C
		uint16_t ccr_value = 0;
		tempreg = 0;
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode
			ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );   //t_total = thigh + tlow = 2 * CCR * TPCLK1 => CCR = t_total/(2*TPCLK1) => CCR = Source_Freq/(2*wanted_speed)
			tempreg |= (ccr_value & 0xFFF);                                                     //aqui zeramos os bits acima de 12, pois não nos interessam
		}else
		{
			//mode is fast mode - o reference manual diz que temos que manter o bit 15 em high
			tempreg |= ( 1 << 15);
			tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);             //setamos aqui no bit 14 se queremos slow mode ou fast mode no duty cicle
			if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}else
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}
			tempreg |= (ccr_value & 0xFFF);   //aqui de novo estamos apenas interessados nos primeiros 12 bits, então zeramos os outros no masking
		}
		pI2CHandle->pI2Cx->CCR = tempreg;

		/*
		 * Agora vamos configurar o tempo de subida desejado na comunicação, levando em conta que precisamos saber de antemão o tempo máximo tanto no SM quanto no FM
		 * dados no reference manual, também devemos lembrar que na comunicação I2C devemos calcular os resistores de pull up do barramento, levando em conta a capacitância
		 * deste barramento (podendo ser medida ou estimada) e o trise máximo em cada modo de funcionamento, bem como as tensões de Vcc e referência (gnd) utilizadas.
		 * Entretanto, para setar o Trise no regulador correspondente (trise register) devemos usar a formula do reference manual e configurar o registrador
		 * Por exemplo: Sabemos que o tempo de subida máximo permitido no modo SM é 1000ns (no fast mode é 300ns), mas se nós programamos a frequencia do barramento para trabalhar em
		 * 125ns, que é a própria frequencia do PCKL1 vinda do clock system, temos que colocar no trise register a sequencia de bits dada pela seguinte fórmula:
		 * trise_reg = ((TriseDoModo/PCLK1) + 1)
		 */
		//TRISE Configuration
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode

			tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

		}else
		{
			//mode is fast mode
			tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

		}

		pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

	}
}

/*
 * ###########################################################################################################################################################
 * A partir daqui teremos a API para comunicação propriamente dita. Primeiramente quando o microcontrolador mandar o start bit para o
 * bus I2C ele se torna o master automaticamente, sempre que um dispositivo mandar o start bit ele é o master.
 *
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);                               //Aqui nós setamos o SB para high, o procedimento a seguir deverá limpá-lo

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));                    //enquanto o SB do SR1 não confirmar que a geração do start bit foi concluída nós esperamos
																			     //Aqui estamos fazendo uma leitura do SR1 via chamada da função I2C_GetFlagStatus e a seguir faremos um write no data register DR, isso
	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);                   // Acima fizemos uma leitura do SR1 via I2C_GetFlagStatus e agora estamos dando um write no data register DR, isso
	                                                                             //limpará o SB

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));                  //mesma espera que o while anterior, enquanto o SR1 não confirmar que a phase address está completa devemos esperar

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);                                        //Com essa função realizamos duas leituras, uma do SR1 e outra do SR2, isso limpa o ADDR bit por hardware (ou seja, automaticamente)

	//6. send the data until len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF) );


	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF

	/*
	 * LMENBRANDO QUE FAZEMOS A GERAÇÃO DO STOP APENAS NA ULTIMA CHAMADA DA FUNÇÃO MASTER RECEIVE, ISSO PORQUE SE GERARMOS O STOP A CADA
	 * SEND OU RECEIVE QUALQUER (E NAO NA ULTIMA CHAMADA) ISSO ABRE MARGEM PRA ALGUM OUTRO DISPOSITIVO MASTER SOLICITAR O BUS I2C PREJUDICANDO
	 * A COMPLETA AQUISIÇÃO DE DADOS SOLICITADA PELO PRIMEIRO MASTER, ENTÃO O QUE FAZEMOS É SÓ ACIONAR O STOP QUANDO TODA A MENSAGEM ESTIVER
	 * SIDO RECEBIDA (OU ENVIADA) EM CADA "RODADA" DE COMUNICAÇÃO SPI
	 */
	if(Sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);                                //aqui nos geramos a condição de stop para finalização da comunicação I2C nessa rodada de comunicação entre master e slave
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	//1. Generate the START condition - é a primeira coisa que devemos fazer para inicializar a rotina de receive
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);                       //apenas levanta o bit 8 do CR1

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/w bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);                       //aqui mandamos o endereço do slave e o bit de read (ele enviará de volta um ack)

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );               //aqui o addressBit = 1 pode ser esperado para sabermos que o ack foi recebido do slave para o master

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{

		/*
		 * Como neste caso queremos apenas um byte, já temos que mandar um nack para o slave saber que não queremos mais comunicar além desta rodada
		 * após isso daremos um clear no ADDR para não prejudicar a próxima chamada (sempre que chamamos o masterReceive temos que ter o ADDR =0),
		 * então os próximos 2 comandos cuidam do que foi dito acima (mandar nack e dar um clear no ADDR flag)
		 *
		 * Feito isso, esperamos o RXNE ficar em alto para sabermos que tudo ocorreu .
		 * a partir daí geramos a condição de stop e se for o caso reabilitamos o ack
		 */
		//Disable Acking                                                   //aqui damos um clear no ACK porque é só 1 byte que receberemos
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);                              //damos um clear no ADDR para que o próximo receive não seja prejudicado com essa flag em alto

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

		/*
		 * LMENBRANDO QUE FAZEMOS A GERAÇÃO DO STOP APENAS NA ULTIMA CHAMADA DA FUNÇÃO MASTER RECEIVE, ISSO PORQUE SE GERARMOS O STOP A CADA
		 * SEND OU RECEIVE QUALQUER (E NAO NA ULTIMA CHAMADA) ISSO ABRE MARGEM PRA ALGUM OUTRO DISPOSITIVO MASTER SOLICITAR O BUS I2C PREJUDICANDO
		 * A COMPLETA AQUISIÇÃO DE DADOS SOLICITADA PELO PRIMEIRO MASTER, ENTÃO O QUE FAZEMOS É SÓ ACIONAR O STOP QUANDO TODA A MENSAGEM ESTIVER
		 * SIDO RECEBIDA (OU ENVIADA) EM CADA "RODADA" DE COMUNICAÇÃO SPI
		 */

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR )
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);                     //aqui nos geramos a condição de stop para finalização da comunicação I2C

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		/*
		 * No caso de len ser maior que 1, temos que só mandaremos o nack junto com o stop bit no fim da recepção, enquanto não for o fim da recepção
		 * não queremos mandar nack nem o stop bit, apenas vamos lendo os bytes que chegam
		 */
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);            //aqui fazemos um disable do ack porque não queremos mais leitura , esperaremos o ultimo byte, geraremos a condição de stop e só depois reabilita-se o ack pra receive típico do receive (sempre que entramos nesta função queremos um ack levantado para o caso de mais de 1 byte esteja na fila para ser recebido)

				/*
				 * LMENBRANDO QUE FAZEMOS A GERAÇÃO DO STOP APENAS NA ULTIMA CHAMADA DA FUNÇÃO MASTER RECEIVE, ISSO PORQUE SE GERARMOS O STOP A CADA
				 * SEND OU RECEIVE QUALQUER (E NAO NA ULTIMA CHAMADA) ISSO ABRE MARGEM PRA ALGUM OUTRO DISPOSITIVO MASTER SOLICITAR O BUS I2C PREJUDICANDO
				 * A COMPLETA AQUISIÇÃO DE DADOS SOLICITADA PELO PRIMEIRO MASTER, ENTÃO O QUE FAZEMOS É SÓ ACIONAR O STOP QUANDO TODA A MENSAGEM ESTIVER
				 * SIDO RECEBIDA (OU ENVIADA) EM CADA "RODADA" DE COMUNICAÇÃO SPI
				 */

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR )
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);              //aqui nos geramos a condição de stop para finalização da comunicação I2C

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}

	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}

}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	/*
	 * LEMBRANDO QUE NESTA FUNÇÃO ABAIXO NÃO IREMOS FAZER EFETIVAMENTE O SEND DATA, MAS SIM HABILITAR AS
	 * FLAGS DE INTERRUPÇÃO QUE FARÃO A IRQ SER ACIONADA E SÓ AÍ FAREMOS O SEND DENTRO DA IRQ HANDLING CORRESPONDENTE
	 */

	/*
	 * ABAIXO PASSAREMOS TODAS AS INFORMAÇÕES DE CONFIGURAÇÃO E MENSAGEM DO USUÁRIO PARA OS LUGARES DE MEMÓRIA PREVIAMENTE ENCONTRADOS NO
	 * MEMORY MAP, SÓ AÍ GERAREMOS A INTERRUPÇÃO QUE CHAMARÁ A IRQ DO I2C CORRESPONDENTE. DAÍ LIDAREMOS COM O ENVIO/RECEPÇÃO DE MENSAGENS DENTRO
	 * DA IRQ
	 */

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))          //se não tiver preso mandando nem recebendo mensagem então podemos continuar
	{
		pI2CHandle->pTxBuffer = pTxBuffer;                                      //mandamos os dados para o lugar certo do memory map que vai fazer o handle da mensagem enviada (mandar do data register para o shift register etc)
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;                                        //informamos aqui o endereço do slave solicitado pelo usuário
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);     //com essa função chamada, o start bit será gerado, e a flag SB = 1, então gerará a interrupção já que as flags de
														   //interrupção por evento, buffer e error vão ser habilitadas logo abaixo. Com a interrupção feita Com a interrupção feita logo após o set das flags, deveremos dentro da IRQ handler
														   //verificar qual foi a natureza dessa interrupção e tomar as atitudes correspondentes
		/*
		 * A partir do momento que estas flags abaixo forem setadas, vários eventos irão acarretar a interrupção do processador chamando a I2C IRQ handler
		 * esses eventos podem ser por exemplo a geração de um start bit, ou um address sent, address receive etc (para maior detalhes ver o reference manual)
		 */

		//Implement the code to enable ITBUFEN Control Bit                       //aqui temos as 3 flags das portas AND para deixarmos os sinais alcançarem o NVIC e interromper o processador
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);                      //com o start bit setado em high SB=1, a interrupção acontecerá aqui, chamando a função IRQ handler de número correspondente e já devidamente habilitada previamente
        																		//dentro da IRQ handler é que faremos efetivamente a transmissão da mensagem

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	/*
	 * LEMBRANDO QUE NESTA FUNÇÃO ABAIXO NÃO IREMOS FAZER EFETIVAMENTE O SEND DATA, MAS SIM HABILITAR AS
	 * FLAGS DE INTERRUPÇÃO QUE FARÃO A IRQ SER ACIONADA E SÓ AÍ FAREMOS O SEND DENTRO DA IRQ HANDLING CORRESPONDENTE
	 */

	/*
	 * ABAIXO PASSAREMOS TODAS AS INFORMAÇÕES DE CONFIGURAÇÃO E MENSAGEM DO USUÁRIO PARA OS LUGARES DE MEMÓRIA PREVIAMENTE ENCONTRADOS NO
	 * MEMORY MAP, SÓ AÍ GERAREMOS A INTERRUPÇÃO QUE CHAMARÁ A IRQ DO I2C CORRESPONDENTE. DAÍ LIDAREMOS COM O ENVIO/RECEPÇÃO DE MENSAGENS DENTRO
	 * DA IRQ
	 */

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);     //com essa função chamada, o start bit será gerado, e a flag SB = 1, então gerará a interrupção já que as flags de
														   //interrupção por evento, buffer e error vão ser habilitadas logo abaixo. Com a interrupção feita logo após o set das flags, deveremos dentro da IRQ handler
														   //verificar qual foi a natureza dessa interrupção e tomar as atitudes correspondentes
		/*
		* A partir do momento que estas flags abaixo forem setadas, vários eventos irão acarretar a interrupção do processador chamando a I2C IRQ handler
		* esses eventos podem ser por exemplo a geração de um start bit, ou um address sent, address receive etc (para maior detalhes ver o reference manual)
		*/

		//Implement the code to enable ITBUFEN Control Bit                //aqui temos as 3 flags das portas AND para deixarmos os sinais alcançarem o NVIC e interromper o processador
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);                //com o start bit setado em high SB=1, a interrupção acontecerá aqui, chamando a função IRQ handler de número correspondente e já devidamente habilitada previamente
		                                                                  //dentro da IRQ handler é que faremos efetivamente a recepção da mensagem

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->DR = data; //simples assim, quando o master faz o request de dado o disciple só insere dado no data register
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
    return (uint8_t) pI2C->DR;   //simples assim, quando o master faz o request de dado o disciple só retorna dado do data register para uma variável declarada no proprio disciple
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device

	/*
	* LEMBRETE: QUANDO UMA INTERRUPÇÃO POR I2C FOR GERADA, COMO SEMPRE TEREMOS UMA FLAG DE EVENTO OU ERRO GERADA, SE AS FLAG DE INTERRUPT EVENT ENABLE
	* OU ERROR ENABLE ESTIVEREM EM HIGH, A INTERRUPÇÃO OCORRE E O PROCESSADOR É JOGADO NO CONTEXTO DE CHAMADA DA FUNÇÃO I2C HANDLER (OU SEJA, CHAMA O I2C HANDLER
    * DE NUMERO CORRESPONDENTE E A HANDLER LIDA COM A INTERRUPÇÃO TOMANDO AS ATITUDES NECESSÁRIAS SEJAM ELAS QUAIS FOREM (irq 31 SE FOR EVENTO E IRQ21 SE FOR ERRO).
	* POR EXEMPLO:
	* SE FORMOS MANDAR  UMA MENSAGEM (SEND) VIA INTERRUPÇÃO I2C, A FLAG START BIT SERÁ ACIONADA NA CHAMADA DA FUNÇÃO SEND (ALGUMA LINHA FORÇARÁ SB=1), A FLAG I2C_CR2_ITEVTEN TAMBÉM DEVERÁ SER ACIONADA ATRAVÉS DA CHAMDA
	* pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN), POIS É UMA FLAG DE ENABLE NO CR2, NISSO OCORRERÁ UMA INTERUPÇÃO E A FUNÇÃO I2C_IRQ_NO_31 SERÁ CHAMADA (POIS É UM EVENTO, SE FOSSE UM ERRO SERIA DISPARADA A IRQ32),
	* DENTRO DA FUNÇÃO IRQ NO 31 CHAMAREMOS A FUNÇÃO I2C_EV_IRQHandling(I2C1) QUE LIDARÁ COM A NATUREZA DA
	* CHAMADA DE INTERRUPÇÃO, . NOTE PRIMEIRAMENTE QUE O DISPARO DA INTERRUPÇÃO OCORRE INTERNAMENTE POR HARDWARE E O HARDWARE SÓ SABE QUE A INTERRUPÇÃO FOI FEITA OU POR UM EVENTO OU POR UM ERRO, MAS NÃO SABE QUAL EVENTO E QUAL ERRO,
	* MAS ISSO SERÁ DESCOBERTO DENTRO DA CHAMADA DA FUNÇÃO I2C_IRQ_NO_31 OU I2C_IRQ_NO_32 POIS COMO ESTAMOS PASSANDO QUAL I2Cx CHAMA A INTERRUPÇÃO, ENTÃO SABEMOS TODAS AS CONFIGURAÇÕES DA INTERRUPÇÃO, PODENDO ANALISAR SUAS FLAGS E
	* PROCESSÁ-LAS. NO CASO ABORDADO O EVENTO QUE DISPAROU A FUNÇÃO FOI O START BIT, MAS SE FOSSE OUTRA SABERIAMOS POIS A FUNÇÃO I2C_EV_IRQHandling RECEBE O PARÂMETRO I2CX COMO VISTO,
	* ENTÃO AO ANALISARMOS QUAIS FLAGS ESTÃO EM HIGH SABEREMOS QUAL INTERRUPT ESTAMOS LIDANDO E TOMAREMOS AS ATITUDES CERTAS.
	*
	* Lembrando que a função Master_SEND_IT_TX que gerará a interrupção irá setar também a flag Busy_IN_TX, para sabermos se na hora de processar a interrupção queremos um write.
	* Analogamente também faremos isso com a função Mater_RECEIVE_IT e outras interrupções
	*
	*Resumo do resumo: a interrupção por erro ou por evento acontecerá e esta função decodificará o motivo dela ter acontecido
	*/

	uint8_t temp1,temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN); //NOTE QUE COMO JÁ ESTAMOS DENTRO DA CHAMADA DE INTERRUPÇÃO ENTÃO ESTA FLAG JÁ ESTÁ EM ALTO, (JÁ QUE A I2C_EV_IRQHandling SÓ É CHAMADA SE I2C_CR2_ITEVTEN estiver acionada, PORTANTO AQUI DENTRO ESTAMOS FAZENDO APENAS UM DOUBLE CHECK POR PRECAUÇÃO)
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if(temp1 && temp3)
	{  //se a interrupção foi trigada pelo evento start bit com a flag FLAG I2C_CR2_ITEVTEN acionada
	   //SB flag está setada em high
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}

	}

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		//ADDR flag está setada em high
		// interrupt is generated because of ADDR event
		//Lembrando que temos que dar um clear no ADDR flag se não o clock fica pra sempre em stretch
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		//BTF flag está setada em high - isso significa que se estemos em uma transmissão então TXE =1 e BTF = 1, então o shift register e o data register estão vazios, podemos transmitir mais uma vez (transmissão anterior terminou, por isso byte transfer finished),
		//isso significa que o TXE junto com o BTF pode ser usado para ver se a transmissão acabou.
		//Mas se estamos em uma recepção, então RXNE = 1 e BTF=1, então Shift register e data register estão ambos cheios e o clock estará em stretch, podemos ler os dados

		//Se chegou aqui então BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) //se for uma transmissão entrará nessa condicional
		{
			//make sure that TXE is also set .
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
			{
				//BTF and TXE = 1
				if(pI2CHandle->TxLen == 0 )
				{
					//1. generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)                            //só acionaremos o stop condition se não queremos mais gerar o repeated start, que é o caso em que o master segura o disciple para ele continuar enviando dados, sem que outro master solicite o barramento de dados
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);  //DEFINIREMOS A FUNÇÃO DE CALLBACK NO ARQUIVO MAIN DE CADA APLICAÇÃO (GERALMENTE SERÁ A MESMA FUNÇÃO)

				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			; //se a aplicação está ocupada em transmissão então esta função de send não faz nada (mesmo sendo uma interrupção)
		}
	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode, only the slave can execute this block
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);  //isso equivale a um read no SR1 - lembrando que isso só acontece no modo disciple, porque o hardware só mexe nessa flag em slaveMode
	if(temp1 && temp3)
	{
		//STOPF flag está setada em high
		//STOF flag is set
		//Clear the STOPF, i.e 1) read SR1 2) Write to CR1 ) - fizemos um read no SR1 na atribuição o temp3 logo acima

		pI2CHandle->pI2Cx->CR1 |= 0x0000;  //O uso do or bitwise aqui é para dar um dummy write com a vantagem de não mexer nos dados do control register 1 (CR1).
										   //

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP); //DEFINIREMOS A FUNÇÃO DE CALLBACK NO ARQUIVO MAIN DE CADA APLICAÇÃO (GERALMENTE SERÁ A MESMA FUNÇÃO)

	}
	//5. Handle For interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		//TXE flag está setada em high - we have to do data transmission
		//TXE em high é uma indicação de que o data register está vazio e pronto para trabalhar (ser preenchido com um dado, passar para o shift register e o shift register enviar)
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master - Lembrando que estamos criando esta função para transmissão feita pelo master apenas, não de um disciple
			//TXE flag is set
			//We have to do the data transmission - o master interrompe o seu própro processador para fazer a transmissão de dados
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//disciple
			//make sure that the disciple is really in transmitter mode - aqui o STM32 é disciple. O master externo faz a interrupção por I2C solicitando um byte e aqui o enviamos
			if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ); //A interrupção ocorreu porque o master quer que o disciple envie byte, com essa callback retornamos pro master os ACKs, status e outras flags necessárias dependendo do caso
			}															  //dentro da callback analisaremos qual flag foi passada como parametro e processaremos o evento dessa flag
		}
	}

	//6. Handle For interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		//RXNE flag está setada em high
		//check device mode .
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master - Lembrando que estamos criando esta função para receive do master apenas, não de um disciple

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}

		}
		else
		{
			//the device is disciple
			//make sure that the disciple is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV); //A interrupção ocorreu porque o master quer enviar byte para este slave, com essa callback retornamos pro master os ACKs, bytes de informação, status e outras flags necessárias dependendo do caso
			}														      //dentro da callback analisaremos qual flag foi passada como parametro e processaremos o evento dessa flag
		}
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	/*
	 * Esta função é analoga ao I2C_EV_IRQHandling, uando ela for chamada, verificamos nos control register e status register quais são as flags que acusam o tipo de erro que ocorreu
	 * e tomamos as atitudes condizentes
	 */

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN); //se ocorreu interrupção e essa função foi chamada, essa flag estará levantada, mas faremos um double check dentro dos if por precaução


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);  //dentro da callback analisaremos qual flag foi passada como parametro e processaremos o evento dessa flag

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	/*
	 * Para fechar a recepção de dados precisamos limpar todas as flags de interrupção, sejam elas quais forem
	 */

	//Implement the code to disable ITBUFEN Control Bit - aqui eu evito as interrupções devido RXNE e TXE
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit - aqui eu evito todas as interrupções do tipo evento
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) //o set de configuração do periférico está configurado como high no enable, então precisamos preservar isso (faremos no proximo comando)
	{                                                           //mas como na função de receiveIT fizemos um disable do ACK para avisar ao disciple que não queremos mais dados, então agora precisamos desfazer esse disable reabilitando o ACK
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	/*
	 * Para fechar a recepção de dados precisamos limpar todas as flags de interrupção, sejam elas quais forem
	 */

	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}
