/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jan 17, 2022
 *      Author: matte
 */

/*
 * Init and De-Init
 */
#include <stdint.h>
#include "stm32f407xx_gpio_driver.h"

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp;
	/*Vamos separar as funções de init entre Modos Com e Sem interrupção e depois setar
	  cada modo de acordo com a estrutura passada como parâmetro pelo usuário */

	/*Primeiro vamos habilitar o clock dessa GPIO para não esquecermos (muita gente esquece e o periférico não funciona)	 */
	PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Set do Modo
	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) <= GPIO_MODE_ANALOG){                                           //Se menor que ANALOG, então não tem interrupção
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));    //ex se o PinMode = 2, será lido como 10 (em binário) e será shiftado para a direita 2*PinNumber vezes, pois o registrador tem 32 bits e cada pino precisa de 2 bits pro modo
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));                   //Limpando (zerando) os dois bits para ter certeza que estão zerados antes de fazer o bitwise
		pGPIOHandle->pGPIOx->MODER |= temp;                                                                       //com a outra estrutura do parametro passado pelo usuário, acessamos diretamente o Base Address do periférico desejado e setamos seu registrador
	}
	else{                                                                                                        //Neste else trataremos as interrupções
		//SYSCFG vai ser usado para escolher qual port vai gerar a interrupção (PORT A, PORT B, PORT C...)
		//EXTI vai ser usado para dizer o modo de interrupção (Falling edge, Rising Edge Trigger ou ambos)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//Configurar o FTSR (Falling Trigger Selection Register)
			EXTI->FTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);                                    //Habilitando a interrupção com Falling Trigger no pino PinNumber
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);                                    //Garantindo que o Rising Trigger está desabilitado
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//Configurar o RTSR (Rising Trigger Selection Register)
			EXTI->RTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);                                    //Habilitando a interrupção com Rising Trigger no pino PinNumber
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);                                    //Garantindo que o Falling Trigger está desabilitado
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//Configurar o FTSR e o RTSR simultaneamente
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);                                    //Habilitando ambas interrupções com Rising e Falling Trigger no pino PinNumber
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Estamos dentro da interupção: Configurar o SYSCFG_EXTICR (ou seja, fazer o port selection)
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;                                                //isso nos dará qual registrador SYSCFG_EXTICR[temp1] deveremos escolher
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;                                                //isso nos dará qual bloco do registrador temos que colocar os 4 bits
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLOCK_EN();
		SYSCFG->EXTICR[temp1] |= (portCode << ( 4*temp2 ));

		//Habilitar o interrupt delivery using Interrupt Mask Register (IMR) para entregar a interrupção para o NVIC
		//De acordo com o pino escolhido, dependendo do pino o EXTI será entrega para um número no NVIC
		//Ex:Se o pino for GPIOn0, quem será o responsável pela entrega será o EXTIO0, que está
		//ligado fisicamente no IRQ 6 do NVIC e é lá que o processador irá chamar o Handler do ETXIO0, onde colocaremos a função
		//para tratar dessa interupção
		EXTI->IMR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;

	//2. Set da velocidade
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));                   //Limpando (zerando) os dois bits para ter certeza que estão zerados antes de fazer o bitwise
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp=0;

	//3. SET de PULLUP PULLDOWN
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));                          //Limpando (zerando) os dois bits para ter certeza que estão zerados antes de fazer o bitwise
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;

	//4. Configure OUTPUT TYPE, PUSH PULL or OPEN DRAIN
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));                          //Limpando (zerando) o bit para ter certeza que estão zerados antes de fazer o bitwise
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;

	//5. Configure Alternate Functionality
	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;                                            //sempre igual ou a 0 ou a 1
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->PUPDR &= ~(0xF << (4*temp2));                                                 //Limpando (zerando) os bits para ter certeza que estão zerados antes de fazer o bitwise
		pGPIOHandle->pGPIOx->AFR[temp1] |= ((pGPIOHandle->GPIO_PinConfig.GPIO_AltFunMode) << (4*temp2));   //cada registrador de 32 bits nesse caso tem 8 pinos, temp1 recebe o resto da divisão inteira por 8 vai dizer se devemos mexer no registrador 0 ou no 1
		                                                                                                   //dado qual registrador (0 ou 1) temos que saber a posição do pino que queremos mexer (normalizado entre 0 e 8), multiplicamos por 4 pois são 4 bits para right shift
		   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   //ex, digamos que queremos escrever o modo alternativo correspondente a 1010 no pino 9 do GPIOA. Entao 9/8 = 1, ou seja, registrador AFR[1]. Qual posição? A posição 9%8 = 1, portanto posição 1 do registrador 1
	}																									   //mas como temos 4 bits por posição, temos que dar um shift left na posição multiplicado por 4, então 4*posicao = 4*1 = 4 de shift.
}
void deInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)         GPIOA_REG_RST();
	else if (pGPIOx == GPIOB)	GPIOB_REG_RST();
	else if (pGPIOx == GPIOC)	GPIOC_REG_RST();
	else if (pGPIOx == GPIOD)	GPIOD_REG_RST();
	else if (pGPIOx == GPIOE)	GPIOE_REG_RST();
	else if (pGPIOx == GPIOF)	GPIOF_REG_RST();
	else if (pGPIOx == GPIOG)	GPIOG_REG_RST();
	else if (pGPIOx == GPIOH)	GPIOH_REG_RST();
	else if (pGPIOx == GPIOI)	GPIOI_REG_RST();

}

/*
 * Peripheral Clock Setup
 */
void PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En) //Controlar o Clock de um periféico é equivalente a habilitá-lo ou desabilitá-lo
{
	if(En == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLOCK_EN();
		}
		else if (pGPIOx == GPIOB){
			GPIOB_PCLOCK_EN();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLOCK_EN();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLOCK_EN();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLOCK_EN();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLOCK_EN();
		}
		else if (pGPIOx == GPIOG){
			GPIOG_PCLOCK_EN();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLOCK_EN();
		}
		else if (pGPIOx == GPIOI){
			GPIOI_PCLOCK_EN();
		}
	}

	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLOCK_DI();
		}
		else if (pGPIOx == GPIOB){
			GPIOB_PCLOCK_DI();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLOCK_DI();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLOCK_DI();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLOCK_DI();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLOCK_DI();
		}
		else if (pGPIOx == GPIOG){
			GPIOG_PCLOCK_DI();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLOCK_DI();
		}
		else if (pGPIOx == GPIOI){
			GPIOI_PCLOCK_DI();
		}
	}
}
/*
 * Read and Write
 */
uint8_t ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);           //se queremos ler o pino 7 (pinNumber = 7) então (pGPIOx->IDR >> pinNumber) vai shiftar o valor do pino 7 até à primeira casa (que é onde temos acesso para ler) e o mascaramento com o & retorna o valor, armazenando-o em value
	return value;                                                         //apenas a operacao de shift nao muda nada, precisamos fazer o and para retornar o valor que ficou no registrador
}


uint16_t ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR); //aqui estamos armazenando todos os bits (um de cada pino), diferentemente da função anterior, que queriamos apenas um pino
	return value;					 //Afinal de contas é um input PORT, não um input PIN
}
void WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)  pGPIOx->ODR |=  (1 << pinNumber);
	else                       pGPIOx->ODR &= ~(1 << pinNumber);
}

void WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	//inverter o bit naquele pino usando o bitwise A XOR B = a ^ B
	pGPIOx->ODR ^= (1 << pinNumber);
}

/*
 * Interrupt Service Routine (ISR) Handling and Interrupt Routine Queue Configuration
 */
void GPIO_IRQInterruptConfig(uint8_t IRQnumber, uint8_t  IRQEn)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;                                              //para descobrirmos qual registrador IPR entre 0 - 59 cada um com 4 seções de 8 bits (Se IQRNumber == 23 então IQRNumber/4 == 5 então deveremos mexr no IPR5)
	uint8_t iprx_section = IRQNumber % 4;                                      //para descobrirmos qual a seção
	uint8_t shift_amount = 8*iprx_section + (8 - NO_PR_BITS_IMPLEMENTED);		 //Esta linha primeiramente faz o shift maior colocando os 8 bits na seção certa, depois faz mais um shift para à esquerda de 4 bits pois os primeiros 4 bits de cada seção não são utilizados
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);               //para colocarmos efetivamente o IRQPriority na seção de 8 bits

	/*Como NVIC_PR_BASE_ADDR foi declarado como uint32_t cada +1 que somamos no seu endereço pula 32bits (4 bytes), ou seja
	  iprx pode ser diretamente somando ao NVIC_PR_BASE_ADDR que já pulará 4bytes, fazendo com que seja desnecessário o
	  uso da multiplicação por 4 nesse cenário. Caso o NVIC_PR_BASE_ADDR fosse declarado como uint8_t então o iprx teria que ser multiplicado por 4 mesmo*/
}

void GPIO_IRQHandler(uint8_t pin)                                               //Interrupção externa ativda por esse pino será gerenciada
{
	//Clear the Pending Register
	if(EXTI ->PR & (1 << pin))
	{
		EXTI ->PR |= (1 << pin); //pode parecer estranho mas é isso mesmo,
	}							 //para limpar o bit 1 do Pending Register (PR) devemos forçar um bit 1 nele mesmo
}
