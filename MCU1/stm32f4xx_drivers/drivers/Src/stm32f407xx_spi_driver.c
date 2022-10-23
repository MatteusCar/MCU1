/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 14 de ago de 2022
 *      Author: Matteus
 */

#include <stdint.h>
#include "stm32f407xx_spi_driver.h"

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)   //O usuario vai instanciar uma struct do tipo SPI_Handle_t e vai configurar e passar como parametro para inicialização
{

	//Inicializando também o clock desse periférico para não esquecer
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Vamos primeiramente configurar os registradores do SPI dado o parâmetro recebido acima
	uint32_t temp = 0;

	// 1. Primeiramente começando pelo Modo
	temp |= (pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MODE);

	//2. Bus Config
	if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		temp &= ~(1 << 15); //bit resetado para habilitar full duplex mode
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		temp |= (1 << 15); //bit setado para habilitar half duplex mode
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){
		temp &= ~(1 << 15); //bit resetado para habilitar full duplex mode
		temp |= (1 << 10); // simplex mode
	}

	//3. SPI Clock Speed
	temp |= (pSPIHandle->SPI_PinConfig.SPI_SCLKSpeed << SPI_CR1_BR);

	//4. SPI Data Frame Format - DFF
	temp |= (pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF);

	//5. SPI CPOL
	temp |= (pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. SPI CPHA
	temp |= (pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA);

	//Aqui transferimos a configuração para o periférico mapeado naquele SPI propriamente dito
	pSPIHandle->pSPIx->CR1 |= temp;
}
void SPI_deInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1){
		SPI1_REG_RST();
	}
	else if (pSPIx == SPI2){
		SPI2_REG_RST();
	}
	else if (pSPIx == SPI3){
		SPI3_REG_RST();
	}
}

//nesta função deveremos passar o periférico que queremos monitorar e qual a flag monitorada
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t nomeDaFlag){
	if(pSPIx->SR & nomeDaFlag){   //lembrando que a flag nomeDaFlag já é um bit deslocado de acordo com o próprio nome da flag que se quer monitorar, no caso nomeDaFlag == SPI_TXE_FLAG == (1 << SPI_SR_TXE) == (1 << 1)
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En)                        //dado um base address de um SPI a gente habilita ou desabilita seu clock
{														                           //En = 1 habilita, En = 0 desabilita
	if(En == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLOCK_EN();
		}
		else if (pSPIx == SPI2){
			SPI2_PCLOCK_EN();
		}
		else if (pSPIx == SPI3){
			SPI3_PCLOCK_EN();
		}
	}

	else{
		if(pSPIx == SPI1){
			SPI1_PCLOCK_DI();
		}
		else if (pSPIx == SPI2){
			SPI2_PCLOCK_DI();
		}
		else if (pSPIx == SPI3){
			SPI3_PCLOCK_DI();
		}
	}
}
/*
 * Para TX e RX temos basicamente três formas de comunicação
 * Polling - blocking based
 * Interrupção - non-blocking based
 * DMA
 */

/*
 * Como podemos ver abaixo na função send, ela fica bloqueada até o loop sair do segundo while, ou seja, até que a flag
 * do Status Register avise que o TX buffer está vazio para escrita. Isso significa que se houver algum mau funcionamento
 * de hardware ou mesmo de firmware que dê um set nessa flag o programa vai ficar parado nessa função, essa é uma das desvantagens
 * de se usar polling para send ou receive de mensagens
 */
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)   //o primeiro parametro é o endereço de onde está mapeado (pelo memoery map) aquele SPI em questão
{
	while(len>0){                                                      //enquanto len não for nulo (ainda não enviou toda a mensagem)

		//1. Wait until TXE is set
		//while(!(pSPIx->SR & (1 << SPI_SR_TXE)));                     //espere até a flag do Tx avisar que o TX buffer está vazio (pronto para enviar byte) para sair do loop
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);   //essa espera é o que caracteriza o polling do código

		//2. Format is 8 or 16 bits?
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){                           //verificar se é 1 byte por vez ou 2

			//16 bits format
			pSPIx->DR = *(uint16_t*)pTxBuffer;                         //colocar o dado no registrador de dados (DR) que vai enviar, note que primeiro fazemos um type cast de uint8_t para uint16_t. Depois pegamos o conteúdo dentro do ponteiro usando o operador de de-referenciação
			len--;
			len--;                                                     //duas vezes pois são dois bytes enviados
			//(uint16_t*)pTxBuffer++;                                    //incrementar duas posições na memória, por isso o casting antes
			pTxBuffer+=2;
		}
		else {

			//8 bits format
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;                                              //incrementar uma única posição
		}
	}
}

void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{

}

//Mesmo após as configurações do SPIx (1,2,3..) e da ativação dos seus clocks, o SPI ainda não está preparado para ser usado
//pois ainda existe uma flag a ser setada, que é justamente a flag de enable. Essa flag serve para que as configurações de clock e
//do SPI propriamente dito sejam alteradas apenas quando o periférico estiver desabilitado. A partir do momento que essa flag é habilitada
//não é possível mais fazer alterações na configuração do SPI, sendo que para configurar é necessário desabilitar mais uma vez essa flag e
//mexer nas configurações

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*
 * Interrupt Routine Queue handlers
 */

void SPI_IRQInterruptConfig(uint8_t IRQnumber, uint8_t  IRQEn)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void SPI_IRQHandler(SPI_Handle_t *pHandle)                                               //Interrupção externa ativda por esse pino será gerenciada
{

}
