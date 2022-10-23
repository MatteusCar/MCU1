/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 14 de ago de 2022
 *      Author: Matteus
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

#define SPI_MODE_MASTER    1
#define SPI_MODE_SLAVE     0

#define SPI_BUS_CONFIG_FD          1 //full duplex
#define SPI_BUS_CONFIG_HD		   2 //half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RX  3 //simplex receiver

#define SPI_SCLK_SPEED_DIV2	    0 //baudrate divisor de frequencia para velocidade de comunicação SPI
#define SPI_SCLK_SPEED_DIV4     1
#define SPI_SCLK_SPEED_DIV8     2
#define SPI_SCLK_SPEED_DIV16    3
#define SPI_SCLK_SPEED_DIV32    4
#define SPI_SCLK_SPEED_DIV64    5
#define SPI_SCLK_SPEED_DIV128   6
#define SPI_SCLK_SPEED_DIV256   7

#define SPI_DFF_8BITS   0  //data frame format (8 ou 16 bits por transmissão)
#define SPI_DFF_16BITS  1

#define SPI_CPOL_HIGH   1 //idle state in high by default
#define SPI_CPOL_LOW    0 //idle state in low

#define SPI_CPHA_HIGH   1 //se este bit está em 1 então a segunda borda do sinal de clock amostra um bit do dado transmitido (se vai ser uma borda de descida ou subida depende do CPOL
#define SPI_CPHA_LOW    0 //se este bit está em 0 então a priemira borda do sinal de clock amostra um bit do dado transmitido (se vai ser uma borda de descida ou subida depende do CPOL

#define SPI_SSM_EN      1
#define SPI_SSM_DI      0

#define SPI_TXE_FLAG    (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   (1 << SPI_SR_BSY)
/*
 * Structure Para configuração do SPI
 */
typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Structure que servirá de handler para um periférico SPI
 */

typedef struct                              //em pSPIx o usuario vai setar as informações desejadas que serão usadas pelo nosso driver para configurar os periféricos usando a estrutura SPI_Config, que já tem os Base Addrs definidos
{
	SPI_RegDef_t *pSPIx;                    //ponteiro para acesso rápido ao SPIx em questão, ex: pSPIx = SPI1
	SPI_Config_t SPI_PinConfig;             //faz pGPOIx apontar para o endereço do SPI1 (seu base Address) e
						                    //já acessa direto o registrador certo do SPI em questão
}SPI_Handle_t;

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);        //O usuario vai instanciar uma struct do tipo SPI_Handle_t e vai configurar e passar como parametro para inicialização
void SPI_deInit(SPI_RegDef_t *pSPIx);

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En);                        //dado um base address de um SPI a gente habilita ou desabilita seu clock
														                           //En = 1 habilita, En = 0 desabilita

/*
 * Para TX e RX temos basicamente três formas de comunicação
 * Polling - blocking based
 * Interrupção - non-blocking based
 * DMA
 */

void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len); //o primeiro parametro é o endereço de onde está mapeado (pelo memoery map) aquele SPI em questão
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * SPI Peripheral Enable/Disable
 */

//Mesmo após as configurações do SPIx (1,2,3..) e da ativação dos seus clocks, o SPI ainda não está preparado para ser usado
//pois ainda existe uma flag a ser setada, que é justamente a flag de enable. Essa flag serve para que as configurações de clock e
//do SPI propriamente dito sejam alteradas apenas quando o periférico estiver desabilitado. A partir do momento que essa flag é habilitada
//não é possível mais fazer alterações na configuração do SPI, sendo que para configurar é necessário desabilitar mais uma vez essa flag e
//mexer nas configurações

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Interrupt Routine Queue handlers
 */

void SPI_IRQInterruptConfig(uint8_t IRQnumber, uint8_t  IRQEn);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandler(SPI_Handle_t *pHandle);                                               //Interrupção externa ativda por esse pino será gerenciada


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */

