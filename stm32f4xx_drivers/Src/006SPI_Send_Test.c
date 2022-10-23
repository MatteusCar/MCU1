/*
 * SPI_Send_Test.c
 *
 *  Created on: 16 de ago de 2022
 *      Author: Matteus
 */


/*
 * Aqui será testado um código para SPI e a placa de teste aqui será no modo master.
 * Para descobrir quais pinos GPIOs podem ser MISO / MOSI / NSS / SCLK do SPI devemos
 * consultar o datasheet desse processador e ver quais são as funções secundárias e terciárias
 * de cada pino, também conhecidas como funções alternativas (alternative functionalities)
 *
 * Após a consulta no datasheet, podemos ver que uma das possibilidades de pinos para essas funções secundárias são:
 *
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 *
 * Lembrar que para este processador que estamos usando (ARM Cortex - M4 na placa STM32 discovery)
 * ALT function mode é 5 para ser usado o SPI2
 */

#include <stdint.h>
#include <stm32f407xx.h>
#include <stdio.h>
#include <string.h>
#include <stm32f407xx_gpio_driver.h>
#include <stm32f407xx_spi_driver.h>

//Primeiro aqui Dizemos quais pinos de GPIO farão o quê no SPI
void SPI2_GPIOInit(void){
	GPIO_Handle_t SPIPins;

	//Primeiramente inicializar a GPIO certa (GPIOB com as configurações de SPI2, ou seja, função alternativa 5

	SPIPins.pGPIOx = GPIOB;                                       //pronto, setamos o base addr desejado, agora é setar as configurações
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;        //Aqui dizemos que esse pino terá uma das funções alternativas
	SPIPins.GPIO_PinConfig.GPIO_AltFunMode = 5;                   //Após consulta no datasheet vemos que a função alternativa 5 é a SPI2, como desejado
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//Agor iremos dizer quais GPIOBs irão funcionar como SPI

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);
}

//Aqui Iniciaremos o SPI em si
void SPI2_Init(void){
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;                                       //aqui escolhemos o base addr, para avisar ao programa que queremos o base addr do SPI2
	SPI2Handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;    //Full Duplex
	SPI2Handle.SPI_PinConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2Handle.SPI_PinConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV2;  //como estamos usando o cristal interno de 16MHz então o DIV2 vai produzir um SPI com 8MHz e essa é a maior frequência necessária
	SPI2Handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;              //8 bits de dados por transmissão
	SPI2Handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_HIGH;              //low iddle state
	SPI2Handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;              //first edge sampling
	SPI2Handle.SPI_PinConfig.SPI_SSM = SPI_SSM_EN;                 //software disciple selection

	SPI_Init(&SPI2Handle);
}

int main(void){

	char userData[] = "hello world";

	//Essa função serve para fazer com que as GPIOs escolhidas se comportem como SPI (SPI 2 no caso), qual pino será MISO, MOSI, CLK etc
	SPI2_GPIOInit();

	//Essa função serve para inicializar as configurações propriamente ditas, first ou second edge, Master ou slave, velocidade etc
	SPI2_Init();

	//Essa função coloca o NSS em High, lembrando que para o SPI do Master funcionar, esse pino precisa estar em alto
	SPI_SSIControl(SPI2, ENABLE);

	//Aqui habilita o SPI, após a chamada desta função não é mais possível alterar o funcionamento do SPI, a não ser que a desabilitemos de novo
	SPI_PeripheralControl(SPI2, ENABLE);

	//Primeiramente precisamos mandar o número de bytes que o slave deve ler, pois ele não sabe de antemão
	uint8_t dataLen = strlen(userData);
	SPI_Send(SPI2, &dataLen, 1);

	//manda a mensagem que se quer mandar
	SPI_Send(SPI2, (uint8_t*)userData, strlen(userData));  //precisei fazer esse type casting maluco pois estava dando um warning

	//Antes de dar um disable vamos confirmar que o periférico não está ocupado (usando a flag BSY no Status Register - SR)
	while((SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
