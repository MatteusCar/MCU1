/*
 * 009_IT_by_SPI.c
 *
 *  Created on: 29 de ago de 2022
 *      Author: Matteus
 */

/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */
/*
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board , acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */
#include<stdio.h>
#include<string.h>

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

SPI_Handle_t SPI2handle;

#define MAX_LEN 500                                                            //máximo tamanho de mensagem que se pode enviar por SPI

char RcvBuff[MAX_LEN];

volatile char ReadByte;
volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */


/*
 * Usaremos a SPI2 inits para informar qual GPIO terá qual função, velocidade etc
 */
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_AltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);


	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);
}

/*
 * Aqui como sempre configuraremos o SPI propriamente dito, full-duplex, master mode, speedDIV32 etc
 */
void SPI2_Inits(void)
{
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_PinConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2handle.SPI_PinConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle); //inicia o clock do bus SPI
}


/*This function configures the gpio pin over which SPI peripheral issues data available interrupt
 * Ou seja, temos aqui uma função que configura um pino GPIOD6 para triggar a interrupção externa
 */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin,0,sizeof(spiIntPin));

	//this is led gpio configuration
	spiIntPin.pGPIOx = GPIOD;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_Init(&spiIntPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
														  //Então temos o seguinte: como queremos a interrupção pelo GPIOD6, então temos que configurar o EXTIO6 (se fosse qualquer outro periférico
														  //teríamos que configurar o acônimo correspondente a ele. Para isso é necessário ver o manual do usuário e pegar a vector table que relaciona
														  //cada periférico com sua IRQ) que tem IRQ number 23 (todos do 5 ao 9, EXTIO9_5 todos estão atrelados ao 23) então é esse número que temos que passar como
														  //parametro para o interrupt Config. Se tudo der certo a função EXTIO9_5_IRQHandler será chamada assim que houver
														  //detecção de borda de fall trigger no GPIOD6. Basicamente é assim que ocorre uma interrupção por GPIO em MCUs, nesse caso a interrupção é SPI													  //aciona o SPI mas é isso mesmo
}

/*
 * Resumo: quando o hardware disciple quer mandar um byte, o GPIO6 terá uma borda de descida que gerará uma
 * interrupção, que chamará a função EXTI9_5_IRQ_Handler, que irá setar a flag dataAvailable, liberando o fluxo
 * do código, assim desabilitamos a interrupção, impossibilitando-a de acionar de novo para podermos processar tudo, habilitamos o SPI2
 * para lermos o dado, e então damos um dummy write para depois recebermos o byte, esse dummy write e posterior read vao habilitar
 * mais uma interrupção internamente, que chamará o SPI2 handler, que lidará com a recepcção em si
 */
int main(void)
{
	uint8_t dummy = 0xff;

	Slave_GPIO_InterruptPinInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEControl(SPI2,ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);  //essa função habilita a interrupção dentro das funções SendIT e ReceiveIT

	while(1){

		rcvStop = 0;

		while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

		/*
		 * Quando o disciple colocar algum dado no data register dele próprio, ele avisará o master que fez isso, avisando através da interrupção
		 * do botão PD6 do STM32. Essa interrupção do PD6 chamará a função EXTI9_5_IRQHandler, que lidará com essa interrupção e setará a flag dataAvailable,
		 * fazendo com que a condição while(!dataAvailable) seja quebrada, fazendo o código avançar. A partir daí desabilitamos a interrupção pelo PD6 enquanto estivermos
		 * dentro da condição que a gerou (para não termos duas interrupções iguais no mesmo contexto). Daí habilitamos o SPI, e enquanto a recepção não terminar nós
		 * fazemos a transferência de dados entre data registers bem conhecida, ou seja, dummyWrite seguido de ReadByte. Cada dupla SPI_SendIT seguida
		 * de SPI_ReceiveIT faz a transferência de um único byte, mas como provavelmente queremos ler vários bytes, então teremos 2 loops de sendIT e receiveIT.
		 * Dentro da função SPI_SendIT nós apenas habilitamos a interrupção de receive setando o bit RXNE, o que faz com que o handler do SPI seja chamado, ou seja, acontece
		 * a chamada da função SPI2_IRQHandler, que chamará a SPI_IRQHandling, que por sua vez perceberá (através de condicionais internas) que a interrupção ocorrida foi uma
		 * interrupção para receive, chamando então a função rxne_spi_interrupt_handler, que vai ler o byte recebido que agora está no DR do master, fechar a transmissão atual
		 * e fazer uma callback, que vai dar um "append" do byte lido no RcvBuffer e, caso não se tenha mais bytes para serem lidos, a callback irá setar a flag rcvStop e aí
		 * saberemos que todos os bytes que estavam no DR foram recebidos, caso contrário o loop while(!rcvStop) continua e repete todo o processo
		 *
		 */

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,DISABLE);  //desabilita a interrupção para não atrapalhar esse fluxo. Não ter a mesma interrupção em cima de interrupção "igual"

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		while(!rcvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while ( SPI_SendIT(&SPI2handle,&dummy,1) == SPI_BUSY_IN_TX);
			while ( SPI_ReceiveIT(&SPI2handle,(uint8_t*)&ReadByte,1) == SPI_BUSY_IN_RX);
		}

		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
	}
	return 0;
}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || ( i == MAX_LEN))
		{
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
	}
}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandler(GPIO_PIN_6);
	dataAvailable = 1;
}
