/*
 * SPI_Send_Test.c
 *
 *  Created on: 22 de ago de 2022
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

extern void initialise_monitor_handles();

//CMD codes
#define COMMAND_LED_CTRL     0x50
#define COMMAND_SENSOR_READ  0x51
#define COMMAND_LED_READ     0x52
#define COMMAND_PRINT        0x53
#define COMMAND_ID_READ      0x54

//
#define LED_ON   1
#define LED_OFF  0

//Arduino analog Pins
#define ANALOG_PIN0  0
#define ANALOG_PIN1  1
#define ANALOG_PIN2  2
#define ANALOG_PIN3  3
#define ANALOG_PIN4  4

#define LED_PIN      9

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
	SPI2Handle.SPI_PinConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV8;  //como estamos usando o cristal interno de 16MHz então o DIV2 vai produzir um SPI com 8MHz e essa é a maior frequência necessária
	SPI2Handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;              //8 bits de dados por transmissão
	SPI2Handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;              //low iddle state
	SPI2Handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;              //first edge sampling
	SPI2Handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI;                 //hardware disciple selection

	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(){

	GPIO_Handle_t GpioBtn, GpioLed;

	GpioBtn.pGPIOx = GPIOA;                                        //GPIOA0 - já está ligado ao botao na placa discovery, o botão está com um pino no VCC e no outro um resistor de PullDown
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;            //O mesmo ponto do resistor de PullDown está ligado ao GPIOA0 por um resistor.
	GpioBtn.GPIO_PinConfig.GPIO_PinMode =   GPIO_MODE_IN;          //INPUT
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;        //Fast Transition between Low and High
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;    //Without PullUp or PullDown

	GPIO_Init(&GpioBtn);

	GpioLed.pGPIOx = GPIOD;                                      //GPIOD
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;         //GPIOD12
	GpioLed.GPIO_PinConfig.GPIO_PinMode =   GPIO_MODE_OUT;       //OUTPUT
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;       //LOW Transition between Low and High
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;     //Open Drain
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;  //Without PullUp or PullDown

	PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);
}

void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
}

uint8_t SPI_VerifyResponse(uint8_t response){

	if(response == (uint8_t)0xF5){
		//temos um ack
		return 1;
	}
	else{
		return 0; //não precisaria do else, era só retornar zero mas quero ser bem quadrado aqui para lembrar depois
	}
}

int main(void){

	uint8_t dummyWrite = 0xff;
	uint8_t dummyRead;
	//

	initialise_monitor_handles();

	printf("application is running\n\r");
	GPIO_ButtonInit();

	//Essa função serve para fazer com que as GPIOs escolhidas se comportem como SPI (SPI 2 no caso), qual pino será MISO, MOSI, CLK etc
	SPI2_GPIOInit();

	//Essa função serve para inicializar as configurações propriamente ditas, first ou second edge, Master ou slave, velocidade etc
	SPI2_Init();

	//Função para habilitar o output NSS em high ou low, ele só habilita, quem controla quando vai ser high ou low é o SPE igual a 1 ou 0
	SPI_SSOEControl(SPI2, ENABLE);

	while(1){
		while(!(ReadFromInputPin(GPIOA, GPIO_PIN_0)));

		delay();

		//Aqui habilita o SPI, após a chamada desta função não é mais possível alterar o funcionamento do SPI, a não ser que a desabilitemos de novo
		SPI_PeripheralControl(SPI2, ENABLE);

		//###############################################################  1. CMD_LED_CTRL <pin no(1)>    <value(1)>  ##############################################################################

		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ackByte;
		uint8_t args[2];
																				   //Repare que a função send a seguir coloca o dado da variável command no Data Register (DR) e já realiza o swap, então após o retorno da função SEND, já podemos dar um READ no DR do master pois já existe dado que estava no DR do disciple
		SPI_Send(SPI2, &commandCode, 1);                                           //quando o slave receber isso ele vai verificar se pode ou não receber esse comando pois quando o Master fizer um read, o disciple também fará um read
		SPI_Receive(SPI2, &dummyRead,1);										   //Se o master faz um read o disciple também faz um read para processar o dado recebido. No caso do master é um dummy read e do disciple é um readCommand (a ser programado no disciple)
																				   //devemos lembrar que sempre que um master ou disciple conseguem enviar um byte no SPI, eles também recebem um byte (primeiramente lixo)
																				   //Sempre que o master mandar um byte e a comunicação for bem sucedida o disciple vai mandar um garbage do shift register pois é assim que o SPI funciona, se vc manda vc recebe instantaneamente.
																				   //Mais aprofundadamente, quando um SPI_Send é realizado, o bit RXNE (RX Not Empty) no Status register é colocado em alto e só é colocado em baixo de novo se dermos um read aleatório (dummy read) para limpar o data Register no modo receptor
		                                                                           //Então para limpar o RXNE, de acordo com o datasheet, precisamos dar um read depois de cada Send, para limpar o Status register, colocando em baixo de novo o RXNE automaticamente nesse read (bit RXNE = 0 significa RX empty e pronto para mais uma recepção)
																				   //Em resumo, depois de cada send devemos dar um read
																				   //A sequencia fica: mandamos um byte avisando qual o comando (commandCode) usando a função send e ao mesmo tempo recebemos um byte garbage
																				   //Assim que o disciple recebe o command code ele está programado previsamente para colocar um ack no data register
																				   //Para limpar o garbage recebido no primeiro send devemos dar um dummy read no data register no master
																				   //Para receber o ack do disciple devemos enviar um dummy send e depois dar um read pra pegar o ack e assim por diante

		SPI_Send(SPI2, &dummyWrite, 1);                                            //Quando o master faz um dummy write o disciple faz um ACK write (ou NACK)
		SPI_Receive(SPI2, &ackByte, 1);                                            //Sendo assim o Master deve mandar um dummy byte pra receber pelo shift register um ACK ou NACK do slave, lembrando
		                                                                           //que o SPI no funcionamento full duplex só aciona quando um byte é mandado pelo master
		                                                                           //quando a chamada do dummy voltar (retornar) então uma mensagem de ack ou nack estará disponível no DR do master para ser lida,
		                                                                           //portanto vamos enviar o dummy byte e logo depois vamos ler o que ficou disponível no SPI para ser lido
																				   //Devemos lembrar que o disciple deve estar preparado para mandar um ACK assim que detectar um comando válido vindo do Master
		if(SPI_VerifyResponse(ackByte)){
			//se a resposta for um ack então podemos mandar a mensagem requerida
			args[0] = LED_PIN;                                                     //no caso do arduino estará ligado no pino 9
			args[1] = LED_ON;                                                      // ligar

			                                                                       //aqui podemos mandar esses dados, já há um shake hands entre master e disciple definindo previamente que a ordem será <comando, pino, valor>
			SPI_Send(SPI2, args, 2);

		}

		//###############################################################  2. CMD_Sensor_Read <pin no(1)>  <value(1)>  ##############################################################################

		while(!(ReadFromInputPin(GPIOA, GPIO_PIN_0)));
		delay();
		commandCode = COMMAND_SENSOR_READ;

		//Send command e depois faz o dummy read pra limpar o buffer de recepção
		SPI_Send(SPI2, &commandCode, 1);
		SPI_Receive(SPI2, &dummyRead,1);

		//mais uma vez dummy write para receber a resposta (ack), e a recebemos com um read
		SPI_Send(SPI2, &dummyWrite, 1);
		SPI_Receive(SPI2, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte)){
			//se a resposta for um ack então podemos mandar a mensagem requerida
			args[0] = ANALOG_PIN0;                                                     //
				                                                                       //aqui podemos mandar esses dados, já há um shake hands entre master e disciple definindo previamente que a ordem será <comando, pino, valor>
			SPI_Send(SPI2, args, 1);                                                   //send avisando qual pino deve ser lido


			SPI_Receive(SPI2, &dummyRead, 1);                                          //atrelado ao send anterior para limpar o garbage recebido simultaneamente ao send

			delay();                                                                   //esse delay serve para o disciple ter tempo hábil para os dados

			SPI_Send(SPI2, &dummyWrite, 1);                                            //dummy write para ler a próxima mensagem

			uint8_t analog_read;
			SPI_Receive(SPI2, &analog_read, 1);                                        //get da mensagem obtida usando o send dummy write acima

			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}

		//###############################################################  3.  CMD_LED_READ 	 <pin no(1) >  ##############################################################################

		//Repare que todos os blocos tem praticamente a mesma estrutura, mudando apenas o comando (e possivelmente o número de bytes enviados)

		//wait till button is pressed
		while( ! ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//coloca-se um deboucing de 200ms
		delay();

		commandCode = COMMAND_LED_READ;

		//send command
		SPI_Send(SPI2,&commandCode,1);

		//do dummy read to clear off the RXNE
		SPI_Receive(SPI2,&dummyRead,1);                                           //Aqui o disciple ao mesmo tempo deu um read no comando recebido anterior e deve estar preparado para enviar um ACK para confirmação do master

		//Send some dummy byte to fetch the response from the slave.
		//lembrando que no dummy write o disciple já deve ter identificado o Command e já colocou um ACK (ou NACK) no Data Register (DR) para ser lido
		SPI_Send(SPI2,&dummyWrite,1);

		//read the ack byte received
		SPI_Receive(SPI2,&ackByte,1);                                            //após o swap anterior lemos aqui o ACK (ou nack) recebido

		if( SPI_VerifyResponse(ackByte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_Send(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_Receive(SPI2,&dummyRead,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_Send(SPI2,&dummyWrite,1);

			uint8_t led_status;
			SPI_Receive(SPI2,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);

		}

		//###############################################################  //4. CMD_PRINT 		<len(2)>  <message(len) >  ##############################################################################

		//Repare que todos os blocos tem praticamente a mesma estrutura, mudando apenas o comando (e possivelmente o número de bytes enviados)

		//wait till button is pressed
		while( ! ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_PRINT;

		//send command
		SPI_Send(SPI2,&commandCode,1);

		//do dummy read to clear off the RXNE, ao mesmo tempo o disciple recebe e le o comando
		SPI_Receive(SPI2,&dummyRead,1);

		//Send some dummy byte to fetch the response from the slave, ao mesmo tempo o disciple coloca e envia um ACK no DR para o swap típico do SPI
		SPI_Send(SPI2,&dummyWrite,1);

		//read the ack byte received
		//Aqui o swap já foi feito e só precisamos ler o que foi recebido
		SPI_Receive(SPI2,&ackByte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(ackByte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_Send(SPI2,args,1); //sending length, LEMBRANDO QUE PASSAR O VETOR È A MESMA COISA QUE PASSAR O PONTEIRO PARA A "CABEÇA" DO VETOR, POR ISSO A AUSENCIA DO & E DO ZERO AQUI, O QUE DEIXARIA &args[0]

			//do dummy read to clear off the RXNE
			//Mais uma vez para limpar o DR na recepção
			SPI_Receive(SPI2,&dummyRead,1);

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_Send(SPI2,&message[i],1);
				SPI_Receive(SPI2,&dummyRead,1);
			}

			printf("COMMAND_PRINT Executed \n");

		}

		//###############################################################  5. CMD_ID_READ  ##############################################################################

		//wait till button is pressed
		while( ! ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_ID_READ;

		//send command
		SPI_Send(SPI2,&commandCode,1);

		//do dummy read to clear off the RXNE
		SPI_Receive(SPI2,&dummyRead,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_Send(SPI2,&dummyWrite,1);

		//read the ack byte received
		SPI_Receive(SPI2,&ackByte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackByte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_Send(SPI2,&dummyWrite,1);
				SPI_Receive(SPI2,&id[i],1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}

		//Antes de dar um disable vamos confirmar que o periférico não está ocupado (usando a flag BSY no Status Register - SR)
		while((SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)));

		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}


