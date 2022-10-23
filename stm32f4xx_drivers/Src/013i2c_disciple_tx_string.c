/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Feb 24, 2019
 *      Author: admin
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

#define SLAVE_ADDR  0x68
#define MY_ADDR     SLAVE_ADDR

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t Tx_buf[32] = "STM32 slave mode testing.."; //se for usar com arduino como master e a discovery como disciple nao passar de 32 bytes aqui

/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	I2CPins.GPIO_PinConfig.GPIO_AltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;

	GPIO_Init(&GPIOBtn);
}


int main(void)
{

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//I2C IRQ configurations - interrupt enable
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE); //para aplcações em que o STM32 discovery é disciple devemos setar as configurações de interrupção, lembramos que um disciple sempre está com interrupções ativadas
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);


	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE); //acima configuramos os tipos de interrupção que queriamos (numero de IRQ handler), agora temos os enables habilitados

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1 (peripheral enable)
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	//até aqui tudo foi habilitado, agora esse while fica esperando alguma interrupção do Master acontecer e chamar a callback para lidar com a interrupção
	while(1);

}

/*
 * A INTERRUPÇÃO NO CASO DO SLAVE É FEITA EXTERNAMENTE PELO MASTER. SE O BARRAMENTO DE EVENTOS DO I2C FOR ATIVADO ENTÃO É UM EVENTO E SERÁ CHAMADA A I2C1_EV_IRQHANDLING (i2c1 pois foi ela que habilitamos na main), QUE LERÁ AS
 * FLAGS DE INTERRUPÇÃO E LIDARÁ COM O QUE FOR NECESSÁRIO (SEJA REQUEST DE DADO DO MASTER QUERENDO DADO, OU SE FOR DADO DO MASTER PARA SER RECEBIDO POR ESTE SLAVE)
 * CASO CONTRARIO SE FOR UMA INTERRUPÇÃO PELO BARRAMENTO DE ERRO ENTÃO SERÁ CHAMADA A FUNÇAO I2C1_ER_IRQHandler (i2c1 pois foi ela que habilitamos na main).
 * DENTRO DE AMBAS ESSAS FUNÇÕES É QUE ALÉM DO HANDLER TEREMOS AS CALLBACKS PARA AVISAR O MASTER O QUE ESTÁ ACONTECENDO DENTRO DA INTERRUPÇÃO
 */


void I2C1_EV_IRQHandler (void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler (void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}


/*
 * A LOGICA SERÁ A SEGUINTE:
 * PRIMEIRAMENTE O MASTER DÁ UM COMANDO DE WRITE, QUE O DISCIPLE RECEBERÁ O BYTE DE COMANDO E ARMAZENARÁ ESTE BYTE NA VARIAVEL commandCode.
 * COMO PRIMEIRA COISA O BYTE RECEBIDO SERÁ O 0x51 QUE É O BYTE QUE SOLICITA O LENGTH DA MENSAGEM. EM SEGUNDO LUGAR O MASTER ENVIARÁ DE NOVO
 * UMA SOLICITAÇÃO MAS AGORA UM READ, QUE OBRIGARÁ O DISCIPLE A LER O commandCode, ver que era o ox51 e enviar o lenght. DEPOIS
 * O MASTER ENVIARÁ MAIS UM WRITE COM O BYTE 0x52 E UM READ PARA OBRIGAR O DISCIPLE A LER O 0x52 E ENVIAR DESTA VEZ A MENSAGEM.
 * AÍ NESTE ENVIO O DISCIPLE ENVIARÁ A MENSAGEM BYTE A BYTE, A CADA BYTE ENVIADO O MASTER ENVIA UM ACK DIZENDO QUE QUER QUE O DISCIPLE CONTINUE
 * ENVIANDO.
 *
 * RESUMINDO DO PONTO DE VISTA DO MASTER
 *
 * 		WRITE --> 0x51
 * 		READ  <-- LEN
 * 		WRITE --> 0X52
 * 		READ  <-- MSG
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv) //Função para notificar a aplicação o que está acontecendo dentro da interrupção, tanto que ela é chamada apenas na IRQ EV handler e IRQ ER Handler
{																		  //Se o STM32 for um disciple então é aqui que enviaremos os ACKs e outras flags de aviso pro master sobre o que está acontecendo

	static uint8_t commandCode = 0;  //variável estática dentro de uma função, ela não será destruída e manterá os valores atribuidos ao longo do tempo. Essa variável estará na memória ROM do programa
	static  uint8_t Cnt = 0;         //elas serão como variáveis globais mas não podem ser acessadas fora dessa função (this->function)

	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants some data. slave has to send it
		if(commandCode == 0x51)
		{
			//send the length information to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx,strlen((char*)Tx_buf));
		}else if (commandCode == 0x52)
		{
			//Send the contents of Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx,Tx_buf[Cnt++]); //para cada solicitação do master o disciple manda um byte por solicitação

		}
	}else if (AppEv == I2C_EV_DATA_RCV)
	{
		//Data is waiting for the slave to read . slave has to read it
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}else if (AppEv == I2C_ERROR_AF)
	{
		//This happens only during slave txing .
		//Master has sent the NACK. so slave should understand that master doesnt need
		//more data.
		commandCode = 0xff; //como o master não quer mais dados então mandará um NACK, assim podemos resetar essas flags e o contador dpo buffer de transmissão
		Cnt = 0;
	}
	else if (AppEv == I2C_EV_STOP)
	{
		//This happens only during slave reception .
		//Master has ended the I2C communication with the slave.
	}
}
