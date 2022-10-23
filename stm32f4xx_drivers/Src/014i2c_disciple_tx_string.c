
/*
 * 014i2c_disciple_tx_string.c
 *
 *  Created on: Feb 24, 2019
 *      Author: admin
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

//extern void initialise_monitor_handles();



#define MY_ADDR 0x68;
 uint32_t data_len=0;
#define SLAVE_ADDR  0x68

 //very large message
uint8_t Tx_buf[] = "HiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHi...123";

uint8_t CommandCode;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];

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
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
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
	GPIO_Handle_t GPIOBtn;

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
	//initialise_monitor_handles();

	//printf("Application is running\n");

	 data_len = strlen((char*)Tx_buf);

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE); //para aplcações em que o STM32 discovery é disciple devemos setar as configurações de interrupção, lembramos que um disciple sempre está com interrupções ativadas
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE); //acima configuramos os tipos de interrupção que queriamos (numero de IRQ handler), agora temos os enables habilitados

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

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{


	static uint32_t cnt = 0;
	static uint32_t w_ptr = 0;



	if(AppEv == I2C_ERROR_AF)
	{
		//This will happen during slave transmitting data to master .
		// slave should understand master needs no more data
		//slave concludes end of Tx


		//if the current active code is 0x52 then dont invalidate
		if(! (CommandCode == 0x52))
			CommandCode = 0XFF;

		//reset the cnt variable because its end of transmission
		cnt = 0;

		//Slave concludes it sent all the bytes when w_ptr reaches data_len
		if(w_ptr >= (data_len))
		{
			w_ptr=0;
			CommandCode = 0xff;
		}

	}else if (AppEv == I2C_EV_STOP)
	{
		//This will happen during end slave reception
		//slave concludes end of Rx

		cnt = 0;

	}else if (AppEv == I2C_EV_DATA_REQ)
	{
		//Master is requesting for the data . send data
		if(CommandCode == 0x51)
		{
			//Here we are sending 4 bytes of length information
			I2C_SlaveSendData(I2C1,((data_len >> ((cnt%4) * 8)) & 0xFF));
		    cnt++;
		}else if (CommandCode == 0x52)
		{
			//sending Tx_buf contents indexed by w_ptr variable
			I2C_SlaveSendData(I2C1,Tx_buf[w_ptr++]);
		}
	}else if (AppEv == I2C_EV_DATA_RCV)
	{
		//Master has sent command code, read it
		 CommandCode = I2C_SlaveReceiveData(I2C1);

	}
}

