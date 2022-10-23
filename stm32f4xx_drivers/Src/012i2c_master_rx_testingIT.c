/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Feb 24, 2019
 *      Author: admin
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"
//#include "stm32f407xx_i2c_driver.h"

extern void initialise_monitor_handles();

//Flag variable
uint8_t rxComplt = RESET;

#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

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

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;

	PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

}


int main(void)
{

	uint8_t commandcode;

	uint8_t len;

	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//I2C IRQ configurations - interrupt enable
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1 (peripheral enable)
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(!ReadFromInputPin(GPIOA,GPIO_PIN_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = 0x51;


		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); //esse while significa que se tiver outra interrupção sendo executada então o I2C
		                                                                                               //vai ficar insistindo em tentar mandar a mensagem até conseguir. Quando conseguir
		                                                                                               //ele irá sair desse loop.
																									   //Aqui mandamos o commandcode 0x51 para o slave com endereço SLAVE_ADDR,
		                                                                                               //o comando será detectado, lido e processado pelo slave que detectará e enviará primeiro
		                                                                                               //o número de bytes que devemos receber no comando de recepção (comando seguinte)


		while(I2C_MasterReceiveDataIT(&I2C1Handle,&len,1,SLAVE_ADDR,I2C_ENABLE_SR)!= I2C_READY);       //Aqui o slave já detectou o send anterior, viu que era o comando com byte 0x51 então
		                                                                                               //sabe que tem que retornar o tamanho da string que colocará no buffer de envio (SR)

		commandcode = 0x52;                                                                            //com esse comando o slave saberá que deve enviar a string em si
		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY); //aqui o master insistirá até conseguir enviar o comando


		while(I2C_MasterReceiveDataIT(&I2C1Handle,rcv_buf,len,SLAVE_ADDR,I2C_DISABLE_SR)!= I2C_READY); //aqui recebemos a string com tamanho len obtido com o comando anterior

		rxComplt = RESET;                         //INFORMAÇÃO MUITO IMPORTANTE: A FUNÇÃO RECEIVE ANTERIOR APENAS HABILITA A INTERRUPÇÃO POR I2C E RETORNA, MAS DEVEMOS LEMBRAR QUE O I2C É LENTO,
												  //MUITO MAIS LENTO QUE O PROCESSADOR, ENTÃO ASSIM QUE A FUNÇÃO RECEIVE ANTERIOR É CHAMADA E MESMO ESPERANDO ATÉ CONSEGUIR HABILITAR A
											      //INTERRUPÇÃO E CONSEGUIR SAIR DO WHILE, O COMANDO rxComplt = RESET SERÁ EXECUTADO MUITO ANTES DA PRÓPRIA INTERRUPÇÃO POR I2C QUE FOI
		                                          //HABILITADA DENTRO DA RECEIVEIT. POR ISSO QUE ESSE rxComplt = RESET FAZ UM RESET NESTA VARIÁVEL E QUANDO O I2C ACABAR DE RECEBER A MENSAGEM
		                                          //SETARÁ ESSA VARIÁVEL DE NOVO E O FLUXO ESCAPARÁ DO WHILE (rxComplt != SET)
		//wait till rx completes
        while(rxComplt != SET)
        {

        }

		rcv_buf[len+1] = '\0';

		printf("Data : %s",rcv_buf);

		rxComplt = RESET;

	}

}


void I2C1_EV_IRQHandler (void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler (void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv) //Função para notificar a aplicação o que está acontecendo dentro da interrupção, tanto que ela é chamada apenas na IRQ EV handler e IRQ ER Handler
{                                                                         //Se o STM32 for um disciple então é aqui que enviaremos os ACKs e outras flags de aviso pro master sobre o que está acontecendo
     if(AppEv == I2C_EV_TX_CMPLT)
     {
    	 printf("Tx is completed\n");
     }else if (AppEv == I2C_EV_RX_CMPLT)
     {
    	 printf("Rx is completed\n");
    	 rxComplt = SET;
     }else if (AppEv == I2C_ERROR_AF)
     {
    	 printf("Error : Ack failure\n");
    	 //in master ack failure happens when slave fails to send ack for the byte
    	 //sent from the master.
    	 I2C_CloseSendData(pI2CHandle);

    	 //generate the stop condition to release the bus
    	 I2C_GenerateStopCondition(I2C1);

    	 //If there is an error hang in infinite loop
    	 while(1);
     }
}

