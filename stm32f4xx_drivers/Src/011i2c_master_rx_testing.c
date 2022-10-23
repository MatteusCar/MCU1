/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: 11 de set de 2022
 *      Author: Matteus
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

//extern void initialise_monitor_handles();

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

	uint8_t commandcode;

	uint8_t len;

	//initialise_monitor_handles();

	//printf("Application is running\n");

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral - aqui também se faz PE=1, sem ele o ACK não é colocado em ENABLE
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1 - apenas aqui podemos colocar ACK em high porque aqui já temos o PE = 1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(!ReadFromInputPin(GPIOA,GPIO_PIN_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = 0x51;

		/*
		 * LMENBRANDO QUE FAZEMOS A GERAÇÃO DO STOP APENAS NA ULTIMA CHAMADA DA FUNÇÃO MASTER RECEIVE, ISSO PORQUE SE GERARMOS O STOP A CADA
		 * SEND OU RECEIVE QUALQUER (E NAO NA ULTIMA CHAMADA) ISSO ABRE MARGEM PRA ALGUM OUTRO DISPOSITIVO MASTER SOLICITAR O BUS I2C PREJUDICANDO
		 * A COMPLETA AQUISIÇÃO DE DADOS SOLICITADA PELO PRIMEIRO MASTER, ENTÃO O QUE FAZEMOS É SÓ ACIONAR O STOP QUANDO TODA A MENSAGEM ESTIVER
		 * SIDO RECEBIDA (OU ENVIADA) EM CADA "RODADA" DE COMUNICAÇÃO SPI
		 */

		I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);         //aqui mandamos o commandcode 0x51 para o slave com endereço SLAVE_ADDR, o comando será lido e processado pelo slave que enviará primeiro o número de bytes que devemos receber no comando de recepção

		I2C_MasterReceiveData(&I2C1Handle,&len,1,SLAVE_ADDR,I2C_ENABLE_SR);              //aqui o "buffer de recepção" e o próprio &len, então o retorno desta função estará preenchido com o len fornecido pelo slave

		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);


		I2C_MasterReceiveData(&I2C1Handle,rcv_buf,len,SLAVE_ADDR,I2C_DISABLE_SR);        //aqui neste receive além da recepção dos dados nós geramos a condição de stop para finalização da comunicação I2C

		rcv_buf[len+1] = '\0';  //para usar o printf com %s devemos atribuir um caractere nulo no fim do receive buffer

		//printf("Data : %s",rcv_buf);

	}
}
