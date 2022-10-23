/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: 10 de set de 2022
 *      Author: Matteus
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

#define MY_ADDR 0x61;   //este é um endereço de disciple permitido, mas devemos sempre verificar no reference manual ou no datasheet quais as restrições em relação a address

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";
/*
 * PB6-> SCL
 * PB9 or PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	/*Note : Internal pull-up resistors are used */
	/*
	 * Esta função apenas faz os pinos se comportarem como I2C, mas ainda não é a configuração do I2C
	 *
	 * Aqui estamos configurando o pino PB6 para funcionar como altfunction I2C1 - SCLK (ver o alt function map no datasheet/reference manual).
	 * Também estamos configurando o PB9 para funcionar como I2C1 - SDA
	 */
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;            //PRECISAMOS SETAR O I2C COMO OPEN DRAIN PARA HABILITAR O PULL UP INTERNO (OU EXTERNO), A FIM DE RESPEITAR AS CONDIÇÕES DE TRISE E BUS CAPACITANCE EXISTENTES
	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	 */
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;                  //O PULL UP INTERNO CALCULADO PELAS EQUAÇÕES FORNECIDAS DADO BUS CAPACITANCE DE 400nF e trise = 1000ns, Vcc = 3v3 e Vgnd = 0V nos fornecem um pullup resistence que obedece o intervalo 2.9k ohms <= R <= 3.5k ohms, portanto podemos usar o pullup interno do MCU de 3k3 ohms
	I2CPins.GPIO_PinConfig.GPIO_AltFunMode = 4;                            //essa alt function significa I2C1
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;                   //como setamos esse GPIO como alternate function 4, a inicialização deste pino será como I2C1_SCLK
	GPIO_Init(&I2CPins);

	//sda
	//Note : since we found a glitch on PB9 , you can also try with PB7 - vendo no esquemático da placa discovery descobrimos que esse pino está conectado a outros circuitos que podem estar causando um mau funcionamento do PB9
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;                  //como setamos esse GPIO como alternate function 4, a inicialização deste pino será como I2C1_SDA
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

/*
 * A função abaixo é exatamente igual ao do SPI, apenas configura um pino para ser lido por um botão externo
 */
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

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//send some data to the slave
		I2C_MasterSendData(&I2C1Handle,some_data,strlen((char*)some_data),SLAVE_ADDR,0);
	}

}
