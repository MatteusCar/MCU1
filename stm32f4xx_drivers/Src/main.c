/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stm32f407xx.h>
#include <stdio.h>
#include <string.h>

void delay()
{
	for(uint32_t i=0; i<500000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	//copia o valor do meio em todas as entradas do espaço de memória passado, no caso, o GpioLed.
	//É uma boa prática fazer isso, porque a variável inicia sem lixo da memória
	memset(&GpioLed, 0, sizeof(GpioLed));

	GpioLed.pGPIOx = GPIOD;                                      //GPIOD
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;         //GPIOD12
	GpioLed.GPIO_PinConfig.GPIO_PinMode =   GPIO_MODE_OUT;       //OUTPUT
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;      //Fast Transition between Low and High
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;     //Output with Push Pull
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;  //Without PullUp or PullDown

	PeriClockControl(GPIOD, ENABLE); //faz um enable do clock daquele GPIO (basicamente um enable do barramento de clock (RCC) desse GPIO)
	GPIO_Init(&GpioLed);

	while(1){
		ToggleOutputPin(GPIOD, GPIO_PIN_12);
		delay();
	}
    return 0;
}

//Se a interrupção foi confugurada corretamente para o EXTIO0 o processador irá localizar a
//função abaixo
void EXTI0_IRQHandler(void)
{
	//Lembrar que as IRQHandler não aceitam nem retornam nenhum parâmetro, elas (como o próprio nome já diz) apenas lidam com a respectiva interrupção
	//Handle the interruption - Primeiramente a função abaixo irá limpar os registradores de interrupção
	//para ela não ficar infinitamente ativa (o que travaria o código)
	//A passagem do parâmetro abaixo é o pino ao qual se espera a interrupção
	GPIO_IRQHandler(0);

	//A partir daqui escreva o código que rodará assim que a interrupção for detectada
}








