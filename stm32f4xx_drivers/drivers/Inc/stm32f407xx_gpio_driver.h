/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jan 17, 2022
 *      Author: matte
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*Lembrando que cada GPIO desse microcontrolador tem 16 pinos (devido ao seu design)

  Por exemplo GPIOA0...GPIOA15 (cada um de 1 bit obviamente)
              GPIOB0...GPIOB15
              etc
  Como essa placa STM32F407VG só tem os ports A, B, C, D, E externalizados do processador ela totaliza 80 pinos de GPIO externalizados*/

/*PinMode: controla  a natureza do GPIO: Input (sendo simples input ou interrupção
 	 	 	 	 	 	 	 	 	 	Output
 	 	 	 	 	 	 	 	 	 	Alternate Function (SPI, I2C, CAN, UART_TX, UART_RX, Timer, ADC, DAC, RTC etc)
 	 	 	 	 	 	 	 	 	 	Analog (para mexer com DAC ou ADC)

Sumario

PinMode = INPUT:

(1) O buffer de output é desabilitado
(2) O Schmitt Trigger de entrada é habilitade para evitar leituras espúrias ou problema de boucing ou ringing
(3) Os resistores de pullUp e PulDown estão desabilitados por default, para controla-los devemos controlar o registrador GPIOx_PUPDR
(4) O dado de entrada (seja 0 ou 1) no pino de entrada será amostrado para o buffer de dados a cada um ciclo de clock do Bus conectado ao GPIO (no caso desse uC o bus é o AHB1)

PinMode = OUTPUT:

(1) Dois modos geralmente: a saída é dada por uma dupla de transistores na topologia push-pull (primeira opção de modo) ou open Drain (segundaopção de modo),
entretanto é recomendado o modo Push-Pull já que o open drain precisa de um pullUp (interno ou externo dependendo do que se quer acionar, o pullUp interno é
mais limitado pois seu valor é fixo), caso contrário você não iria conseguir escrever o bit 1.

Outra coisa é que mesmo na configuração Push-Pull podemos colocar algum resistor de pullUp ou PullDown para estabelecer um valor default no estado de output
e não deixar o output flutuando (recomendado pullDown)
 --------------------------------------------------------------------------------------------------------------------------------------------

 PinPuPd: controla os resistores de PullUp e PullDown do pino. Eles são inicializados (default) em modo flutuante, mas é recomendável que se ative em PullUp ou
 PullDown para não termos current leakage nos transistores push-pull na entrada devido à ruído espúrio, que podem estar na região de transição entre High e Low
 na entrada do Push-Pull e fazer com que ambos estejam ativos, gerando um caminho do VCC pro terra e assim consumindo potência.
 Outra vantagem é acionar um deles no modo input para que nenhum ruído espúrio seja interpretado como zeros e uns aleatoriamente.

 Caso queira colocar um resistor de pullUp/pullDown externo é só procurar no datasheet do microcontrolador ou do processador quais os valores

-----------------------------------------------------------------------------------------------------------------------------------------------

 PinSpeed (OSPEEDRy): Você praticamente controla o slew rate daquele GPIO, ou seja, a velocidade em que ele transista de 0 pra 1 (L pra H), ou ainda em outras palavras,
 o tempo de subid e o tempo de descida.

 Os registradores deste microcontrolador nos fornecem 2 bits de opção

 00 - Low Speed
 01 - Medium Speed
 10 - High Speed
 11 - Ultra High Speed


*/
typedef struct{
	uint8_t GPIO_PinNumber;					 //@POSSIBLE_PIN_NUMBERS
	uint8_t GPIO_PinMode;                    //@POSSIBLE_MODES
	uint8_t GPIO_PinSpeed;					 //@POSSIBLE_SPEEDS
	uint8_t GPIO_PinPuPdControl;             //@POSSIBLE_PULLUP_PULLDOWN
	uint8_t GPIO_PinOPType;                  //@POSSIBLE_OUTPUT_TYPES
	uint8_t GPIO_AltFunMode;
}GPIO_PinConfig_t;

typedef struct                                //em GPIO_PinConfig o usuario vai setar as informações desejadas que serão usadas pelo nosso driver para configurar os periféricos usando a estrutura pGPIOx, que já tem os Base Addrs definidos
{
	GPIO_RegDef_t *pGPIOx;                    //ponteiro para acesso rápido à GPIOx em questão, ex: pGPIOx = GPIOA
	GPIO_PinConfig_t GPIO_PinConfig;          //faz pGPOIx apontar para o endereço do GPIOA (seu base Address)
						                      //então pGPIOx -> MODER = 25 já acessa direto o registrador MODER da GPIOA
}GPIO_Handle_t;

/*
 * @POSSIBLE_PIN_NUMBERS
 */
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15

/*
 * GPIO Modes @POSSIBLE_MODES
 */
#define GPIO_MODE_IN       0  //Input Mode
#define GPIO_MODE_OUT      1  //Output Mode
#define GPIO_MODE_ALTFN    2  //Alternate Function (Depending on the specific Pin, see datasheet)
#define GPIO_MODE_ANALOG   3  //Analog Mode
#define GPIO_MODE_IT_FT    4  //Interrupt input falling edge trigger
#define GPIO_MODE_IT_RT    5  //Interrupt input rising  edge trigger
#define GPIO_MODE_IT_RFT      6  //...rising falling edge trigger

/*
 * GPIO Output Types like (Push Pull or Open Drain) @POSSIBLE_OUTPUT_TYPES
 */
#define GPIO_OP_TYPE_PP	0
#define GPIO_OP_TYPE_OD	1

/*
 * GPIO possible speeds @POSSIBLE_SPEEDS
 */
#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_FAST    2
#define GPIO_SPEED_HIGH    3

/*
 * GPIO possible pull up or pull down mode @POSSIBLE_PULLUP_PULLDOWN
 */
#define GPIO_NO_PU_PD  0
#define GPIO_PU        1
#define GPIO_PD        2

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);        //O usuario vai instanciar uma struct do tipo GPIO_Handle_t e vai configurar e passar como parametro para inicialização
void deInit(GPIO_RegDef_t *pGPIOx);

/*
 * Peripheral Clock Setup
 */
void PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En);                        //dado um base address de uma GPIO a gente habilita ou desabilita seu clock
														                         //En = 1 habilita, En = 0 desabilita
/*
 * Read and Write
 */
uint8_t ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin);                    //primeiro parametro diz qual GPIOx, e o segundo diz qual pino desse GPIOx
uint16_t ReadFromInputPort(GPIO_RegDef_t *pGPIOx);                               //aqui vai retornar 16 bits ou seja, o valor de cada porta da GPIOx
void WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t value);       //value = 0 ou 1
void WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin);

/*
 * Interrupt Service Routine (ISR) Handling and Interrupt Routine Queue Configuration
 */
void GPIO_IRQInterruptConfig(uint8_t IRQnumber, uint8_t  IRQEn);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandler(uint8_t pin);                                               //Interrupção externa ativda por esse pino será gerenciada


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
