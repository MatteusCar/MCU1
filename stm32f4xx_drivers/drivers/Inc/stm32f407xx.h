/*
 * stm32f407xx.h
 *
 *  Created on: Jan 15, 2022
 *      Author: matte
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Regra geral (grosso modo): definir os base adress desde a memoria flash, RAM, ROM etc, definir os base adress dos barramentos (APBx, AHBx), definir os base adress dos periféricos a partir dos base adress dos barramentos
 * definir as structs para facilitar o acessi aos registradores, definir clock enables usando os registradores dos barramentos para habilitar ou desabilitar os periféricos. Depois é só construir as structs que lidam com
 * as configurações e handle do periférico (periph_Config_t e periph_Handle_t), fazer as funções que lidam com definição e config e pronto (fazendo macros para facilitar as opções disponíveis), está feito o driver, depois
 * é só prestar atenção nas questões de interrupção
 *
 */

/*
 * Macros específicas deste processador. Interrupt Service Registers para acessarmos os registradores de enable e clear de
 * interrupção
 */

#define NO_PR_BITS_IMPLEMENTED  4

/*
 * ARM Cortex Mx Processor NVIC ISERx (Interrupt Set Enable Register) Address
 */
#define NVIC_ISER0              ( (volatile uint32_t*)0xE000E100  )
#define NVIC_ISER1              ( (volatile uint32_t*)0xE000E104  )
#define NVIC_ISER2              ( (volatile uint32_t*)0xE000E108  )
#define NVIC_ISER3              ( (volatile uint32_t*)0xE000E10C  ) //há mais base Address destes registradores,
																	//mas vamos utilizar poucas interrupções para prototipagem

/*
 * ARM Cortex Mx Processor NVIC ICERx (Interrupt Clear Register) Address
 */
#define NVIC_ICER0              ( (volatile uint32_t*)0XE000E180  )
#define NVIC_ICER1              ( (volatile uint32_t*)0XE000E184  )
#define NVIC_ICER2              ( (volatile uint32_t*)0XE000E188  )
#define NVIC_ICER3              ( (volatile uint32_t*)0XE000E18C  ) //há mais base Address destes registradores,
																	//mas vamos utilizar poucas interrupções para prototipagem

/*
 * ARM Cortex Mx Priority Registers para alocar a IRQ no Stack de interrupções de acordo com sua prioridade (IRQPriority)
 */
#define NVIC_PR_BASE_ADDR       ( (volatile uint32_t*)0xE000E400 )

/*
*Base Address of memory
*/
#define	FLASH_BASEADDR			0x08000000
#define SRAM1_BASEADDR			0x20000000
#define SRAM2_BASEADDR			0x2001C000
#define ROM_BASEADDR			0x1FFF0000
#define SRAM					SRAM1_BASEADDR

/*
*Bus Base Address for peripherals
*/
#define PERIPH_BASE			    0x40000000
#define APB1PERIPH_BASEADDR		PERIPH_BASE
#define APB2PERIPH_BASEADDR		0x40010000
#define AHB1PERIPH_BASEADDR		0x40020000
#define AHB2PERIPH_BASEADDR		0x50000000

/*
*Base Address for GPIO
*/
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)


//Podemos escolher via configurações do RCC no firmware se queremos o HSE (high speed external clock), HSI(High speed internal clock) ou PLL para multiplicação
//de frequencia. No caso da discovery board ela já tem um cristal externo que fornece clock de 16MHz para o microcontrolador, e como por default o RCC já vem
//pre configurado para clock externo então não precisamos nos preocupar com isso na programação dos drivers

#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800)
/*
*Base Address for APB1 BUS peripherals
*/
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

/*
*Base Address for APB2 BUS peripherals
*/
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

typedef struct  //estrutura genérica para uma GPIO qualquer, o acesso à GPIO específica se dará pelo Base Addr a seguir
{

	volatile uint32_t MODER;          //por padrao o compilador colocará cada variável da struct em locais distanciados de 0x04 em sequencia
	volatile uint32_t OTYPER;		 //lembrar que cada uint32_t é um registrador de 32 bits distanciados de 0x04
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

} GPIO_RegDef_t;

#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR) //estrutura cujo primeiro endereço é o próprio GPIOA_BASEADDR
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI	((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ	((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK	((GPIO_RegDef_t*)GPIOK_BASEADDR)

/*GPIO_RegDef_t *pGPIOA = GPIOA;     //ponteiro para o Base Address do GPIOA para acesso rapido
                         		     //exemplo: pGPIOA->MODER = 25 já acessa o GPIOA direto, o que é equivalente a fazer: *(0x40020000 + 0x00) = 25
								     //Lembrando que a essa altura do campeonato ja devemos ter os base Address de todos os GPIOx
								     //para podermos usar a macro GPIOA, que no caso é o pimeiro periférico GPIOx ou seja, GPIOA == 0x40020000*/
									 //Lembrando que como estamos numa arquitetura de 32 bits (4 bytes) e cada endereço da memória é um índice pra um
                                     //array de 32 bits, então os endereços deverão pular de 4 em 4 bytes para arrays contíguos
                                     //exemplo 0x40020000 é o endereço do array 0000 0000 0000 0000 0000 0000 0000 0000*/
                                     //então   0x40020004 será o endereço do próximo array (já que o array anterior tem 4 bytes, ou seja, 32 bits)
/*Structure for RCC peripheral
 * É AQUI QUE TODOS OS CLOCKS DOS BARRAMENTOS DE DADOS SÃO HABILITADOS OU DESABILITADOS, ESTA ESTRUTURA É ESSENCIAL PARA O FUNCIONAMENTO DE QUALQUER
 * PERIFÉRICO E ESSA ABSTRAÇÃO SERVE PARA A MAIORIA DOS MICROCONTROLADORES: PRIMEIRO FAZEMOS A ESTRUTURA QUE HABILITA OU DESABILITA OS CLOCKS DOS BARRAMENTOS
 * DE CADA PERIFÉRICO (gpio, timerS, adcs etc) E DEPOIS FAZEMOS UMA ESTRUTURA QUE CHAMARÁ CADA UM E JÁ ABARCARÁ SEU MAPA DE REGISTRADORES para definição e
 * instanciação daquele periférico (RegDef)
*/

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
} RCC_RegDef_t;

#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR) //Inicialização de uma variável do tipo struct cujo primeiro endereço é o RCC_BASEADDR, os endereços após isso seguem o
                                             //o padrão acima na struct do tipo RCC_RegDef_t

#define EXTI   ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SPI1   ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2   ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3   ((SPI_RegDef_t*)SPI3_BASEADDR)

typedef struct  //estrutura genérica para uma GPIO qualquer, o acesso à GPIO específica se dará pelo Base Addr a seguir
{

	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

} EXTI_RegDef_t;

/*
 * SYS_CFG Struct Registers
 */

typedef struct
{

	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t          RESERVED[2];           //Gap between 0x14 to 0x18 + 0x1C to 0x20 (4 + 4 bytes)
	volatile uint32_t CMPCR;

} SYSCFG_RegDef_t;

#define SYSCFG      ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;

}SPI_RegDef_t;

typedef struct{

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;

}I2C_RegDef_t;

#define I2C1   ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2   ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3   ((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * GPIOx Peripheral Clock Enable
 */
#define GPIOA_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLOCK_EN()    (RCC->AHB1ENR |= (1 << 10))

/*
 * I2Cx Peripheral Clock Enable
 */
#define I2C1_PCLOCK_EN()     (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLOCK_EN()     (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLOCK_EN()     (RCC->APB1ENR |= (1 << 23))

/*
 * UARTx Peripheral Clock Enable
 */
#define UART4_PCLOCK_EN()    (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLOCK_EN()    (RCC->APB1ENR |= (1 << 20))
#define UART7_PCLOCK_EN()    (RCC->APB1ENR |= (1 << 30))
#define UART8_PCLOCK_EN()    (RCC->APB1ENR |= (1 << 31))

/*
 * SPI Peripheral Clock Enable
 */
#define SPI1_PCLOCK_EN()	 (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLOCK_EN()     (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLOCK_EN()     (RCC->APB1ENR |= (1 << 15))

/*
 * SYSCFG Peripheral Clock Enable
 */
#define SYSCFG_PCLOCK_EN()	 (RCC->APB2ENR |= (1 << 14))

/*
 * USARTx Peripheral Clock
 */
#define USART1_PCLOCK_EN()   (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLOCK_EN()   (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLOCK_EN()   (RCC->APB1ENR |= (1 << 18))
#define USART6_PCLOCK_EN()   (RCC->APB2ENR |= (1 << 5))

//---------------------------------------------------------------------------------------------

/*
 * GPIOx Peripheral Clock Disable
 */
#define GPIOA_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLOCK_DI()    (RCC->AHB1ENR &= ~(1 << 10))

/*
 * I2Cx Peripheral Clock Disable
 */
#define I2C1_PCLOCK_DI()     (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLOCK_DI()     (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLOCK_DI()     (RCC->APB1ENR &= ~(1 << 23))

/*
 * UARTx Peripheral Clock Disable
 */
#define UART4_PCLOCK_DI()    (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLOCK_DI()    (RCC->APB1ENR &= ~(1 << 20))
#define UART7_PCLOCK_DI()    (RCC->APB1ENR &= ~(1 << 30))
#define UART8_PCLOCK_DI()    (RCC->APB1ENR &= ~(1 << 31))

/*
 * SPI Peripheral Clock Disable
 */
#define SPI1_PCLOCK_DI()	 (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLOCK_DI()     (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLOCK_DI()     (RCC->APB1ENR &= ~(1 << 15))

/*
 * SYSCFG Peripheral Clock Disable
 */
#define SYSCFG_PCLOCK_DI()	 (RCC->APB2ENR &= ~(1 << 14))

/*
 * USARTx Peripheral Clock Disable
 */
#define USART1_PCLOCK_DI()   (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLOCK_DI()   (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLOCK_DI()   (RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLOCK_DI()   (RCC->APB2ENR &= ~(1 << 5))

/*
 * Defining Macros for Reset GPIOx
 */
#define GPIOA_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0)); }  while(0)   //do-while zero condition loop define
#define GPIOB_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1)); }  while(0)
#define GPIOC_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2)); }  while(0)
#define GPIOD_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3)); }  while(0)
#define GPIOE_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4)); }  while(0)
#define GPIOF_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 5));  (RCC->AHB1RSTR &= ~(1 << 5)); }  while(0)
#define GPIOG_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 6));  (RCC->AHB1RSTR &= ~(1 << 6)); }  while(0)
#define GPIOH_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7)); }  while(0)
#define GPIOI_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 8));  (RCC->AHB1RSTR &= ~(1 << 8)); }  while(0)
#define GPIOJ_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 9));  (RCC->AHB1RSTR &= ~(1 << 9)); }  while(0)
#define GPIOK_REG_RST()      do{ (RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~(1 << 10)); } while(0)

/*
 * Defining Macros for Reset GPIOx
 */
#define SPI1_REG_RST()      do{ (RCC->APB2RSTR |= (1 << 12));  (RCC->AHB1RSTR &= ~(1 << 12)); }  while(0)   //do-while zero condition loop define
#define SPI2_REG_RST()      do{ (RCC->APB1RSTR |= (1 << 14));  (RCC->AHB1RSTR &= ~(1 << 14)); }  while(0)
#define SPI3_REG_RST()      do{ (RCC->APB1RSTR |= (1 << 15));  (RCC->AHB1RSTR &= ~(1 << 15)); }  while(0)

#define I2C1_REG_RST()      do{ (RCC->APB2RSTR |= (1 << 21));  (RCC->AHB1RSTR &= ~(1 << 21)); }  while(0)   //do-while zero condition loop define
#define I2C2_REG_RST()      do{ (RCC->APB1RSTR |= (1 << 22));  (RCC->AHB1RSTR &= ~(1 << 22)); }  while(0)
#define I2C3_REG_RST()      do{ (RCC->APB1RSTR |= (1 << 23));  (RCC->AHB1RSTR &= ~(1 << 23)); }  while(0)

/*
 * Se a condição for verdadeira retorna zero, caso contrario cheque a próxima condição, barra invertida
 * serve para checar a próxima condição
 */
#define GPIO_BASEADDR_TO_CODE(x)    ((x == GPIOA)? 0  : \
                                    (x == GPIOB) ? 1  : \
                                    (x == GPIOC) ? 2  : \
                                    (x == GPIOD) ? 3  : \
                                    (x == GPIOE) ? 4  : \
                                    (x == GPIOF) ? 5  : \
          						    (x == GPIOG) ? 6  : \
                                   	(x == GPIOH) ? 7  : \
                                    (x == GPIOI) ? 8  : \
                                    (x == GPIOJ) ? 9  : \
                                    (x == GPIOK) ? 10 : 0)

/*
 * IRQ Macros for External Interrupt (EXTIx), ou seja, são interrupções via GPIO (EXTI é um edge detector)
 * Sendo assim, sempre que eu quiser habilitar uma interrupção externa, eu vou dizer qual o periférico a que ele está associado
 * IRQ number que o fará habilitar a detecção de borda de subida ou descida
 */

#define IRQ_NO_EXTI0    	6  //IRQ Numbers das trilhas de interrupção,
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3	    9
#define IRQ_NO_EXTI4	    10
#define IRQ_NO_EXTI9_5      23

#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4         84
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

/*
 * Macros para Prioridade de Interrupções
 */

//#define NVIC_PRI_NO_15   15

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15

/*
 * Generic Macros
 */

//Macros para GPIO
#define ENABLE         1
#define DISABLE        0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET     RESET
#define FLAG_SET       SET


//Macros para SPI CR1
#define SPI_CR1_CPHA       0
#define SPI_CR1_CPOL       1
#define SPI_CR1_MODE       2
#define SPI_CR1_BR         3
#define SPI_CR1_SPE        6
#define SPI_CR1_LSB        7
#define SPI_CR1_SSI        8
#define SPI_CR1_SSM        9
#define SPI_CR1_RXONLY     10
#define SPI_CR1_DFF	       11
#define SPI_CR1_CRC_NEXT   12
#define SPI_CR1_CRC_EN     13
#define SPI_CR1_BIDI_OE    14
#define SPI_CR1_BIDI_MODE  15

//Macros for SPI CR2
#define SPI_CR2_RXDMAEN    0
#define SPI_CR2_TXDMAEN    1
#define SPI_CR2_SSOE       2
#define SPI_CR2_FRF        4
#define SPI_CR2_ERRIE      5
#define SPI_CR2_RXNEIE     6
#define SPI_CR2_TXEIE      7

//Macros for SPI SR
#define SPI_SR_RXNE        0
#define SPI_SR_TXE         1
#define SPI_SR_CHSIDE      2
#define SPI_SR_UDR         3
#define SPI_SR_CRCERR      4
#define SPI_SR_MODF        5
#define SPI_SR_OVR         6
#define SPI_SR_BSY         7
#define SPI_SR_FRE         8

/******************************************************************************************
 *Bit position definitions of I2C peripheral - Para configuiração dos registradores I2C
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1 - bits do status register 1 para monitorarmos
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

#include <stm32f407xx_gpio_driver.h>
#include <stm32f407xx_spi_driver.h>
#include <stm32f407xx_i2c_driver.h>
#endif /* INC_STM32F407XX_H_ */
