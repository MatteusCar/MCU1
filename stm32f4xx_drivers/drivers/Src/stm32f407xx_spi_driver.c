/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 14 de ago de 2022
 *      Author: Matteus
 */

#include <stdint.h>
#include "stm32f407xx_spi_driver.h"

static void txe_spi_interrupt_handler(SPI_Handle_t *pHandle);  //aqui o static é só para dizer que o usuário não vai ter acesso a essas funções, elas são de uso exclusivo do driver
static void rxne_spi_interrupt_handler(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pHandle);

/*
 * Ponto importante sobre a comunicação SPI, lembrar que os únicos registrasdores que temos acesso para controlar e modificar são
 * o dataRegister (DR), Control Register 1 e 2 (CR1 e CR2), para enviar ou receber bytes e habilitar ou desabilitar as interrupções. Já o
 * Status Register é um registrador que é modificado internamente por hardware (pelo próprio protocolo SPI), ou seja, podemos apenas monitorar
 * as flags internas dele e tomar atitudes de acordo com as informações que ele fornece, mas para controlar mesmo a comunicação SPI temos apenas
 * DR, CR1 e CR2
 *
 * Detalhe: Para mexer em qualquer periférico disciple e trocar mensagens via SPI, devemos usar uma GPIO para drivar o pino NSS do periférico para baixo e só aí transferir dados
 *
 * ############################################################################################################################################################################
 *
 * 				    Sempre que for mexer com SPI tomar cuidado com as seguintes coisas:
 *
 * 					- Master mode bit must be enabled in the configuration register if you are configuring the peripheral as master
					- SPI peripheral enable bit must be enabled
 					- SPI peripheral clock must be enabled. By default clocks to almost all
					- peripheral in the microcontroller will be disabled to save power.

 * ##############################################################################################################################################################################
 *
 *                  Problemas comuns na comunicação SPI
 *
 *                  Case 1: Master cannot able to produce clock and data

				################################################################################################
                #   Reason-1: Non-Proper Configuration of I/O lines for Alternate functionality                #
                #                                                                                              #
				#	Debug Tip: Re-check the GPIO Configuration registers to see what values they have          #
				################################################################################################

				################################################################################################
				#	Reason-2: Configuration Overriding														   #
				#	Debug Tip:                                                                                 #
				#	Dump out all the required register contents just before you begin                          #
				#	the transmission                                                                           #
 *              ################################################################################################
 *
 *                  Case 2: Master is sending data, but slave is not receiving data !
 *
 *              ################################################################################################
				#	Reason-1: Not pulling down the slave select pin to ground before sending data to the slave #
				#	Debug Tip:                                                                                 #
				#	Probe through the logic analyzer to confirm slave select line is                           #
				#	really low during data communication                                                       #
                ################################################################################################

                ################################################################################################
                #   Reason-2 :																				   #
				#	Non-Proper Configuration of I/O lines for Alternate functionality                          #
				#	Debug Tip:                                                                                 #
				#	Probe the alternate function register                                                      #
                ################################################################################################

                ################################################################################################
				#	Reason-3 : Non enabling the peripheral IRQ number in the NVIC                              #
				#	Debug Tip:                                                                                 #
				#	Probe the NVIC Interrupt Mask register to see whether the bit                              #
				#	position corresponding to the IRQ number is set or not                                     #
                ################################################################################################

                    Case 3: SPI interrupts are not getting triggered

                ################################################################################################
				#	Reason-1 : Not enabling the TXE or RXNE interrupt in the SPI configuration register        #
				#	Debug Tip:																				   #
				#	Check the SPI configuration register to see TXEIE and RXNEIE bits                          #
				#	are really set to enable the interrupt !                                                   #
                ################################################################################################

                ################################################################################################
				#	Reason-2: Non enabling the peripheral IRQ number in the NVIC                               #
				#	Debug Tip:                                                                                 #
				#	Probe the NVIC Interrupt Mask Register to see whether the bit                              #
				#	position corresponding to the IRQ number is set or not                                     #
                ################################################################################################

                    Case 4: Master is producing right data but slave is receiving the different data

                ################################################################################################
				#	Reason-1 : Using Long Wires in high frequency communication                                #
				#	Debug Tip:                                                                                 #
				#	use shorter wires or reduce the SPI serial frequency to 500KHz to                          #
				#	check things work well                                                                     #
                ################################################################################################
*/
/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)   //O usuario vai instanciar uma struct do tipo SPI_Handle_t e vai configurar e passar como parametro para inicialização
{

	//Inicializando também o clock desse periférico para não esquecer
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Vamos primeiramente configurar os registradores do SPI dado o parâmetro recebido acima
	uint32_t temp = 0;

	// 1. Primeiramente começando pelo Modo
	temp |= (pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MODE);

	//2. Bus Config
	if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		temp &= ~(1 << 15); //bit resetado para habilitar full duplex mode
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		temp |= (1 << 15); //bit setado para habilitar half duplex mode
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX){
		temp &= ~(1 << 15); //bit resetado para habilitar full duplex mode
		temp |= (1 << 10); // simplex mode
	}

	//3. SPI Clock Speed
	temp |= (pSPIHandle->SPI_PinConfig.SPI_SCLKSpeed << SPI_CR1_BR);

	//4. SPI Data Frame Format - DFF
	temp |= (pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF);

	//5. SPI CPOL
	temp |= (pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. SPI CPHA
	temp |= (pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA);

	//SPI SSM
	temp |= (pSPIHandle->SPI_PinConfig.SPI_SSM << SPI_CR1_SSM);

	//Aqui transferimos a configuração para o periférico mapeado naquele SPI propriamente dito
	pSPIHandle->pSPIx->CR1 |= temp;
}
void SPI_deInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1){
		SPI1_REG_RST();
	}
	else if (pSPIx == SPI2){
		SPI2_REG_RST();
	}
	else if (pSPIx == SPI3){
		SPI3_REG_RST();
	}
}

//nesta função deveremos passar o periférico que queremos monitorar e qual a flag monitorada
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t nomeDaFlag){
	if(pSPIx->SR & nomeDaFlag){   //lembrando que a flag nomeDaFlag já é um bit deslocado de acordo com o próprio nome da flag que se quer monitorar, no caso nomeDaFlag == SPI_TXE_FLAG == (1 << SPI_SR_TXE) == (1 << 1)
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En)                        //dado um base address de um SPI a gente habilita ou desabilita seu clock
{														                           //En = 1 habilita, En = 0 desabilita
	if(En == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLOCK_EN();
		}
		else if (pSPIx == SPI2){
			SPI2_PCLOCK_EN();
		}
		else if (pSPIx == SPI3){
			SPI3_PCLOCK_EN();
		}
	}

	else{
		if(pSPIx == SPI1){
			SPI1_PCLOCK_DI();
		}
		else if (pSPIx == SPI2){
			SPI2_PCLOCK_DI();
		}
		else if (pSPIx == SPI3){
			SPI3_PCLOCK_DI();
		}
	}
}
/*
 * Para TX e RX temos basicamente três formas de comunicação
 * Polling - blocking based
 * Interrupção - non-blocking based
 * DMA
 */

/*
 * Como podemos ver abaixo na função send, ela fica bloqueada até o loop sair do segundo while, ou seja, até que a flag
 * do Status Register avise que o TX buffer está vazio para escrita. Isso significa que se houver algum mau funcionamento
 * de hardware ou mesmo de firmware que dê um set nessa flag o programa vai ficar parado nessa função, essa é uma das desvantagens
 * de se usar polling para send ou receive de mensagens.
 *
 * Um aspecto importante da função send é que ela apenas insere o dado no data register (DR), mas quem se encarrega de fazer o swap entre
 * os shift registers do SPI é o protocolo em nível de hardware, ou seja, não devemos nos preocupar, só saber como funciona e usar a função abaixo,
 * que espera o buffer de transmissão ficar em alto e depois insere o dado no DR
 */
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)   //o primeiro parametro é o endereço de onde está mapeado (pelo memoery map) aquele SPI em questão
{
	while(len>0){                                                      //enquanto len não for nulo (ainda não enviou toda a mensagem)

		//1. Wait until TXE is set
		//while(!(pSPIx->SR & (1 << SPI_SR_TXE)));                     //espere até a flag do Tx avisar que o TX buffer está vazio (pronto para enviar byte) para sair do loop
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);   //essa espera é o que caracteriza o polling do código

		//2. Format is 8 or 16 bits?
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){                           //verificar se é 1 byte por vez ou 2

			//16 bits format
			pSPIx->DR = *((uint16_t*)pTxBuffer);                         //colocar o dado no registrador de dados (DR) que vai enviar, note que primeiro fazemos um type cast de uint8_t para uint16_t. Depois pegamos o conteúdo dentro do ponteiro usando o operador de de-referenciação
			len--;
			len--;                                                     //duas vezes pois são dois bytes enviados
			(uint16_t*)pTxBuffer++;                                    //incrementar duas posições na memória, por isso o casting antes
			//pTxBuffer+=2;
		}
		else {

			//8 bits format
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;                                              //incrementar uma única posição
		}
	}
}

void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while(len>0){                                                      //enquanto len não for nulo (ainda não enviou toda a mensagem)

			//1. Wait until RXNE is set
			//while(!(pSPIx->SR & (1 << SPI_SR_TXE)));                      //espere até a flag do Tx avisar que o TX buffer está vazio (pronto para enviar byte) para sair do loop
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);   //essa espera é o que caracteriza o polling do código
																			//Quando RXNE for 0 sai do loop e continua na próxima linha

			//2. Format is 8 or 16 bits?
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){                            //verificar se é 1 byte por vez ou 2

				//Ler o dado no data register (DR)
				*((uint16_t*)pRxBuffer) = pSPIx->DR;                          //colocar o dado no registrador de dados (DR) que vai enviar, note que primeiro fazemos um type cast de uint8_t para uint16_t. Depois pegamos o conteúdo dentro do ponteiro usando o operador de de-referenciação
				len--;
				len--;                                                      //duas vezes pois são dois bytes enviados
				(uint16_t*)pRxBuffer++;                                     //incrementar duas posições na memória, por isso o casting antes
				//pTxBuffer+=2;
			}
			else {

				//8 bits format
				*pRxBuffer = pSPIx->DR;
				len--;
				pRxBuffer++;                                              //incrementar uma única posição
			}
		}
}


/*
 * Na função de interrupção por SPI abaixo passaremos como parâmetro o handler do SPI e depois colocaremos o pTxBuffer (já com a mensagem) e a largura
 * da mensagem enviada dentro da estrutura do SPI handle e depois habilitaremos a interrupção desse SPI para que a mensagem seja enviada,
 * entretanto, o envio efetivo da mensagem contida no pTxBuffer passado será feito no IRQ handler dessa interrupção e não neste sendIT
 *
 */
uint8_t SPI_SendIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR,
		//sempre que o hardware disser que o TXE (transmission buffer) está vazio, então pode interromper
		//é isso que o comando abaixo faz. TXEIE é o interrupt enable do SPI para transmissão e a interrupção ocorre
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	}
	return state;
}
uint8_t SPI_ReceiveIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable the RXEIE control bit to get interrupt whenever RXE flag is set in SR,
		//sempre que o hardware disser que o RXE (reception buffer) está vazio, então pode interromper
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
	}
	return state;
}

//Mesmo após as configurações do SPIx (1,2,3..) e da ativação dos seus clocks, o SPI ainda não está preparado para ser usado
//pois ainda existe uma flag a ser setada, que é justamente a flag de enable. Essa flag serve para que as configurações de clock e
//do SPI propriamente dito sejam alteradas apenas quando o periférico estiver desabilitado. A partir do momento que essa flag é habilitada
//não é possível mais fazer alterações na configuração do SPI, sendo que para configurar é necessário desabilitar mais uma vez essa flag e
//mexer nas configurações

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

//Função para habilitar o output NSS em high ou low, ele só habilita, quem controla quando vai ser high ou low é o SPE igual a 1 ou 0
void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}
		else{
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}

/*
 * Interrupt Routine Queue handlers
 */

void SPI_IRQInterruptConfig(uint8_t IRQnumber, uint8_t  IRQEn)
{
	//A função GPIO_Init irá configurar o SYSCFG (que seta o PORT (A,B...) que deverá gerar a interrupção
	//E o EXTIx que seta o modo de detecção dessa interrupção (Falling ou Rising Trigger)
	//Agora esta função GPIO_IRQConfig irá efetivamente gerar a interrupção do processador,
	//Usando os IRQ set-enable/Clear-enable Registers (ISR e ICR)

	if(IRQEn == ENABLE){ //enable
		if(IRQnumber <= 31){
			//programa o ISR0 (Interrupt Set-Enable Register 0) que habilita as interrupções de número 0 até 31.
			//habilitar a interrupção de número IRQnumber significa setar 1 no espaço de memória de índice IRQnumber
			//do registrador  NVIC_ISER0
			*NVIC_ISER0 |= (1 << IRQnumber);
		}
		else if(IRQnumber > 31 && IRQnumber < 64){ //entre 32 e 63
			//c.c programar o ISR1 (Interrupt Set-Enable Register 1) que habilita as interrupções de número 32 até 63
			uint8_t temp1 = IRQnumber % 32;       //precisamos disso pois os registradores vão até 32 e os IRQnumbers superam esse valor
			*NVIC_ISER1 |= (1 << temp1);
		}
		else if(IRQnumber >= 64 && IRQnumber < 96 ){
			uint8_t temp1 = IRQnumber % 64;
			//c.c programar o ISR2 (Interrupt Set-Enable Register 2) que habilita as interrupções de número 64 até 96
			*NVIC_ISER2 |= (1 << temp1);
		}
	}
	else{ //Disable
		if(IRQnumber <= 31){
			//programar o ICR0 (Interrupt Clear-Enable Register 0) que desabilita as interrupções de número 0 até 31
			*NVIC_ICER0 |= (1 << IRQnumber);
		}
		else if(IRQnumber > 31 && IRQnumber < 64){ //entre 32 e 63
			//programar o ICR1 (Interrupt Clear-Enable Register 1) que desabilita as interrupções de número 32 até 63
			*NVIC_ICER1 |= (1 << IRQnumber%32); //aqui não usarei a variável auxiliar temp1 para ser mais rápido
		}
		else if(IRQnumber >= 64 && IRQnumber < 96 ){
			//programar o ICR2 (Interrupt Clear-Enable Register 2) que desabilita as interrupções de número 64 até 93
			*NVIC_ICER2 |= (1 << IRQnumber%64);
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;                                              //para descobrirmos qual registrador IPR entre 0 - 59 cada um com 4 seções de 8 bits (Se IQRNumber == 23 então IQRNumber/4 == 5 então deveremos mexr no IPR5)
	uint8_t iprx_section = IRQNumber % 4;                                      //para descobrirmos qual a seção
	uint8_t shift_amount = 8*iprx_section + (8 - NO_PR_BITS_IMPLEMENTED);		 //Esta linha primeiramente faz o shift maior colocando os 8 bits na seção certa, depois faz mais um shift para à esquerda de 4 bits pois os primeiros 4 bits de cada seção não são utilizados
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);               //para colocarmos efetivamente o IRQPriority na seção de 8 bits

	/*Como NVIC_PR_BASE_ADDR foi declarado como uint32_t cada +1 que somamos no seu endereço pula 32bits (4 bytes), ou seja
	  iprx pode ser diretamente somando ao NVIC_PR_BASE_ADDR que já pulará 4bytes, fazendo com que seja desnecessário o
	  uso da multiplicação por 4 nesse cenário. Caso o NVIC_PR_BASE_ADDR fosse declarado como uint8_t então o iprx teria que ser multiplicado por 4 mesmo*/
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)                                               //Interrupção externa ativada por esse pino será gerenciada
{
	//Nesta função vamos verificar o motivo da interrupção ter sido feita
	//e tomar a atitude correspondente. Para verificar qual é o caso em questão, devemos
	//olhar para a flags do status register (SR) e lidar com a origem da interrupção,
	//São 3 casos: TXE ativo (queremos enviar), RXNE ativo (precisamos estar preparados
	//para receber dados), fault (erro)

	uint8_t temp1, temp2;
	//1. Primeiramente vamos ver se o TXE está ativo
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_TXE));                            //estamos verificando se a flag na posição SPI_SR_TXE está em set ou reset (1 ou 0 respectivamente)
	temp2 = (pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE));                        //aqui verificamos se a interrupção do SPI para transmissão está habilitada

	if(temp1 && temp2){
		//caso o hardware nos diga que TXE está em set, ou seja, vazio e pronto para enviar  (TXE = 1 significa TX buffer empty)
		//handle the TXE
		txe_spi_interrupt_handler(pHandle);
	}

	//2. Segundo caso vamos ver se o RXNE está ativo, ou seja, Rx buffer not empty e se a interrupção para recepção está habilitada, como no caso do Tx
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_RXNE));                            //estamos verificando se a flag na posição SPI_SR_RXNE está em set ou reset (1 ou 0 respectivamente)
	temp2 = (pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE));                        //aqui verificamos se a interrupção do SPI para recepção está habilitada

	if(temp1 && temp2){
		//caso o hardware nos diga que RXNXE está em set, ou seja, vazio e pronto para recebre  (RXNE = 1 significa RX buffer empty)
		//handle the RXNE
		rxne_spi_interrupt_handler(pHandle);
	}

	//3. Verificar se teve erro de overrun, ou seja, se um dado vindo do mundo exterior foi disponibilizado
	//mas por motivos de demora na leitura o dado anterior ainda não foi lido, fazendo com que o dado atual seja perdido
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_OVR));
	temp2 = (pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE));
	if(temp1 && temp2){
		//caso o hardware nos diga que RXNXE está em set, ou seja, vazio e pronto para recebre  (RXNE = 1 significa RX buffer empty)
		//handle the RXNE
		spi_ovr_err_interrupt_handler(pHandle);
	}
}

/*
 * Funções helper para handle de interrupções do SPI
 */

static void txe_spi_interrupt_handler(SPI_Handle_t *pHandle){                //quando uma interrupção por spi acontecer, essa função deverá ser chamada se for o caso de interrupção para Tx
	//2. Format is 8 or 16 bits?
	if(pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){                           //verificar se é 1 byte por vez ou 2

		//16 bits format
		pHandle->pSPIx->DR = *((uint16_t*)pHandle->pTxBuffer);                         //colocar o dado no registrador de dados (DR) que vai enviar, note que primeiro fazemos um type cast de uint8_t para uint16_t. Depois pegamos o conteúdo dentro do ponteiro usando o operador de de-referenciação
		pHandle->TxLen--;
		pHandle->TxLen--;                                                     //duas vezes pois são dois bytes enviados
		(uint16_t*)pHandle->pTxBuffer++;                                    //incrementar duas posições na memória, por isso o casting antes
		//pTxBuffer+=2;
	}
	else {

		//8 bits format
		pHandle->pSPIx->DR = *pHandle->pTxBuffer;
		pHandle->TxLen--;
		pHandle->pTxBuffer++;                                              //incrementar uma única posição
	}

	if(!(pHandle->TxLen)){
		//a comunicação SPI acabou, portanto podemos fechar a comunicação
		//primeiramente desabilitando a interrupção, isso previne mais interrupções da flag TXE do SR
		SPI_CloseTransmisson(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);     //avisar a aplicação que a transmissão acabou
	}
}

static void rxne_spi_interrupt_handler(SPI_Handle_t *pHandle)
{
	//quando uma interrupção por spi acontecer, essa função deverá ser chamada se for o caso de interrupção para Rx
	if(pHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pHandle->pRxBuffer) = (uint16_t) pHandle->pSPIx->DR;     //Aqui efetivamente ocorre a leitura do byte externo
		pHandle->RxLen -= 2;
		pHandle->pRxBuffer++;
		pHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pHandle->pRxBuffer) = (uint8_t) pHandle->pSPIx->DR;
		pHandle->RxLen--;
		pHandle->pRxBuffer++;
	}

	if(! pHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pHandle->TxState != SPI_BUSY_IN_TX)                             //Se uma interrupção de overrun acontecer, mas o periférico estiver ocupado transmitindo algo, então o clear nos registradores SR e DR não acontece
	{
		temp = pHandle->pSPIx->DR;                                     //segundo o manual do usuário devemos dar um read no data register (DR)
		temp = pHandle->pSPIx->SR;									  //seguido de um read no status register (SR) para limpar a flag de overrun
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmisson(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);    //limpar o bit que habilita a interupção (ou seja, estamos desabilitando essa interrupção)
	pHandle->pTxBuffer = NULL;
	pHandle->TxLen = 0;
	pHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pHandle)
{
	pHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pHandle->pRxBuffer = NULL;
	pHandle->RxLen = 0;
	pHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*
 * A função abaixo é uma atribuição fraca de uma app calback colocada aqui pois a implementação efetiva
 * depende da aplicação (grosso modo, da main), mas para não termos warning enquanto a implementação efetiva
 * não for realizada, teremos aqui então esta implementação
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}


//teste de alteração
//teste alteração 2
