/*
 * carHw.c
 *
 *  Created on: 10 feb. 2019
 *      Author: erlin
 */
#include "carHw.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "cmsis_os.h"
extern struct UART_HandleTypeDef huart3;
void USART3_IRQHandler(void);
/* Private macro -------------------------------------------------------------*/
#define semtstSTACK_SIZE      configMINIMAL_STACK_SIZE

/* Private variables ---------------------------------------------------------*/
//osSemaphoreId osSemaphore;

// Circular buffer pointers
// volatile makes read-modify-write atomic
//volatile int tx_in=0;
//volatile int tx_out=0;
//volatile int rx_in=0;
//volatile int rx_out=0;
extern uint8_t received[];
uint8_t receivedParam[100];
uint8_t *receivedParamStartPtr = &receivedParam[0];
uint8_t *receivedParamEndPtr = &receivedParam[0];
static uint8_t tmp;
extern uint8_t num_rx_rounds;
extern int param1;
extern int param2;
extern int param3;
extern char parameterType[];
int parametersReceived =0;
/* Private function prototypes -----------------------------------------------*/
//static void SemaphoreTest(void const *argument);
void SystemClock_Config(void);
/* Initialize LED */
//BSP_LED_Init(LED2);

/* Initialize buttons */
//BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

/* Define used semaphore */
osSemaphoreDef(SEM);

/* Create the semaphore used by the two threads */
//osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);

/* Create the Thread that toggle LED2 */
//osThreadDef(SEM_Thread, SemaphoreTest, osPriorityNormal, 0, semtstSTACK_SIZE);
//osThreadCreate(osThread(SEM_Thread), (void *) osSemaphore);

/* UART handler declaration */
UART_HandleTypeDef UartHandle;
const int RXBUFFERSIZE = 10;
const int TXBUFFERSIZE = 60;
/* Buffer used for transmission */
uint8_t aTxBuffer3[] = " **** UART_TwoBoards_ComPolling ****   **** UART_TwoBoards_ComPolling **** ";

/* Buffer used for reception */
uint8_t aRxBuffer3[] = "                        ";

/* function declaration */
void sendCommand( struct Command *command );
int myItoa(int value,char *ptr);


void sendCommand( struct Command *command ) {


}
void runCarHw(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huartTx) {
	//	   struct Command Power;        /* Declare power command -255 0 255 */
	//   struct Command Direction;        /* Declare steering -100 0 100 */
	long power;
	long steeringAngle;
	power = 100;
	steeringAngle = 150;
	/* Power definition */
	//	   Power.startMarker ='<';
	//	   Power.command = power;
	//	   Power.delimiter = ',';
	//	   Power.param= steeringAngle;
	//	   Power.endMarker= '>';

	aTxBuffer3[0] = (uint8_t) '<';
	char buf[20];
	int length = myItoa (power,buf);
	int pos =1;
	memcpy(&aTxBuffer3[pos], &buf[0], length+1);
	pos = length+1;
	aTxBuffer3[pos] = (uint8_t) ',';
	length = myItoa (steeringAngle,buf);
	memcpy(&aTxBuffer3[pos], &buf[0], length+1);
	pos = pos + length+1;
	aTxBuffer3[pos] = (uint8_t) '>';

	//			  if(HAL_UART_Transmit(huart1, (uint8_t*)aTxBuffer, 70, 5000)!= HAL_OK)
	//			    {
	//			  //    Error_Handler();
	//			    }
	if(num_rx_rounds<=1){
		if(HAL_UART_Transmit(huartTx, (uint8_t*)aTxBuffer3, pos+1, 5000)!= HAL_OK)
		{
			Error_Handler();
		}
	} else {
		aTxBuffer3[0] = parameterType[0];
		length = myItoa (param1,buf);
		memcpy(&aTxBuffer3[1], &buf[0], length+1);
		pos = length+2;
		length = myItoa (param2,buf);
		memcpy(&aTxBuffer3[pos], &buf[0], length+1);
		pos = pos + length+1;
		length = myItoa (param3,buf);
		memcpy(&aTxBuffer3[pos], &buf[0], length+1);
		pos = pos + length+1;
		if (parametersReceived == 1){

			memcpy(&aTxBuffer3[pos], &receivedParam[0], num_rx_rounds);
			if(HAL_UART_Transmit(huartTx, (uint8_t*)aTxBuffer3, num_rx_rounds+ pos, 5000)!= HAL_OK)
			{
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				Error_Handler();
			}
		}
	}

	//	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	osDelay(1000);


	//	if(HAL_UART_Receive(huartRx, (uint8_t*)aRxBuffer3, pos+1, 5000)!= HAL_OK)
	//	{
	//		Error_Handler();
	//	}

}

/*Function return size of string and convert signed  *
 *integer to ascii value and store them in array of  *
 *character with NULL at the end of the array        */

int myItoa(int value,char *ptr)
{
	int count=0,temp;
	if(ptr==NULL)
		return 0;
	if(value==0)
	{
		*ptr='0';
		return 1;
	}

	if(value<0)
	{
		value*=(-1);
		*ptr++='-';
		count++;
	}
	for(temp=value;temp>0;temp/=10,ptr++);
	*ptr='\0';
	for(temp=value;temp>0;temp/=10)
	{
		*--ptr=temp%10+'0';
		count++;
	}
	return count;
}
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3) {
		tmp = *(huart->pRxBuffPtr);
		if (tmp == '<'){
			receivedParamEndPtr =&receivedParam[0];
			num_rx_rounds = 0;
		} else if (tmp == '>'){
			parametersReceived = 1;
			*receivedParamEndPtr++ = tmp;
			*receivedParamEndPtr = '\0'; // String end
			num_rx_rounds+=2;
		}else {
			*receivedParamEndPtr++ = tmp;
			num_rx_rounds++;
		}
	}
	if (huart->Instance == USART2) {
		tmp = *(huart->pRxBuffPtr);
		if (tmp == '<'){

		}
	}
	// don't need to do anything. DMA is circular
}
void HAL_UART_Error (UART_HandleTypeDef *huart)
{
	num_rx_rounds++;
	// don't need to do anything. DMA is circular
	for(;;)
	{
		//		GPIO_Write(LD2_GPIO_Port, LD2_Pin,GPIO_BIT_SET);
		//		HAL_GPIO_DeInit(LD2_GPIO_Port, LD2_Pin);
		//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		osDelay(100);

	}
}
/**
 * @brief  Semaphore Test.
 * @param  argument: Not used
 * @retval None
 */
//static void SemaphoreTest(void const *argument)
//{
//	for (;;)
//	{
//
//		if (osSemaphore != NULL)
//		{
//			/* Try to obtain the semaphore */
//			if (osSemaphoreWait(osSemaphore , 0) == osOK)
//			{
////				BSP_LED_Toggle(LED2);
//
//			}
//		}
//	}
//}

/* For what it's worth, I have a working UART implementation using the DMA - but *only* in circular mode.  It
avoids the ISR LOCK problem.

Thus, if used, it works without having to modify the library code or accommodate the issue in any other way.

The only potential error I found is that the pointer passed into the Receive function doesn't cause the
received data to be placed into the location pointed to, it always arrives in the location originally
pointed to provided in the first call - but this just may be a bit of laziness in the original programmer's
part where they didn't want to write a separate function that excluded the pointer.  However, this
requires a separate small receive buffer.

Example of (possible) error:
 **/

/* initial call */
//
//p = &RecvBuffer [ 0 ];
//
//HAL_UART_Receive_DMA ( &huart3, p, 1 );
//
///* in the Callback function */
//
//p = &RecvBuffer [  TailPtr ];
//
//HAL_UART_Receive_DMA ( &huart3, p, 1 );

/* Character still shows up in RecvBuffer [ 0 ] regardless of TailPtr's value. */

/*
 * No need to go through all these other gyrations in this thread - just use the Circular DMA feature - but
use a separate receive buffer and transfer the data from it to your ring buffer in the callback function.

I have had 100% success with using only the RECEIVE on DMA.  The transmit still uses _IT - but that's
largely because the transmit isn't re-cocked inside the ISR - whereas the receive must be.  I
have had several UARTS simultaneously receiving and transmitting for days on end at 115.2K baud
using common (shared) call-back functions.
 **/

