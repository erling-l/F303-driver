#include "main.h"
#include "cmsis_os.h"
#include "messageQueue.h"
void MessageQueueProducer(const void *argument);
/**
  * @brief  Message Queue Producer Thread.
  * @param  argument: Not used
  * @retval None
  */
void MessageQueueProducer(const void *argument)
{
  for (;;)
  {
    if (osMessagePut(osQueue, ProducerValue, 100) != osOK)
    {
      ++ProducerErrors;
       
      /* Switch On continuously LED2 to indicate error */
		//		GPIO_Write(LD2_GPIO_Port, LD2_Pin,GPIO_BIT_SET);
		HAL_GPIO_DeInit(LD2_GPIO_Port, LD2_Pin);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
    else
    {
      /* Increment the variable we are going to post next time round.  The
      consumer will expect the numbers to follow in numerical order */
      ++ProducerValue;

      if( (ProducerErrors == 0) && (ConsumerErrors == 0) )
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//        BSP_LED_Toggle(LED2);
      osDelay(1000);
    }
  }
}