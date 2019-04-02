#include "main.h"
#include "cmsis_os.h"
#include "messageQueue.h"
//#include "stm32f3xx_hal_gpio.h"
void MessageQueueConsumer(const void *argument);
/**
  * @brief  Message Queue Consumer Thread.
  * @param  argument: Not used
  * @retval None
  */
void MessageQueueConsumer(const void *argument)
{
  osEvent event;

  for (;;)
  {
    /* Get the message from the queue */
    event = osMessageGet(osQueue, 100);

    if (event.status == osEventMessage)
    {
      if (event.value.v != ConsumerValue)
      {
        /* Catch-up */
        ConsumerValue = event.value.v;

        ++ConsumerErrors;

        /* Switch On continuously LED2 to indicate error */
		//		GPIO_Write(LD2_GPIO_Port, LD2_Pin,GPIO_BIT_SET);
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      }
      else
      {
        /* Increment the value we expect to remove from the queue next time
        round */
        ++ConsumerValue;
      }
    }
  }
}
