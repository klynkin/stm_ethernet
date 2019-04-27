/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern GPIO_InitTypeDef GPIO_InitStruct;
extern TIM_HandleTypeDef htim9;
xSemaphoreHandle xBinarySemaphoreStop;
xSemaphoreHandle xBinarySemaphoreStart;
xQueueHandle xQueue;
xQueueHandle xQueueStart;
xQueueHandle xQueueSwitch;
xQueueHandle xQueueEncoder;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef struct struct_conn_t {
  uint32_t conn;
  uint32_t buf;
} struct_conn;
struct_conn conn01;
//-----------------------------------------------------------------------------------------------------------------------------------------------
extern osThreadId defaultTaskHandle;
osThreadId Task01Handle, TaskEncoderHandle, TaskNextionHandle,task_startHandle;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void vHandlerTask1( void *pvParameters );
static void tcp_thread( void *pvParameters );
static void task_start(void *pvParameters);
static void task_stop(void *pvParameters);
static void task_nextion(void *pvParameters);
static void task_encoder(void *pvParameters);

//static void task_DRV(void *pvParameters);
void StartDefaultTask(void const * argument);
void MX_FREERTOS_Init(void);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1000);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
vSemaphoreCreateBinary( xBinarySemaphoreStart );
vSemaphoreCreateBinary( xBinarySemaphoreStop );
xQueue = xQueueCreate(1, sizeof( int ) );
xQueueStart = xQueueCreate(1, sizeof( int ) );
xQueueEncoder=xQueueCreate(1, sizeof( int ) );
xQueueSwitch=xQueueCreate(20, sizeof( int ) );
if(xBinarySemaphoreStart != NULL && xBinarySemaphoreStop != NULL)
{
    osThreadDef(task_start, task_start, 2, 0, 1000);
    task_startHandle = osThreadCreate(osThread(task_start), NULL);
}
vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN 5 */
  struct netconn *conn;
  err_t err;
  ip_addr_t ServerIPaddr;
  IP4_ADDR(&ServerIPaddr, 192, 168, 0, 189);
  conn = netconn_new(NETCONN_TCP);
  	if(conn!=NULL)
  	{
  		err = netconn_bind(conn, NULL, 4555);
  	  	if (err == ERR_OK)
  	  	{
  	  		err = netconn_connect(conn, &ServerIPaddr, 5000);
  	  		while(err!=ERR_OK)
  	  		{
  	  			netconn_delete(conn);
  	  		    conn = netconn_new(NETCONN_TCP);
    		    	if(conn!=NULL)
    		    	{
  	  		    	err = netconn_bind(conn, NULL, 4555);
    		    		if (err == ERR_OK)
  	  		    	{
  	  		    		err = netconn_connect(conn, &ServerIPaddr, 5000);
  	  		    	}
    		    	}
  	  		    vTaskDelay(10);
  	  		}
  	  		if (err==ERR_OK)
  	  		{
  				conn01.conn = conn;
  				err=netconn_write(conn, "hello     \n", 11, NETCONN_COPY);
  				sys_thread_new("tcp_thread", tcp_thread, (void*)&conn01, 2000, osPriorityHigh );
  			}
  	  	}
  		else
  		{
  			netconn_delete(conn);
  		}
		osThreadTerminate(NULL);


  	}
  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay(portMAX_DELAY);
  }
  /* USER CODE END 5 */
}

static void tcp_thread(void *arg)
{
struct_conn *arg_conn;
struct netconn *conn;
err_t sent_err;
err_t ent_err;
arg_conn = (struct_conn*) arg;
conn = (void*)arg_conn->conn;
portBASE_TYPE xStatus;
int *buf;
int lReceivedValue;
static int i=1;

  	xStatus = xQueueReceive( xQueueStart, &lReceivedValue, portMAX_DELAY);
	sent_err=netconn_write(conn, "ZO2       \n", 11, NETCONN_COPY);
	if (sent_err!=ERR_OK)
    	{
			netconn_delete(conn);
    	}
		else
		{
			sys_thread_new("task_stop", task_stop, (void*)conn, 1000, 2 );
			TaskNextionHandle=sys_thread_new("task_nextion", task_nextion, (void*)conn, 2500, 1 );
			Task01Handle=sys_thread_new("vHandlerTask1", vHandlerTask1, (void*)conn, 1024, 1);
			TaskEncoderHandle=sys_thread_new("task_encoder", task_encoder, (void*)conn, 4000, 2);
			osThreadTerminate(NULL);
		}
	for(;;)
			{
		vTaskDelay(portMAX_DELAY);
			}

}
//-----------------------------------------------------------------------------------------------------------------------------------------------

static void vHandlerTask1( void *pvParameters )
{
	err_t err;
	struct netconn *conn;
	conn=(struct netconn *) pvParameters;
	portBASE_TYPE xStatus;
	int *buf;
	int lReceivedValue;
	char str[11];
	str[0]='Z';
	str[1]='O';
	str[2]='3';
	str[3]=' ';
	str[5]=' ';
	str[7]=' ';
	str[8]=' ';
	str[9]=' ';
	str[10]='\n';
	for(;;)
	{
	  	xStatus = xQueueReceive(xQueueSwitch, &lReceivedValue, portMAX_DELAY);
	  	if (lReceivedValue<6)
	  	{
	  		str[6]='1';
	  		str[4]=(lReceivedValue)+'0';
	//sprintf(str, "ZO3 %d 1\r\n", lReceivedValue);
	//	taskENTER_CRITICAL();
    //	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_0);
    	err=netconn_write(conn, str, 11, NETCONN_COPY);
		/*	if (err!=ERR_OK)
			   	{
				netconn_delete(conn);
				}*/
	//	taskEXIT_CRITICAL();
	  	}
	  	else
	  	{
	  		str[6]='0';
	  		str[4]=(lReceivedValue-6)+'0';
	  //sprintf(str, "ZO3 %d 0\r\n", lReceivedValue-6);
	  //		taskENTER_CRITICAL();
	  //	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_0);
	  		err=netconn_write(conn, str, 11, NETCONN_COPY);
	  	/*	if (err!=ERR_OK)
	  			{
	  			netconn_delete(conn);
	  			}*/
	  //		taskEXIT_CRITICAL();
	  	}
	}

}

//-----------------------------------------------------------------------------------------------------------------------------------------------
static void task_start(void *pVparameters)
{
	int i=1;
	xSemaphoreTake(xBinarySemaphoreStart, 0);
	xSemaphoreTake(xBinarySemaphoreStart, portMAX_DELAY );
	xQueueSendToBack( xQueueStart, &i, 0 );


	/*GPIO_InitStruct.Pin = Stop_3_Pin|Stop_4_Pin|Stop_5_Pin|Stop_6_Pin
	                          |Stop_7_Pin|Stop_8_Pin|Stop_9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);*/
	osThreadTerminate(NULL);
		for(;;)
		{
			vTaskDelay(portMAX_DELAY);
		}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------

static void task_stop(void *pVparameters)

{
err_t err;
struct netconn *conn;
conn=(struct netconn *) pVparameters;
xSemaphoreTake(xBinarySemaphoreStop, 1);
//HAL_UART_Transmit(&huart2, "enter to Stop\r\n", strlen("enter to Stop\r\n"), 0x100);
xSemaphoreTake(xBinarySemaphoreStop, portMAX_DELAY );
err=netconn_write(conn, "ZO1       \n", 11, NETCONN_COPY);
TIM9->CCR1=0;
TIM9->CCR2=0;
HAL_TIM_PWM_Stop(&htim9,TIM_CHANNEL_1);
HAL_TIM_PWM_Stop(&htim9,TIM_CHANNEL_2);
netconn_close(conn);
netconn_delete(conn);
HAL_NVIC_DisableIRQ(EXTI2_IRQn);
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
//HAL_NVIC_DisableIRQ(EXTI4_IRQn);
//HAL_NVIC_DisableIRQ(EXTI3_IRQn);
osThreadTerminate(Task01Handle);
//osThreadTerminate(tcp_thread);
osThreadTerminate(defaultTaskHandle);
osThreadTerminate(TaskEncoderHandle);
osThreadTerminate(TaskNextionHandle);

for(;;)
{

}

}
//-----------------------------------------------------------------------------------------------------------------------------------------------

static void task_nextion(void *pVparameters)
{
	err_t recv_err;
	struct netbuf *inbuf;
	struct netconn *conn;
	uint8_t* buf;
	u16_t buflen;
	uint8_t end=0xff;
	conn=(struct netconn *) pVparameters;
	char str[16];
	str[0]='n';
	str[1]='u';
	str[2]='m';
	str[3]='b';
	str[4]='e';
	str[5]='r';
	str[6]='1';
	str[7]='.';
	str[8]='v';
	str[9]='a';
	str[10]='l';
	str[11]='=';
	int counter;
			for(;;)
				{

					recv_err = netconn_recv(conn, &inbuf);
					if (recv_err == ERR_OK)
					{
						if (netconn_err(conn) == ERR_OK)
						{
							netbuf_data(inbuf, (void**)&buf, &buflen);
							if(buflen>1)
							{
								if	((*(uint8_t *)buf)=='Z' && (uint8_t)buf[1]=='I')
								{
							    if ((uint8_t)buf[2]=='5')
							    {
									if (buflen==8)
							    	{
							    	str[12]=(char)buf[4];
							    	str[13]=(char)buf[5];
							    	str[14]=(char)buf[6];
				  		    		taskENTER_CRITICAL();
							    	HAL_UART_Transmit(&huart3, str, 15, 0x100);
							    	HAL_UART_Transmit(&huart3, &end, sizeof(uint8_t), 0x100);
							    	HAL_UART_Transmit(&huart3, &end, sizeof(uint8_t), 0x100);
							    	HAL_UART_Transmit(&huart3, &end, sizeof(uint8_t), 0x100);
				  		    		taskEXIT_CRITICAL();
							    	}

								}
								else if	((uint8_t)buf[2]=='4')
								  {
									if (buflen==8)
									{
										counter=((uint8_t)buf[4]-48)*100 +((uint8_t)buf[5]-48)*10+((uint8_t)buf[6]-48);
								    	xQueueSendToBack(xQueueEncoder, &counter, 0 );
									}
								  }
								 else if	((uint8_t)buf[2]=='3')
								    	{
								    		if((uint8_t)buf[4]=='1')
								    		{
								    			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
								    			vTaskDelay(1);
								    			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
								    			vTaskDelay(10);
								    			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
								    			vTaskDelay(1);
								    			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
								    			vTaskDelay(10);
								    			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
								    			vTaskDelay(1);
								    			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
								    			vTaskDelay(10);
								    			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
								    			vTaskDelay(1);
								    			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
								    			vTaskDelay(10);
								    			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
								    			vTaskDelay(1);
								    			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
								    		}
								    		else if ((uint8_t)buf[4]=='0')
								    		{
								    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
								    			vTaskDelay(1);
								    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
								    			vTaskDelay(10);
								    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
								    			vTaskDelay(1);
								    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
								    			vTaskDelay(10);
								    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
								    			vTaskDelay(1);
								    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
								    			vTaskDelay(10);
								    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
								    			vTaskDelay(1);
								    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
								    			vTaskDelay(10);
								    			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
								    			vTaskDelay(1);
								    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
								    		}
								    	}
								   else if	((uint8_t)buf[2]=='1')
								   {
									   if((uint8_t)buf[4]=='1')
									   {
										  TIM9->CCR1=20000;
										  TIM9->CCR2=0;
									   }
									   else if ((uint8_t)buf[4]=='0')
									   {
										   TIM9->CCR1=0;
										   TIM9->CCR2=0;
									   }
								   }

								    else if	((uint8_t)buf[2]=='6')
								  {
								    	counter=0;
								    	xQueueSendToBack(xQueueEncoder, &counter, 0 );
								  }
								}
								else if (((uint8_t )buf[0])=='P' && (uint8_t)buf[1]=='I' && ((uint8_t )buf[2])=='N' && (uint8_t)buf[3]=='G' )
								{
									netconn_write(conn, "PONG      \n", 11, NETCONN_COPY);
								}
							}

						netbuf_delete(inbuf);
						}
					}

				}
	}



static void task_encoder(void *pVparametrs)
{
	int encoder_val1;
	int encoder_val2;
	char str[21];
	portBASE_TYPE xStatus;
	int *buf;
	uint8_t lReceivedValue;
	static int i=1;
	static int counter;
	err_t err;
	struct netconn *conn;
	conn=(struct netconn *) pVparametrs;
	xStatus = xQueueReceive( xQueueEncoder, &counter, portMAX_DELAY);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
	int a;
	str[0]='Z';
	str[1]='O';
	str[2]='4';
	str[3]=' ';
	str[4]='0';
	str[5]=' ';
	str[10]='\n';

	str[11]='Z';
	str[12]='O';
	str[13]='4';
	str[14]=' ';
	str[15]='1';
	str[16]=' ';
	str[21]='\n';


	  		      for(;;)
	  		      {
	  		    	a=1000/counter;
	  		    	xStatus = xQueueReceive( xQueueEncoder, &counter, a/portTICK_RATE_MS );
	  		    	if (counter==0)
	  		    	{
	  		    		TIM3->CNT=0;
	  		    		TIM4->CNT=0;
	  		    		vTaskDelay(1);
	  		    	}
	  		    	else
	  		    	{
	  		    		encoder_val1=TIM4->CNT;
	  		    		encoder_val2=TIM3->CNT;
	  		    		//sprintf(str, "ZO4 0 %d\r\n",encoder_val1);
	  		    		//sprintf(str2, "ZO4 1 %d\r\n",encoder_val2);
	  		    		str[9]=(encoder_val1%10)+'0';
	  		    		encoder_val1=(int)encoder_val1/10;
	  		    		str[8]=(encoder_val1%10)+'0';
	  		    		encoder_val1=(int)encoder_val1/10;
	  		    		str[7]=(encoder_val1%10)+'0';
	  		    		encoder_val1=(int)encoder_val1/10;
	  		    		str[6]=(encoder_val1)+'0';

	  		    		str[20]=(encoder_val2%10)+'0';
  		    			encoder_val2=(int)encoder_val2/10;
	  		    		str[19]=(encoder_val2%10)+'0';
	  		    		encoder_val2=(int)encoder_val2/10;
	  		    		str[18]=(encoder_val2%10)+'0';
	  		    		encoder_val2=(int)encoder_val2/10;
	  		    		str[17]=(encoder_val2)+'0';


	  		    		//taskENTER_CRITICAL();
	  		    		err=netconn_write(conn, str, 22, NETCONN_COPY);
	  		    		//if (err != ERR_OK)
	  		    		//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
	  		    		//err=netconn_write(conn, str2, 11, NETCONN_COPY);
	  		    		//delay(1000*a);
	  		    		//if (err!=ERR_OK)
		  		    	//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);

	  		    	//	{
	  		    		//netconn_delete(conn);
	  		    	//	}
	  		    		//taskEXIT_CRITICAL();
	  		      }
	  		      }
	}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
