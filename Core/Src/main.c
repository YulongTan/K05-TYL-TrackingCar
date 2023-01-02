/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"stdio.h"
#include"stdint.h"
#include"string.h"
#include"stdlib.h"
#include"oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t my_re_buf1[24]={0x00};
uint8_t rx_buf1[1]={0};
uint8_t rx_buf2[1]={0};
uint8_t rx_buf3[1]={0};
uint8_t openmvdata[8]={0x00};
uint8_t data_static2[6]={255,255,255,255,255,255};
volatile uint32_t G_Systime;//定时器中断中计时
int temp=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct gesture
{
	float angr;//角度
	float angx;
	float angy;
	float w;//角速度
	float v1;//左轮速度
	float v2;//右轮速度
	float x1;//左轮位移
	float x2;//右轮位移
	float x;//总位移
};

struct pidstruct
{
	float kp,ki,kd;
	float p ,i ,d;
	float thisde, lastde;
};  //thisde当前
struct pidstruct initPID(float kp, float ki, float kd, struct pidstruct e)//初始化PID
{
	e.kp=kp;
	e.ki=ki;
	e.kd=kd;
	e.p=0;//有必要时不用初始化给0
	e.i=0;
	e.d=0;
	e.lastde =0;
	e.thisde=0;
	return e;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct gesture gest;//当前姿态
struct gesture tgest;//目标姿态
struct gesture globalgest;//初始姿态
struct pidstruct angrpid;//角度PID 
struct pidstruct angxpid;//x轴角度PID
struct pidstruct angypid;
struct pidstruct angzpid;
struct pidstruct x1pid;//位移PID
struct pidstruct x2pid;
struct pidstruct v1pid;
struct pidstruct v2pid;


float PWM1;
float PWM2;
float PWM;
int En=1;//小车运行停止
int Enflag=1;    //控制轮子停止后反转保证停稳
int Enflag2=1;
int startpid=1;

int mark=0;//小车停稳
//char a[50];
//int count=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void Wheel(int num,int pwm);
void My_Delay_1ms(int t);
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
	return ch;
}

float PID(struct pidstruct *e,float err, float outhigh , float outlow)//返回PID的值 outhigh和outlow是限幅
{
	float out;
	e->thisde=err;
	e->p = e->kp * e->thisde ;
	e->i = e->ki * (e->i+e->lastde );
	e->d = e->kd * (e->thisde -e->lastde );
	e->lastde = e->thisde ;
	out=e->p+e->i +e->d;
	if(out>outhigh ) out=outhigh ;//限幅
	if(out<outlow ) out =outlow ;
	return out;
}

void My_Delay_1ms(int t)
{
	uint32_t InitTime=G_Systime ;
	while(G_Systime <InitTime +t)
	{
	}
}


void Stop()
{
//	if(mark==0)
//	{
//	Wheel(1,-60);
//	Wheel(0,0);	
//	My_Delay_1ms(500);
//	mark=1;
//	}
//	if(mark==1)
//	{
//	Wheel(1,0);
//	Wheel(0,0);	
//	}
     Wheel(1,-30);
     Wheel(0,10);	
//	My_Delay_1ms (500);
//    HAL_GPIO_TogglePin (GPIOC,GPIO_PIN_13);
}

void Move_pid()
{
	float err1,err2,angxerr;
    angxerr =tgest.angx -gest.angx ;
	PWM=PID (&angxpid ,angxerr ,250,-250);
	err1=tgest.v1-gest.v1;
	PWM1=PID(&v1pid,err1,1000,-1000);
	err2=tgest.v2 -gest.v2 ;
	PWM2=PID(&v2pid,err2,1000,-1000);
	Wheel(1,PWM1);
	Wheel(0,PWM2);
	mark=0;//小车停稳标志重置
}
void Delay_stop(int target_delaytime)  //放在定时器中断中1ms进入一次，只能连续使用一次
{
	static int dtime;//dtime是延时停车
    if(mark==0)
	{
        dtime++;
		if(dtime<target_delaytime )
		{
			HAL_GPIO_TogglePin (GPIOC,GPIO_PIN_13);
			Move_pid ();
		}
		else
		{
			mark=1;
			dtime =0;
		}
	}
	if(mark==1)
	{
		Stop();
	}
	
}

void DataGet1(uint8_t *data)//蓝牙
{ 
//	int en,i=0;
//	char b[1];
//	b[0]=data[0];
//	en=atoi(b);
//	if(en==0)
//	{
//		i++;
//		En=i%2;
//	}
//	if(En==1)
//	{
//	HAL_GPIO_TogglePin (GPIOC,GPIO_PIN_13);
//	}
	int en;
	static int i;
	en=(int)(data[0]-'0');
	if(en==0)
	{
		i++;
		En=i%2;
		startpid=0;
		Enflag=0;
	}
}
void DataGet2(uint8_t *data)//陀螺仪
{ 
	static int flag=0;
	if(data[0]==0x55&&flag ==0)
	{
		HAL_UART_Receive_DMA (&huart2 ,rx_buf2 ,11);
		flag=1;
	}
	else if(data [0]!=0x55&&flag==0)
	{
		HAL_UART_Receive_IT (&huart2,rx_buf2 ,1);
	}
	else if(flag==1)
	{
		HAL_UART_Receive_DMA (&huart2,rx_buf2 ,11);
	}
	
}
void DataGet3(uint8_t *data)//
{ 
	//printf("%c",data[0]);
	//HAL_GPIO_TogglePin (GPIOC,GPIO_PIN_13);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口中断
{
	if(huart == &huart1)
	{
		DataGet1(rx_buf1);
		HAL_UART_AbortReceive_IT(&huart1);
		HAL_UART_Receive_IT(&huart1, rx_buf1, 1);
	}
	if(huart == &huart2)
	{
		DataGet2(rx_buf2);
		HAL_UART_AbortReceive_IT(&huart2);
		HAL_UART_Receive_IT(&huart2, rx_buf2, 1);
	}
	if(huart == &huart3)
	{
		static  char a[20];
        static  int count=0;

		DataGet3(rx_buf3);
		//printf("a=%c\n",rx_buf3[0]);
		if(rx_buf3[0]=='\n')
		{
			a[count]='\0';
			gest.angx=atof(a);
			count=0;
			a[0]='\0';
		}
		else if(rx_buf3[0]=='a')
		{
			En=0;
		    Enflag=0;
            temp=1;
		  HAL_GPIO_TogglePin (GPIOC,GPIO_PIN_13);
		}
//		else if(rx_buf3[0]=='w')
//		{
//			En=2;
//			Enflag2=0;
//			HAL_GPIO_TogglePin (GPIOC,GPIO_PIN_13);
//		}
		else
		{
			a[count]=rx_buf3[0];
			count++;
		}
		HAL_UART_AbortReceive_IT(&huart3);//继续接受
		HAL_UART_Receive_IT(&huart3, rx_buf3, 1);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//输入捕获中断
{
	
	static float catch1[3],catch2[3];
	//int judge1,judge2;
	if(htim==&htim1&&htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_3)//左轮
	{
		//judge1 =HAL_GPIO_ReadPin (GPIOB,GPIO_PIN_14);
		//printf("judge1=%d\r\n",judge1 );
		catch1[1]=HAL_TIM_ReadCapturedValue (htim,TIM_CHANNEL_3 );
		catch1[2]=catch1[1]-catch1[0];
		if(catch1[2]<=0)
		{
			catch1[2]+=0xffff;
		}
		
		catch1[0]=catch1[1];
		
		//if(judge1==1)
		if(HAL_GPIO_ReadPin (GPIOB,GPIO_PIN_14))
		{
			gest.x1+=0.52f;
			gest.v1=644429.2/catch1[2];
			//printf("x1=%f\r\n",gest.x1);

		}
		//else if(judge1==0)
		else
		{
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			gest.x1-=0.52f;
			gest.v1=-644429.2/catch1[2];
			//printf("x1=%f\r\n",gest.x2);

			
		}
		
	}	
	if(htim==&htim1&&htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_4)//右轮
	{
		//judge2 =HAL_GPIO_ReadPin (GPIOB,GPIO_PIN_15);
		catch2[1]=HAL_TIM_ReadCapturedValue (htim,TIM_CHANNEL_4 );
		catch2[2]=catch2[1]-catch2[0];
		if(catch2[2]<=0)
		{
			catch2[2]+=0xffff;
		}
		catch2[0]=catch2[1];
		
		if(HAL_GPIO_ReadPin (GPIOB,GPIO_PIN_15))
		{
			gest.x2-=0.52f;
			gest.v2=-644429.2/catch2[2];
			//printf("v2=%f\r\n",gest.v2);
		}
		else
		{
			gest.x2+=0.52f;
			gest.v2=644429.2/catch2[2];

			//printf("v2=%f\r\n",gest.v2);
		}
		
	}	
     gest.x=(gest.x1+gest.x2)/2;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器中断（1ms） （callback回调函数 谨慎用printf 1ms进一次 不要超过字节）
{
    static int time,stoptime,delaytime;
	static int Enfla1,Enfla2;
	G_Systime++;
	//int i;
//	float err1,err2,angxerr;
//	angxerr =tgest.angx -gest.angx ;
//	PWM=PID (&angxpid ,angxerr ,250,-250);
//	err1=tgest.v1-gest.v1;
//	PWM1=PID(&v1pid,err1,1000,-1000);
//    //Wheel(1,PWM1);
//	err2=tgest.v2 -gest.v2 ;
//	PWM2=PID(&v2pid,err2,1000,-1000);		
	
	
	if(htim == (&htim4))
	{

		  
		if(En==1)
		{
			if(startpid==0)
			{
			v1pid=initPID(10.5,1,0.01,v1pid);
            v2pid=initPID(11,1,0,v2pid);
            angxpid=initPID(1,0,0,angxpid);
			startpid =1;
            Move_pid ();
			}
		    else
			{
            Move_pid();
			}
		}
	    else if(En==0)
		{
//      //    Stop(En,1000);
//			if(Enfla1==0){
////				
////			angxerr =tgest.angx -gest.angx ;
////		   PWM=PID (&angxpid ,angxerr ,250,-250);
////		   err1=tgest.v1-gest.v1;
////		   PWM1=PID(&v1pid,err1,1000,-1000);
////            //Wheel(1,PWM1);
////		   err2=tgest.v2 -gest.v2 ;
////		    PWM2=PID(&v2pid,err2,1000,-1000);
//            Move_pid();
////			Wheel(1,PWM1);
////			Wheel(0,PWM2);
//			delaytime++;	
//			}			
//		    if(delaytime>=500)
//				{
//                if(Enflag==0)
//				{
//					Enfla1=1;
//					Wheel(1,-60);
//					Wheel(0,0);	
//					stoptime++;
//					if(stoptime==500)
//						{
//							stoptime =0;
//							Enflag=1;

//						}
//				}
//			    else
//		        {
//				Wheel(1,0);
//				Wheel(0,0);	
//		        }

//	    	}
//      Stop();
			Delay_stop (2000);
		}
			
		else if(En==2)
		{
			
			if(Enfla2==0)
			{
			Move_pid();
//			Wheel(1,PWM1);
//			Wheel(0,PWM2);
			delaytime++;	
			}
		    if(delaytime>=650)
			{
                if(Enflag2==0)
				{
					Enfla2=1;
					Wheel(1,-60);
					Wheel(0,0);	
					stoptime++;
					if(stoptime==400)
						{
							stoptime =0;
							Enflag2=1;

						}
				}
			    else
		        {
				Wheel(1,0);
				Wheel(0,0);	
				time++;
				  if(time>=5000)
				  {
					  
					  
			   v1pid=initPID(10.5,1,0.01,v1pid);
               v2pid=initPID(11,1,0,v2pid);  
			   angxpid=initPID(1,0,0,angxpid); 

			   Move_pid();
//			   Wheel(1,PWM1);
//			   Wheel(0,PWM2);
					  
					  time=0;
                      En=1;
					  delaytime=0;
					  
					  Enfla2=0;
				  }
		        }

	    	}
			
			
			
//			Wheel(1,0);
//			Wheel(0,0);
//			time++;
//		if(time==5000)
//		{	
//				En=1;
//				time=0;
//		}
//		}
		
//			Wheel(1,PWM1);
//			Wheel(0,PWM2);
		//Move(En,PWM1,PWM2);
	}
   }

   
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define LENGTH 4     //接受缓冲区大小
//uint8_t	RxBuffer[LENGTH];   //接受缓冲区 
//uint8_t RxFlag = 0;       //接收完成标志；0表示接受未完成，1表示接收完成



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//  v1pid=initPID(9,1,0,v1pid);
//  v2pid=initPID(9,1,0,v2pid);
 
  v1pid=initPID(10.5,1,0.01,v1pid);
  v2pid=initPID(11,1,0,v2pid);
  angxpid=initPID(1,0,0,angxpid);
	  // angxpid=initPID(0.8,0.1,0.001,angxpid);//tgest.v=400
 //angxpid=initPID(1.3,0.12,0.01,angxpid);//tgest.v=800
  //angxpid=initPID(1,0,0,angxpid);//tgest.v=800
//  angypid =initPID (0,0,0,angypid);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  OLED_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
    HAL_UART_Receive_IT (&huart1,rx_buf1 ,1);//串口 IT中断
    HAL_UART_Receive_IT (&huart2,rx_buf2 ,1);
	HAL_UART_Receive_IT (&huart3,rx_buf3 ,1);

//    HAL_TIM_Base_Start_IT (&htim4);//定时器中断初始化
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//pwm
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

    HAL_TIM_IC_Start_IT (&htim1,TIM_CHANNEL_3 );//输入捕获
	HAL_TIM_IC_Start_IT (&htim1,TIM_CHANNEL_4 );
	HAL_TIM_Base_Start_IT (&htim4);//定时器中断初始化
	HAL_TIM_Base_Start_IT (&htim3);//定时器中断初始化
//HAL_UART_Transmit_DMA (&huart1,"Uart System Init\r\n",sizeof ("Uart System Init\r\n"));
//HAL_UART_Receive_DMA(&huart1,buf,3);
	//HAL_UART_Receive_DMA (&huart2 ,rx_buf2 ,11); //一次11个满了进中断
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	tgest.angx =0;
	tgest.v1=400+PWM;
	tgest.v2 =400-PWM;
	  

//	 OLED_ShowString(40,0,"MPU6050",16);
//	if(En==1)
//	{
//	tgest.v1=800+PWM;
//	tgest.v2 =800-PWM;
//	}
//	else
//	{
//	tgest.v1=0;
//	tgest.v2=0;
//	}
////	Wheel(1,1000);
////	Wheel(0,1000);
 //   printf("a=%c\n",rx_buf3[0]);
//	printf("gest.angr=%f\r\n",);
//	printf("--------------------\n");
//	printf("PWM1=%f\r\n",PWM1);
//	printf("PWM2=%f\r\n",PWM2);
//	printf("v1=%f\r\n",gest.v1);  
//	printf("v2=%f\r\n",gest.v2);
 printf("%f,%f,%.2f\n",gest.v1,gest.v2,gest.angx);
//	printf("%f\n",gest.angx);
//	printf("%d\n",temp);
//	printf("%c",temp1[0]);
  //  printf("%f",PWM);
//	    printf("v1=%f\r\n",gest.v1);
//	    printf("PWM1=%f\r\n",PWM1 );
//		err2=tgest.v2 -gest.v2 ;
//		PWM2=PID(&v2pid,err2,1000,-1000);
//		Wheel(0,PWM2);
//	  	HAL_GPIO_TogglePin (GPIOC,GPIO_PIN_13);
       // printf("PWM2=%f\r\n",PWM2);
	  //	printf("v2=%f\r\n",gest.v2);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,500);
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,500);
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,1);
	  
//	
//	for(i=-1000;i<=1000;i++)
//	  {
//	  Wheel(1,i);
//	  HAL_Delay(20);
//	  }
//	  
//        HAL_UART_Transmit(&huart1, (uint8_t *)"hello windows!!!\r\n", 16 , HAL_MAX_DELAY);
//        HAL_Delay(1000);  
//	  if(RxFlag == 1)  
//	  {
//		   HAL_UART_Transmit(&huart1,(uint8_t *)"Recevie Success!\n",17,HAL_MAX_DELAY);  
//		   break; 
//	  }

//	 Wheel(1,500);
//	 Wheel(0,500);
   }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */








void Wheel(int num,int pwm)
{
	int direct;
	if(pwm>1000)
	{
		pwm=1000;
	}
	if(pwm<-1000)
	{
		pwm=-1000;
	}
	if(pwm>=0)
	{
		direct=1;
	}
	else
	{
		direct=0;
	}
	switch(num){
		case 1:
		switch(direct){
		    case 1:
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
			break;
		    case 0:
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,1000+pwm);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
			break;
	      }
	     break;
		case 0:
			switch(direct){
			case 1:
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,1000-pwm);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
				break;
			case 0:
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,-pwm);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
				break;
		  
            }
	      break;
		}
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //串口接收中断回调函数
//{
//	if(huart->Instance == USART1)   //判断发生接收中断的串口
//	{
//		RxFlag=1;   //置为接收完成标志
//	}
//}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
