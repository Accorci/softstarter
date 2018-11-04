/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId EmergenciaHandle;
osThreadId CorrenteHandle;
osThreadId LedsHandle;
osMutexId myMutex01Handle;
osMutexId myMutex02Handle;
osMutexId myMutex03Handle;
osMutexId myMutex04Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int
		//!flag utilizada para mandar o motor iniciar: 1 significa ligando, 0 significa desligando e 2 significa parar tudo. o valor 5 é o inicial pois não é nenhum dos valores aceitos pelos condicionais do código
		y=5,
		//!flag utilizada para verificar quantas vezes alteramos o valor do registrador CCR na rampa de descida: 1 significa que devemos alterá-lo novamente, após isso a flag vira 0 e só volta a 1 na próxima rampa de descida
		w,
		//!indica quantas vezes o registrador CCR é alterado, esse valor deve subir até que o código encerre
		ciclo=0,
		//!vetor utilizado para enviar dados via serial
		msg[5000],
		//!variável utilizada para receber o valor do registrador CCR
		ccr,
		//!variavel utilizada para settar o valor atual do pulso
		inicioDePulso,
		//!flag utilizada para verificar se o motor está na rampa de subida: 1 significa sim, 0 significa não
		subindo,
		//!flag utilizada para verificar se o motor está na rampa de descida: 1 significa sim, 0 significa não
		descendo,
		 //!contador para verificar a contiuidade (ou não) do valor lido na entrada analógica, se contagem for alta significa que há um valor contínuo, isso é, não é ruido
		contagem=0,
		 //!contador para verificar a contiuidade (ou não) do valor lido na entrada analógica, se contagem for alta significa que há um valor contínuo, isso é, não é ruido
		contagem2=0,
		 //!contador para verificar a contiuidade (ou não) do valor lido na entrada analógica, se contagem for alta significa que há um valor contínuo, isso é, não é ruido
		contagem3=0,
		//!flag dos 150
        centoecinquenta=0;

float
		//!valor inicial para o registrador CCR na rampa de subida
		z=24500,
		//!valor inicial para o registrador CCR na rampa de descida
        x = 3000,
		//!tempo de descida
        tempo1,
		//!tempo de subida
        tempo2,
		//!tempo auxiliar
        tempo1aux,
		//!tempo auxiliar
        tempo2aux,
		//!x auxiliar
        xaux=3000,
		 //!z auxiliar
        zaux=24500,
		//!valor de tensão lido na entrada analógica em volts
        valor;
		//!flag de emergencia dps da interrupção de RX completed
		int emergencia=0;
		//!!leds de emergencia, 150 e 200%
		int led1=0, led2=0, led3=0;
		//!variável que recebe algum valor da serial
		uint8_t receive[3];
		//!valor de tensão lido na entrada analógica em bits (resolução = 12bits)
		uint32_t value;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void Emergencia_Task(void const * argument);
//void Serial_Out_Task(void const * argument);
void Corrente_Task(void const * argument);
void Leds_Task(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//!verifica o estouro do timer
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	 //!verifica se o modo de comparação tá funcionando
    ciclo++;
    if(y==0) {
    	//!settando a saida do timer pra inativo
        htim->Instance->CCMR1&= ~TIM_CCMR1_OC1M;
        htim->Instance->CCMR1|= TIM_OCMODE_INACTIVE;
        //!os ifs abaixo são responsáveis pela largura do pulso na rampa de descida
        if (htim->Instance->CCR1<inicioDePulso+1) {
            if((htim->Instance->CCR1+500)<25450) {
                htim->Instance->CCR1+=500;
            } else if((htim->Instance->CCR1+400)<25450) {
                htim->Instance->CCR1+=400;

            } else if((htim->Instance->CCR1+300)<25450) {
                htim->Instance->CCR1+=300;

            } else if((htim->Instance->CCR1+200)<25450) {
                htim->Instance->CCR1+=200;

            } else if((htim->Instance->CCR1+100)<25450) {
                htim->Instance->CCR1+=100;

            } else if((htim->Instance->CCR1+50)<25450) {
                htim->Instance->CCR1+=50;

            } else if((htim->Instance->CCR1+200)<25450) {
                htim->Instance->CCR1+=20;
            }

        } else {
            htim->Instance->CCR1=inicioDePulso;

        }
    }
    if(w==1) {
        w=0;
        htim->Instance->CCR1+=10;
    }
    if(y==1) {
    	//!settando a saida do timer pra inativa
        htim->Instance->CCMR1&= ~TIM_CCMR1_OC1M;
        htim->Instance->CCMR1|= TIM_OCMODE_INACTIVE;
        //!os ifs abaixo servem para definir a largura de pulso da rampa de subida
        if (htim->Instance->CCR1<(inicioDePulso+tempo2+1)) {
            if((htim->Instance->CCR1+500)<25450) {
                htim->Instance->CCR1+=500;
            } else if((htim->Instance->CCR1+400)<25450) {
                htim->Instance->CCR1+=400;

            } else if((htim->Instance->CCR1+300)<25450) {
                htim->Instance->CCR1+=300;

            } else if((htim->Instance->CCR1+200)<25450) {
                htim->Instance->CCR1+=200;

            } else if((htim->Instance->CCR1+100)<25450) {
                htim->Instance->CCR1+=100;

            } else if((htim->Instance->CCR1+50)<25450) {
                htim->Instance->CCR1+=50;

            } else if((htim->Instance->CCR1+200)<25450) {
                htim->Instance->CCR1+=20;
            }

        } else {
            htim->Instance->CCR1=inicioDePulso;

        }
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
    	 //!temporiza as threads
        HAL_IncTick();
    }
    if (htim->Instance==TIM3) {
    	//!togga um pino pra conferir se o trigger do timer tá sincronizado com a rede
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

        if(y==0) {
            descendo = 1;
            //!setta a saida do timer pra ativo
            htim->Instance->CCMR1&= ~TIM_CCMR1_OC1M;
            htim->Instance->CCMR1|= TIM_OCMODE_ACTIVE;
            //!desaciona o relé só pra garantir
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
            inicioDePulso = (int)x;
            //!faz o incremento pra posição do pulso
            x+=tempo1aux;
            //!valor máximo pra posição do pulso antes de terminar tudo
            if(x>=23000) {
                descendo=0;
                subindo=0;
                w=1;
                z=zaux;
                HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
                HAL_TIM_Base_Stop_IT(&htim3);
            }
        }

        if(y==1) {
            subindo = 1;
            //!setta a saida do timer pra ativo
            htim->Instance->CCMR1&= ~TIM_CCMR1_OC1M;
            htim->Instance->CCMR1|= TIM_OCMODE_ACTIVE;
            w=0;
            inicioDePulso = (int)z;
            //!verifica se pode mexer na posição do pulso caso corrente < 150%Inominal
            if(centoecinquenta==0) {
                z-=tempo2;
            }
            //!valor máximo pro pulso antes de acionar o relé de bypass
            if(z<(tempo2+2700)) {
                subindo=0;
                descendo=0;
                x=xaux;
                HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
                HAL_TIM_Base_Stop_IT(&htim3);
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
            }
        }
    }
}
//!interrupção chamada sempre que receber algo pela serial

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim3);

    //!switch pro últiom valor do vetor de recebimento
    switch(receive[2]) {
    //!se o ultimo numero for 0, os dois anteriores são o tempo de descida, ai faz os cálculos e manda o motor começar a descida
    case '0':
        tempo1=(receive[0]-48)*10;
        tempo1+=(receive[1]-48);
        tempo1 = tempo1/0.0083;
        tempo1 = 25000/tempo1;
        tempo1aux = tempo1;
        inicioDePulso=x;
        led2=0;
        led3=0;
        led1=0;
        descendo=1;
        y=0;
        HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
        HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
        //!aqui é pra ver se o motor tá no meio da rampa de subida, ai ele começa a descer do ponto em que está, inves de começar do zero
        if(subindo==1) {
            subindo=0;
            x=z;
        } else {
            x=xaux;
        }
        break;
        //!se for 1, os dois anteriores são o tempo de subida, aí faz os cálculos e manda o motor começar a subir
    case '1':
        led2=0;
        led3=0;
        led1=0;
        tempo2=(receive[0]-48)*10;
        tempo2+=(receive[1]-48);
        tempo2 = tempo2/0.0083;
        tempo2 = 21000/tempo2;
        tempo2aux=tempo2;
        subindo=1;
        HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
        HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
        y=1;
        HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
        //!aqui é pra ver se o motor tá no meio da rampa de descida, ai ele começa a subir do ponto em que está, inves de começar do zero
        if(descendo == 1 ) {
            descendo=0;
            z=x;
        } else {
            z=zaux;
        }
        break;
        //!se for 2 é emergencia, ai vai pras threads que desligam tudo
    case '2':
        emergencia=1;
        break;

    }
    w=1;

}
//!interrupção chamada sempre que o ADC termina uma conversão
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	 //!converte o valor em bits pra volts
	valor=value*0.0008056640625;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

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
    MX_DMA_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();

    /* USER CODE BEGIN 2 */
    //!inicia o relé desativado
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

    zaux=z;
    //!inicia timer 2
    HAL_TIM_Base_Start(&htim2);
    //!inicia conversão AD
    HAL_ADC_Start_DMA(&hadc1, &value, 4);
    //!inicia aquisição de dados via serial
    HAL_UART_Receive_DMA(&huart2, &receive, 3);



    /* USER CODE END 2 */

    /* Create the mutex(es) */
    /* definition and creation of myMutex01 */
    //!mutexes criadas para estabilizar as threads sem que uma interfira na outra, mas acabou não sendo necessário
    osMutexDef(myMutex01);
    myMutex01Handle = osMutexCreate(osMutex(myMutex01));

    /* definition and creation of myMutex02 */
    //!mutexes criadas para estabilizar as threads sem que uma interfira na outra, mas acabou não sendo necessário
    osMutexDef(myMutex02);
    myMutex02Handle = osMutexCreate(osMutex(myMutex02));

    /* definition and creation of myMutex03 */
    //!mutexes criadas para estabilizar as threads sem que uma interfira na outra, mas acabou não sendo necessário
    osMutexDef(myMutex03);
    myMutex03Handle = osMutexCreate(osMutex(myMutex03));

    /* definition and creation of myMutex04 */
    //!mutexes criadas para estabilizar as threads sem que uma interfira na outra, mas acabou não sendo necessário
    osMutexDef(myMutex04);
    myMutex04Handle = osMutexCreate(osMutex(myMutex04));

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
    /* definition and creation of Emergencia */
    //!iniciando task de emergencia
    osThreadDef(Emergencia,Emergencia_Task, osPriorityHigh, 0, 128);
    EmergenciaHandle = osThreadCreate(osThread(Emergencia), NULL);


    /* definition and creation of Corrente */
    //!iniciando task de corrente
    osThreadDef(Corrente, Corrente_Task, osPriorityNormal, 0, 128);
    CorrenteHandle = osThreadCreate(osThread(Corrente), NULL);

    /* definition and creation of Leds */
    //!iniciando task dos leds (kkkkkk)
    osThreadDef(Leds, Leds_Task, osPriorityIdle, 0, 128);
    LedsHandle = osThreadCreate(osThread(Leds), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */


    /* Start scheduler */
    //! iniciando configurações de kernel
    osKernelStart();
    //!setta o relé pra zero novamente só pra garantir
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);





    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void) //configurações do sistema
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) //configurações adc1
{

    ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM2 init function */
static void MX_TIM2_Init(void) //configurações timer2
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 106;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 6600;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM3 init function */
static void MX_TIM3_Init(void) //configurações timer3
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 26;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 26923;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_ETRF;
    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
    sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    sSlaveConfig.TriggerFilter = 15;
    if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim3);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) //configurações usart2
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) //configurações direct memory access
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    /* DMA1_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void) //configurações dos pinos de entrada e saida
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PC0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PC1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* Emergencia_Task function */
//!task de emergencia
void Emergencia(void const * argument)
{

    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {
    	//!verifica se emergencia foi pressionado, desliga tudo e reseta todos os valores
        if(emergencia==1){
            led2=0;
            led3=0;
            led1=0;
            z=24500,
            x = 150;
            tempo2aux=0;
            tempo1aux=0;
            descendo=0;
            subindo=0;
            tempo2=0; tempo1=0;
            HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
            HAL_TIM_Base_Stop_IT(&htim3);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
            emergencia=0;
        }
        osDelay(1);
    }
    /* USER CODE END 5 */
}


/* Corrente_Task function */
//!task da corrente
void Corrente_Task(void const * argument)
{
    /* USER CODE BEGIN Corrente_Task */
    /* Infinite loop */
    for(;;)
    {
    	//!verifica o nivel pra 200%
        if(valor>=3.0) {
            contagem++;
            //!verifica se aconteceu mtas vezes seguidas pra garantir que nao foi pico de corrente ou ruido
            if(contagem>850) {
                contagem=0;
                y=2;
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
                HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
                HAL_TIM_Base_Stop_IT(&htim3);
                led1=1;
            }
        }
        else if(contagem>10) {
            contagem=0;

        }
        //!verifica 150%
        if(valor>=2.3) {
            contagem2++;
            //!verifica se aconteceu mtas vezes seguidas pra garantir que nao foi pico de corrente ou ruido
            if(contagem2>850) {
                contagem2=0;
                centoecinquenta=1; //!flag que faz os pulsos pararem de incrementar, ficando parados no meio da rampa
                led2=1;
            }
        }
        else if(contagem2>10) {
            contagem2=0;
            centoecinquenta=0;
            led2=0;
        }
        //!verifica a queda de energia
        if((valor<=0.12&&subindo==1)||(valor<=0.12&&descendo==1)) {
            contagem3++;
            //!verifica se aconteceu mtas vezes seguidas pra garantir que nao foi pico de corrente ou ruido
            if(contagem3>50) {
                subindo=0;
                descendo=0;
                y=2;
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
                HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
                HAL_TIM_Base_Stop_IT(&htim3);
                contagem3=0;
                led3=1;
            }

        }
        else if(contagem3>10) {
            contagem3=0;
            led3=0;
        }
        osDelay(1);
    }



    /* USER CODE END Corrente_Task */
}

/* Leds_Task function */
//!task leds
void Leds_Task(void const * argument)
{
    /* USER CODE BEGIN Leds_Task */
    /* Infinite loop */
    for(;;)
    {
        if(led1==1) {
        	//!acende o led dos 200%
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            led3=0;
            led2=0;
            led1=1;
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        if(led2==1) {
        	//!acende o led dos 150%
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
            led3=0;
            led2=1;
            led1=0;
        } else {
            centoecinquenta=0;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        }

        if(led3==1) {
        	//!acende no corte de energia
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
            led3=1;
            led2=0;
            led1=0;
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        }
        osDelay(1);
    }
    /* USER CODE END Leds_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
