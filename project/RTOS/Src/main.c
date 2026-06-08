/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    26-February-2014
  * @brief   This sample code shows how to use STM32F4xx TIM HAL API to generate
  *          4 signals in PWM.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Output
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
#define PERIOD_VALUE 0xFFFF /* Period Value  */
#define PULSE1_VALUE 0xFFFF /* Capture Compare 1 Value  */
#define PULSE2_VALUE 900    /* Capture Compare 2 Value  */
#define PULSE3_VALUE 600    /* Capture Compare 3 Value  */
#define PULSE4_VALUE 450    /* Capture Compare 4 Value  */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef TimHandle1, TimHandle2, TimHandle3, TimHandle4;
TIM_IC_InitTypeDef sICConfig;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig1, sConfig2, sConfig3;

/* Counter Prescaler value */
uint32_t uwPrescalerValue = 0;
uint16_t motorInterrupt1 = 0;
uint16_t motorInterrupt2 = 0;

uint8_t encoder_right = READY;
uint8_t encoder_left = READY;

/* Captured Values */
uint32_t uwIC2Value1 = 0;
uint32_t uwIC2Value2 = 0;
uint32_t uwDiffCapture1 = 0;

uint32_t uwIC2Value3 = 0;
uint32_t uwIC2Value4 = 0;
uint32_t uwDiffCapture2 = 0;

uint32_t uwIC2Value5 = 0;
uint32_t uwIC2Value6 = 0;
uint32_t uwDiffCapture3 = 0;

uint32_t uwFrequency = 0;

/* ADC handler declaration */
ADC_HandleTypeDef AdcHandle1, AdcHandle2, AdcHandle3;
ADC_ChannelConfTypeDef adcConfig1, adcConfig2, adcConfig3;
ADC_ChannelConfTypeDef sConfig;

/* Variable used to get converted value */
__IO uint32_t uhADCxRight;
__IO uint32_t uhADCxForward;
__IO uint32_t uhADCxLeft;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);
void Motor_Speed_Up_Config(void);
void Motor_Speed_Down_Config(void);
static void EXTILine_Config(void);
static void Error_Handler(void);
/* Private functions ---------------------------------------------------------*/

extern UART_HandleTypeDef UartHandle1, UartHandle2;

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

/* -------------------------------------------------------------------------- */
/* Queue 메시지 정의                                                           */
/* -------------------------------------------------------------------------- */

/* IR_Sensor -> Motor_control: 최신 좌우 적외선 감지 결과 */
typedef struct
{
    uint8_t leftDetected;
    uint8_t rightDetected;
} IRSensorMessage;

/*
  * Detect_obstacle -> Motor_control:
  * 최신 초음파 장애물 상태와 유지되는 경로 복귀 요청을 전달
  */
typedef struct
{
    uint8_t frontBlocked;
    uint8_t leftBlocked;
    uint8_t rightBlocked;
    uint8_t requestLeftReturn;
    uint8_t requestRightReturn;
} NavigationMessage;

/*
  * Motor_control -> Detect_obstacle:
  * 제어 명령을 전달
  */
typedef enum
{
    NAV_CMD_RETURN_START = 0,
    NAV_CMD_RETURN_END,
    NAV_CMD_ACK_LEFT_RETURN,
    NAV_CMD_ACK_RIGHT_RETURN,
    NAV_CMD_RESET_TRACKING
} NavigationCommand;

static xQueueHandle irSensorQueue = NULL;
static xQueueHandle navigationQueue = NULL;
static xQueueHandle navigationCommandQueue = NULL;

#define NAV_COMMAND_QUEUE_LENGTH 8U

/*
  * 스케줄러를 시작하기 전에 한 번 호출해야 한다.
  * 모든 Queue 생성에 성공한 경우에만 pdPASS를 반환한다.
  */
portBASE_TYPE Navigation_Queue_Init(void)
{
    // 항상 가장 최신 값을 사용하기 위해 길이를 1로 설정함
    irSensorQueue = xQueueCreate(1U, sizeof(IRSensorMessage));
    navigationQueue = xQueueCreate(1U, sizeof(NavigationMessage));
    navigationCommandQueue = xQueueCreate(NAV_COMMAND_QUEUE_LENGTH, sizeof(NavigationCommand));

    if ((irSensorQueue == NULL) || (navigationQueue == NULL) || (navigationCommandQueue == NULL))
    {
        return pdFAIL;
    }

    {
        const IRSensorMessage initialIR = {0U, 0U};
        const NavigationMessage initialNavigation = {0U, 0U, 0U, 0U, 0U};

        xQueueOverwrite(irSensorQueue, &initialIR);
        xQueueOverwrite(navigationQueue, &initialNavigation);
    }

    return pdPASS;
}

static void Send_Navigation_Command(NavigationCommand command)
{
    (void)xQueueSend(navigationCommandQueue, &command, 0U);
}

/* -------------------------------------------------------------------------- */
/* 모터 제어 로직이 소유하는 주행 상태                                         */
/* -------------------------------------------------------------------------- */

volatile uint32_t rightAngle = 0;
volatile uint32_t leftAngle = 0;

static uint8_t isir = 0;

// IR_THRESHOLD 700 부딪힘. 600/650 도리도리 675 살짝 부딪힘. 670 도 긁음
#define IR_THRESHOLD 660
#define FRONT_TOL_MS 700
#define SIDE_MIN_MS 700

typedef enum
{
    HEADING_FRONT = 0,
    HEADING_LEFT,
    HEADING_RIGHT,
    HEADING_UNKNOWN
} HeadingState;

int32_t Get_Net_Angle(void)
{
    return (int32_t)rightAngle - (int32_t)leftAngle;
}

HeadingState Get_Heading_State(void)
{
    int32_t net = Get_Net_Angle();

    // LED: HeadingState 디버깅용도
    if (net >= SIDE_MIN_MS)
    {
        BSP_LED_On(LED1);
        BSP_LED_Off(LED2);
        BSP_LED_Off(LED3);
        BSP_LED_Off(LED4);
        return HEADING_RIGHT;
    }
    else if (net <= -SIDE_MIN_MS)
    {
        BSP_LED_On(LED4);
        BSP_LED_Off(LED2);
        BSP_LED_Off(LED3);
        BSP_LED_Off(LED1);
        return HEADING_LEFT;
    }
    else if ((net <= FRONT_TOL_MS) && (net >= -FRONT_TOL_MS))
    {
        BSP_LED_Off(LED1);
        BSP_LED_On(LED2);
        BSP_LED_On(LED3);
        BSP_LED_Off(LED4);
        return HEADING_FRONT;
    }
    else
    {
        return HEADING_UNKNOWN;
    }
}

void Turn_Right_Angle(uint32_t delayMs)
{
    uint32_t startTick;
    uint32_t endTick;
    uint32_t realDelay;

    Motor_Stop();
    osDelay(20);

    startTick = osKernelSysTick();

    Motor_Right();
    osDelay(delayMs);

    Motor_Stop();
    endTick = osKernelSysTick();
    realDelay = endTick - startTick;

    if (isir == 1U)
    {
        realDelay *= 0.24;
    }

    rightAngle += realDelay;
}

void Turn_Left_Angle(uint32_t delayMs)
{
    uint32_t startTick;
    uint32_t endTick;
    uint32_t realDelay;

    Motor_Stop();
    osDelay(20);

    startTick = osKernelSysTick();

    Motor_Left();
    osDelay(delayMs);

    Motor_Stop();
    endTick = osKernelSysTick();
    realDelay = endTick - startTick;

    if (isir == 1U)
    {
        realDelay *= 0.24;
    }

    leftAngle += realDelay;
}

void Turn_By_Angle_Balance(uint32_t delayMs)
{
    if (rightAngle > leftAngle)
    {
        Turn_Left_Angle(delayMs);
    }
    else if (leftAngle > rightAngle)
    {
        Turn_Right_Angle(delayMs);
    }
    else
    {
        Turn_Right_Angle(delayMs);
    }
}

/* -------------------------------------------------------------------------- */
/* 초음파 센서 및 코너 복귀 판단 태스크                                        */
/* -------------------------------------------------------------------------- */

void Detect_obstacle()
{
    NavigationMessage navigation = {0U, 0U, 0U, 0U, 0U};
    NavigationCommand command;

    /*
      * 아래 변수들은 Detect_obstacle 태스크만 소유한다.
      * Motor_control은 다른 태스크의 변수를 직접 수정하지 않고
      * navigationCommandQueue를 통해 초기화를 요청한다.
      */
    uint8_t sawLeftWallWhileRight = 0U;
    uint8_t sawRightWallWhileLeft = 0U;
    uint8_t leftOpenCount = 0U;
    uint8_t rightOpenCount = 0U;
    uint8_t returnHandling = 0U;

    osDelay(200);

    for (;;)
    {
        osDelay(50);

        /*
          * 다음 코너 판단을 수행하기 전에 대기 중인 모터 제어 명령을 모두 적용함.
          */
        while (xQueueReceive(navigationCommandQueue, &command, 0U) == pdPASS)
        {
            switch (command)
            {
            case NAV_CMD_RETURN_START:
                returnHandling = 1U;
                break;

            case NAV_CMD_RETURN_END:
                returnHandling = 0U;
                break;

            case NAV_CMD_ACK_LEFT_RETURN:
                navigation.requestLeftReturn = 0U;
                break;

            case NAV_CMD_ACK_RIGHT_RETURN:
                navigation.requestRightReturn = 0U;
                break;

            case NAV_CMD_RESET_TRACKING:
                sawLeftWallWhileRight = 0U;
                sawRightWallWhileLeft = 0U;
                leftOpenCount = 0U;
                rightOpenCount = 0U;
                break;

            default:
                break;
            }
        }

        navigation.frontBlocked =
            ((uwDiffCapture2 != 0U) && ((uwDiffCapture2 / 58U) < 15U)) ? 1U : 0U;
        navigation.rightBlocked =
            ((uwDiffCapture1 != 0U) && ((uwDiffCapture1 / 58U) < 15U)) ? 1U : 0U;
        navigation.leftBlocked =
            ((uwDiffCapture3 != 0U) && ((uwDiffCapture3 / 58U) < 15U)) ? 1U : 0U;

        if ((navigation.frontBlocked == 0U) && (returnHandling == 0U))
        {
            HeadingState heading = Get_Heading_State();

            /*
              * 오른쪽 방향을 바라보며 이동하는 경우:
              * 왼쪽 벽을 관찰했다는 사실을 기억하고, 이후 벽이 연속 두 번
              * 사라진 경우에만 좌회전 복귀를 요청한다.
              */
            if (heading == HEADING_RIGHT)
            {
                if (navigation.leftBlocked == 1U)
                {
                    sawLeftWallWhileRight = 1U;
                    leftOpenCount = 0U;
                }
                else if ((navigation.leftBlocked == 0U) && (sawLeftWallWhileRight == 1U))
                {
                    leftOpenCount++;

                    if (leftOpenCount >= 2U)
                    {
                        /*
                          * Motor_control이 요청을 확인할 때까지 복귀 요청을
                          * 유지한다. 모터 태스크가 다른 회전을 수행하는 동안
                          * 한 주기만 발생한 Queue 요청이 사라지는 것을 방지한다.
                          */
                        navigation.requestLeftReturn = 1U;
                        leftOpenCount = 0U;
                    }
                }
            }
            /*
              * 왼쪽 방향을 바라보며 이동하는 경우:
              * 오른쪽 벽에 대해 좌우가 반대인 동일한 복귀 로직을 적용한다.
              */
            else if (heading == HEADING_LEFT)
            {
                if (navigation.rightBlocked == 1U)
                {
                    sawRightWallWhileLeft = 1U;
                    rightOpenCount = 0U;
                }
                else if ((navigation.rightBlocked == 0U) && (sawRightWallWhileLeft == 1U))
                {
                    rightOpenCount++;

                    if (rightOpenCount >= 2U)
                    {
                        navigation.requestRightReturn = 1U;
                        rightOpenCount = 0U;
                    }
                }
            }
        }

        /*
          * 길이 1 Queue와 overwrite를 사용하여 항상 최신 상태만 유지한다.
          * 관련된 다섯 플래그는 각각 갱신되는 전역 변수가 아니라 하나의
          * 일관된 스냅샷으로 Motor_control에 전달된다.
          */
        xQueueOverwrite(navigationQueue, &navigation);
    }
}

/* -------------------------------------------------------------------------- */
/* 모터 제어 태스크                                                            */
/* -------------------------------------------------------------------------- */

void Motor_control()
{
    IRSensorMessage irState = {0U, 0U};
    NavigationMessage navigation = {0U, 0U, 0U, 0U, 0U};

    osDelay(200);
    Motor_Forward();

    for (;;)
    {
        /*
          * Peek는 Queue에서 메시지를 제거하지 않고 최신 스냅샷을 읽는다.
          * 새로운 메시지가 없으면 마지막으로 수신한 유효 센서 상태를 유지하므로
          * 기존 공유 플래그 방식과 같은 동작이 된다.
          */
        (void)xQueuePeek(irSensorQueue, &irState, 0U);
        (void)xQueuePeek(navigationQueue, &navigation, 0U);

        if ((irState.leftDetected == 1U) && (irState.rightDetected == 0U))
        {
            isir = 1U;
            Turn_Right_Angle(50);
            isir = 0U;
            continue;
        }
        else if ((irState.rightDetected == 1U) && (irState.leftDetected == 0U))
        {
            isir = 1U;
            Turn_Left_Angle(50);
            isir = 0U;
            continue;
        }

        if (navigation.frontBlocked == 0U)
        {
            if (navigation.requestLeftReturn == 1U)
            {
                Send_Navigation_Command(NAV_CMD_RETURN_START);
                Send_Navigation_Command(NAV_CMD_ACK_LEFT_RETURN);

                Motor_Forward();
                osDelay(400);

                Turn_Left_Angle(870);

                Send_Navigation_Command(NAV_CMD_RESET_TRACKING);
                Send_Navigation_Command(NAV_CMD_RETURN_END);

                navigation.requestLeftReturn = 0U;

                Motor_Forward();
                continue;
            }

            if (navigation.requestRightReturn == 1U)
            {
                Send_Navigation_Command(NAV_CMD_RETURN_START);
                Send_Navigation_Command(NAV_CMD_ACK_RIGHT_RETURN);

                Motor_Forward();
                osDelay(400);

                Turn_Right_Angle(870);

                Send_Navigation_Command(NAV_CMD_RESET_TRACKING);
                Send_Navigation_Command(NAV_CMD_RETURN_END);

                navigation.requestRightReturn = 0U;

                Motor_Forward();
                continue;
            }

            Motor_Forward();
            continue;
        }
        else if (navigation.frontBlocked == 1U)
        {
            if ((navigation.leftBlocked == 1U) && (navigation.rightBlocked == 1U))
            {
                Turn_By_Angle_Balance(1200);
            }
            else if (navigation.leftBlocked == 1U)
            {
                Turn_Right_Angle(870);
                Send_Navigation_Command(NAV_CMD_RESET_TRACKING);
            }
            else if (navigation.rightBlocked == 1U)
            {
                Turn_Left_Angle(870);
                Send_Navigation_Command(NAV_CMD_RESET_TRACKING);
            }
            else
            {
                Turn_By_Angle_Balance(870);
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/* 적외선 센서 태스크                                                          */
/* -------------------------------------------------------------------------- */

void IR_Sensor()
{
    IRSensorMessage irState = {0U, 0U};

    osDelay(200);

    for (;;)
    {
        HAL_ADC_Start(&AdcHandle1);
        HAL_ADC_PollForConversion(&AdcHandle1, 0xFF);
        uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);

        if (uhADCxLeft > 2000U)
        {
            uhADCxLeft = 2000U;
        }
        else if (uhADCxLeft < 100U)
        {
            uhADCxLeft = 100U;
        }

        irState.leftDetected = (uhADCxLeft > IR_THRESHOLD) ? 1U : 0U;

        HAL_ADC_Start(&AdcHandle2);
        HAL_ADC_PollForConversion(&AdcHandle2, 0xFF);
        uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);

        if (uhADCxRight > 2000U)
        {
            uhADCxRight = 2000U;
        }
        else if (uhADCxRight < 100U)
        {
            uhADCxRight = 100U;
        }

        irState.rightDetected = (uhADCxRight > IR_THRESHOLD) ? 1U : 0U;

        xQueueOverwrite(irSensorQueue, &irState);

        osDelay(10);
    }
}

/***************************************************************************/
int main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* STM32F4xx HAL library initialization:
      - Configure the Flash prefetch, instruction and Data caches
      - Configure the Systick to generate an interrupt each 1 msec
      - Set NVIC Group Priority to 4
      - Global MSP (MCU Support Package) initialization
    */
    HAL_Init();

    /* Configure the system clock to have a system clock = 180 Mhz */
    SystemClock_Config();

    BSP_COM1_Init();

    /************************************** 모터 설정 시작 **************************************/
    uwPrescalerValue = (SystemCoreClock / 2) / 1000000;

    // PB2 모터 드라이버 Enable 핀 GPIO 초기화
    __GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // MC_EN(PB2) 출력 High 설정

    sConfig1.OCMode = TIM_OCMODE_PWM1;
    sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig1.Pulse = 20000;

    TimHandle1.Instance = TIM8;
    TimHandle1.Init.Prescaler = uwPrescalerValue;
    TimHandle1.Init.Period = 20000;
    TimHandle1.Init.ClockDivision = 0;
    TimHandle1.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle1);

    HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_2);

    /* Common configuration for all channels */
    sConfig2.OCMode = TIM_OCMODE_PWM1;
    sConfig2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig2.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig2.Pulse = 20000;

    TimHandle2.Instance = TIM4;
    TimHandle2.Init.Prescaler = uwPrescalerValue;
    TimHandle2.Init.Period = 20000;
    TimHandle2.Init.ClockDivision = 0;
    TimHandle2.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle2);

    HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_2);

    EXTILine_Config(); // Encoder Interrupt Setting
    /************************************** 모터 설정 종료 **************************************/

    /************************************** 초음파 설정 시작 **************************************/
    uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;

    /* Set TIMx instance */
    TimHandle3.Instance = TIM3;

    TimHandle3.Init.Period = 0xFFFF;
    TimHandle3.Init.Prescaler = uwPrescalerValue;
    TimHandle3.Init.ClockDivision = 0;
    TimHandle3.Init.CounterMode = TIM_COUNTERMODE_UP;

    if (HAL_TIM_IC_Init(&TimHandle3) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure the Input Capture of channel 2 */
    sICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
    sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
    sICConfig.ICFilter = 0;

    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_3);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_4);

    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_4);

    uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;

    TimHandle4.Instance = TIM10;

    TimHandle4.Init.Prescaler = uwPrescalerValue;
    TimHandle4.Init.Period = 0xFFFF;
    TimHandle4.Init.ClockDivision = 0;
    TimHandle4.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle4);

    /* Common configuration for all channels */
    sConfig3.OCMode = TIM_OCMODE_PWM1;
    sConfig3.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig3.OCFastMode = TIM_OCFAST_DISABLE;

    /* Set the pulse value for channel 1 */
    sConfig3.Pulse = 2;
    HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig3, TIM_CHANNEL_1);

    /* Start channel 3 */
    HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);
    /************************************** 초음파 설정 종료 **************************************/

    /************************************** 적외선 센서 설정 시작 **************************************/

    AdcHandle1.Instance = ADC3; // ADC3 설정

    AdcHandle1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle1.Init.Resolution = ADC_RESOLUTION12b;
    AdcHandle1.Init.ScanConvMode = DISABLE;
    // 변환 모드 설정
    AdcHandle1.Init.ContinuousConvMode = DISABLE;
    AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle1.Init.NbrOfDiscConversion = 0;
    AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    AdcHandle1.Init.NbrOfConversion = 1;
    // DMA (Direct Memory Access)
    AdcHandle1.Init.DMAContinuousRequests = DISABLE;
    AdcHandle1.Init.EOCSelection = DISABLE;

    HAL_ADC_Init(&AdcHandle1); // ADC initialized

    adcConfig1.Channel = ADC_CHANNEL_11; // 채널 설정
    adcConfig1.Rank = 1;
    adcConfig1.SamplingTime = ADC_SAMPLETIME_480CYCLES; // 샘플링 시간 설정
    adcConfig1.Offset = 0;

    HAL_ADC_ConfigChannel(&AdcHandle1, &adcConfig1);

    AdcHandle2.Instance = ADC2; // ADC2 설정

    AdcHandle2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle2.Init.Resolution = ADC_RESOLUTION12b;
    AdcHandle2.Init.ScanConvMode = DISABLE;
    AdcHandle2.Init.ContinuousConvMode = DISABLE;
    AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle2.Init.NbrOfDiscConversion = 0;
    AdcHandle2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    AdcHandle2.Init.NbrOfConversion = 1;
    AdcHandle2.Init.DMAContinuousRequests = DISABLE;
    AdcHandle2.Init.EOCSelection = DISABLE;

    HAL_ADC_Init(&AdcHandle2);

    adcConfig2.Channel = ADC_CHANNEL_14;
    adcConfig2.Rank = 1;
    adcConfig2.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig2.Offset = 0;

    HAL_ADC_ConfigChannel(&AdcHandle2, &adcConfig2);

    AdcHandle3.Instance = ADC1; // ADC1 설정

    AdcHandle3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle3.Init.Resolution = ADC_RESOLUTION12b;
    AdcHandle3.Init.ScanConvMode = DISABLE;
    AdcHandle3.Init.ContinuousConvMode = DISABLE;
    AdcHandle3.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle3.Init.NbrOfDiscConversion = 0;
    AdcHandle3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    AdcHandle3.Init.NbrOfConversion = 1;
    AdcHandle3.Init.DMAContinuousRequests = DISABLE;
    AdcHandle3.Init.EOCSelection = DISABLE;

    HAL_ADC_Init(&AdcHandle3);

    adcConfig3.Channel = ADC_CHANNEL_15;
    adcConfig3.Rank = 1;
    adcConfig3.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig3.Offset = 0;
    HAL_ADC_ConfigChannel(&AdcHandle3, &adcConfig3);
    /************************************** 적외선 센서 설정 종료 **************************************/

    if (Navigation_Queue_Init() != pdPASS)
    {
        Error_Handler();
    }

    xTaskCreate(IR_Sensor, "IR_Sensor", 1000, NULL, 1, NULL);
    xTaskCreate(Detect_obstacle, "Detect_obstacle", 1000, NULL, 1, NULL);
    xTaskCreate(Motor_control, "Motor_control", 1000, NULL, 1, NULL);

    vTaskStartScheduler();

    /* Infinite loop */
    while (1)
    {
    }
}

void Motor_Forward(void)
{
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Backward(void)
{
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}

void Motor_Left(void)
{
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Right(void)
{
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}

void Motor_Stop(void)
{
    HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Speed_Up_Config(void)
{
    sConfig1.Pulse = sConfig1.Pulse + 100;
    sConfig2.Pulse = sConfig2.Pulse + 100;
    TIM8->CCR1 = sConfig1.Pulse;
    TIM8->CCR2 = sConfig1.Pulse;
    TIM4->CCR1 = sConfig2.Pulse;
    TIM4->CCR2 = sConfig2.Pulse;
}

void Motor_Speed_Down_Config(void)
{
    sConfig1.Pulse = sConfig1.Pulse - 100;
    sConfig2.Pulse = sConfig2.Pulse - 100;
    TIM8->CCR1 = sConfig1.Pulse;
    TIM8->CCR2 = sConfig1.Pulse;
    TIM4->CCR1 = sConfig2.Pulse;
    TIM4->CCR2 = sConfig2.Pulse;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
    clocked below the maximum system frequency, to update the voltage scaling value
    regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Activate the Over-Drive mode */
    HAL_PWREx_ActivateOverDrive();

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
    clocks dividers */
    RCC_ClkInitStruct.ClockType =
        (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}

#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case GPIO_PIN_15:
        encoder_right = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
        if (encoder_right == 0)
        {
            motorInterrupt1++;
            encoder_right = READY;
        }
        else if (encoder_right == 1)
        {
            motorInterrupt1--;
            encoder_right = READY;
        }
        break;

    case GPIO_PIN_4:
        encoder_left = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
        if (encoder_left == 0)
        {
            motorInterrupt2++;
            encoder_left = READY;
        }
        else if (encoder_left == 1)
        {
            motorInterrupt2--;
            encoder_left = READY;
        }
        break;
    }
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  htim : hadc handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            if ((TIM3->CCER & TIM_CCER_CC2P) == 0)
            {
                /* Get the 1st Input Capture value */
                uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
                TIM3->CCER |= TIM_CCER_CC2P;
            }
            else if ((TIM3->CCER & TIM_CCER_CC2P) == TIM_CCER_CC2P)
            {
                /* Get the 2nd Input Capture value */
                uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

                /* Capture computation */
                if (uwIC2Value2 > uwIC2Value1)
                {
                    uwDiffCapture1 = (uwIC2Value2 - uwIC2Value1);
                }
                else if (uwIC2Value2 < uwIC2Value1)
                {
                    uwDiffCapture1 = ((0xFFFF - uwIC2Value1) + uwIC2Value2);
                }
                else
                {
                    uwDiffCapture1 = 0;
                }
                // printf("\r\n Value Right : %d cm", uwDiffCapture1 / 58);

                uwFrequency = (2 * HAL_RCC_GetPCLK1Freq()) / uwDiffCapture1;
                TIM3->CCER &= ~TIM_CCER_CC2P;
            }
        }

        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            if ((TIM3->CCER & TIM_CCER_CC3P) == 0)
            {
                /* Get the 1st Input Capture value */
                uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
                TIM3->CCER |= TIM_CCER_CC3P;
            }
            else if ((TIM3->CCER & TIM_CCER_CC3P) == TIM_CCER_CC3P)
            {
                /* Get the 2nd Input Capture value */
                uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

                /* Capture computation */
                if (uwIC2Value4 > uwIC2Value3)
                {
                    uwDiffCapture2 = (uwIC2Value4 - uwIC2Value3);
                }
                else if (uwIC2Value4 < uwIC2Value3)
                {
                    uwDiffCapture2 = ((0xFFFF - uwIC2Value3) + uwIC2Value4);
                }
                else
                {
                    uwDiffCapture2 = 0;
                }
                // printf("\r\n Value Forward : %d cm", uwDiffCapture2 / 58);

                uwFrequency = (2 * HAL_RCC_GetPCLK1Freq()) / uwDiffCapture2;
                TIM3->CCER &= ~TIM_CCER_CC3P;
            }
        }

        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            if ((TIM3->CCER & TIM_CCER_CC4P) == 0)
            {
                /* Get the 1st Input Capture value */
                uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                TIM3->CCER |= TIM_CCER_CC4P;
            }
            else if ((TIM3->CCER & TIM_CCER_CC4P) == TIM_CCER_CC4P)
            {
                /* Get the 2nd Input Capture value */
                uwIC2Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

                /* Capture computation */
                if (uwIC2Value6 > uwIC2Value5)
                {
                    uwDiffCapture3 = (uwIC2Value6 - uwIC2Value5);
                }
                else if (uwIC2Value6 < uwIC2Value5)
                {
                    uwDiffCapture3 = ((0xFFFF - uwIC2Value5) + uwIC2Value6);
                }
                else
                {
                    uwDiffCapture3 = 0;
                }
                // printf("\r\n Value Left: %d cm", uwDiffCapture3 / 58);

                uwFrequency = (2 * HAL_RCC_GetPCLK1Freq()) / uwDiffCapture3;
                TIM3->CCER &= ~TIM_CCER_CC4P;
            }
        }
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
    /* Turn LED3 on */
    BSP_LED_On(LED3);
    while (1)
    {
    }
}

/**
  * @brief  Configures EXTI Line (connected to PA15, PB3, PB4, PB5 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTILine_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOA clock */
    __GPIOA_CLK_ENABLE();

    /* Configure PA15 pin  */
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Enable and set EXTI Line15 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* Enable GPIOB clock */
    __GPIOB_CLK_ENABLE();

    /* Configure PB3, PB4, PB5 pin  */
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable and set EXTI Line4 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
