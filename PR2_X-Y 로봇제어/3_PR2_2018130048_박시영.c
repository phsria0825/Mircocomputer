////////////////////////////////////////////////////////////////////////////////
//************************************************
// PR2: X-Y Robot Control
// 제출자 : 2018130048_박시영
// 제출일 : 2022.12.11
// 과제개요 : PC에서 축,거리,속도를 받아서 모터의 펄스를 계산 후, 제어
//          USART4 : RPM제어 1.PC제어, 2.ADC제어
//          USART1 : PC에서의 명령어 수신
//          void Robot_Control(void) : 명령어 해석후 이동량, 펄스 계산
//          void make_Bar(int D)     : 좌표 표시 + 방향
//          void erase_Bar(int D)    : 좌표 표시 - 방향
//          TIM.1,8 : 계산된 펼스로 모터제어
//          ADC3   : 가변저항으로 RPM제어
////////////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

// USART1을 통하여 오믐 명령어들을 10진수로 푠현했을때를 정의
#define X_axis          88             // USART1에서 오는 X축 정의
#define Y_axis          89             // USART1에서 오는 Y축 정의
#define Gripper         71             // USART1에서 오는 그리퍼 정의
#define Speed           82             // USART1에서 오는 RPM 정의
#define N               (str[1] - 48)  // USART1에서 오는 데이터 정의

#define STEP_DEG	2       // step angle

void _GPIO_Init(void);
uint16_t KEY_Scan(void);
void TIMER1_OC_Init(void); //TIM1_CH4 CC 
void TIMER8_OC_Init(void); //TIM8_CH2 CC 
void TIMER3_Init(void);    //TIM3     UI

void DisplayInitScreen(void);
void DisDisplayDataScreen(void); // data 표시함수

// 통신관련 함수 정의
void USART1_Init(void);  // PC통신
void UART4_Init(void);   // 휴대폰 통신 (IOS)
void USART1_BRR_Configuration(uint32_t USART_BaudRate);  // PC통신속도
void USART4_BRR_Configuration(uint32_t USART_BaudRate);  // 휴대폰 통신 (IOS)속도
void SerialSendChar(uint8_t c);
void SerialSendString(char* s);
UINT8 no, CRdata;

void _ADC_Init(void);

// 로봇제어 및 디스플레이 함수정의
void Robot_Control(void);     // 값을받아서 로봇의 거리와 좌표들을 계산하는함수
void make_Bar(int D);         // 현재좌표와 막대그래프를 업데이트 (증가)
void erase_Bar(int D);        // 현재좌표와 막대그래프를 업데이트 (감소)

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

uint16 ADC_flag = 0;     // PC명령과 ADC변환을 정하는 플래그
int ADC_Value, Voltage;  // ADC값, 전압값

// PC에서 수신받는 명령어 변수
UINT16 USART1_INT_flag = 0;  // 
int CX = 0, CY = 0;
int Gx = 0, Gy = 0;
int Dx = 0, Dy = 0;
int TPX = 0, TPY = 0;
int CPX = 0, CPY = 0;
int RPM = 3;

// 축과 이동량 저장변수
int str[3];

int main(void)
{
    _GPIO_Init();
    LCD_Init();
    DelayMS(10);
    DisplayInitScreen();	// LCD 초기화면
    DisDisplayDataScreen(); // LCD에 초기 데이터 표시
    GPIOG->ODR &= 0xFF00;   // 초기값: LED0~7 Off
    USART1_Init();          // USART1  초기화
    UART4_Init();           // USART4  초기화
    _ADC_Init();            // ADC3 초기화

    TIMER3_Init();    //TIM3_UI (ADC)
    TIMER1_OC_Init(); //TIM1_CH4 CC (X축 모터 펄스)
    TIMER8_OC_Init(); //TIM8_CH2 CC (Y축 모터 펄스)
    while (1)
    {

    }

}

void TIMER3_Init(void)
{
    // Clock Enable : TIMER3
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	// TIMER3 Enable 

    // Assign 'Interrupt Period' and 'Output Pulse Period'
    TIM3->PSC = 840 - 1;	// Prescaler 84MHz/840 = 0.1MHz (10us)
    TIM3->ARR = 50000 - 1;	// Auto reload  : 10us * 5K = 500ms(period)

    // CR1 : Up counting
    TIM3->CR1 &= ~(1 << 4);	// DIR=0(Up counter)(reset state)
    TIM3->CR1 &= ~(1 << 1);	// UDIS=0(Update event Enabled): By one of following events
                //	- Counter Overflow/Underflow, 
                // 	- Setting the UG bit Set,
                //	- Update Generation through the slave mode controller 
    TIM3->CR1 &= ~(1 << 2);	// URS=0(Update event source Selection): one of following events
                //	- Counter Overflow/Underflow, 
                // 	- Setting the UG bit Set,
                //	- Update Generation through the slave mode controller 
    TIM3->CR1 &= ~(1 << 3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
    TIM3->CR1 &= ~(1 << 7);	// ARPE=0(ARR is NOT buffered) (reset state)
    TIM3->CR1 &= ~(3 << 8); 	// CKD(Clock division)=00(reset state)
    TIM3->CR1 &= ~(3 << 5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
                // Center-aligned mode: The counter counts Up and DOWN alternatively

    // Event & Interrup Enable : UI  
    TIM3->EGR |= (1 << 0);    // UG: Update generation    
    TIM3->DIER |= (1 << 0);   // 

    NVIC->ISER[0] |= (1 << 29);

    TIM3->CR1 |= (1 << 0);	// CEN: Enable the Tim3 Counter			
}

void TIM3_IRQHandler(void)      //RESET: 0
{
    TIM3->SR &= ~(1 << 0);	// UI Interrupt Claer
    if (ADC_flag == 1)       // USART4 에서 ADC_flag == 1일때
        ADC3->CR2 |= (1 << 30); // ADC SWSTART : ON
}

void _ADC_Init(void)
{
    // ADC3: PA1(pin 41) : 가변저항
    RCC->AHB1ENR |= (1 << 0);	                // (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
    GPIOA->MODER |= (3 << 2 * 1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE				
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;	// ENABLE ADC3 CLK (stm32f4xx.h 참조)

    ADC->CCR &= ~(0X1F << 0);		// MULTI[4:0]: ADC_Mode_Independent
    ADC->CCR |= (1 << 16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)

    ADC3->CR1 &= ~(3 << 24);		// RES[1:0]= 0x00 : 12bit Resolution
    ADC3->CR1 &= ~(1 << 8);		// SCAN=0 : ADC_ScanCovMode Disable
    ADC3->CR1 |= (1 << 5);		// EOCIE=1: Interrupt enable for EOC

    ADC3->CR2 &= ~(1 << 1);		// CONT=0: ADC_Continuous ConvMode Disable
    ADC1->CR2 &= ~(3 << 28);		// EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge_None
    ADC3->CR2 &= ~(1 << 11);		// ALIGN=0: ADC_DataAlign_Right
    ADC3->CR2 &= ~(1 << 10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

    ADC3->SQR1 &= ~(0xF << 20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
                    // 0b0000:1 conversion)

    ADC3->SQR3 |= (1 << 0);		// SQ1[4:0]=0b0001 : CH1
    ADC3->SMPR2 |= (0x7 << 3);	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)

    NVIC->ISER[0] |= (1 << 18);	// Enable ADC global Interrupt
    ADC3->CR2 |= (1 << 0);		// ADON: ADC ON
}

void ADC_IRQHandler(void)
{
    ADC3->SR &= ~(1 << 1);		                   // EOC flag clear

    ADC_Value = ADC3->DR;		                   // Reading ADC3 result
    Voltage = ADC_Value * (3.3 * 10) / 4095;         // 3.3 : 4095 =  Volatge : ADC_Value

        // 주어진 범위에 따라서 RPM 지정
    if (Voltage >= 0 && Voltage <= 8)
    {
        RPM = 1;
    }
    else if (Voltage >= 9 && Voltage <= 16)
    {
        RPM = 2;
    }
    else if (Voltage >= 17 && Voltage <= 24)
    {
        RPM = 3;
    }
    else if (Voltage >= 25 && Voltage <= 33)
    {
        RPM = 4;
    }

    LCD_SetTextColor(RGB_BLUE);    // 글자색 : 파란색
    LCD_DisplayChar(1, 11, 0x30 + RPM);
}

float fARR;
void TIMER1_OC_Init(void) //TIM1_CH4 CC 
{
    // PA11: TIM1_CH4
    // PA11을 출력설정하고 Alternate function(TIM1_CH4)으로 사용 선언
    RCC->AHB1ENR |= (1 << 0);      // RCC_AHB1ENR GPIOA Enable
    GPIOA->MODER |= (2 << 2 * 11);   // GPIOA PA11을 Output Alternate function mode               
    GPIOA->OSPEEDR |= (3 << 2 * 11);   // GPIOA PA11을 Output speed (100MHz High speed)
    GPIOA->OTYPER |= 0x00000000;   // GPIOA PA11을 Output type push-pull (reset state)
    GPIOA->PUPDR |= (1 << 2 * 11);   // GPIOA PA11을 Pull-up
    GPIOA->AFR[1] |= (1 << 12);     // Connect TIM1 pins(PA11) to AF1(TIM1/2)

    // Timerbase Mode
    RCC->APB2ENR |= (1 << 0);   // RCC_APB1ENR TIMER1 Enable

    TIM1->PSC = 168 - 1;        // Prescaler 168,000,000Hz/168= 1MHz (1us)

    fARR = 1000000 * (60 * STEP_DEG / 2) / (360 * RPM);  // us 단위 <pulse / sec>
    TIM1->ARR = fARR - 1;    // 주기 = 1us * fARR = fARR us

    TIM1->CR1 &= ~(1 << 4);   // DIR: Countermode = Upcounter (reset state)
    TIM1->CR1 &= ~(3 << 8);   // CKD: Clock division = 1 (reset state)
    TIM1->CR1 &= ~(3 << 5);    // CMS(Center-aligned mode Sel): No(reset state)

    TIM1->EGR |= (1 << 0);    // UG: Update generation 

    // Output/Compare Mode
    TIM1->CCER &= ~(1 << 12);   // CC4E: OC4 Active 
    TIM1->CCER &= ~(1 << 13);  // CC4P: OCPolarity_Active High

    TIM1->CCR4 = 10;   // TIM1_Pulse

        //TIM.1,8은 BDTR 15bit를 1로 enable을 해야 출력 입력기능을 한다
    TIM1->BDTR |= (1 << 15);  // main output enable

    TIM1->CCMR2 &= ~(3 << 8); // CC4S(CC1channel): Output 
    TIM1->CCMR2 &= ~(1 << 11); // OC4PE: Output Compare 4 preload disable
    TIM1->CCMR2 |= (3 << 12);   // OC4M: Output Compare 4 Mode : toggle

    TIM1->CR1 &= ~(1 << 7);   // ARPE: Auto reload preload disable
    TIM1->DIER |= (1 << 4);   // CC4IE: Enable the Tim1 CC4 interrupt

    NVIC->ISER[0] |= (1 << 27); // TIM1_CC
    TIM1->CR1 &= ~(1 << 0);   // CEN: Disable the Tim1 Counter 
}

void TIM1_CC_IRQHandler(void)      // X축 모터 구동
{
    if ((TIM1->SR & (1 << 4)) != RESET)	// CC4 interrupt flag 
    {
        TIM1->SR &= ~(1 << 4);	// CC4 Interrupt Claer
        CPX++;                  // X축 펄스 카운트

        // 현재 펄스수 표시
        LCD_SetTextColor(RGB_RED);	// 글자색 : RED
        LCD_DisplayChar(5, 4, 0x30 + CPX / 10);
        LCD_DisplayChar(5, 5, 0x30 + CPX % 10);

        GPIOG->ODR ^= (1 << 1);   // 모터구동 LED1

        if (CPX % 5 == 0)     // 펄스의 수가 5의 배수일때 마다 현재 좌표와 그래프 업데이트
                            // 이동거리와 펄스는 2배 차이다. 따라서 현재좌표를 10cm마다 업데이트
                            // 시키기 위해 5배수 마다 업데이트 필요
        {
            // X축 이동량에 따라서 막대와 현재 좌표 업데이트
            LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
            if (Dx >= 0)
            {
                // CW 방향으로 (LED0 =ON)
                GPIOG->ODR |= (1 << 0);
                LCD_DisplayChar(2, 16, '+');
                make_Bar(Dx);
            }
            else if (Dx < 0)
            {
                // CCW 방향으로 (LED0 =OFF)
                GPIOG->ODR &= ~(1 << 0);
                LCD_DisplayChar(2, 16, '-');
                erase_Bar(abs(Dx));
            }
        }

        if (CPX >= TPX)  // 출력 펄스수 제어 <인터럽트 발생수가 목표 pulse가 되면>
        {
            // pulse의 출력도 멈추고, TIM도 중지
            TIM1->CCER &= ~(1 << 12);   // CC4E: OC4 Disable
            TIM1->CR1 &= ~(1 << 0);     // TIM1 Disable

                    // 좌표, 거리 데이터, 현재펄스 초기화
            str[0] = 0;
            str[1] = 0;
            CPX = 0;
            GPIOG->ODR &= ~0XFF;
        }


    }
}

void TIMER8_OC_Init(void) //TIM8_CH2 CC 
{
    // PC7: TIM8_CH2
    // PC7을 출력설정하고 Alternate function(TIM1_CH4)으로 사용 선언
    RCC->AHB1ENR |= (1 << 2);      // RCC_AHB1ENR GPIOC Enable
    GPIOC->MODER |= (2 << 2 * 7);   // GPIOC PC7 Output Alternate function mode               
    GPIOC->OSPEEDR |= (3 << 2 * 7);   // GPIOC PC7 Output speed (100MHz High speed)
    GPIOC->OTYPER |= 0x00000000;  // GPIOC PC7 Output type push-pull (reset state)
    GPIOC->PUPDR |= (1 << 2 * 7);   // GPIOC PC7 Pull-up
    GPIOC->AFR[0] |= (1 << 28);     // Connect TIM1 pins(PC11) to AF1(TIM8)

    // Timerbase Mode
    RCC->APB2ENR |= (1 << 1);   // RCC_APB1ENR TIMER1 Enable

    TIM8->PSC = 168 - 1;        // Prescaler 168,000,000Hz/168= 1MHz (1us)

    fARR = 1000000 * (60 * STEP_DEG / 2) / (360 * RPM);  // us 단위 <pulse / sec>
    TIM8->ARR = fARR - 1;    // 주기 = 1us * fARR = fARR us

    TIM8->CR1 &= ~(1 << 4);   // DIR: Countermode = Upcounter (reset state)
    TIM8->CR1 &= ~(3 << 8);   // CKD: Clock division = 1 (reset state)
    TIM8->CR1 &= ~(3 << 5);    // CMS(Center-aligned mode Sel): No(reset state)

    TIM8->EGR |= (1 << 0);    // UG: Update generation 

    // Output/Compare Mode
    TIM8->CCER &= ~(1 << 4);   // CC2E: OC2 Active 
    TIM8->CCER &= ~(1 << 5);   // CC2P: OCPolarity_Active High

    TIM8->CCR4 = 10;   // TIM1_Pulse

        //TIM.1,8은 BDTR 15bit를 1로 enable을 해야 출력 입력기능을 한다
    TIM8->BDTR |= (1 << 15);  // main output enable

    TIM8->CCMR1 &= ~(3 << 8);   // CC2S(CC2channel): Output 
    TIM8->CCMR1 &= ~(1 << 11);  // OC2PE: Output Compare 2 preload disable
    TIM8->CCMR1 |= (3 << 12);   // OC2M: Output Compare 2 Mode : toggle

    TIM8->CR1 &= ~(1 << 7);   // ARPE: Auto reload preload disable
    TIM8->DIER |= (1 << 2);   // CC2IE: Enable the Tim1 CC2 interrupt

    NVIC->ISER[1] |= (1 << (46 - 32)); // TIM8_CC
    TIM8->CR1 &= ~(1 << 0);   // CEN: Disable the Tim8 Counter 
}

void TIM8_CC_IRQHandler(void)      // Y축 모터 구동
{
    if ((TIM8->SR & (1 << 2)) != RESET)	// CC4 interrupt flag 
    {
        TIM8->SR &= ~(1 << 2);	// CC4 Interrupt Claer

        CPY++;                  // Y축 펄스 카운트

            // 현재 펄스수 표시
        LCD_SetTextColor(RGB_RED);	// 글자색 : RED
        LCD_DisplayChar(5, 11, 0x30 + CPY / 10);
        LCD_DisplayChar(5, 12, 0x30 + CPY % 10);

        GPIOG->ODR ^= (1 << 6);   // 모터구동 LED6

        if (CPY % 5 == 0)     // 펄스의 수가 5의 배수일때 마다 현재 좌표와 그래프 업데이트
        {
            // Y축 이동량에 따라서 막대와 현재 좌표 업데이트
            LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
            if (Dy < 0)
            {
                // CCW 방향으로 (LED7 =OFF)
                LCD_DisplayChar(2, 23, '-');
                GPIOG->ODR &= ~(1 << 7);
                erase_Bar(abs(Dy));
            }
            else if (Dy >= 0)
            {
                // CW 방향으로 (LED7 =ON)
                LCD_DisplayChar(2, 23, '+');
                GPIOG->ODR |= (1 << 7);
                make_Bar(Dy);
            }
        }

        if (CPY >= TPY)  // 출력 펄스수 제어 <인터럽트 발생수가 목표 pulse가 되면>
        {
            // pulse의 출력도 멈추고, TIM도 중지
            TIM8->CCER &= ~(1 << 4);   // CC4E: OC4 Disable
            TIM8->CR1 &= ~(1 << 0);     // TIM1 Disable

                    // 좌표, 거리 데이터, 현재펄스 초기화
            str[0] = 0;
            str[1] = 0;
            CPY = 0;
            GPIOG->ODR &= ~0XFF;
        }
    }
}

void USART1_Init(void)
{
    // USART1 : TX(PA9)
    RCC->AHB1ENR |= (1 << 0);	// RCC_AHB1ENR GPIOA Enable
    GPIOA->MODER |= (2 << 2 * 9);	// GPIOA PIN9 Output Alternate function mode					
    GPIOA->OSPEEDR |= (3 << 2 * 9);	// GPIOA PIN9 Output speed (100MHz Very High speed)
    GPIOA->AFR[1] |= (7 << 4);	// Connect GPIOA pin9 to AF7(USART1)

    // USART1 : RX(PA10)
    GPIOA->MODER |= (2 << 2 * 10);	// GPIOA PIN10 Output Alternate function mode
    GPIOA->OSPEEDR |= (3 << 2 * 10);	// GPIOA PIN10 Output speed (100MHz Very High speed
    GPIOA->AFR[1] |= (7 << 8);	// Connect GPIOA pin10 to AF7(USART1)

    RCC->APB2ENR |= (1 << 4);	// RCC_APB2ENR USART1 Enable

    USART1_BRR_Configuration(9600); // USART Baud rate Configuration

    USART1->CR1 &= ~(1 << 12);	// USART_WordLength 8 Data bit
    USART1->CR1 |= (1 << 10);	// USART_Parity EN
    USART1->CR1 |= (1 << 9);	// USART_Parity Odd

    USART1->CR1 |= (1 << 2);	// 0x0004, USART_Mode_RX Enable
    USART1->CR2 &= ~(3 << 12);	// 0b00, USART_StopBits_1
    USART1->CR3 = 0x0000;	// No HardwareFlowControl, No DMA

    USART1->CR1 |= (1 << 5);	// 0x0020, RXNE interrupt Enable
    NVIC->ISER[1] |= (1 << (37 - 32));// Enable Interrupt USART1 (NVIC 37번)
    USART1->CR1 |= (1 << 13);	//  0x2000, USART1 Enable
}

void UART4_Init(void)
{
    // UART4 : TX(PC10)
    RCC->AHB1ENR |= (1 << 2);	// RCC_AHB1ENR GPIOC Enable
    GPIOC->MODER |= (2 << 2 * 10);	// GPIOC PIN10 Output Alternate function mode					
    GPIOC->OSPEEDR |= (3 << 2 * 10);	// GPIOC PIN10 Output speed (100MHz Very High speed)
    GPIOC->AFR[1] |= (8 << 4 * (10 - 8));// Connect GPIOC pin10 to AF8(USART1)

    // UART4 : RX(PC11)
    GPIOC->MODER |= (2 << 2 * 11);	// GPIOC PIN11 Output Alternate function mode
    GPIOC->OSPEEDR |= (3 << 2 * 11);	// GPIOC PIN11 Output speed (100MHz Very High speed
    GPIOC->AFR[1] |= (8 << 4 * (11 - 8));// Connect GPIOC pin11 to AF8(USART1)

    // BT RESET (PC13) : GPIO
    GPIOC->MODER |= (1 << 2 * 13);	// GPIOC PIN13 Output mode
    GPIOC->OSPEEDR |= (3 << 2 * 13);
    GPIOC->ODR |= (1 << 13);	// BT Reset

    RCC->APB1ENR |= (1 << 19);	// RCC_APB1ENR UART4 Enable

    USART4_BRR_Configuration(9600); // USART Baud rate Configuration

    UART4->CR1 &= ~(1 << 12);	// USART_WordLength 8 Data bit
    UART4->CR1 |= (1 << 10);	// USART_Parity EN
    UART4->CR1 |= (1 << 9);	// USART_Parity Odd

    UART4->CR1 |= (1 << 2);	// 0x0004, USART_Mode_RX Enable
    UART4->CR1 |= (1 << 3);	// 0x0008, USART_Mode_Tx Enable
    UART4->CR2 &= ~(3 << 12);	// 0b00, USART_StopBits_1
    UART4->CR3 = 0x0000;	// No HardwareFlowControl, No DMA

    UART4->CR1 |= (1 << 5);	// 0x0020, RXNE interrupt Enable
    NVIC->ISER[1] |= (1 << (52 - 32));// Enable Interrupt USART1 (NVIC 52번)
    UART4->CR1 |= (1 << 13);	//  0x2000, USART1 Enable
}

void SerialSendChar(uint8_t Ch) // 1문자 보내기 함수
{
    // USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
    // TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
    while ((USART1->SR & USART_SR_TXE) == RESET);
    USART1->DR = (Ch & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendString(char* str) // 여러문자 보내기 함수
{
    while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
    {
        SerialSendChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
        str++; 			// 포인터 수치 증가
    }
}

// Baud rate 설정
void USART1_BRR_Configuration(uint32_t USART_BaudRate)
{
    uint32_t tmpreg = 0x00;
    uint32_t APB2clock = 84000000;	//PCLK2_Frequency
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;

    // Find the integer part 
    if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
    //  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
    {       // USART1->CR1.OVER8 = 1 (8 oversampling)
        // Computing 'Integer part' when the oversampling mode is 8 Samples 
        integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)  
    }
    else  // USART1->CR1.OVER8 = 0 (16 oversampling)
    {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
        integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)    
    }								     // 100*(f_CK) / (8*2*Buadrate) = (25*f_CK)/(4*Buadrate)	
    tmpreg = (integerdivider / 100) << 4;

    // Find the fractional part 
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

    // Implement the fractional part in the register 
    if ((USART1->CR1 & USART_CR1_OVER8) != 0)
    {	// 8 oversampling
        tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
    }
    else	// 16 oversampling
    {
        tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
    }

    // Write to USART BRR register
    USART1->BRR = (uint16_t)tmpreg;
}

void USART4_BRR_Configuration(uint32_t USART_BaudRate)
{
    uint32_t tmpreg = 0x00;
    uint32_t APB1clock = 42000000;	//PCLK2_Frequency
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;

    // Find the integer part 
    if ((UART4->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
        //  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
    {       // UART4->CR1.OVER8 = 1 (8 oversampling)
    // Computing 'Integer part' when the oversampling mode is 8 Samples 
        integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)  
    }
    else  // USART1->CR1.OVER8 = 0 (16 oversampling)
    {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
        integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)    
    }
    tmpreg = (integerdivider / 100) << 4;

    // Find the fractional part 
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

    // Implement the fractional part in the register 
    if ((UART4->CR1 & USART_CR1_OVER8) != 0)
    {	// 8 oversampling
        tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
    }
    else	// 16 oversampling
    {
        tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
    }

    // Write to USART BRR register
    UART4->BRR = (uint16_t)tmpreg;
}

void USART1_IRQHandler(void)	// 문자열 수신?
{
    if ((USART1->SR & USART_SR_RXNE)) // USART_SR_RXNE= 1? RX Buffer Full?    
    {
        if (USART1_INT_flag == 0)   // 첫인터럽트
        {
            str[0] = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 첫번째 문자 저장
            USART1_INT_flag++;        // 인터럽트 플래그 1증가
        }
        else if (USART1_INT_flag == 1) // 두번째 인터럽트
        {
            str[1] = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 두번째 문자 저장
            USART1_INT_flag = 0;         // 인터럽트 플래그 초기화
            Robot_Control();             // 로봇제어 시작
        }

    }
}

void UART4_IRQHandler(void)	// 휴대폰에서 오는 문장을 받는 핸들러
{
    if ((UART4->SR & USART_SR_RXNE)) // USART_SR_RXNE= 1? RX Buffer Full?
        // #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
    {
        char ch;
        ch = UART4->DR;	 // 수신된 문자 저장
        LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE

        if (ch == 0)      // PC로 RPM제어를 할때
        {
            ADC_flag = 0;
            LCD_DisplayText(1, 12, "(PC)");

        }
        else if (ch == 1) // ADC로 RPM제어를 할때
        {
            ADC_flag = 1;
            LCD_DisplayText(1, 12, "(VR)");
        }

    }
    // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 clear 할 필요없음 
}

void Robot_Control(void)  // 로봇 명열어 제어 함수
{
    LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
    switch (str[0])                   // 명령어에 따라서 동작 분류
    {
    case X_axis:                // X축 명령어
        if (N <= 8)         // 최대 80cm 까지 제한
        {
            // X축 이동량을 받아 목표 좌표 계산후 표시
            Gx = 10 * (N);
            LCD_DisplayChar(2, 3, 0x30 + Gx / 10);
            LCD_DisplayChar(2, 4, 0x30 + Gx % 10);

            // X축 이동량 계산후 표시 (음수일때도 있어서 LCD 표시때는 abs값으로 표시)
            Dx = Gx - CX;
            LCD_DisplayChar(2, 17, 0x30 + abs(Dx) / 10);
            LCD_DisplayChar(2, 18, 0x30 + abs(Dx) % 10);

            // X축 모터(TIM1_Ch4)의 목표펄스수 + 모터구동
            TPX = abs(Dx) / STEP_DEG;
            LCD_DisplayChar(3, 4, 0x30 + TPX / 10);
            LCD_DisplayChar(3, 5, 0x30 + TPX % 10);

            if (Dx != 0)   // X의 거리가 0이면 모터 동작 (X)
            {
                TIM1->CCER |= (1 << 12);   // CC4E: OC4 Active 
                TIM1->CR1 |= (1 << 0);     // TIM1 Enable
            }

            break;
        }

    case Y_axis:          // Y축 명령어
        if (N <= 6)      // 최대 60cm 까지 제한
        {
            // Y축 이동량을 받아 목표 좌표 계산후 표시
            Gy = 10 * (N);
            LCD_DisplayChar(2, 10, 0x30 + Gy / 10);
            LCD_DisplayChar(2, 11, 0x30 + Gy % 10);

            // Y축 이동량 계산후 표시 (음수일때도 있어서 LCD 표시때는 abs값으로 표시)
            Dy = Gy - CY;
            LCD_DisplayChar(2, 24, 0x30 + abs(Dy) / 10);
            LCD_DisplayChar(2, 25, 0x30 + abs(Dy) % 10);

            // X축 모터(TIM8_Ch2)의 목표펄스수 + 모터구동
            TPY = abs(Dy) / STEP_DEG;
            LCD_DisplayChar(3, 11, 0x30 + TPY / 10);
            LCD_DisplayChar(3, 12, 0x30 + TPY % 10);

            if (Dy != 0)  // Y의 거리가 0이면 모터 동작 (X)
            {
                TIM8->CCER |= (1 << 4);   // CC2E: OC2 Disable
                TIM8->CR1 |= (1 << 0);   // TIM8 Enable
            }

            break;
        }

    case Gripper:     // Gripper 명령어

        if (N == 1)  // Gripper 명령어 : 잡기
        {
            GPIOG->ODR |= (1 << 3);
        }
        else if (N == 0)  // Gripper 명령어 : 놓기
        {
            GPIOG->ODR &= ~(1 << 3);
        }
        break;

    case Speed:    // RPM값 변경

        if ((N > 0) && (N <= 4) && (ADC_flag == 0))  // RPM 속도를 4이하로 제한 && ADC변환을 사용 하지 않을때
        {
            RPM = N;
            LCD_DisplayChar(1, 11, 0x30 + RPM);
        }
        break;

    }
}

void make_Bar(int D)  // +방향을로 로봇이 움직일때 막대 그래프 생성 + 현재 좌표 업데이트
{
    LCD_SetTextColor(RGB_RED);	// 글자색 : RED
    LCD_SetBrushColor(RGB_RED);       // 막대 그래프 색

    if (str[0] == X_axis)  // X축 명령일때
    {
        // 현재 X좌표 10씩 업데이트후 표시
        CX += 10;
        LCD_DisplayChar(4, 3, 0x30 + CX / 10);
        LCD_DisplayChar(4, 4, 0x30 + CX % 10);

        // 막대 그래프 생성
        LCD_DrawFillRect(20, 90, CX * 1.5, 8);

    }
    else if (str[0] == Y_axis) // Y축 명령일때
    {
        // 현재 Y좌표 10씩 업데이트후 표시
        CY += 10;
        LCD_DisplayChar(4, 10, 0x30 + CY / 10);
        LCD_DisplayChar(4, 11, 0x30 + CY % 10);

        // 막대 그래프 생성
        LCD_DrawFillRect(20, 100, CY * 1.5, 8);
    }
}

void erase_Bar(int D)  // -방향을로 로봇이 움직일때 막대 그래프 생성 + 현재 좌표 업데이트
{
    LCD_SetTextColor(RGB_RED);	// 글자색 : RED
    LCD_SetBrushColor(RGB_WHITE);     // 막대 그래프 색

    if (str[0] == X_axis)    // X축 명령일때
    {
        // 현재 X좌표 10씩 업데이트후 표시
        CX -= 10;
        LCD_DisplayChar(4, 3, 0x30 + CX / 10);
        LCD_DisplayChar(4, 4, 0x30 + CX % 10);

        // 막대 그래프 생성
        LCD_DrawFillRect(20 + 1.5 * CX, 90, 130 - CX * 1.5, 8);
    }
    else if (str[0] == Y_axis) // Y축 명령일때
    {
        // 현재 Y좌표 10씩 업데이트후 표시
        CY -= 10;
        LCD_DisplayChar(4, 10, 0x30 + CY / 10);
        LCD_DisplayChar(4, 11, 0x30 + CY % 10);

        // 막대 그래프 생성
        LCD_DrawFillRect(20 + 1.5 * CY, 100, 130 - CY * 1.5, 8);
    }

}

void _GPIO_Init(void)
{
    // LED (GPIO G) 설정
    RCC->AHB1ENR |= 0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
    GPIOG->MODER |= 0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
    GPIOG->OTYPER &= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
    GPIOG->OSPEEDR |= 0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 

    // SW (GPIO H) 설정 
    RCC->AHB1ENR |= 0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
    GPIOH->MODER &= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
    GPIOH->PUPDR &= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

    // Buzzer (GPIO F) 설정 
    RCC->AHB1ENR |= 0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
    GPIOF->MODER |= 0x00040000;	// GPIOF 9 : Output mode (0b01)						
    GPIOF->OTYPER &= ~0x0200;	// GPIOF 9 : Push-pull  	
    GPIOF->OSPEEDR |= 0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}

void BEEP(void)			// Beep for 20 ms 
{
    GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
    DelayMS(20);		// Delay 20 ms
    GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
    register unsigned short i;
    for (i = 0; i < wMS; i++)
        DelayUS(1000);
}

void DelayUS(unsigned short wUS)
{
    volatile int Dly = (int)wUS * 17;
    for (; Dly; Dly--);
}

void DisplayInitScreen(void)
{
    LCD_Clear(RGB_WHITE);		// 화면 클리어
    LCD_SetFont(&Gulim7);		// 폰트 : 굴림 7
    LCD_SetBackColor(RGB_WHITE);	// 글자배경색 : WHITE
    LCD_SetTextColor(RGB_BLACK);	// 글자색 : Black

        // X-Y로봇 초기화면
    LCD_SetBackColor(RGB_YELLOW);	//글자배경색 : Yellow
    LCD_DisplayText(0, 0, "X-Y Robot Control : PSY");

    LCD_SetBackColor(RGB_WHITE);	//글자배경색 : Yellow
    LCD_DisplayText(1, 0, "SD=   ,RPM=      ");
    LCD_DisplayText(2, 0, "GX=   ,GY=  ,DX=   ,DY=   ");
    LCD_DisplayText(3, 0, "TPX=  ,TPY=   ");
    LCD_DisplayText(4, 0, "CX=   ,CY=   ");
    LCD_DisplayText(5, 0, "CPX=  ,CPY=   ");

    LCD_DisplayText(8, 0, "CX");
    LCD_DisplayText(9, 0, "CY");

}

void DisDisplayDataScreen(void)
{
    LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE

    LCD_DisplayChar(1, 3, 0x30 + STEP_DEG);
    LCD_DisplayChar(1, 11, 0x30 + RPM);
    LCD_DisplayText(1, 12, "(PC)");

    LCD_DisplayChar(2, 3, 0x30 + Gx / 10);
    LCD_DisplayChar(2, 4, 0x30 + Gx % 10);

    LCD_DisplayChar(2, 10, 0x30 + Gy / 10);
    LCD_DisplayChar(2, 11, 0x30 + Gy % 10);

    LCD_DisplayChar(2, 17, 0x30 + Dx / 10);
    LCD_DisplayChar(2, 18, 0x30 + Dx % 10);

    LCD_DisplayChar(2, 24, 0x30 + Dy / 10);
    LCD_DisplayChar(2, 25, 0x30 + Dy % 10);

    LCD_DisplayChar(3, 4, 0x30 + TPX / 10);
    LCD_DisplayChar(3, 5, 0x30 + TPX % 10);

    LCD_DisplayChar(3, 11, 0x30 + TPY / 10);
    LCD_DisplayChar(3, 12, 0x30 + TPY % 10);

    LCD_SetTextColor(RGB_RED);       // 색상 : RED
    LCD_DisplayChar(4, 3, 0x30 + CX / 10);
    LCD_DisplayChar(4, 4, 0x30 + CX % 10);

    LCD_DisplayChar(4, 10, 0x30 + CY / 10);
    LCD_DisplayChar(4, 11, 0x30 + CY % 10);

    LCD_DisplayChar(5, 4, 0x30 + CPX / 10);
    LCD_DisplayChar(5, 5, 0x30 + CPX % 10);

    LCD_DisplayChar(5, 11, 0x30 + CPY / 10);
    LCD_DisplayChar(5, 12, 0x30 + CPY % 10);

    //LCD_SetTextColor(RGB_BLACK); // 블랙으로 초기화
}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{
    uint16_t key;
    key = GPIOH->IDR & 0xFF00;	// any key pressed ?
    if (key == 0xFF00)		// if no key, check key off
    {
        if (key_flag == 0)
            return key;
        else
        {
            DelayMS(10);
            key_flag = 0;
            return key;
        }
    }
    else				// if key input, check continuous key
    {
        if (key_flag != 0)	// if continuous key, treat as no key input
            return 0xFF00;
        else			// if new key,delay for debounce
        {
            key_flag = 1;
            DelayMS(10);
            return key;
        }
    }
}
