////////////////////////////////////////////////////////////////////////////////
//************************************************
// PR2: X-Y Robot Control
// ������ : 2018130048_�ڽÿ�
// ������ : 2022.12.11
// �������� : PC���� ��,�Ÿ�,�ӵ��� �޾Ƽ� ������ �޽��� ��� ��, ����
//          USART4 : RPM���� 1.PC����, 2.ADC����
//          USART1 : PC������ ��ɾ� ����
//          void Robot_Control(void) : ��ɾ� �ؼ��� �̵���, �޽� ���
//          void make_Bar(int D)     : ��ǥ ǥ�� + ����
//          void erase_Bar(int D)    : ��ǥ ǥ�� - ����
//          TIM.1,8 : ���� ��� ��������
//          ADC3   : ������������ RPM����
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

// USART1�� ���Ͽ� ���� ��ɾ���� 10������ Ǧ���������� ����
#define X_axis          88             // USART1���� ���� X�� ����
#define Y_axis          89             // USART1���� ���� Y�� ����
#define Gripper         71             // USART1���� ���� �׸��� ����
#define Speed           82             // USART1���� ���� RPM ����
#define N               (str[1] - 48)  // USART1���� ���� ������ ����

#define STEP_DEG	2       // step angle

void _GPIO_Init(void);
uint16_t KEY_Scan(void);
void TIMER1_OC_Init(void); //TIM1_CH4 CC 
void TIMER8_OC_Init(void); //TIM8_CH2 CC 
void TIMER3_Init(void);    //TIM3     UI

void DisplayInitScreen(void);
void DisDisplayDataScreen(void); // data ǥ���Լ�

// ��Ű��� �Լ� ����
void USART1_Init(void);  // PC���
void UART4_Init(void);   // �޴��� ��� (IOS)
void USART1_BRR_Configuration(uint32_t USART_BaudRate);  // PC��żӵ�
void USART4_BRR_Configuration(uint32_t USART_BaudRate);  // �޴��� ��� (IOS)�ӵ�
void SerialSendChar(uint8_t c);
void SerialSendString(char* s);
UINT8 no, CRdata;

void _ADC_Init(void);

// �κ����� �� ���÷��� �Լ�����
void Robot_Control(void);     // �����޾Ƽ� �κ��� �Ÿ��� ��ǥ���� ����ϴ��Լ�
void make_Bar(int D);         // ������ǥ�� ����׷����� ������Ʈ (����)
void erase_Bar(int D);        // ������ǥ�� ����׷����� ������Ʈ (����)

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

uint16 ADC_flag = 0;     // PC��ɰ� ADC��ȯ�� ���ϴ� �÷���
int ADC_Value, Voltage;  // ADC��, ���а�

// PC���� ���Ź޴� ��ɾ� ����
UINT16 USART1_INT_flag = 0;  // 
int CX = 0, CY = 0;
int Gx = 0, Gy = 0;
int Dx = 0, Dy = 0;
int TPX = 0, TPY = 0;
int CPX = 0, CPY = 0;
int RPM = 3;

// ��� �̵��� ���庯��
int str[3];

int main(void)
{
    _GPIO_Init();
    LCD_Init();
    DelayMS(10);
    DisplayInitScreen();	// LCD �ʱ�ȭ��
    DisDisplayDataScreen(); // LCD�� �ʱ� ������ ǥ��
    GPIOG->ODR &= 0xFF00;   // �ʱⰪ: LED0~7 Off
    USART1_Init();          // USART1  �ʱ�ȭ
    UART4_Init();           // USART4  �ʱ�ȭ
    _ADC_Init();            // ADC3 �ʱ�ȭ

    TIMER3_Init();    //TIM3_UI (ADC)
    TIMER1_OC_Init(); //TIM1_CH4 CC (X�� ���� �޽�)
    TIMER8_OC_Init(); //TIM8_CH2 CC (Y�� ���� �޽�)
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
    if (ADC_flag == 1)       // USART4 ���� ADC_flag == 1�϶�
        ADC3->CR2 |= (1 << 30); // ADC SWSTART : ON
}

void _ADC_Init(void)
{
    // ADC3: PA1(pin 41) : ��������
    RCC->AHB1ENR |= (1 << 0);	                // (1<<0) ENABLE GPIOA CLK (stm32f4xx.h ����)
    GPIOA->MODER |= (3 << 2 * 1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE				
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;	// ENABLE ADC3 CLK (stm32f4xx.h ����)

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

        // �־��� ������ ���� RPM ����
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

    LCD_SetTextColor(RGB_BLUE);    // ���ڻ� : �Ķ���
    LCD_DisplayChar(1, 11, 0x30 + RPM);
}

float fARR;
void TIMER1_OC_Init(void) //TIM1_CH4 CC 
{
    // PA11: TIM1_CH4
    // PA11�� ��¼����ϰ� Alternate function(TIM1_CH4)���� ��� ����
    RCC->AHB1ENR |= (1 << 0);      // RCC_AHB1ENR GPIOA Enable
    GPIOA->MODER |= (2 << 2 * 11);   // GPIOA PA11�� Output Alternate function mode               
    GPIOA->OSPEEDR |= (3 << 2 * 11);   // GPIOA PA11�� Output speed (100MHz High speed)
    GPIOA->OTYPER |= 0x00000000;   // GPIOA PA11�� Output type push-pull (reset state)
    GPIOA->PUPDR |= (1 << 2 * 11);   // GPIOA PA11�� Pull-up
    GPIOA->AFR[1] |= (1 << 12);     // Connect TIM1 pins(PA11) to AF1(TIM1/2)

    // Timerbase Mode
    RCC->APB2ENR |= (1 << 0);   // RCC_APB1ENR TIMER1 Enable

    TIM1->PSC = 168 - 1;        // Prescaler 168,000,000Hz/168= 1MHz (1us)

    fARR = 1000000 * (60 * STEP_DEG / 2) / (360 * RPM);  // us ���� <pulse / sec>
    TIM1->ARR = fARR - 1;    // �ֱ� = 1us * fARR = fARR us

    TIM1->CR1 &= ~(1 << 4);   // DIR: Countermode = Upcounter (reset state)
    TIM1->CR1 &= ~(3 << 8);   // CKD: Clock division = 1 (reset state)
    TIM1->CR1 &= ~(3 << 5);    // CMS(Center-aligned mode Sel): No(reset state)

    TIM1->EGR |= (1 << 0);    // UG: Update generation 

    // Output/Compare Mode
    TIM1->CCER &= ~(1 << 12);   // CC4E: OC4 Active 
    TIM1->CCER &= ~(1 << 13);  // CC4P: OCPolarity_Active High

    TIM1->CCR4 = 10;   // TIM1_Pulse

        //TIM.1,8�� BDTR 15bit�� 1�� enable�� �ؾ� ��� �Է±���� �Ѵ�
    TIM1->BDTR |= (1 << 15);  // main output enable

    TIM1->CCMR2 &= ~(3 << 8); // CC4S(CC1channel): Output 
    TIM1->CCMR2 &= ~(1 << 11); // OC4PE: Output Compare 4 preload disable
    TIM1->CCMR2 |= (3 << 12);   // OC4M: Output Compare 4 Mode : toggle

    TIM1->CR1 &= ~(1 << 7);   // ARPE: Auto reload preload disable
    TIM1->DIER |= (1 << 4);   // CC4IE: Enable the Tim1 CC4 interrupt

    NVIC->ISER[0] |= (1 << 27); // TIM1_CC
    TIM1->CR1 &= ~(1 << 0);   // CEN: Disable the Tim1 Counter 
}

void TIM1_CC_IRQHandler(void)      // X�� ���� ����
{
    if ((TIM1->SR & (1 << 4)) != RESET)	// CC4 interrupt flag 
    {
        TIM1->SR &= ~(1 << 4);	// CC4 Interrupt Claer
        CPX++;                  // X�� �޽� ī��Ʈ

        // ���� �޽��� ǥ��
        LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
        LCD_DisplayChar(5, 4, 0x30 + CPX / 10);
        LCD_DisplayChar(5, 5, 0x30 + CPX % 10);

        GPIOG->ODR ^= (1 << 1);   // ���ͱ��� LED1

        if (CPX % 5 == 0)     // �޽��� ���� 5�� ����϶� ���� ���� ��ǥ�� �׷��� ������Ʈ
                            // �̵��Ÿ��� �޽��� 2�� ���̴�. ���� ������ǥ�� 10cm���� ������Ʈ
                            // ��Ű�� ���� 5��� ���� ������Ʈ �ʿ�
        {
            // X�� �̵����� ���� ����� ���� ��ǥ ������Ʈ
            LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
            if (Dx >= 0)
            {
                // CW �������� (LED0 =ON)
                GPIOG->ODR |= (1 << 0);
                LCD_DisplayChar(2, 16, '+');
                make_Bar(Dx);
            }
            else if (Dx < 0)
            {
                // CCW �������� (LED0 =OFF)
                GPIOG->ODR &= ~(1 << 0);
                LCD_DisplayChar(2, 16, '-');
                erase_Bar(abs(Dx));
            }
        }

        if (CPX >= TPX)  // ��� �޽��� ���� <���ͷ�Ʈ �߻����� ��ǥ pulse�� �Ǹ�>
        {
            // pulse�� ��µ� ���߰�, TIM�� ����
            TIM1->CCER &= ~(1 << 12);   // CC4E: OC4 Disable
            TIM1->CR1 &= ~(1 << 0);     // TIM1 Disable

                    // ��ǥ, �Ÿ� ������, �����޽� �ʱ�ȭ
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
    // PC7�� ��¼����ϰ� Alternate function(TIM1_CH4)���� ��� ����
    RCC->AHB1ENR |= (1 << 2);      // RCC_AHB1ENR GPIOC Enable
    GPIOC->MODER |= (2 << 2 * 7);   // GPIOC PC7 Output Alternate function mode               
    GPIOC->OSPEEDR |= (3 << 2 * 7);   // GPIOC PC7 Output speed (100MHz High speed)
    GPIOC->OTYPER |= 0x00000000;  // GPIOC PC7 Output type push-pull (reset state)
    GPIOC->PUPDR |= (1 << 2 * 7);   // GPIOC PC7 Pull-up
    GPIOC->AFR[0] |= (1 << 28);     // Connect TIM1 pins(PC11) to AF1(TIM8)

    // Timerbase Mode
    RCC->APB2ENR |= (1 << 1);   // RCC_APB1ENR TIMER1 Enable

    TIM8->PSC = 168 - 1;        // Prescaler 168,000,000Hz/168= 1MHz (1us)

    fARR = 1000000 * (60 * STEP_DEG / 2) / (360 * RPM);  // us ���� <pulse / sec>
    TIM8->ARR = fARR - 1;    // �ֱ� = 1us * fARR = fARR us

    TIM8->CR1 &= ~(1 << 4);   // DIR: Countermode = Upcounter (reset state)
    TIM8->CR1 &= ~(3 << 8);   // CKD: Clock division = 1 (reset state)
    TIM8->CR1 &= ~(3 << 5);    // CMS(Center-aligned mode Sel): No(reset state)

    TIM8->EGR |= (1 << 0);    // UG: Update generation 

    // Output/Compare Mode
    TIM8->CCER &= ~(1 << 4);   // CC2E: OC2 Active 
    TIM8->CCER &= ~(1 << 5);   // CC2P: OCPolarity_Active High

    TIM8->CCR4 = 10;   // TIM1_Pulse

        //TIM.1,8�� BDTR 15bit�� 1�� enable�� �ؾ� ��� �Է±���� �Ѵ�
    TIM8->BDTR |= (1 << 15);  // main output enable

    TIM8->CCMR1 &= ~(3 << 8);   // CC2S(CC2channel): Output 
    TIM8->CCMR1 &= ~(1 << 11);  // OC2PE: Output Compare 2 preload disable
    TIM8->CCMR1 |= (3 << 12);   // OC2M: Output Compare 2 Mode : toggle

    TIM8->CR1 &= ~(1 << 7);   // ARPE: Auto reload preload disable
    TIM8->DIER |= (1 << 2);   // CC2IE: Enable the Tim1 CC2 interrupt

    NVIC->ISER[1] |= (1 << (46 - 32)); // TIM8_CC
    TIM8->CR1 &= ~(1 << 0);   // CEN: Disable the Tim8 Counter 
}

void TIM8_CC_IRQHandler(void)      // Y�� ���� ����
{
    if ((TIM8->SR & (1 << 2)) != RESET)	// CC4 interrupt flag 
    {
        TIM8->SR &= ~(1 << 2);	// CC4 Interrupt Claer

        CPY++;                  // Y�� �޽� ī��Ʈ

            // ���� �޽��� ǥ��
        LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
        LCD_DisplayChar(5, 11, 0x30 + CPY / 10);
        LCD_DisplayChar(5, 12, 0x30 + CPY % 10);

        GPIOG->ODR ^= (1 << 6);   // ���ͱ��� LED6

        if (CPY % 5 == 0)     // �޽��� ���� 5�� ����϶� ���� ���� ��ǥ�� �׷��� ������Ʈ
        {
            // Y�� �̵����� ���� ����� ���� ��ǥ ������Ʈ
            LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
            if (Dy < 0)
            {
                // CCW �������� (LED7 =OFF)
                LCD_DisplayChar(2, 23, '-');
                GPIOG->ODR &= ~(1 << 7);
                erase_Bar(abs(Dy));
            }
            else if (Dy >= 0)
            {
                // CW �������� (LED7 =ON)
                LCD_DisplayChar(2, 23, '+');
                GPIOG->ODR |= (1 << 7);
                make_Bar(Dy);
            }
        }

        if (CPY >= TPY)  // ��� �޽��� ���� <���ͷ�Ʈ �߻����� ��ǥ pulse�� �Ǹ�>
        {
            // pulse�� ��µ� ���߰�, TIM�� ����
            TIM8->CCER &= ~(1 << 4);   // CC4E: OC4 Disable
            TIM8->CR1 &= ~(1 << 0);     // TIM1 Disable

                    // ��ǥ, �Ÿ� ������, �����޽� �ʱ�ȭ
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
    NVIC->ISER[1] |= (1 << (37 - 32));// Enable Interrupt USART1 (NVIC 37��)
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
    NVIC->ISER[1] |= (1 << (52 - 32));// Enable Interrupt USART1 (NVIC 52��)
    UART4->CR1 |= (1 << 13);	//  0x2000, USART1 Enable
}

void SerialSendChar(uint8_t Ch) // 1���� ������ �Լ�
{
    // USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
    // TX buffer Empty���� ������ ��� ���(�۽� ������ ���±��� ���)
    while ((USART1->SR & USART_SR_TXE) == RESET);
    USART1->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}

void SerialSendString(char* str) // �������� ������ �Լ�
{
    while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
    {
        SerialSendChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
        str++; 			// ������ ��ġ ����
    }
}

// Baud rate ����
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
        integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)  
    }
    else  // USART1->CR1.OVER8 = 0 (16 oversampling)
    {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
        integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)    
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
        integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)  
    }
    else  // USART1->CR1.OVER8 = 0 (16 oversampling)
    {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
        integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)    
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

void USART1_IRQHandler(void)	// ���ڿ� ����?
{
    if ((USART1->SR & USART_SR_RXNE)) // USART_SR_RXNE= 1? RX Buffer Full?    
    {
        if (USART1_INT_flag == 0)   // ù���ͷ�Ʈ
        {
            str[0] = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ù��° ���� ����
            USART1_INT_flag++;        // ���ͷ�Ʈ �÷��� 1����
        }
        else if (USART1_INT_flag == 1) // �ι�° ���ͷ�Ʈ
        {
            str[1] = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� �ι�° ���� ����
            USART1_INT_flag = 0;         // ���ͷ�Ʈ �÷��� �ʱ�ȭ
            Robot_Control();             // �κ����� ����
        }

    }
}

void UART4_IRQHandler(void)	// �޴������� ���� ������ �޴� �ڵ鷯
{
    if ((UART4->SR & USART_SR_RXNE)) // USART_SR_RXNE= 1? RX Buffer Full?
        // #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
    {
        char ch;
        ch = UART4->DR;	 // ���ŵ� ���� ����
        LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE

        if (ch == 0)      // PC�� RPM��� �Ҷ�
        {
            ADC_flag = 0;
            LCD_DisplayText(1, 12, "(PC)");

        }
        else if (ch == 1) // ADC�� RPM��� �Ҷ�
        {
            ADC_flag = 1;
            LCD_DisplayText(1, 12, "(VR)");
        }

    }
    // DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
}

void Robot_Control(void)  // �κ� ���� ���� �Լ�
{
    LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
    switch (str[0])                   // ��ɾ ���� ���� �з�
    {
    case X_axis:                // X�� ��ɾ�
        if (N <= 8)         // �ִ� 80cm ���� ����
        {
            // X�� �̵����� �޾� ��ǥ ��ǥ ����� ǥ��
            Gx = 10 * (N);
            LCD_DisplayChar(2, 3, 0x30 + Gx / 10);
            LCD_DisplayChar(2, 4, 0x30 + Gx % 10);

            // X�� �̵��� ����� ǥ�� (�����϶��� �־ LCD ǥ�ö��� abs������ ǥ��)
            Dx = Gx - CX;
            LCD_DisplayChar(2, 17, 0x30 + abs(Dx) / 10);
            LCD_DisplayChar(2, 18, 0x30 + abs(Dx) % 10);

            // X�� ����(TIM1_Ch4)�� ��ǥ�޽��� + ���ͱ���
            TPX = abs(Dx) / STEP_DEG;
            LCD_DisplayChar(3, 4, 0x30 + TPX / 10);
            LCD_DisplayChar(3, 5, 0x30 + TPX % 10);

            if (Dx != 0)   // X�� �Ÿ��� 0�̸� ���� ���� (X)
            {
                TIM1->CCER |= (1 << 12);   // CC4E: OC4 Active 
                TIM1->CR1 |= (1 << 0);     // TIM1 Enable
            }

            break;
        }

    case Y_axis:          // Y�� ��ɾ�
        if (N <= 6)      // �ִ� 60cm ���� ����
        {
            // Y�� �̵����� �޾� ��ǥ ��ǥ ����� ǥ��
            Gy = 10 * (N);
            LCD_DisplayChar(2, 10, 0x30 + Gy / 10);
            LCD_DisplayChar(2, 11, 0x30 + Gy % 10);

            // Y�� �̵��� ����� ǥ�� (�����϶��� �־ LCD ǥ�ö��� abs������ ǥ��)
            Dy = Gy - CY;
            LCD_DisplayChar(2, 24, 0x30 + abs(Dy) / 10);
            LCD_DisplayChar(2, 25, 0x30 + abs(Dy) % 10);

            // X�� ����(TIM8_Ch2)�� ��ǥ�޽��� + ���ͱ���
            TPY = abs(Dy) / STEP_DEG;
            LCD_DisplayChar(3, 11, 0x30 + TPY / 10);
            LCD_DisplayChar(3, 12, 0x30 + TPY % 10);

            if (Dy != 0)  // Y�� �Ÿ��� 0�̸� ���� ���� (X)
            {
                TIM8->CCER |= (1 << 4);   // CC2E: OC2 Disable
                TIM8->CR1 |= (1 << 0);   // TIM8 Enable
            }

            break;
        }

    case Gripper:     // Gripper ��ɾ�

        if (N == 1)  // Gripper ��ɾ� : ���
        {
            GPIOG->ODR |= (1 << 3);
        }
        else if (N == 0)  // Gripper ��ɾ� : ����
        {
            GPIOG->ODR &= ~(1 << 3);
        }
        break;

    case Speed:    // RPM�� ����

        if ((N > 0) && (N <= 4) && (ADC_flag == 0))  // RPM �ӵ��� 4���Ϸ� ���� && ADC��ȯ�� ��� ���� ������
        {
            RPM = N;
            LCD_DisplayChar(1, 11, 0x30 + RPM);
        }
        break;

    }
}

void make_Bar(int D)  // +�������� �κ��� �����϶� ���� �׷��� ���� + ���� ��ǥ ������Ʈ
{
    LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
    LCD_SetBrushColor(RGB_RED);       // ���� �׷��� ��

    if (str[0] == X_axis)  // X�� ����϶�
    {
        // ���� X��ǥ 10�� ������Ʈ�� ǥ��
        CX += 10;
        LCD_DisplayChar(4, 3, 0x30 + CX / 10);
        LCD_DisplayChar(4, 4, 0x30 + CX % 10);

        // ���� �׷��� ����
        LCD_DrawFillRect(20, 90, CX * 1.5, 8);

    }
    else if (str[0] == Y_axis) // Y�� ����϶�
    {
        // ���� Y��ǥ 10�� ������Ʈ�� ǥ��
        CY += 10;
        LCD_DisplayChar(4, 10, 0x30 + CY / 10);
        LCD_DisplayChar(4, 11, 0x30 + CY % 10);

        // ���� �׷��� ����
        LCD_DrawFillRect(20, 100, CY * 1.5, 8);
    }
}

void erase_Bar(int D)  // -�������� �κ��� �����϶� ���� �׷��� ���� + ���� ��ǥ ������Ʈ
{
    LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
    LCD_SetBrushColor(RGB_WHITE);     // ���� �׷��� ��

    if (str[0] == X_axis)    // X�� ����϶�
    {
        // ���� X��ǥ 10�� ������Ʈ�� ǥ��
        CX -= 10;
        LCD_DisplayChar(4, 3, 0x30 + CX / 10);
        LCD_DisplayChar(4, 4, 0x30 + CX % 10);

        // ���� �׷��� ����
        LCD_DrawFillRect(20 + 1.5 * CX, 90, 130 - CX * 1.5, 8);
    }
    else if (str[0] == Y_axis) // Y�� ����϶�
    {
        // ���� Y��ǥ 10�� ������Ʈ�� ǥ��
        CY -= 10;
        LCD_DisplayChar(4, 10, 0x30 + CY / 10);
        LCD_DisplayChar(4, 11, 0x30 + CY % 10);

        // ���� �׷��� ����
        LCD_DrawFillRect(20 + 1.5 * CY, 100, 130 - CY * 1.5, 8);
    }

}

void _GPIO_Init(void)
{
    // LED (GPIO G) ����
    RCC->AHB1ENR |= 0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
    GPIOG->MODER |= 0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
    GPIOG->OTYPER &= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
    GPIOG->OSPEEDR |= 0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 

    // SW (GPIO H) ���� 
    RCC->AHB1ENR |= 0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
    GPIOH->MODER &= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
    GPIOH->PUPDR &= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

    // Buzzer (GPIO F) ���� 
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
    LCD_Clear(RGB_WHITE);		// ȭ�� Ŭ����
    LCD_SetFont(&Gulim7);		// ��Ʈ : ���� 7
    LCD_SetBackColor(RGB_WHITE);	// ���ڹ��� : WHITE
    LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : Black

        // X-Y�κ� �ʱ�ȭ��
    LCD_SetBackColor(RGB_YELLOW);	//���ڹ��� : Yellow
    LCD_DisplayText(0, 0, "X-Y Robot Control : PSY");

    LCD_SetBackColor(RGB_WHITE);	//���ڹ��� : Yellow
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
    LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE

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

    LCD_SetTextColor(RGB_RED);       // ���� : RED
    LCD_DisplayChar(4, 3, 0x30 + CX / 10);
    LCD_DisplayChar(4, 4, 0x30 + CX % 10);

    LCD_DisplayChar(4, 10, 0x30 + CY / 10);
    LCD_DisplayChar(4, 11, 0x30 + CY % 10);

    LCD_DisplayChar(5, 4, 0x30 + CPX / 10);
    LCD_DisplayChar(5, 5, 0x30 + CPX % 10);

    LCD_DisplayChar(5, 11, 0x30 + CPY / 10);
    LCD_DisplayChar(5, 12, 0x30 + CPY % 10);

    //LCD_SetTextColor(RGB_BLACK); // ������ �ʱ�ȭ
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
