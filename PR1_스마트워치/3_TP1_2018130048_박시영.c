////////////////////////////////////////////////////////////////////////////////
// *****************************************************************************
// PR1: 스마트 워치
// 제출자 : 2018130048 박시영
// 제출일 : 2022.12.08
// 과제개요 : <1.알람, 2.계산기, 3.온도계를 EXTI(SW7)을 이용하여 화면제어>
//          void EXTI15_10_IRQHandler()에서 화면 전환
//
//          1.알람 화면에서 알람시간 설정후, 시간이 현재시간과 같은면 알람을 울린다.
//            main함수의 while(1) : 알람시간 설정, 
//            void ASCII_Time()  : 10진수와 아스키 코드 사이의 변환
//            void TIM7_IRQHandler(): 현재시간과 알람이 울리는 기능
//
//          2.계산기 화면에서 PC와 USART1통신을 통하여 계산기기능을 수행한다. 
//            USART1_IRQHandler : 계산기의 연산을 PC에서 수신후 연산한다음 LCD출력
//
//          3.온도계 화면에서 온도막대와 히터 쿨러 표시
//            ADC_IRQHandler(TIM3로 제어): 온도값 수신후 변환, LCD에 막대그래프, 히터/쿨러강도, 온도표시
//            TIM4_CH1  :  PWM으로 히터/쿨러 강도에 따라서 출력제어
// *****************************************************************************
////////////////////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

#define DISP_Alarm      0       // DISP_Flag에서 Alarm 화면을 표시하기위한 값 정의
#define DISP_Calculator 1       // DISP_Flag에서 Calculator 화면을 표시하기위한 값 정의
#define DISP_Thermostat 2       // DISP_Flag에서 Thermostat 화면을 표시하기위한 값 정의

void _GPIO_Init(void);
void DisplayTitle(void);
void _EXTI_Init(void);    // SW7 인터럽트 초기화 함수 정의
void _ADC_Init(void);     // ADC 초기화 함수 정의
uint16_t KEY_Scan(void);  
void TIMER7_Init(void);   // 현재시각 타이머 정의
void TIMER3_Init(void);   // ADC제어 타이머 정의
void TIMER4_PWM_Init(void); // 히터,쿨러 가동 타이머 정의

// 통신관련 함수 정의
void USART1_Init(void);
void USART_BRR_Configuration(uint32_t USART_BaudRate);
void SerialSendChar(uint8_t c);
void SerialSendString(char *s);

// 스마트 워치의 관련함수 정의
void ALARM(void);
void Calculator(void);
void Thermostat(void);
void ASCII_Time(char *time);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

uint16_t ADC_Value, Voltage;
uint16_t DISP_Flag = 1;       // 화면 선택 Flag
char time_h = 70,time_m = 65; // 시계 초기 값
char operand_flag = 0, operand1, operand2, result; // Calculator의 계산순서 플래그, 피연산자1, 피연산자2, 결과
char H, C;  // 히터, 쿨러 초기화
uint8_t SW_flag = 1;          // SW.0,1이 작동 변수
uint8_t str[20];

int main(void)
{
  
	LCD_Init();	// LCD 구동 함수
	DelayMS(10);	// LCD구동 딜레이
 	DisplayTitle();	//LCD 초기화면구동 함수
        _EXTI_Init();   // EXTI 초기화
        _GPIO_Init();  
	_ADC_Init();    // ADC초기화
        USART1_Init();  // USART1  초기화
        
        
        Fram_Init();            // FRAM 초기화 H/W 초기화
	Fram_Status_Config();   // FRAM 초기화 S/W 초기화
        
        ALARM();                // 초기화면 표시
        
        // 타이머들 초기화
        TIMER7_Init();
        TIMER3_Init();
        TIMER4_PWM_Init();

        char  Alarm_h = Fram_Read(1208), Alarm_m = Fram_Read(1209);                  // Alarm의 시, 분 시작 값
 	while(1)
	{
          
          if (SW_flag == 1)    // DISP_Alarm 일때, 즉 알람 화면 일때만 동작
          {
              switch(KEY_Scan())
              {
                  case SW0_PUSH:                      // SW0이 눌리면
                      Alarm_h++;                      // 알람의 시간 증가
                      ASCII_Time(&Alarm_h);            // 시간값을 계산  (void ASCII_Time(char time) 참고)
                      LCD_SetTextColor(RGB_RED);        //글자색
                      LCD_DisplayChar(1,6,Alarm_h);   // 시 표시
                      BEEP();
                      break;
                      
                  case SW1_PUSH:                      // SW1이 눌리면
                      Alarm_m++;                      // 알람의 분 증가
                      ASCII_Time(&Alarm_m);            // 시간값을 계산  (void ASCII_Time(char time) 참고)
                      LCD_SetTextColor(RGB_RED);        //글자색
                      LCD_DisplayChar(1,8,Alarm_m);   // 분 표시
                      BEEP();
                      break;
                      
                  case SW2_PUSH:                      // SW2가 눌리면
                      Fram_Write(1208,Alarm_h);       // 알람 시간을 Fram 저장
                      Fram_Write(1209,Alarm_m);
                      BEEP();
                      DelayMS(100);
                      BEEP();
                      break;
              }
          
          }
	}
}

void ALARM(void)
{
      // 알람 디스플레이 
      LCD_DisplayText(0,0,"1.ALARM(PSY)");
      LCD_DisplayText(1,0,"Alarm  : ");
      
      LCD_SetTextColor(RGB_RED);        //글자색
      LCD_DisplayChar(1,6,Fram_Read(1208));
      LCD_DisplayChar(1,8,Fram_Read(1209));
}

void Calculator(void)
{
      // 계산기 디스플레이 
      LCD_SetTextColor(RGB_BLACK);	//글자색
      LCD_DisplayText(0,0,"2.Calculator");
      LCD_DisplayText(1,0,"  + = ");
      LCD_SetTextColor(RGB_RED);	//글자색
      LCD_DisplayChar(1,1,0x30 + 0);
      LCD_DisplayChar(1,3,0x30 + 0);
      LCD_DisplayChar(1,5,0x30 + 0);
}

void Thermostat(void)
{
      // // 온도계 디스플레이 
      LCD_SetTextColor(RGB_BLACK);	//글자색
      LCD_DisplayText(0,0,"3.Thermostat");
      LCD_DisplayText(1,0,"T:    ");
      LCD_DisplayText(2,0,"H:  C: ");
}

//* 아스키 코드로 시간들 계산하는 함수 *//
void ASCII_Time(char *time)
{
      if (*time == 71 )  // time이 문자상으로 G가되면 0으로 초기화
      {
          *time = 48;     // 0으로 초기화
      }
                      
      if (*time == 58)   // time이 문자상으로 10이되면 A로 초기화
      {
          *time += 7;    // 아스키코드에서 10과 A의 차이가 7이다
      }
}

void ADC_IRQHandler(void)
{
	ADC2->SR &= ~(1<<1);		// EOC flag clear
        
        float Temperature;              // 온도값 변수
	ADC_Value = ADC2->DR;		// Reading ADC result

       	//sprintf(str,"%4d",ADC_Value);	// ADC result : 12bits (0~4095)
       	//LCD_DisplayText(0,6,str);
	Voltage = ADC_Value * (3.3 * 100) / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
                                                    // 100:  소수점아래 두자리까지 표시하기 위한 값  
        
        /** 선형 보간법 **/
        /// (x1, y1) == (0, -10), (x2, y2) == (330, 40) 주어진 두점 : (0 ~ 330 : Voltage * 100 <input 값>) (-10 ~ 40 : 온도값 <output 값>)
        /// 공식 : result = {(y2 - y1) / (x2 - x1)} * (Voltage - x1) + y1
        /// Temperature = {(40 - (-10)) / (330 - 0)} * (Voltage - 0) + (-10) == 0.15 * Voltage -10
        
        Temperature = 0.15 * (Voltage) -10;
        
        // 온도그래프의 막대 색상 지정 (Temperature 값에 따라서 색 지정)
        if (Temperature >= -10 && Temperature <= 0)
        {
            H = 2;  // 히터 강도
            C = 0;  // 쿨러 강도
            LCD_SetBrushColor(RGB_BLUE);
        }
        else if (Temperature >= 1 && Temperature <= 10)
        {   
            H = 1;  // 히터 강도
            C = 0;  // 쿨러 강도
            LCD_SetBrushColor(RGB_BLUE);
        }
        else if (Temperature >= 11 && Temperature <= 20)
        {
            H = 0;  // 히터 강도
            C = 0;  // 쿨러 강도
            LCD_SetBrushColor(RGB_GREEN);
        }
        else if (Temperature >= 21 && Temperature <= 30)
        {
            H = 0;  // 히터 강도
            C = 1;  // 쿨러 강도
            LCD_SetBrushColor(RGB_RED);
        }
        else if (Temperature >= 31 && Temperature <= 40)
        {
            H = 0;  // 히터 강도
            C = 2;  // 쿨러 강도
            LCD_SetBrushColor(RGB_RED);
        }
        
        // 온도막대 그래프 표시
        LCD_DrawFillRect(50, 22, (42 + 1.2 * Temperature),10);
        LCD_SetBrushColor(RGB_WHITE);
        LCD_DrawFillRect((42 + 1.2 * Temperature) + 50, 22, 140 - (42 + 1.2 * Temperature),10);
        
        // 히터, 쿨러 강도? 표시
        if (H != 0)
        {
            GPIOG->ODR &= ~0xFF;
            GPIOG->ODR |= (1<<5);
        }
        else if (C != 0)
        {
            GPIOG->ODR &= ~0xFF;
            GPIOG->ODR |= (1<<6);
        }
        else
        {
            GPIOG->ODR &= ~0xFF;
        }
        LCD_SetTextColor(RGB_RED);        //글자색
        LCD_DisplayChar(2,2,0x30 + H);
        LCD_SetTextColor(RGB_BLUE);        //글자색
        LCD_DisplayChar(2,6,0x30 + C);
        
        // 히터, 쿨러 가동 TIM4_PWM
        if (H == 2 || C == 2)
        {
            TIM4->CCR1	= 18000;		// CCR1 value : DR = 90 %
        }
        else if (H == 1 || C == 1)
        {
            TIM4->CCR1	= 2000;		// CCR1 value : DR = 10 %
        }
        else if (H == 0 || C == 0)
        {
            TIM4->CCR1	= 0;		// CCR1 value : DR = 00 %
        }
        
        // 온도의 부호를 결정하는  if문
        LCD_SetTextColor(RGB_GREEN);        //글자색
        if (Temperature < 0)
        {
            LCD_DisplayChar(1,2,'-');
            Temperature = -Temperature;  // 온도가 음수일 때, 양수로 바꿔서 LCD에 표시하기위해 -
        }
        else
        {
            LCD_DisplayChar(1,2,' ');
        }
        
        // 온도 표시
        LCD_DisplayChar(1,3,(int)Temperature/10 + 0x30);
	LCD_DisplayChar(1,4,(int)Temperature%10 + 0x30);
        
}

void EXTI15_10_IRQHandler(void)
{
      if (EXTI->PR & (1<<15))
      {
          EXTI->PR |= (1<<15);
          switch (DISP_Flag)
          {
                case DISP_Alarm:         // Alarm 화면 일때 (DISP_Flag = 0)
                      USART1->CR1 &= ~(1<<13);	//  0x2000, USART1 DisEnable (Calculator때만 통신으로 값을 읽어올수있다)
                      ADC2->CR2   &= ~(1<<0);   //  ADC2 DisEnable           (Thermostat 일때만 동작)
                      BEEP();
                      DisplayTitle();    // 화면 초기화
                      ALARM();           // Alarm 화면 표시
                      SW_flag = 1;       // ALARM화면 에서만 SW.0,1이 작동
                      DISP_Flag++;       // DISP_Flag = 1로 계산기 화면 준비
                      break;
                      
                case DISP_Calculator:    // Calculator 화면 일때 (DISP_Flag = 1)
                      USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable (Calculator때만 통신으로 값을 읽어올수있다)
                      ADC2->CR2   &= ~(1<<0);   //  ADC2 DisEnable           (Thermostat 일때만 동작)
                      BEEP();
                      SW_flag = 0;       // Calculator 에서 SW.0,1이 작동중지
                      DisplayTitle();    // 화면 초기화
                      Calculator();      // Calculator 화면 표시
                      DISP_Flag++;       // DISP_Flag = 2로 온도계 화면 준비
                      break;
               
                case DISP_Thermostat:
                      USART1->CR1 &= ~(1<<13);	//  0x2000, USART1 DisEnable (Calculator때만 통신으로 값을 읽어올수있다)
                      ADC2->CR2   |=  (1<<0);   //  ADC2 Enable
                      BEEP();
                      SW_flag = 0;              // Calculator 에서 SW.0,1이 작동중지
                      DisplayTitle();           // 화면 초기화
                      Thermostat();             // Thermostat 화면 표시
                      DISP_Flag = 0;            // DISP_Flag = 0으로 알람 화면 준비
                      break;
                      
                      
          }
      }
}

void USART1_IRQHandler(void)	// 자리, 더하기 연산
{       
        char ch;
        
	if ( (USART1->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
	// #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
            ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장
            
            LCD_SetTextColor(RGB_RED);        //글자색
            if (operand_flag == 0)
            {
                operand1 = ch - 0x30;
		LCD_DisplayChar(1,1,operand1 + 0x30); 	// 수신된 문자를 LCD에 display
                operand_flag++;
            }
            else if (operand_flag == 1)
            {
                operand2 = ch - 0x30;
		LCD_DisplayChar(1,3,operand2 + 0x30); 	// 수신된 문자를 LCD에 display
                operand_flag++;
            }
            else if ((operand_flag == 2)&&(ch == '='))
            {
                result = operand1 + operand2;
                LCD_DisplayChar(1,5,result + 0x30); 	// 수신된 문자를 LCD에 display
                operand_flag = 0;
            }
	} 
	// DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 clear 할 필요없음 
}

void TIM7_IRQHandler(void)
{
      if ((TIM7->SR & 0x01) != RESET)	// Update interrupt flag (1s)
	{
                TIM7->SR &= ~(1<<0);    // Update interrupt flag 초기화
                
		time_m++;               // 분 증가
                if (time_m == 71)       // 분이 F가 되면
                {
                    //time_m = 48;
                    time_h++;           // 시 증가
                }
                ASCII_Time(&time_m);    // 아스키 코드 연산
                ASCII_Time(&time_h);    // 아스키 코드 연산
                
                // 현재 시간 표시
                LCD_SetTextColor(RGB_BLUE);        //글자색
                LCD_DisplayChar(0,15,time_h);
                LCD_DisplayChar(0,17,time_m);
                
                // 알람 작동
                if ((time_h == Fram_Read(1208))&&(time_m == Fram_Read(1209)))  // 현재시각과 Fram에 저장된 알람시간과 비교
                {
                    BEEP();
                    DelayMS(1000);
                    BEEP();
                    DelayMS(1000);
                    BEEP();
                    DelayMS(1000);
                }
	}
}

void _ADC_Init(void)
{   	// ADC2: PA1(pin 41)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// ENABLE GPIOA CLK (stm32f4xx.h 참조)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
						
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;	// ENABLE ADC2 CLK (stm32f4xx.h 참조)

	ADC->CCR &= ~(0X1F<<0);		// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |=  (1<<16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
        
	ADC2->CR1 &= ~(3<<24);		// RES[1:0]= 0x00 : 12bit Resolution
	ADC2->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable
	ADC2->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC

	ADC2->CR2 &= ~(1<<1);		// CONT=0: ADC_Continuous ConvMode Disable
	ADC2->CR2 |=  (2<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
	ADC2->CR2 |= (0x07<<24);	// EXTSEL[3:0]: ADC_ExternalTrig (TIM.3_CH1 OC mode)
	ADC2->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
	ADC2->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC2->SQR1 &= ~(0xF<<20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
					// 0b0000:1 conversion)
 	//Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
	ADC2->SQR3 |= (1<<0);		// SQ1[4:0]=0b0001 : CH1
        ADC2->SMPR2 |= (0x7<<(3*1));	// ADC2_CH1 Sample TIme_480Cycles (3*Channel_1)
 	//Channel selection, The Conversion Sequence of PIN1(ADC2_CH1) is first, Config sequence Range is possible from 0 to 17

	NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt

	//ADC2->CR2 |= (1<<0);		// ADON: ADC ON
}

void TIMER3_Init(void)
{
// TIM3_CH1 (PA6) : OC mode. 400ms 이벤트 발생
// Clock Enable : GPIOA & TIMER5
	RCC->AHB1ENR	|= RCC_AHB1ENR_GPIOAEN;	        // GPIOA Enable
	RCC->APB1ENR 	|= RCC_APB1ENR_TIM3EN;	// TIMER3 Enable 
    						
// PA6을 출력설정하고 Alternate function(TIM3_CH1)으로 사용 선언 
	GPIOA->MODER 	|= (2<<2*6);	// PA6 Output Alternate function mode					
	GPIOA->OSPEEDR 	|= (3<<2*6);	// PA6 Output speed (100MHz High speed)
	GPIOA->OTYPER	&= ~(1<<6);	// PA6 Output type push-pull (reset state)
	GPIOA->AFR[0]	|= (2<<4*6); 	// (AFR[0].(11~8)=0b0010): Connect TIM5 pins(PA6) to AF2(TIM3..5)
					

	// Assign 'Interrupt Period' and 'Output Pulse Period'
	TIM3->PSC = 840-1;	// Prescaler 84MHz/840 = 0.1MHz (10us)
	TIM3->ARR = 40000 - 1;	// Auto reload  : 10us * 4K = 400ms(period)

	// CR1 : Up counting
	TIM3->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM3->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM3->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM3->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM3->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM3->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM3->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
				// Center-aligned mode: The counter counts Up and DOWN alternatively

	// Event & Interrup Enable : UI  
	TIM3->EGR |= (1<<0);    // UG: Update generation    
    
	// Define the corresponding pin by 'Output'  
	TIM3->CCER |= (1<<0);	// CC1E=1: CC1 channel Output Enable
				// OC1(TIM3_CH1) Active: 해당핀을 통해 신호출력
	TIM3->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1으로 반전없이 출력)  

	// 'Mode' Selection : Output mode, toggle  
	TIM3->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output 
	TIM3->CCMR1 &= ~(1<<3); // OC1P=0: Output Compare 1 preload disable
	TIM3->CCMR1 |= (3<<4);	// OC1M=0b011: Output Compare 1 Mode : toggle
				// OC1REF toggles when CNT = CCR1

 	TIM3->CCR1 = 10;	// TIM3 CCR1 TIM3_Pulse
	TIM3->CR1 |= (1<<0);	// CEN: Enable the Tim3 Counter  					
}

void TIMER4_PWM_Init(void)
{   
// TIM4 CH1 : PB6 (164번 핀)
// Clock Enable : GPIOB & TIMER4
	RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
	RCC->APB1ENR 	|= (1<<2);	// TIMER4 CLOCK Enable 
    						
// PB6을 출력설정하고 Alternate function(TIM4_CH1)으로 사용 선언 : PWM 출력
	GPIOB->MODER 	|= (2<<6*2);	// PB6 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<6*2);	// PB6 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<6);	// PB6 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1<<6*2);	// PB6 Pull-up
 	GPIOB->AFR[0]	|= (2<<6*4);	// (AFR[0].(3~0)=0b0010): Connect TIM4 pins(PB6) to AF2(TIM3..5)
                                        
    
// TIM4 Channel 1 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM4->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM4->ARR	= 20000-1;	// Auto reload  (0.1ms * 20K = 2s : PWM Period)

	// Setting CR1 : 0x0000 (Up counting)
	TIM4->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM4->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enable 
	TIM4->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM4->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : use Edge-aligned mode(reset state)
				
	// Define the corresponding pin by 'Output'  
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM4->CCER	|= (1<<0);	// CC1E=1: OC1(TIM4_CH1) Active(Capture/Compare 1 output enable)
					// 해당핀(164번)을 통해 신호출력
	TIM4->CCER	&= ~(1<<1);	// CC1P=0: CC1 Output Polarity (OCPolarity_High : OC1으로 반전없이 출력)

	// Duty Ratio 
	TIM4->CCR1	= 1800;	// CCR1 value : DR = 90 %

	// 'Mode' Selection : Output mode, PWM 1
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2 <채널 설정>
	TIM4->CCMR1 &= ~(3<<0); // CC1S(CC1 channel)= '0b00' : Output 
	TIM4->CCMR1 |= (1<<3); 	// OC1PE=1: Output Compare 1 preload Enable
	TIM4->CCMR1 |= (7<<4);	// Output compare 1 mode: PWM 2 mode  < (6<<4):PWM 1 mode, (7<<4):PWM 2 mode>
	TIM4->CCMR1 |= (1<<7);	// OC1CE=1: Output compare 1 Clear enable
	
	//Counter TIM4 enable
	TIM4->CR1 |= (1<<0);	// CEN: Counter TIM4 enable
}

void TIMER7_Init(void)
{
// TIM7 : UI mode. 1s 이벤트 발생
// Clock Enable : TIMER7
	RCC->APB1ENR 	|= (1<<5);	// TIMER7 Enable 
    						
	// Assign 'Interrupt Period'
	TIM7->PSC = 8400-1;	// Prescaler 84MHz/840 = 0.1MHz (10us)
	TIM7->ARR = 10000;	// Auto reload  : 10us * 100K = 1000ms(period)

	// CR1 : Up counting
	TIM7->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM7->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM7->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM7->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM7->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM7->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM7->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
				// Center-aligned mode: The counter counts Up and DOWN alternatively

	// Event & Interrup Enable : UI  
	TIM7->EGR     |= (1<<0);    // UG: Update generation
        TIM7->DIER    |= (1<<0);   // UIE: Enable Tim7 Update interrupt
        NVIC->ISER[1] |= (1<<(55 - 32)); // Enable Timer7 global Interrupt on NVIC 

	TIM7->CR1 |= (1<<0);	// CEN: Enable the Tim7 Counter  					
}

void _EXTI_Init(void)
{
      // SW7 EXTI.15 초기화
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;  // RCC_AHB1ENR GPIOH Enable
      RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable System Configuration Controller Clock
      
      GPIOH->MODER &= ~(0xFFFF<<16);        // GPIOH 8~15 : Input mode (reset state)
      
      SYSCFG->EXTICR[3] |= (7<<12);         // EXTI15에 대한 소스 입력은 GPIOH로 설정
      
      EXTI->RTSR |= (1<<15);                // EXTI.15: Rising Trigger  Enable
      EXTI->IMR  |= (1<<15);                // EXTI.15 인터럽트 mask (Interrupt Enable) 설정
      
      NVIC->ISER[1] |= (1<<(40 - 32));      // Enable 'Global Interrupt EXTI15_10'
}

void USART1_Init(void)
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR	|= (1<<0);	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER	|= (2<<2*9);	// GPIOA PIN9 Output Alternate function mode					
	GPIOA->OSPEEDR	|= (3<<2*9);	// GPIOA PIN9 Output speed (100MHz Very High speed)
	GPIOA->AFR[1]	|= (7<<4);	// Connect GPIOA pin9 to AF7(USART1)
    
	// USART1 : RX(PA10)
	GPIOA->MODER 	|= (2<<2*10);	// GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR	|= (3<<2*10);	// GPIOA PIN10 Output speed (100MHz Very High speed
	GPIOA->AFR[1]	|= (7<<8);	// Connect GPIOA pin10 to AF7(USART1)

	RCC->APB2ENR	|= (1<<4);	// RCC_APB2ENR USART1 Enable
    
	USART_BRR_Configuration(9600); // USART Baud rate Configuration
    
	USART1->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	USART1->CR1	&= ~(1<<10);	// NO USART_Parity

	USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
	USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37번)
        
        /** USAET1을 계산기에서만 동작 시키게 하기 위해 초기화때 Enable 시키지 않았습니다 **/
	// USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}

void SerialSendChar(uint8_t Ch) // 1문자 보내기 함수
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
        while((USART1->SR & USART_SR_TXE) == RESET); 
	USART1->DR = (Ch & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendString(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialSendChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
}

// Baud rate 설정
void USART_BRR_Configuration(uint32_t USART_BaudRate)
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


void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) 설정 
    	RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
	
	//NAVI.SW(PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	= 0x00000000;	// GPIOI PIN8~PIN15 Input mode (reset state)
	GPIOI->PUPDR    = 0x00000000;	// GPIOI PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
}	

void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~(1<<9);	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
		for(; Dly; Dly--);
}

void DisplayTitle(void)
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim10);		//폰트 
	LCD_SetBackColor(RGB_WHITE);	//글자배경색
	LCD_SetTextColor(RGB_BLACK);	//글자색
        LCD_DisplayText(0,15," : ");
}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	if(key_flag == 0)
        		return key;
      		else
		{	DelayMS(10);
        		key_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
        		return 0xFF00;
      		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
}