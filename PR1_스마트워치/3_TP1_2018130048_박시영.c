////////////////////////////////////////////////////////////////////////////////
// *****************************************************************************
// PR1: ����Ʈ ��ġ
// ������ : 2018130048 �ڽÿ�
// ������ : 2022.12.08
// �������� : <1.�˶�, 2.����, 3.�µ��踦 EXTI(SW7)�� �̿��Ͽ� ȭ������>
//          void EXTI15_10_IRQHandler()���� ȭ�� ��ȯ
//
//          1.�˶� ȭ�鿡�� �˶��ð� ������, �ð��� ����ð��� ������ �˶��� �︰��.
//            main�Լ��� while(1) : �˶��ð� ����, 
//            void ASCII_Time()  : 10������ �ƽ�Ű �ڵ� ������ ��ȯ
//            void TIM7_IRQHandler(): ����ð��� �˶��� �︮�� ���
//
//          2.���� ȭ�鿡�� PC�� USART1����� ���Ͽ� �������� �����Ѵ�. 
//            USART1_IRQHandler : ������ ������ PC���� ������ �����Ѵ��� LCD���
//
//          3.�µ��� ȭ�鿡�� �µ������ ���� �� ǥ��
//            ADC_IRQHandler(TIM3�� ����): �µ��� ������ ��ȯ, LCD�� ����׷���, ����/�𷯰���, �µ�ǥ��
//            TIM4_CH1  :  PWM���� ����/�� ������ ���� �������
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

#define DISP_Alarm      0       // DISP_Flag���� Alarm ȭ���� ǥ���ϱ����� �� ����
#define DISP_Calculator 1       // DISP_Flag���� Calculator ȭ���� ǥ���ϱ����� �� ����
#define DISP_Thermostat 2       // DISP_Flag���� Thermostat ȭ���� ǥ���ϱ����� �� ����

void _GPIO_Init(void);
void DisplayTitle(void);
void _EXTI_Init(void);    // SW7 ���ͷ�Ʈ �ʱ�ȭ �Լ� ����
void _ADC_Init(void);     // ADC �ʱ�ȭ �Լ� ����
uint16_t KEY_Scan(void);  
void TIMER7_Init(void);   // ����ð� Ÿ�̸� ����
void TIMER3_Init(void);   // ADC���� Ÿ�̸� ����
void TIMER4_PWM_Init(void); // ����,�� ���� Ÿ�̸� ����

// ��Ű��� �Լ� ����
void USART1_Init(void);
void USART_BRR_Configuration(uint32_t USART_BaudRate);
void SerialSendChar(uint8_t c);
void SerialSendString(char *s);

// ����Ʈ ��ġ�� �����Լ� ����
void ALARM(void);
void Calculator(void);
void Thermostat(void);
void ASCII_Time(char *time);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

uint16_t ADC_Value, Voltage;
uint16_t DISP_Flag = 1;       // ȭ�� ���� Flag
char time_h = 70,time_m = 65; // �ð� �ʱ� ��
char operand_flag = 0, operand1, operand2, result; // Calculator�� ������ �÷���, �ǿ�����1, �ǿ�����2, ���
char H, C;  // ����, �� �ʱ�ȭ
uint8_t SW_flag = 1;          // SW.0,1�� �۵� ����
uint8_t str[20];

int main(void)
{
  
	LCD_Init();	// LCD ���� �Լ�
	DelayMS(10);	// LCD���� ������
 	DisplayTitle();	//LCD �ʱ�ȭ�鱸�� �Լ�
        _EXTI_Init();   // EXTI �ʱ�ȭ
        _GPIO_Init();  
	_ADC_Init();    // ADC�ʱ�ȭ
        USART1_Init();  // USART1  �ʱ�ȭ
        
        
        Fram_Init();            // FRAM �ʱ�ȭ H/W �ʱ�ȭ
	Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
        
        ALARM();                // �ʱ�ȭ�� ǥ��
        
        // Ÿ�̸ӵ� �ʱ�ȭ
        TIMER7_Init();
        TIMER3_Init();
        TIMER4_PWM_Init();

        char  Alarm_h = Fram_Read(1208), Alarm_m = Fram_Read(1209);                  // Alarm�� ��, �� ���� ��
 	while(1)
	{
          
          if (SW_flag == 1)    // DISP_Alarm �϶�, �� �˶� ȭ�� �϶��� ����
          {
              switch(KEY_Scan())
              {
                  case SW0_PUSH:                      // SW0�� ������
                      Alarm_h++;                      // �˶��� �ð� ����
                      ASCII_Time(&Alarm_h);            // �ð����� ���  (void ASCII_Time(char time) ����)
                      LCD_SetTextColor(RGB_RED);        //���ڻ�
                      LCD_DisplayChar(1,6,Alarm_h);   // �� ǥ��
                      BEEP();
                      break;
                      
                  case SW1_PUSH:                      // SW1�� ������
                      Alarm_m++;                      // �˶��� �� ����
                      ASCII_Time(&Alarm_m);            // �ð����� ���  (void ASCII_Time(char time) ����)
                      LCD_SetTextColor(RGB_RED);        //���ڻ�
                      LCD_DisplayChar(1,8,Alarm_m);   // �� ǥ��
                      BEEP();
                      break;
                      
                  case SW2_PUSH:                      // SW2�� ������
                      Fram_Write(1208,Alarm_h);       // �˶� �ð��� Fram ����
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
      // �˶� ���÷��� 
      LCD_DisplayText(0,0,"1.ALARM(PSY)");
      LCD_DisplayText(1,0,"Alarm  : ");
      
      LCD_SetTextColor(RGB_RED);        //���ڻ�
      LCD_DisplayChar(1,6,Fram_Read(1208));
      LCD_DisplayChar(1,8,Fram_Read(1209));
}

void Calculator(void)
{
      // ���� ���÷��� 
      LCD_SetTextColor(RGB_BLACK);	//���ڻ�
      LCD_DisplayText(0,0,"2.Calculator");
      LCD_DisplayText(1,0,"  + = ");
      LCD_SetTextColor(RGB_RED);	//���ڻ�
      LCD_DisplayChar(1,1,0x30 + 0);
      LCD_DisplayChar(1,3,0x30 + 0);
      LCD_DisplayChar(1,5,0x30 + 0);
}

void Thermostat(void)
{
      // // �µ��� ���÷��� 
      LCD_SetTextColor(RGB_BLACK);	//���ڻ�
      LCD_DisplayText(0,0,"3.Thermostat");
      LCD_DisplayText(1,0,"T:    ");
      LCD_DisplayText(2,0,"H:  C: ");
}

//* �ƽ�Ű �ڵ�� �ð��� ����ϴ� �Լ� *//
void ASCII_Time(char *time)
{
      if (*time == 71 )  // time�� ���ڻ����� G���Ǹ� 0���� �ʱ�ȭ
      {
          *time = 48;     // 0���� �ʱ�ȭ
      }
                      
      if (*time == 58)   // time�� ���ڻ����� 10�̵Ǹ� A�� �ʱ�ȭ
      {
          *time += 7;    // �ƽ�Ű�ڵ忡�� 10�� A�� ���̰� 7�̴�
      }
}

void ADC_IRQHandler(void)
{
	ADC2->SR &= ~(1<<1);		// EOC flag clear
        
        float Temperature;              // �µ��� ����
	ADC_Value = ADC2->DR;		// Reading ADC result

       	//sprintf(str,"%4d",ADC_Value);	// ADC result : 12bits (0~4095)
       	//LCD_DisplayText(0,6,str);
	Voltage = ADC_Value * (3.3 * 100) / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
                                                    // 100:  �Ҽ����Ʒ� ���ڸ����� ǥ���ϱ� ���� ��  
        
        /** ���� ������ **/
        /// (x1, y1) == (0, -10), (x2, y2) == (330, 40) �־��� ���� : (0 ~ 330 : Voltage * 100 <input ��>) (-10 ~ 40 : �µ��� <output ��>)
        /// ���� : result = {(y2 - y1) / (x2 - x1)} * (Voltage - x1) + y1
        /// Temperature = {(40 - (-10)) / (330 - 0)} * (Voltage - 0) + (-10) == 0.15 * Voltage -10
        
        Temperature = 0.15 * (Voltage) -10;
        
        // �µ��׷����� ���� ���� ���� (Temperature ���� ���� �� ����)
        if (Temperature >= -10 && Temperature <= 0)
        {
            H = 2;  // ���� ����
            C = 0;  // �� ����
            LCD_SetBrushColor(RGB_BLUE);
        }
        else if (Temperature >= 1 && Temperature <= 10)
        {   
            H = 1;  // ���� ����
            C = 0;  // �� ����
            LCD_SetBrushColor(RGB_BLUE);
        }
        else if (Temperature >= 11 && Temperature <= 20)
        {
            H = 0;  // ���� ����
            C = 0;  // �� ����
            LCD_SetBrushColor(RGB_GREEN);
        }
        else if (Temperature >= 21 && Temperature <= 30)
        {
            H = 0;  // ���� ����
            C = 1;  // �� ����
            LCD_SetBrushColor(RGB_RED);
        }
        else if (Temperature >= 31 && Temperature <= 40)
        {
            H = 0;  // ���� ����
            C = 2;  // �� ����
            LCD_SetBrushColor(RGB_RED);
        }
        
        // �µ����� �׷��� ǥ��
        LCD_DrawFillRect(50, 22, (42 + 1.2 * Temperature),10);
        LCD_SetBrushColor(RGB_WHITE);
        LCD_DrawFillRect((42 + 1.2 * Temperature) + 50, 22, 140 - (42 + 1.2 * Temperature),10);
        
        // ����, �� ����? ǥ��
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
        LCD_SetTextColor(RGB_RED);        //���ڻ�
        LCD_DisplayChar(2,2,0x30 + H);
        LCD_SetTextColor(RGB_BLUE);        //���ڻ�
        LCD_DisplayChar(2,6,0x30 + C);
        
        // ����, �� ���� TIM4_PWM
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
        
        // �µ��� ��ȣ�� �����ϴ�  if��
        LCD_SetTextColor(RGB_GREEN);        //���ڻ�
        if (Temperature < 0)
        {
            LCD_DisplayChar(1,2,'-');
            Temperature = -Temperature;  // �µ��� ������ ��, ����� �ٲ㼭 LCD�� ǥ���ϱ����� -
        }
        else
        {
            LCD_DisplayChar(1,2,' ');
        }
        
        // �µ� ǥ��
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
                case DISP_Alarm:         // Alarm ȭ�� �϶� (DISP_Flag = 0)
                      USART1->CR1 &= ~(1<<13);	//  0x2000, USART1 DisEnable (Calculator���� ������� ���� �о�ü��ִ�)
                      ADC2->CR2   &= ~(1<<0);   //  ADC2 DisEnable           (Thermostat �϶��� ����)
                      BEEP();
                      DisplayTitle();    // ȭ�� �ʱ�ȭ
                      ALARM();           // Alarm ȭ�� ǥ��
                      SW_flag = 1;       // ALARMȭ�� ������ SW.0,1�� �۵�
                      DISP_Flag++;       // DISP_Flag = 1�� ���� ȭ�� �غ�
                      break;
                      
                case DISP_Calculator:    // Calculator ȭ�� �϶� (DISP_Flag = 1)
                      USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable (Calculator���� ������� ���� �о�ü��ִ�)
                      ADC2->CR2   &= ~(1<<0);   //  ADC2 DisEnable           (Thermostat �϶��� ����)
                      BEEP();
                      SW_flag = 0;       // Calculator ���� SW.0,1�� �۵�����
                      DisplayTitle();    // ȭ�� �ʱ�ȭ
                      Calculator();      // Calculator ȭ�� ǥ��
                      DISP_Flag++;       // DISP_Flag = 2�� �µ��� ȭ�� �غ�
                      break;
               
                case DISP_Thermostat:
                      USART1->CR1 &= ~(1<<13);	//  0x2000, USART1 DisEnable (Calculator���� ������� ���� �о�ü��ִ�)
                      ADC2->CR2   |=  (1<<0);   //  ADC2 Enable
                      BEEP();
                      SW_flag = 0;              // Calculator ���� SW.0,1�� �۵�����
                      DisplayTitle();           // ȭ�� �ʱ�ȭ
                      Thermostat();             // Thermostat ȭ�� ǥ��
                      DISP_Flag = 0;            // DISP_Flag = 0���� �˶� ȭ�� �غ�
                      break;
                      
                      
          }
      }
}

void USART1_IRQHandler(void)	// �ڸ�, ���ϱ� ����
{       
        char ch;
        
	if ( (USART1->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
	// #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
            ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
            
            LCD_SetTextColor(RGB_RED);        //���ڻ�
            if (operand_flag == 0)
            {
                operand1 = ch - 0x30;
		LCD_DisplayChar(1,1,operand1 + 0x30); 	// ���ŵ� ���ڸ� LCD�� display
                operand_flag++;
            }
            else if (operand_flag == 1)
            {
                operand2 = ch - 0x30;
		LCD_DisplayChar(1,3,operand2 + 0x30); 	// ���ŵ� ���ڸ� LCD�� display
                operand_flag++;
            }
            else if ((operand_flag == 2)&&(ch == '='))
            {
                result = operand1 + operand2;
                LCD_DisplayChar(1,5,result + 0x30); 	// ���ŵ� ���ڸ� LCD�� display
                operand_flag = 0;
            }
	} 
	// DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
}

void TIM7_IRQHandler(void)
{
      if ((TIM7->SR & 0x01) != RESET)	// Update interrupt flag (1s)
	{
                TIM7->SR &= ~(1<<0);    // Update interrupt flag �ʱ�ȭ
                
		time_m++;               // �� ����
                if (time_m == 71)       // ���� F�� �Ǹ�
                {
                    //time_m = 48;
                    time_h++;           // �� ����
                }
                ASCII_Time(&time_m);    // �ƽ�Ű �ڵ� ����
                ASCII_Time(&time_h);    // �ƽ�Ű �ڵ� ����
                
                // ���� �ð� ǥ��
                LCD_SetTextColor(RGB_BLUE);        //���ڻ�
                LCD_DisplayChar(0,15,time_h);
                LCD_DisplayChar(0,17,time_m);
                
                // �˶� �۵�
                if ((time_h == Fram_Read(1208))&&(time_m == Fram_Read(1209)))  // ����ð��� Fram�� ����� �˶��ð��� ��
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
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// ENABLE GPIOA CLK (stm32f4xx.h ����)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
						
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;	// ENABLE ADC2 CLK (stm32f4xx.h ����)

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
// TIM3_CH1 (PA6) : OC mode. 400ms �̺�Ʈ �߻�
// Clock Enable : GPIOA & TIMER5
	RCC->AHB1ENR	|= RCC_AHB1ENR_GPIOAEN;	        // GPIOA Enable
	RCC->APB1ENR 	|= RCC_APB1ENR_TIM3EN;	// TIMER3 Enable 
    						
// PA6�� ��¼����ϰ� Alternate function(TIM3_CH1)���� ��� ���� 
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
				// OC1(TIM3_CH1) Active: �ش����� ���� ��ȣ���
	TIM3->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1���� �������� ���)  

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
// TIM4 CH1 : PB6 (164�� ��)
// Clock Enable : GPIOB & TIMER4
	RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
	RCC->APB1ENR 	|= (1<<2);	// TIMER4 CLOCK Enable 
    						
// PB6�� ��¼����ϰ� Alternate function(TIM4_CH1)���� ��� ���� : PWM ���
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
					// �ش���(164��)�� ���� ��ȣ���
	TIM4->CCER	&= ~(1<<1);	// CC1P=0: CC1 Output Polarity (OCPolarity_High : OC1���� �������� ���)

	// Duty Ratio 
	TIM4->CCR1	= 1800;	// CCR1 value : DR = 90 %

	// 'Mode' Selection : Output mode, PWM 1
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2 <ä�� ����>
	TIM4->CCMR1 &= ~(3<<0); // CC1S(CC1 channel)= '0b00' : Output 
	TIM4->CCMR1 |= (1<<3); 	// OC1PE=1: Output Compare 1 preload Enable
	TIM4->CCMR1 |= (7<<4);	// Output compare 1 mode: PWM 2 mode  < (6<<4):PWM 1 mode, (7<<4):PWM 2 mode>
	TIM4->CCMR1 |= (1<<7);	// OC1CE=1: Output compare 1 Clear enable
	
	//Counter TIM4 enable
	TIM4->CR1 |= (1<<0);	// CEN: Counter TIM4 enable
}

void TIMER7_Init(void)
{
// TIM7 : UI mode. 1s �̺�Ʈ �߻�
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
      // SW7 EXTI.15 �ʱ�ȭ
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;  // RCC_AHB1ENR GPIOH Enable
      RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable System Configuration Controller Clock
      
      GPIOH->MODER &= ~(0xFFFF<<16);        // GPIOH 8~15 : Input mode (reset state)
      
      SYSCFG->EXTICR[3] |= (7<<12);         // EXTI15�� ���� �ҽ� �Է��� GPIOH�� ����
      
      EXTI->RTSR |= (1<<15);                // EXTI.15: Rising Trigger  Enable
      EXTI->IMR  |= (1<<15);                // EXTI.15 ���ͷ�Ʈ mask (Interrupt Enable) ����
      
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
	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37��)
        
        /** USAET1�� ���⿡���� ���� ��Ű�� �ϱ� ���� �ʱ�ȭ�� Enable ��Ű�� �ʾҽ��ϴ� **/
	// USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}

void SerialSendChar(uint8_t Ch) // 1���� ������ �Լ�
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty���� ������ ��� ���(�۽� ������ ���±��� ���)
        while((USART1->SR & USART_SR_TXE) == RESET); 
	USART1->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}

void SerialSendString(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialSendChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}

// Baud rate ����
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


void _GPIO_Init(void)
{
	// LED (GPIO G) ����
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) ���� 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) ���� 
    	RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
	
	//NAVI.SW(PORT I) ����
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
	LCD_SetFont(&Gulim10);		//��Ʈ 
	LCD_SetBackColor(RGB_WHITE);	//���ڹ���
	LCD_SetTextColor(RGB_BLACK);	//���ڻ�
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