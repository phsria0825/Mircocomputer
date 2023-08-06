//////////////////////////////////////////////////////////////////////
// ADC Interrupt by Triggering of TIM5 CH3
// 3.1
// ADC1:CH1 (PA1, pin 41) : ��������
// ��ȯ����: TIM8_CH1�� �̿��� 150ms �ֱ⸶�� start trigger ��� <Ÿ�̸Ӹ� ����ϸ� �ݵ�� OC��� ���>
//         TIM8���� �߻��ϴ� CCE�����ؼ� ADC�� �۵� <Ÿ�̸��� ���ͷ�Ʈ�� �۵� ��Ű�°� �ƴϴ�.>
// ��ȯ�Ϸᶧ EOC ���ͷ�Ʈ �߻��Ͽ� �Ƴ��α� ���а��� �����а��� LCD�� ǥ�� + TIMER14_CH1�� ���Ͽ� BUZZER�� ����? �� ���
// 3.2
// ADC3:CH16 : MCU���� �µ��踦 �̿��Ͽ� LCD�� �µ� ǥ��
//////////////////////////////////////////////////////////////////////

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
#define V_25            0.76    //MCU������ 25'C���� ���а� ����
#define Avg_slope       2.5     //��պ�ȭ ����

void _GPIO_Init(void);
void DisplayTitle(void);

void _ADC_Init(void);
uint16_t KEY_Scan(void);
void TIMER8_init(void);   // TIMER8_init �Լ�����
void TIMER14_Init(void);  // TIMER14_init �Լ�����

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

uint16_t ADC3_Value, Voltage, ADC1_Value;  // ADC3��, ���а�, ADC1��
float Temperature, MCU_Temperature, Volt, V_sense, target_ARR;  // ��¥�µ���, ��¥�µ���, ���а�, �µ����� ��, ��ǥARR��
int freq,SW7_Flag;   // ���ļ�, ����ġ7��_�÷���

uint8_t str[20];

void ADC_IRQHandler(void)
{
  if (ADC3->SR && ADC_SR_EOC == ADC_SR_EOC) // ADC3 EOC int (Timer8�� ���� 150ms�ֱ�� ����) : �������װ��� �о �ܺοµ�, ���ļ�, ���е��� ǥ���ϰ� Buzzer���
  {
       ADC3->SR &= ~(1<<1);		                   // EOC flag clear

	ADC3_Value = ADC3->DR;		                   // Reading ADC3 result
	Voltage = ADC3_Value * (3.3 * 100) / 4095;         // 3.3 : 4095 =  Volatge : ADC_Value 
                                                           // 100:  �Ҽ����Ʒ� ���ڸ����� ǥ���ϱ� ���� ��  
	LCD_DisplayChar(1,10,Voltage / 100 + 0x30);        // VOL ù���� �ڸ� ǥ��
	LCD_DisplayChar(1,12,Voltage % 100 / 10 + 0x30);   // VOL �Ҽ��� ù���� �ڸ� ǥ��
	LCD_DisplayChar(1,13,Voltage % 10 + 0x30);         // VOL �Ҽ��� �ι��� �ڸ� ǥ��
        
        
// �µ���� : T = 3.5 * V*V + 1 (�µ� ���� ��ȯ��)
        Volt =  ADC3_Value * 3.3 / 4095;             // �µ� ����� ���� float ������ ���� ���
        Temperature = 3.5 * (Volt * Volt) + 1;             // �µ� ���
        
        // LCD�� �µ��� ǥ��
        // int������ Temperature������ ĳ������ ���Ͽ� �ڸ��� ǥ�� 
        LCD_DisplayChar(2,10,(int) Temperature / 10 + 0x30);          // Temperature ���� �ڸ� ǥ��
	LCD_DisplayChar(2,11,(int) Temperature % 10 + 0x30);          // Temperature ���� �ڸ� ǥ��
	LCD_DisplayChar(2,13,(int) (Temperature*10) % 10 + 0x30);     // Temperature �Ҽ��� ù���� �ڸ� ǥ��
        
        
        // LCD�� ���ļ��� ǥ��
        freq = Temperature * 100;                   // �µ�, ���ļ� ��ȯ��
        LCD_DisplayChar(3,10, freq / 1000 + 0x30);                   // freq 1000�� �ڸ� ǥ��
	LCD_DisplayChar(3,11, freq % 1000 / 100 + 0x30);             // freq 100�� �ڸ� ǥ��
	LCD_DisplayChar(3,12, freq % 100 / 10 + 0x30);               // freq 10�� �ڸ� ǥ��
        LCD_DisplayChar(3,13, freq % 10 + 0x30);                     // freq 1�� �ڸ� ǥ��
        
        // LCD�� ���ļ�����? ǥ��
        LCD_SetBrushColor(RGB_GREEN);                                                          // ����� : �ʷϻ�
        LCD_DrawFillRect(10, 70, (139*freq)/(3910), 10);                                       // ����� ����       freq : x = 3910 : 1390�� ���� x��� (��ʽ�)
        LCD_SetBrushColor(RGB_YELLOW);                                                         // ����� : �ʷϻ�
        LCD_DrawFillRect((139*freq)/(3910) + 10, 70, 159 - (139*freq)/(3910) ,10);             // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ����
        
        target_ARR = 1000000 / freq;          // ���ļ� ��ȭ�� ���� ��ǥ ARR�� ���
                                                  // ARR == [��ǥ�ֱ� / PSC�� ����� CLK�� �ֱ�] == [PSC�� ����� CLK�� ���ļ� / ��ǥ ���ļ�]
        
        TIM14->ARR = (target_ARR/2) - 1;      // Auto reload  : TIM14_ARR ������Ʈ : ���ͷ�Ʈ�ֱ⳪ ��½�ȣ�� �ֱ� ����
                                                  // target_ARR/2 �� �ؾ� CC1�� PULSE�� �ֱⰡ freq�� �ֱ�� ��������. (CC1�� pulse�� �ֱ�� CC1I ���ͷ�Ʈ �ֱ��� 2��) 
        
        
        // <ȭ����� �׽�Ʈ��>
        //sprintf(str,"%.1f",Temperature);
       	//LCD_DisplayText(5,6,str);
  }
  
  
  if (ADC1->SR && ADC_SR_EOC == ADC_SR_EOC)   // ADC1 EOC int (SW������ ����) : MCU������ �µ��� �����Ͽ� LCD�� ǥ��
  {
        ADC1->SR &= ~(1<<1);                  // EOC flag clear
        
        ADC1_Value = ADC1->DR;                // Reading ADC1 result
        V_sense = ADC1_Value * 3.3 / 4095;    //  3.3 : 4095 =  Volatge : ADC1_Value
        
        MCU_Temperature = (((V_sense - V_25)*1000) / Avg_slope) + 25.0; // �µ� ��ȯ��
        
        // LCD�� ���οµ��� ǥ��
        // int������ MCU_Temperature������ ĳ������ ���Ͽ� �ڸ��� ǥ�� 
        LCD_DisplayChar(5,10,(int) MCU_Temperature / 10 + 0x30);          // MCU_Temperature ���� �ڸ� ǥ��
	LCD_DisplayChar(5,11,(int) MCU_Temperature % 10 + 0x30);          // MCU_Temperature ���� �ڸ� ǥ��
	LCD_DisplayChar(5,13,(int) (MCU_Temperature*10) % 10 + 0x30);     // MCU_Temperature �Ҽ��� ù���� �ڸ� ǥ��
  }
                                                          
       	// NO SWSTART !!!
}

int main(void)
{
	LCD_Init();	// LCD ���� �Լ�
	DelayMS(10);	// LCD���� ������
 	DisplayTitle();	//LCD �ʱ�ȭ�鱸�� �Լ�
      
        _GPIO_Init();
        TIMER14_Init();
	_ADC_Init();
        TIMER8_init();
        
        //Starts conversion of regular channels
        ADC1->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30)
        
 	while(1)
	{   
		switch(KEY_Scan())
		{
			case SW7_PUSH  ://SW7
                          if (SW7_Flag == 1){ // BUZZER�� �����Ҷ�
                              TIM14->CCER |= (1<<0);	// CC1E=1: CC1 channel Output Enable
                              SW7_Flag = 0;   // Flag �ʱ�ȭ
                          }
                          else{               // BUZZER�� ���۾��Ҷ�
                              TIM14->CCER &= ~(1<<0);	// CC1E=1: CC1 channel Output disable
                              SW7_Flag = 1;   // Flag ��
                          }
 			break;
        	}
	}
}

void _ADC_Init(void)
{   	
// ADC3: PA1(pin 41) : ��������
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h ����)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE				
	RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;	// ENABLE ADC3 CLK (stm32f4xx.h ����)

	ADC->CCR &= ~(0X1F<<0);		// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= (1<<16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)

	ADC3->CR1 &= ~(3<<24);		// RES[1:0]= 0x00 : 12bit Resolution
	ADC3->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable
	ADC3->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC

	ADC3->CR2 &= ~(1<<1);		// CONT=0: ADC_Continuous ConvMode Disable
	ADC3->CR2 |=  (2<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
	ADC3->CR2 |= (0x0D<<24);	// EXTSEL[3:0]: ADC_ExternalTrig (TIM.8 OC mode)
	ADC3->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
	ADC3->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC3->SQR1 &= ~(0xF<<20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
					// 0b0000:1 conversion)
        
	ADC3->SQR3 |= (1<<0);		// SQ1[4:0]=0b0001 : CH1
        ADC3->SMPR2 |= (0x7<<3);	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
        
// ADC1: MCU������ �µ�����
        // ADC1 ��� �̿� (ADC1_CH16)
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // (1<<8) ENABLE ADC1 CLK (stm32f4xx.h ����)
        
        ADC->CCR &= ~(0X1F<<0);		// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= (1<<16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
        ADC->CCR |= (1<<23);            // TSVREFE=1 : Temperrature sensor and Vrefint chammel enable (���οµ� ���� Enable)
        
        ADC1->CR1 &= ~(3<<24);          // RES[1:0]=0b00 : 12bit Resolution
        ADC1->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable
	ADC1->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC
        
        ADC1->CR2 |= (1<<30);	        //SWSTART : STARTS CONVERSION OF REGULAR CHANNELS
	ADC1->CR2 &= ~(1<<1);		// CONT=0: ADC_Continuous ConvMode Disable
	ADC1->CR2 &= ~(1<<11);        // ALIGN=0: ADC_DataAlign_Right
	ADC1->CR2 &= ~(1<<10);	// EOCS=1: The EOC bit is set at the end of each regular conversion
        
        ADC1->SQR1 &= ~(0xF<<20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
						// 0b0000:1 conversion)
						// Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, 
						// Config sequence Range is possible from 0 to 17
        
        ADC1->SQR3 |= (0x10<<0);        // SQ1[4:0]=0b0001 : CH16  
	ADC1->SMPR1 |= (0x7<<18);	// ADC1_CH16 Sample TIme_480Cycles (3*Channel_1)
                                                //Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
	
// ADC ���ۺ�
        NVIC->ISER[0] |= (1 << 18);	// Enable ADC global Interrupt
	ADC3->CR2 |= (1<<0);		// ADON: ADC ON
	ADC1->CR2 |= (1<<0);		// ADON=1: ADC ON
        
}

void TIMER8_init(void){

// TIM8_CH1 (PI5) : OC mode. 150ms �̺�Ʈ �߻� // 
        RCC->AHB1ENR |= (1<<8);  // GPIOI Enable
        RCC->APB2ENR |= (1<<1);  // TIMER8 Enable
        
// PI5�� ��¼����ϰ� Alternate function(TIM8_CH1)���� ��� ���� 
	GPIOI->MODER 	|= (2<<10);	// PI5 Output Alternate function mode					
	GPIOI->OSPEEDR 	|= (3<<10);	// PI5 Output speed (100MHz High speed)
	GPIOI->OTYPER	&= ~(1<<5);	// PI5 Output type push-pull (reset state)
        GPIOI->AFR[0]   |= (3<<20);     // TIM8_CH1 Connect PI5
        
// Assign 'Interrupt Period' and 'Output Pulse Period'
        TIM8->PSC = 16800 - 1;          // Prescaler 168MHz/16800 = 10kHz (1us)
        TIM8->ARR = 1500 - 1;           // Auto reload  : 0.1ms * 1500 = 150ms(period)
        
        // CR1 : Up counting
        TIM8->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM8->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM8->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM8->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM8->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM8->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM8->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
				// Center-aligned mode: The counter counts Up and DOWN alternatively
        
        // Event & Interrup Enable : UI  
	TIM8->EGR |= (1<<0);    // UG: Update generation
        
        // Define PI5 by 'Output'  
        TIM8->CCER |= (1<<0);   // CC1E=1: CC1 channel Output Enable
        TIM8->CCER &= ~(1<<1);  // CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1���� �������� ���)
        
        TIM8->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output
        TIM8->CCMR1 &= ~(1<<3); // OC1P=0: Output Compare 1 preload disable
        TIM8->CCMR1 |= (3<<4);	// OC1M=0b011: Output Compare 1 Mode : toggle
				// OC1REF toggles when CNT = CCR1
        TIM8->CCR1 = 10;	// TIM8 CCR1 TIM8_Pulse
        TIM8->BDTR |= (1<<15);  // ��������� TIM8�� OC�� ����
        TIM8->CR1 |= (1<<0);    // CEN: Enable the Tim8 Counter 
        
}


void TIMER14_Init(void)
{
// TIM14_CH1 (PF9, PIN27) : OC mode. Pulse ���
// Clock Enable : GPIOF & TIMER14
	RCC->AHB1ENR |= (1<<5);  // GPIOF Enable
        RCC->APB1ENR |= (1<<8);  // TIMER14 Enable
    						
// PF9�� ��¼����ϰ� Alternate function(TIM4_CH1)���� ��� ���� 							
	GPIOF->MODER    |= (2<<18);	// Alternate function mode
	GPIOF->OSPEEDR 	|= (1<<18);	// Output speed
	GPIOF->OTYPER	&= ~(1<<9);	// Output type push-pull (reset state)
        GPIOF->AFR[1]   |= (9<<4);      // AF9 ����

	// Setting CR1 : 0x0000
	TIM14->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM14->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                                          // Counter Overflow/Underflow, 
                                          // Setting the UG bit Set,
                                          // Update Generation through the slave mode controller 
                                          // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
        
	TIM14->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
                                          // Counter Overflow/Underflow, 
                                          // Setting the UG bit Set,
                                          // Update Generation through the slave mode controller 
                                          // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM14->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM14->CR1 &= ~(1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enalbe 
	TIM14->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM14->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)

	// Setting the Period
	TIM14->PSC = 84 - 1;	// Prescaler=84, 84MHz/84 = 1MHz (1us)

	// Update(Clear) the Counter
	TIM14->EGR |= (1<<0);    // UG: Update generation    

// Output Compare ���� <CC ��带 �����ϴ� ��������>
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2 <ä�� ����>
        // Ch1�� ����ϴϱ� CCR1
	TIM14->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output 
	TIM14->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM14->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable(CCR1�� �������� ���ο� ���� loading ����) 
	TIM14->CCMR1 |= (3<<4);	// OC1M=0b001 (Output Compare 1 Mode : toggle)
				// OC1REF toggle when CNT = CCR1
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM14->CCER &= ~(1<<0);	// CC1E=1: CC1 channel Output Enable
				// OC1(TIM14_CH1) Active: �ش���(27��)�� ���� ��ȣ���
	TIM14->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1���� �������� ���)  

	// CC1I(CC ���ͷ�Ʈ) ���ͷ�Ʈ �߻��ð� �Ǵ� ��ȣ��ȭ(���)�ñ� ����: ��ȣ�� ����(phase) ����
	TIM14->CCR1 = 10;	// TIM14 CCR1 TIM14_Pulse,  �޽��� ���۽ð��� �޶�����.
        
	TIM14->CR1 |= (1<<0);	// CEN: Enable the Tim14 Counter				
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
	
	//NAVI.SW(PORT I) ����
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	= 0x00000000;	// GPIOI PIN8~PIN15 Input mode (reset state)
	GPIOI->PUPDR    = 0x00000000;	// GPIOI PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
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
	LCD_Clear(RGB_YELLOW);
	LCD_SetFont(&Gulim10);		          // ��Ʈ 
        LCD_SetBackColor(RGB_YELLOW);             // LCD ȭ�� ����
	LCD_SetTextColor(RGB_BLACK);	          // ���ڻ�
       	LCD_DisplayText(0,0,"2018130048 PSY");    // �й�, �̸� ǥ��
      	LCD_DisplayText(1,0,"EXT VOL :  .   V");  // ����ǥ��
        LCD_DisplayText(2,0,"EXT TMP :   .  C");  // �µ�ǥ��
        LCD_DisplayText(3,0,"FREQ :         Hz"); // ���ļ� ǥ��
        LCD_DisplayText(5,0,"INT TMP :   .  C"); // MCU ���οµ� ǥ��
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

/*
MCU ���� �µ���
ADC. CH 16 : SQR3 |= (16<<0);
*/