///////////////////////////////////////////////////////////////////////////////////////////////
// ������: HW.2 �����нð�����(StopWatch�������)
// ��������: �����нð�, �����ġ�� TIMER�� OC(OUTPUT COMPARE), overflow(UPDATE EVENT)�� �̿��Ͽ� ����
//      // ������ �ð�� TIM.3 <APB1: 84MHz, 16bit(General perpose), 4ch> -> UPDATE EVENT mode
//      // �����ġ��    TIM.4 <APB1: 84MHz, 16bit(General perpose), 4ch> -> OUTPUT COMPARE mode
// ����� �ϵ����(���): GPIO, EXTI, GLCD, TIMER, BUZZER
// ������: 2022. 10. 04
// ������ Ŭ����:  ����Ϲ�
// �й�: 2018130048
// �̸�: �ڽÿ�
///////////////////////////////////////////////////////////////////////////////////////////////

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

void _RCC_Init(void);
void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);
void TIMER3_Init(void);     // TIM3: Overflow mode �Լ�����
void TIMER4_OC_Init(void);  // TIM4: Oc mode �Լ�����

void DisplayInitScreen(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

// �����нð� ����
int time100ms;          // 1/10��
int time1s = 7;         // 1��
int time10s;            // 10��
int time_24h = 19;      // 24h�ð� : AM,PM�� ��Ÿ�������� ����ϴ� ����

// Ÿ�̸� ����
int stop_watch[2];      // 10��, 1�� �迭  stop_watch[2]={ time10s, time1s }
int stop_watch_count;   // 1/10�ʸ� ī��Ʈ�ϴ� ����
int stop_watch100ms;    // 1/10��

uint8_t	SW4_Flag, SW5_Flag; // �����ġ ����,  �����ġ �ʱ�ȭ �÷���

int main(void)
{
    _GPIO_Init();  		// GPIO �ʱ�ȭ
    _EXTI_Init();		// �ܺ����ͷ�Ʈ �ʱ�ȭ
    LCD_Init();		        // GLCD �ʱ�ȭ
    DelayMS(10);			
    BEEP();			// Beep �ѹ� 

    GPIOG->ODR &= 0xFF00;	// �ʱⰪ: LED0~7 Off
    DisplayInitScreen();	// LCD �ʱ�ȭ��
    
    TIMER3_Init();	        // ����Ÿ�̸�(TIM3) �ʱ�ȭ : up counting mode
    TIMER4_OC_Init();           // ����Ÿ�̸�(TIM4) �ʱ�ȭ : output compare mode


    while(1)
    {
        switch(KEY_Scan())
        {
            
        }  // switch(KEY_Scan())
        
        //EXTI SW4�� High���� Low�� �� �� (Falling edge Trigger mode)
        if(SW4_Flag)
        {

        }
        //EXTI SW5�� High���� Low�� �� �� (Rising edge Trigger mode)
        if(SW5_Flag)
        {

        }
    }
}

void TIMER3_Init(void)
{
	RCC->APB1ENR |= 0x02;	// RCC_APB1ENR TIMER3 Enable
        //RCC->APB1ENR |= (1<<1);
        
	// Setting CR1 : 0x0000 
	TIM3->CR1 &= ~(1<<4);  // DIR=0(Up counter)(reset state)
	TIM3->CR1 &= ~(1<<1);  // UDIS=0(Update event Enabled): By one of following events
                                            //  Counter Overflow/Underflow, 
                                            //  Setting the UG bit Set,
                                            //  Update Generation through the slave mode controller 
                                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	
        TIM3->CR1 &= ~(1<<2); // URS=0(Update Request Source  Selection):  By one of following events
                                           //	Counter Overflow/Underflow, 
                                           //   Setting the UG bit Set,
                                           //	Update Generation through the slave mode controller 
                                           // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	
        TIM3->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	
        TIM3->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	
        TIM3->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	
        TIM3->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
                                                // Center-aligned mode: The counter counts UP and DOWN alternatively


    // Deciding the Period
	TIM3->PSC = 840 - 1;	// Prescaler 84,000,000Hz/840 = 10,0000 Hz (0.01ms)  (1~65536)
	TIM3->ARR = 10000 - 1;	// Auto reload  0.01ms * 10000 = 100ms

   	// Clear the Counter
	TIM3->EGR |= (1<<0);	// UG(Update generation)=1 
                        // Re-initialize the counter(CNT=0) & generates an update of registers   

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1<<29); // Enable Timer3 global Interrupt
 	TIM3->DIER |= (1<<0);	// Enable the Tim3 Update interrupt

	TIM3->CR1 |= (1<<0);	// Enable the Tim3 Counter (clock enable)   
}

void TIM3_IRQHandler(void)  	// 1ms Interrupt
{
  if((TIM3->SR & 0x01) != RESET){
	TIM3->SR &= ~(1<<0);	// Interrupt flag Clear  							   	
        time100ms++;            // 1/10s ī��Ʈ
        
     // 1s�� ī��Ʈ�ϴ� if
        if (time100ms  > 9){    // 1/10s�� 9�� �ѱ��
            time100ms  =  0;    // 1/10s�� 0���� �ʱ�ȭ
            time1s++;           // 1s�� 1 ����
            time_24h++;         // 24h �ð� ����
        }
        
     // 10s�� ī��Ʈ�ϴ� if   
        if (time1s > 9){        // 1s�� 9�� �ѱ��
             time1s = 0;        // 1s�� 0���� �ʱ�ȭ
             time10s++;         // 10s�� 1 ����
         }
        
     // PM 11:9 -> AM 00:0 �� ��������� if (���Ŀ��� �������� �Ѿ�� if��)
        if(time_24h == 24){     // 24�ð����� ǥ�������� 24�ð� �Ǹ�
            time_24h = 0;       // time_24h == 24�̸� �Ϸ簡 �����⶧���� 0���� �ʱ�ȭ 
            time1s = 0;         // 00:0�� ��������� 1s�� 0���� ����
            time10s = 0;        // 00:0�� ��������� 10s�� 0���� ����
            LCD_SetTextColor(RGB_BLACK);// ���ڻ� : BLACK
            LCD_DisplayText(3, 6, "AM");  // LCD�� AM ǥ��
        }
        
     // AM -> PM�� ��������� if (�������� ���ķ� �Ѿ�� if��)
        else if(time_24h == 12){         // 24�ð����� ǥ�������� 12�ð� �Ǹ�
            LCD_SetTextColor(RGB_BLACK); // ���ڻ� : BLACK
            LCD_DisplayText(3, 6, "PM"); // LCD�� PM ǥ��
        }
     
     // PM 12:9 -> PM 01:0�� ����� ���� if
        else if(time_24h == 13){   // 24�ð����� ǥ�������� 13�ð� �Ǹ�
            time1s = 1;            // 01:0�� ��������� 1s�� 1���� ����
            time10s = 0;           // 01:0�� ��������� 1s�� 0���� ����
        }
        
     // �ð�ǥ��
        LCD_SetTextColor(RGB_BLACK);// ���ڻ� : BLACK
        LCD_DisplayChar(3, 12, time100ms + 0x30);  // 1/10s
        LCD_DisplayChar(3, 11, ':');
        LCD_DisplayChar(3, 10, time1s + 0x30);     // 1s
        LCD_DisplayChar(3, 9, time10s + 0x30);     // 10s
  }
}

void TIMER4_OC_Init(void){   
      // PB8: TIM4_CH3
      // PB8�� ��¼����ϰ� Alternate function(TIM4_CH3)���� ��� ����
      RCC->AHB1ENR |= (1<<1);   // RCC_AHB1ENR GPIOB Enable
      RCC->APB1ENR |= (1<<2); // RCC_APB1ENR TIM.4 Enable
      
      GPIOB->MODER    |= (2<<16);   // PB8 -> Set Alternate function mode
      GPIOB->OSPEEDR  |=(2<<16);    // 50MHz (HIGH mode)
      GPIOB->OTYPER   &= ~(1<<8);  // OUTPUT PUSH-PULL mode
      GPIOB->PUPDR    &= ~(1<<16);   // GPIOB PIN8 No Pull-up
      
      // PB8 ==> TIM4_CH3
      GPIOB->AFR[1] |= (2<<0); // (AFR[1].(19~16)=0b0010): Connect TIM4 pins(PB8) to AF2(TIM3..5)
      
      // Setting CR1 : 0x0000 
      TIM4->CR1 &= ~(1<<4); // UP-Counter ���
      TIM4->CR1 &= ~(1<<1); // Update event Enabled <UVE : ������Ʈ �̺�Ʈ>
      TIM4->CR1 &= ~(1<<2); // Update Request Source  Selection
      TIM4->CR1 &= ~(1<<3); // The counter is NOT stopped at update event
      TIM4->CR1 |=  (1<<7); // ARR Preload Enalbe 
      TIM4->CR1 &= ~(3<<8); // No Clock division
      TIM4->CR1 &= ~(3<<5); // Center-aligned mode
      
      // Setting the Period
      TIM4->PSC = 16800-1;	// Prescaler=84, 84MHz/16800 = 5KHz (0.2ms)
      TIM4->ARR = 1000-1;	// Auto reload  : 200us * 1000 = 200ms(period) : ���ͷ�Ʈ�ֱ⳪ ��½�ȣ�� �ֱ� ����
      
      // Clear the Counter
      TIM4->EGR |= (1<<0); // Update generation
      
      // Output Compare ���� <CC ��带 �����ϴ� ��������>
      // CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4 <ä�� ����>
      // Ch3�� ����ϴϱ� CCMR2
      TIM4->CCMR2 &= ~(3<<0);  // CC3S(CC3 channel) = '0b00' : Output 
      TIM4->CCMR2 &= ~(1<<2);  // OC3FE=0: Output Compare 3 Fast disable 
      TIM4->CCMR2 &= ~(1<<3);  // OC3PE=0: Output Compare 3 preload disable(CCR2�� �������� ���ο� ���� loading ����) 
      TIM4->CCMR2 |=  (3<<4);  // OC3M=0b011 (Output Compare 3 Mode : toggle)
      
      // CCER(Capture/Compare Enable Register) : Enable "Channel 3"
      TIM4->CCER |= (1<<8);    // CC3E=1: CC3 channel Output Enable <OC3(TIM4_CH3) Active>
      TIM4->CCER &= ~(1<<9);    // OCPolarity_High : OC3���� �������� ���
      
      // CC3I(CC ���ͷ�Ʈ) ���ͷ�Ʈ �߻��ð� �Ǵ� ��ȣ��ȭ(���)�ñ� ����: ��ȣ�� ����(phase) ����
      // ���ͷ�Ʈ �߻��ð�(1000 �޽� == 200ms)�� 50%(500�޽� == 100ms) �ð����� compare match �߻�
      TIM4->CCR3 = 500;
      
      // DIER == local(mask) �����ؾ� ���ͷ�Ʈ�� ���� �ȴ�.
      TIM4->DIER |= (1<<0);    // UIE: Enable Tim4 Update interrupt
      TIM4->DIER |= (1<<3);    // CC3IE: Enable the Tim4 CC3 interrupt
      
      // GLOBAL ���ͷ�Ʈ ����
      NVIC->ISER[0] |= (1<<30);  // Enable Timer4 global Interrupt on NVIC
      
      // Conter Set
      TIM4->CR1 |= (1<<0);  // Enable the Tim4 Counter
}

void TIM4_IRQHandler(void)      //RESET: 0
{    
  if(SW4_Flag==1){                 // �����ġ ���� ����ġ ���ͷ�Ʈ �÷��� (����)
    
        TIM4->SR &= ~(1<<0);	// Update Interrupt Claer
	TIM4->SR &= ~(1<<3);	// CC3 Interrupt Claer
        stop_watch100ms++;      // �����ġ�� 1/10 ��
        
        SW5_Flag = 0;           // �����ġ ������ �ʱ�ȭŰ�� �����°��� ����
        
        if(stop_watch100ms > 9){ // 1/10�ʰ� 9�� ������
           stop_watch100ms = 0;  // 1/10�� 0���� �ʱ�ȭ
           stop_watch_count++;   // 1�� ī��Ʈ
        }
        
        if(stop_watch_count > 99){ // 1��ī��Ʈ�� 99�ʰ�  ������
          stop_watch_count = 0;    // 0���� �ʱ�ȭ
        }
        
        stop_watch[0] = stop_watch_count / 10;   // 10s �ڸ����
        stop_watch[1] = stop_watch_count % 10;   // 1s  �ڸ����
        
        // �����ġ LCD ��� //
        LCD_SetTextColor(RGB_BLACK);// ���ڻ� : BLACK
        LCD_DisplayChar(4, 12, stop_watch100ms + 0x30);  // (1/10)s
        LCD_DisplayChar(4, 11, ':');
        LCD_DisplayChar(4, 10, stop_watch[1] + 0x30);    // 1s
        LCD_DisplayChar(4, 9, stop_watch[0] + 0x30);     // 10s
  }
  else{                         // �����ġ ���� ����ġ ���ͷ�Ʈ �÷��� (�Ͻ�����)
    if(SW5_Flag){               // �����ġ �ʱ�ȭ ���ͷ�Ʈ �÷��� (�ʱ�ȭ)
      // �����ġ ���� ������ ���� �ʱ�ȭ
        stop_watch_count = 0;
        stop_watch[0] = 0;
        stop_watch[1] = 0;
        stop_watch100ms = 0;
        
        // �����ġ LCD ��� //
        LCD_SetTextColor(RGB_BLACK);// ���ڻ� : BLACK
        LCD_DisplayChar(4, 12, stop_watch100ms + 0x30);  // (1/10)s
        LCD_DisplayChar(4, 11, ':');
        LCD_DisplayChar(4, 10, stop_watch[1] + 0x30);    // 1s
        LCD_DisplayChar(4, 9, stop_watch[0] + 0x30);     // 10s
        
        SW5_Flag = 0;   // �ʱ�ȭ�÷��� �ʱ�ȭ
    }
  }
}

void _GPIO_Init(void)
{
	// LED (GPIO G) ����
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	    // GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) ���� 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) ���� 
        RCC->AHB1ENR	|=  0x00000020;     // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	    // GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}	

void _EXTI_Init(void)
{
        RCC->AHB1ENR 	|= 0x0080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x4000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= 0x0000FFFF;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
	
	SYSCFG->EXTICR[3] |= 0x0077; 	// EXTI12,13�� ���� �ҽ� �Է��� GPIOH�� ���� (EXTICR[3]) (reset value: 0x0000)	
	
	EXTI->RTSR 	|= 0x003000 ;	// Rising Trigger  Enable  (EXTI 12,13:PH12,13) 
        EXTI->IMR  	|= 0x003000 ; 	// EXTI 12,13 ���ͷ�Ʈ mask (Interrupt Enable)
		
	NVIC->ISER[1] |= ( 1 << (40-32) );   // Enable Interrupt EXTI12,13 Vector table Position ����
}

void EXTI15_10_IRQHandler(void)		// EXTI 10~15 ���ͷ�Ʈ �ڵ鷯
{
	if(EXTI->PR & 0x1000) 		// EXTI12 nterrupt Pending?
	{
		EXTI->PR |= 0x1000; 	// Pending bit Clear
                if(SW4_Flag == 0){      // ������ SW4�÷��װ�  0�� ������
                    SW4_Flag = 1;       // SW4�÷��� 1
                    GPIOG->ODR |= 0x0030;
                    BEEP();
                }
                else{                   // ������ SW4�÷��װ�  1�� ������
                    SW4_Flag = 0;       // SW4�÷��� 0
                    GPIOG->ODR &= ~0x0010;
                    BEEP();
                }
	}
	else if(EXTI->PR & 0x2000) 	// EXTI13 Interrupt Pending?
	{
		EXTI->PR |= 0x2000; 	// Pending bit Clear
		SW5_Flag = 1;	        // SW5�÷��� 1
                GPIOG->ODR &= ~0x0010;
                BEEP();
	}
}

void BEEP(void)			/* beep for 30 ms */
{ 	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(50);		// Delay 30 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
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

void DisplayInitScreen(void)
{
    LCD_Clear(RGB_YELLOW);		// ���ȭ��
    LCD_SetFont(&Gulim8);		// ���� 8
    LCD_SetBackColor(RGB_YELLOW);	// ���ڹ�� : Yellow
    LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : Black
    LCD_SetPenColor(RGB_BLACK);         // �׸����

    LCD_SetPenColor(RGB_BLACK);         // �׸����
    LCD_DrawRectangle(1,1,130,30);
    
    LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
    LCD_DisplayText(0,1,"Digital Watch");
    
    LCD_SetTextColor(RGB_GREEN);// ���ڻ� : GREEN
    LCD_DisplayText(1,1,"2018130048 PSY");
    
    LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
    LCD_DisplayText(3,1,"TIME" );
    LCD_DisplayText(4,1,"SW" );
    
    LCD_SetTextColor(RGB_BLACK);// ���ڻ� : BLACK
    LCD_DisplayText(3, 6, "PM");   
    
    
    // �����ġ LCD ��� //
    LCD_DisplayChar(4, 12, 0 + 0x30);  // (1/10)s
    LCD_DisplayChar(4, 11, ':');
    LCD_DisplayChar(4, 10, 0 + 0x30);    // 1s
    LCD_DisplayChar(4, 9, 0 + 0x30);     // 10s
    
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
        {   DelayMS(10);
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