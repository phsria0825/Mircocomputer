///////////////////////////////////////////////////////////////////////////////////////////////
// 과제명: HW.2 디지털시계제작(StopWatch기능포함)
// 과제개요: 디지털시계, 스톱워치를 TIMER의 OC(OUTPUT COMPARE), overflow(UPDATE EVENT)를 이용하여 구현
//      // 디지털 시계는 TIM.3 <APB1: 84MHz, 16bit(General perpose), 4ch> -> UPDATE EVENT mode
//      // 스톱워치는    TIM.4 <APB1: 84MHz, 16bit(General perpose), 4ch> -> OUTPUT COMPARE mode
// 사용한 하드웨어(기능): GPIO, EXTI, GLCD, TIMER, BUZZER
// 제출일: 2022. 10. 04
// 제출자 클래스:  목요일반
// 학번: 2018130048
// 이름: 박시영
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
void TIMER3_Init(void);     // TIM3: Overflow mode 함수선언
void TIMER4_OC_Init(void);  // TIM4: Oc mode 함수선언

void DisplayInitScreen(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

// 디지털시계 변수
int time100ms;          // 1/10초
int time1s = 7;         // 1초
int time10s;            // 10초
int time_24h = 19;      // 24h시계 : AM,PM을 나타내기위해 사용하는 변수

// 타이머 변수
int stop_watch[2];      // 10초, 1초 배열  stop_watch[2]={ time10s, time1s }
int stop_watch_count;   // 1/10초를 카운트하는 변수
int stop_watch100ms;    // 1/10초

uint8_t	SW4_Flag, SW5_Flag; // 스톱워치 동작,  스톱워치 초기화 플래그

int main(void)
{
    _GPIO_Init();  		// GPIO 초기화
    _EXTI_Init();		// 외부인터럽트 초기화
    LCD_Init();		        // GLCD 초기화
    DelayMS(10);			
    BEEP();			// Beep 한번 

    GPIOG->ODR &= 0xFF00;	// 초기값: LED0~7 Off
    DisplayInitScreen();	// LCD 초기화면
    
    TIMER3_Init();	        // 범용타이머(TIM3) 초기화 : up counting mode
    TIMER4_OC_Init();           // 범용타이머(TIM4) 초기화 : output compare mode


    while(1)
    {
        switch(KEY_Scan())
        {
            
        }  // switch(KEY_Scan())
        
        //EXTI SW4가 High에서 Low가 될 때 (Falling edge Trigger mode)
        if(SW4_Flag)
        {

        }
        //EXTI SW5가 High에서 Low가 될 때 (Rising edge Trigger mode)
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
        time100ms++;            // 1/10s 카운트
        
     // 1s를 카운트하는 if
        if (time100ms  > 9){    // 1/10s가 9를 넘기면
            time100ms  =  0;    // 1/10s를 0으로 초기화
            time1s++;           // 1s를 1 증가
            time_24h++;         // 24h 시간 증가
        }
        
     // 10s를 카운트하는 if   
        if (time1s > 9){        // 1s가 9를 넘기면
             time1s = 0;        // 1s를 0으로 초기화
             time10s++;         // 10s를 1 증가
         }
        
     // PM 11:9 -> AM 00:0 을 만들기위한 if (오후에서 오전으로 넘어가는 if문)
        if(time_24h == 24){     // 24시간제로 표시했을때 24시가 되면
            time_24h = 0;       // time_24h == 24이면 하루가 지났기때문에 0으로 초기화 
            time1s = 0;         // 00:0을 만들기위해 1s를 0으로 설정
            time10s = 0;        // 00:0을 만들기위해 10s를 0으로 설정
            LCD_SetTextColor(RGB_BLACK);// 글자색 : BLACK
            LCD_DisplayText(3, 6, "AM");  // LCD에 AM 표기
        }
        
     // AM -> PM을 만들기위한 if (오전에서 오후로 넘어가는 if문)
        else if(time_24h == 12){         // 24시간제로 표시했을때 12시가 되면
            LCD_SetTextColor(RGB_BLACK); // 글자색 : BLACK
            LCD_DisplayText(3, 6, "PM"); // LCD에 PM 표기
        }
     
     // PM 12:9 -> PM 01:0을 만들기 위한 if
        else if(time_24h == 13){   // 24시간제로 표시했을때 13시가 되면
            time1s = 1;            // 01:0을 만들기위해 1s를 1으로 설정
            time10s = 0;           // 01:0을 만들기위해 1s를 0으로 설정
        }
        
     // 시간표시
        LCD_SetTextColor(RGB_BLACK);// 글자색 : BLACK
        LCD_DisplayChar(3, 12, time100ms + 0x30);  // 1/10s
        LCD_DisplayChar(3, 11, ':');
        LCD_DisplayChar(3, 10, time1s + 0x30);     // 1s
        LCD_DisplayChar(3, 9, time10s + 0x30);     // 10s
  }
}

void TIMER4_OC_Init(void){   
      // PB8: TIM4_CH3
      // PB8을 출력설정하고 Alternate function(TIM4_CH3)으로 사용 선언
      RCC->AHB1ENR |= (1<<1);   // RCC_AHB1ENR GPIOB Enable
      RCC->APB1ENR |= (1<<2); // RCC_APB1ENR TIM.4 Enable
      
      GPIOB->MODER    |= (2<<16);   // PB8 -> Set Alternate function mode
      GPIOB->OSPEEDR  |=(2<<16);    // 50MHz (HIGH mode)
      GPIOB->OTYPER   &= ~(1<<8);  // OUTPUT PUSH-PULL mode
      GPIOB->PUPDR    &= ~(1<<16);   // GPIOB PIN8 No Pull-up
      
      // PB8 ==> TIM4_CH3
      GPIOB->AFR[1] |= (2<<0); // (AFR[1].(19~16)=0b0010): Connect TIM4 pins(PB8) to AF2(TIM3..5)
      
      // Setting CR1 : 0x0000 
      TIM4->CR1 &= ~(1<<4); // UP-Counter 사용
      TIM4->CR1 &= ~(1<<1); // Update event Enabled <UVE : 업데이트 이벤트>
      TIM4->CR1 &= ~(1<<2); // Update Request Source  Selection
      TIM4->CR1 &= ~(1<<3); // The counter is NOT stopped at update event
      TIM4->CR1 |=  (1<<7); // ARR Preload Enalbe 
      TIM4->CR1 &= ~(3<<8); // No Clock division
      TIM4->CR1 &= ~(3<<5); // Center-aligned mode
      
      // Setting the Period
      TIM4->PSC = 16800-1;	// Prescaler=84, 84MHz/16800 = 5KHz (0.2ms)
      TIM4->ARR = 1000-1;	// Auto reload  : 200us * 1000 = 200ms(period) : 인터럽트주기나 출력신호의 주기 결정
      
      // Clear the Counter
      TIM4->EGR |= (1<<0); // Update generation
      
      // Output Compare 설정 <CC 모드를 설정하는 레지스터>
      // CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4 <채널 설정>
      // Ch3을 사용하니까 CCMR2
      TIM4->CCMR2 &= ~(3<<0);  // CC3S(CC3 channel) = '0b00' : Output 
      TIM4->CCMR2 &= ~(1<<2);  // OC3FE=0: Output Compare 3 Fast disable 
      TIM4->CCMR2 &= ~(1<<3);  // OC3PE=0: Output Compare 3 preload disable(CCR2에 언제든지 새로운 값을 loading 가능) 
      TIM4->CCMR2 |=  (3<<4);  // OC3M=0b011 (Output Compare 3 Mode : toggle)
      
      // CCER(Capture/Compare Enable Register) : Enable "Channel 3"
      TIM4->CCER |= (1<<8);    // CC3E=1: CC3 channel Output Enable <OC3(TIM4_CH3) Active>
      TIM4->CCER &= ~(1<<9);    // OCPolarity_High : OC3으로 반전없이 출력
      
      // CC3I(CC 인터럽트) 인터럽트 발생시각 또는 신호변화(토글)시기 결정: 신호의 위상(phase) 결정
      // 인터럽트 발생시간(1000 펄스 == 200ms)의 50%(500펄스 == 100ms) 시각에서 compare match 발생
      TIM4->CCR3 = 500;
      
      // DIER == local(mask) 설정해야 인터럽트로 연결 된다.
      TIM4->DIER |= (1<<0);    // UIE: Enable Tim4 Update interrupt
      TIM4->DIER |= (1<<3);    // CC3IE: Enable the Tim4 CC3 interrupt
      
      // GLOBAL 인터럽트 설정
      NVIC->ISER[0] |= (1<<30);  // Enable Timer4 global Interrupt on NVIC
      
      // Conter Set
      TIM4->CR1 |= (1<<0);  // Enable the Tim4 Counter
}

void TIM4_IRQHandler(void)      //RESET: 0
{    
  if(SW4_Flag==1){                 // 스톱워치 동작 스위치 인터럽트 플래그 (동작)
    
        TIM4->SR &= ~(1<<0);	// Update Interrupt Claer
	TIM4->SR &= ~(1<<3);	// CC3 Interrupt Claer
        stop_watch100ms++;      // 스톱워치의 1/10 초
        
        SW5_Flag = 0;           // 스톱워치 동작중 초기화키가 눌리는것을 방지
        
        if(stop_watch100ms > 9){ // 1/10초가 9를 넘으면
           stop_watch100ms = 0;  // 1/10초 0으로 초기화
           stop_watch_count++;   // 1초 카운트
        }
        
        if(stop_watch_count > 99){ // 1초카운트가 99초가  넘으면
          stop_watch_count = 0;    // 0으로 초기화
        }
        
        stop_watch[0] = stop_watch_count / 10;   // 10s 자리계산
        stop_watch[1] = stop_watch_count % 10;   // 1s  자리계산
        
        // 스톱워치 LCD 출력 //
        LCD_SetTextColor(RGB_BLACK);// 글자색 : BLACK
        LCD_DisplayChar(4, 12, stop_watch100ms + 0x30);  // (1/10)s
        LCD_DisplayChar(4, 11, ':');
        LCD_DisplayChar(4, 10, stop_watch[1] + 0x30);    // 1s
        LCD_DisplayChar(4, 9, stop_watch[0] + 0x30);     // 10s
  }
  else{                         // 스톱워치 동작 스위치 인터럽트 플래그 (일시정지)
    if(SW5_Flag){               // 스톱워치 초기화 인터럽트 플래그 (초기화)
      // 스톱워치 관련 변수들 전부 초기화
        stop_watch_count = 0;
        stop_watch[0] = 0;
        stop_watch[1] = 0;
        stop_watch100ms = 0;
        
        // 스톱워치 LCD 출력 //
        LCD_SetTextColor(RGB_BLACK);// 글자색 : BLACK
        LCD_DisplayChar(4, 12, stop_watch100ms + 0x30);  // (1/10)s
        LCD_DisplayChar(4, 11, ':');
        LCD_DisplayChar(4, 10, stop_watch[1] + 0x30);    // 1s
        LCD_DisplayChar(4, 9, stop_watch[0] + 0x30);     // 10s
        
        SW5_Flag = 0;   // 초기화플래그 초기화
    }
  }
}

void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	    // GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) 설정 
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
	
	SYSCFG->EXTICR[3] |= 0x0077; 	// EXTI12,13에 대한 소스 입력은 GPIOH로 설정 (EXTICR[3]) (reset value: 0x0000)	
	
	EXTI->RTSR 	|= 0x003000 ;	// Rising Trigger  Enable  (EXTI 12,13:PH12,13) 
        EXTI->IMR  	|= 0x003000 ; 	// EXTI 12,13 인터럽트 mask (Interrupt Enable)
		
	NVIC->ISER[1] |= ( 1 << (40-32) );   // Enable Interrupt EXTI12,13 Vector table Position 참조
}

void EXTI15_10_IRQHandler(void)		// EXTI 10~15 인터럽트 핸들러
{
	if(EXTI->PR & 0x1000) 		// EXTI12 nterrupt Pending?
	{
		EXTI->PR |= 0x1000; 	// Pending bit Clear
                if(SW4_Flag == 0){      // 이전의 SW4플래그가  0이 였으면
                    SW4_Flag = 1;       // SW4플래그 1
                    GPIOG->ODR |= 0x0030;
                    BEEP();
                }
                else{                   // 이전의 SW4플래그가  1이 였으면
                    SW4_Flag = 0;       // SW4플래그 0
                    GPIOG->ODR &= ~0x0010;
                    BEEP();
                }
	}
	else if(EXTI->PR & 0x2000) 	// EXTI13 Interrupt Pending?
	{
		EXTI->PR |= 0x2000; 	// Pending bit Clear
		SW5_Flag = 1;	        // SW5플래그 1
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
    LCD_Clear(RGB_YELLOW);		// 배경화면
    LCD_SetFont(&Gulim8);		// 굴림 8
    LCD_SetBackColor(RGB_YELLOW);	// 글자배경 : Yellow
    LCD_SetTextColor(RGB_BLACK);	// 글자색 : Black
    LCD_SetPenColor(RGB_BLACK);         // 그리기색

    LCD_SetPenColor(RGB_BLACK);         // 그리기색
    LCD_DrawRectangle(1,1,130,30);
    
    LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
    LCD_DisplayText(0,1,"Digital Watch");
    
    LCD_SetTextColor(RGB_GREEN);// 글자색 : GREEN
    LCD_DisplayText(1,1,"2018130048 PSY");
    
    LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
    LCD_DisplayText(3,1,"TIME" );
    LCD_DisplayText(4,1,"SW" );
    
    LCD_SetTextColor(RGB_BLACK);// 글자색 : BLACK
    LCD_DisplayText(3, 6, "PM");   
    
    
    // 스톱워치 LCD 출력 //
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