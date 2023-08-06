/////////////////////////////////////////////////////////////////////////////////////
//***********************************************************************************
//HW4: SPI가속도 센서를 LCD 출력 + MFC와 USART 출력
//과제개요:
//       SPI 통신을 이용한 가속도센서 측정
//       NSS pin:  PA8 (PA4(SPI1_CS) 대신에 사용)
//       SCK pin:  PA5 (SPI1_SCK)
//       MOSI pin: PA7 (SPI1_MOSI)
//       MISO pin: PA6 (SPI1_MISO)
//       SPI mode: MASTER
//       SPI통신으로 가속도센서(LIS2HH12, Slave mode) X,Y,Z 값을 400ms마다 측정
//       USART1통신으로 MFC와 통신을 측정할 가속도 축 선택 + 800ms마다 측정된 가속도 출력
//       TIMER.10 CC인터럽드를 이용하여 400ms와 800ms 주기생성 
//       main.c만 수정 + Fram(400)에 저장된 축정보와 & 연산으로 축정보 분리(0x01, 0x02, 0x04)
//***********************************************************************************
/////////////////////////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "GLCD.h"
#include "ACC.h"
#include "FRAM.h"

#define No_send_data 	20

void DisplayTitle(void);
void _GPIO_Init(void);

void SPI1_Init(void);
void USART1_Init(void);
void TIMER10_Init(void);

void SerialSendChar(uint8_t c);
void SerialSendString(char *s);

void Display_Process(int16 *pBuf, char Axisz);
void make_space(unsigned int *N);  // PC통신에서 엔터역할을 하는 함수
void USART_BRR_Configuration(uint32_t USART_BaudRate);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

//// void SPI1_Process(UINT16 *pBuf);  // ACC.c (ACC.h) 
//// void ACC_Init(void) // ACC.c (ACC.h)
//// void LCD_Init(void); // GLCD.c (GLCD.h)

UINT8 bControl, b2Control = 0; // bControl : 400ms 측정, b2Control : 800ms 측정
char ch, Old_ch;   //  축정보 변수
int16 buffer[3];   // = { X, Y, Z } 의 가속도가 저장되는 변수
uint8_t str_x[8];  // x축 string 배열
uint8_t str_y[8];  // y축 string 배열
uint8_t str_z[8];  // z축 string 배열

int main(void)
{
        unsigned int space = 0; // PC로 str_x ~ str_z까지 출력을 하고 엔터를 시키기 위한 변수
        
	LCD_Init();		// LCD 구동 함수
	DelayMS(10);		// LCD구동 딜레이
	DisplayTitle();		// LCD 초기화면구동 함수
        Fram_Init();            // FRAM 초기화 H/W 초기화
	Fram_Status_Config();   // FRAM 초기화 S/W 초기화
    
	_GPIO_Init();		// LED, SW 초기화
        TIMER10_Init();         // 가속도센서 스캔 주기(400ms)생성
	SPI1_Init();        	// SPI1 초기화
        USART1_Init();          // USART1 초기화
	ACC_Init();		// 가속도센서 초기화
   
	while(1)
	{
          if(bControl) // 400ms 마다 동작
	  {
              bControl = FALSE;     
              SPI1_Process(&buffer[0]);	// SPI통신을 이용하여 가속도센서 측정
              Display_Process(&buffer[0], Fram_Read(400));	// 측정값을 LCD에 표시
	  }
          
          if(b2Control == 2)  // 800ms 마다 동작
              {
                  b2Control = 0;  // 0으로 초기화 (800ms가 되면 초기화)
                  
                  if (Fram_Read(400) & 0x01){    // Fram(400)의 값과  0x01을 and 하여 bit.0의 값이 TRUE면 if문 실행
                        SerialSendString(str_x); // X축 가속도 출력
                        SerialSendString("   "); // 출력 사이 띄우기
                        space++;
                  }
                  
                  if (Fram_Read(400) & 0x02){    // Fram(400)의 값과  0x02을 and 하여 bit.1의 값이 TRUE면 if문 실행
                        SerialSendString(str_y); // Y축 가속도 출력
                        SerialSendString("   "); // 출력 사이 띄우기
                        space++;
                  }
                  
                  if (Fram_Read(400) & 0x04){    // Fram(400)의 값과  0x04을 and 하여 bit.2의 값이 TRUE면 if문 실행
                        SerialSendString(str_z); // Z축 가속도 출력
                        SerialSendString("   "); // 출력 사이 띄우기
                        space++;
                  }
                  
                  make_space(&space);
              }
      
	}
}

void USART1_IRQHandler(void)	
{       
	// RX Buffer Full interrupt
	if ( (USART1->SR & USART_SR_RXNE) )  // USART_SR_RXNE=(1<<5) 
	{
		
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장
                Fram_Write(400,ch);
		// DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 clear 할 필요없음 
	} 
}

void TIM1_UP_TIM10_IRQHandler(void)      //RESET: 0
{
	TIM10->SR &= ~(1<<1);	// CC1 Interrupt Claer
        bControl = TRUE;	// 400ms마다 센서 측정
        b2Control++;            // 800ms마다 PC로 전송
}

void Display_Process(int16 *pBuf, char Axis)
{
        UINT16 G_VALUE;
        
        // LCD에서 선택된 축의 '*'표시를 지우는 역할
        for(int i=0;i<3;i++){
            LCD_DisplayChar(i+1,0,' ');
        }
        
        // (X,Y,Z)축의 센서값을 표시
        if (Axis & 0x01){    // Fram(400)의 값과  0x01을 and 하여 bit.0의 값이 TRUE면 if문 실행
              LCD_DisplayChar(1,0,'*');
              
              // X 축 가속도 표시		
              if (pBuf[0] < 0)  //음수
              {
                      G_VALUE = abs(pBuf[0]);
                      LCD_DisplayChar(1,4,'-'); // g 부호 표시
                      str_x[3] = '-';           // PC화면 부호 표시
                      
                      // X축 막대그래프 생성
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(88, 80, 125, 10); // + 뱡향에서 그래프가 존재했을 때 지우는 막대
                      
                      LCD_SetBrushColor(RGB_RED);   // Y축 색 설정
                      LCD_DrawFillRect(85 - 0.3*(100 * G_VALUE / 0x4009),80,0.3*(100 * G_VALUE / 0x4009)+1,10);  // - 방향의 막대 생성
                      LCD_SetBrushColor(RGB_WHITE); // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대 색
                      LCD_DrawFillRect(0, 80, 85 - 0.3*(100 * G_VALUE / 0x4009), 10); // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대
              }
              else				// 양수
              {
                      G_VALUE = pBuf[0];
                      LCD_DisplayChar(1,4,'+'); // g 부호 표시
                      str_x[3] = '+';           // PC화면 부호 표시

                      // X축 막대그래프 생성
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(40,80,83,10);
                      
                      LCD_SetBrushColor(RGB_RED);   // X축 색 설정
                      LCD_DrawFillRect(88,80,0.3*(100 * G_VALUE / 0x4009),10);  // + 방향의 막대 생성
                      LCD_SetBrushColor(RGB_WHITE); // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대 색
                      LCD_DrawFillRect(88 + 0.3*(100 * G_VALUE / 0x4009), 80, 125 - 0.3*(100 * G_VALUE / 0x4009), 10);  // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대
                      
              }
              G_VALUE = 10 * G_VALUE / 0x4009; // 가속도 --> g 변환
              
              // X축 가속도 값 표시
              LCD_DisplayChar(1,5, G_VALUE/10 +0x30);
              LCD_DisplayChar(1,6,'.');
              LCD_DisplayChar(1,7, G_VALUE%10 +0x30);
              
              // PC전송용 string 생성 (X:G_VALUE)
              str_x[0] = 'A';
              str_x[1] = 'x';
              str_x[2] = ':';
              str_x[4] = 0x30 + G_VALUE/10;
              str_x[5] = '.';
              str_x[6] = 0x30 + G_VALUE%10;
        }
        
        
        if (Axis & 0x02){     // Fram(400)의 값과  0x02을 and 하여 bit.1의 값이 TEUE면 if문 실행
              LCD_DisplayChar(2,0,'*');
              
              // Y 축 가속도 표시	
              if (pBuf[1] < 0)  //음수
              {
                      G_VALUE = abs(pBuf[1]);
                      LCD_DisplayChar(2,4,'-'); // g 부호 표시
                      str_y[3] = '-';           // PC화면 부호 표시
                      
                      // Y축 막대그래프 생성
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(88, 95, 125, 10); // + 뱡향에서 그래프가 존재했을 때 지우는 막대
                      
                      LCD_SetBrushColor(RGB_GREEN);   // Y축 색 설정
                      LCD_DrawFillRect(85 - 0.3*(100 * G_VALUE / 0x4009),95,0.3*(100 * G_VALUE / 0x4009)+1,10);  // - 방향의 막대 생성
                      LCD_SetBrushColor(RGB_WHITE); // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대 색
                      LCD_DrawFillRect(0, 95, 85 - 0.3*(100 * G_VALUE / 0x4009), 10); // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대
              }
              else				// 양수
              {
                      G_VALUE = pBuf[1];
                      LCD_DisplayChar(2,4,'+'); // g 부호 표시
                      str_y[3] = '+';           // PC화면 부호 표시
                      
                       // Y축 막대그래프 생성
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(40,95,83,10);
                      
                      LCD_SetBrushColor(RGB_GREEN);   // Y축 색 설정
                      LCD_DrawFillRect(88,95,0.3*(100 * G_VALUE / 0x4009),10);  // + 방향의 막대 생성
                      LCD_SetBrushColor(RGB_WHITE); // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대 색
                      LCD_DrawFillRect(88 + 0.3*(100 * G_VALUE / 0x4009), 95, 125 - 0.3*(100 * G_VALUE / 0x4009), 10);  // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대
              }
              G_VALUE = 10 * G_VALUE / 0x4009; 
              
              // Y축 가속도 값 표시
              LCD_DisplayChar(2,5, G_VALUE/10 +0x30);
              LCD_DisplayChar(2,6,'.');
              LCD_DisplayChar(2,7, G_VALUE%10 +0x30);
              
              // PC전송용 string 생성  (Y:G_VALUE)
              str_y[0] = 'A';
              str_y[1] = 'y';
              str_y[2] = ':';
              str_y[4] = 0x30 + G_VALUE/10;
              str_y[5] = '.';
              str_y[6] = 0x30 + G_VALUE%10;
        }
        
        
        if (Axis & 0x04){      // Fram(400)의 값과  0x04을 and 하여 bit.2의 값이 TEUE면 if문 실행
              LCD_DisplayChar(3,0,'*');
              
              // Z 축 가속도 표시	
              if (pBuf[2] < 0)  //음수
              {
                      G_VALUE = abs(pBuf[2]);
                      LCD_DisplayChar(3,4,'-'); // g 부호 표시
                      str_z[3] = '-';           // PC화면 부호 표시
                      
                      // Z축 막대그래프 생성
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(88, 110, 125, 10); // + 뱡향에서 그래프가 존재했을 때 지우는 막대
                      
                      LCD_SetBrushColor(RGB_BLUE);   // Z축 색 설정
                      LCD_DrawFillRect(85 - 0.3*(100 * G_VALUE / 0x4009),110,0.3*(100 * G_VALUE / 0x4009)+1,10);  // - 방향의 막대 생성
                      LCD_SetBrushColor(RGB_WHITE); // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대 색
                      LCD_DrawFillRect(0, 80, 85 - 0.3*(100 * G_VALUE / 0x4009), 10); // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대
              }
              else				// 양수
              {
                      G_VALUE = pBuf[2];
                      LCD_DisplayChar(3,4,'+'); // g 부호 표시
                      str_z[3] = '+';           // PC화면 부호 표시
                      
                       // Z축 막대그래프 생성
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(40,110,83,10);
                      
                      LCD_SetBrushColor(RGB_BLUE);   // Z축 색 설정
                      LCD_DrawFillRect(88,110,0.3*(100 * G_VALUE / 0x4009),10);  // + 방향의 막대 생성
                      LCD_SetBrushColor(RGB_WHITE); // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대 색
                      LCD_DrawFillRect(88 + 0.3*(100 * G_VALUE / 0x4009), 110, 125 - 0.3*(100 * G_VALUE / 0x4009), 10);  // 막대가 줄어들때, 줄어든 부분을 배경으로 채우는 막대
              }
              G_VALUE = 10 * G_VALUE / 0x4009; 
              
              // Z축 가속도 값 표시
              LCD_DisplayChar(3,5, G_VALUE/10 +0x30);
              LCD_DisplayChar(3,6,'.');
              LCD_DisplayChar(3,7, G_VALUE%10 +0x30);

              // PC전송용 string 생성  (Z:G_VALUE)
              str_z[0] = 'A';
              str_z[1] = 'z';
              str_z[2] = ':';
              str_z[4] = 0x30 + G_VALUE/10;
              str_z[5] = '.';
              str_z[6] = 0x30 + G_VALUE%10;
        }
        
        // 막대그래프 기준 축 재생성
        LCD_SetBrushColor(RGB_BLACK); // 막대축 색상
        LCD_DrawFillRect(85,80,3,40);  // 막대축
        LCD_DisplayText(6,1,"Ax:");	//X AXIS
	LCD_DisplayText(7,1,"Ay:");	//Y AXIS
	LCD_DisplayText(8,1,"Az:");	//Z AXIS
}

///////////////////////////////////////////////////////
// Master mode, Full-duplex, 8bit frame(MSB first), 
// fPCLK/32 Baud rate, Software slave management EN
void SPI1_Init(void)
{
	/*!< Clock Enable  *********************************************************/
	RCC->APB2ENR 	|= (1<<12);	// 0x1000, SPI1 Clock EN
	RCC->AHB1ENR 	|= (1<<0);	// 0x0001, GPIOA Clock EN
  
	/*!< SPI1 pins configuration ************************************************/
	
	/*!< SPI1 NSS pin(PA8) configuration : GPIO 핀  */
	GPIOA->MODER 	|= (1<<(2*8));	// 0x00010000, PA8 Output mode
	GPIOA->OTYPER 	&= ~(1<<8); 	// 0x0100, push-pull(reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*8));	// 0x00030000, PA8 Output speed (100MHZ) 
	GPIOA->PUPDR 	&= ~(3<<(2*8));	// 0x00030000, NO Pullup Pulldown(reset state)
    
	/*!< SPI1 SCK pin(PA5) configuration : SPI1_SCK */
	GPIOA->MODER 	|= (2<<(2*5)); 	// 0x00000800, PA5 Alternate function mode
	GPIOA->OTYPER 	&= ~(1<<5); 	// 0020, PA5 Output type push-pull (reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*5));	// 0x00000C00, PA5 Output speed (100MHz)
	GPIOA->PUPDR 	|= (2<<(2*5)); 	// 0x00000800, PA5 Pull-down
	GPIOA->AFR[0] 	|= (5<<(4*5));	// 0x00500000, Connect PA5 to AF5(SPI1)
    
	/*!< SPI1 MOSI pin(PA7) configuration : SPI1_MOSI */    
	GPIOA->MODER 	|= (2<<(2*7));	// 0x00008000, PA7 Alternate function mode
	GPIOA->OTYPER	&= ~(1<<7);	// 0x0080, PA7 Output type push-pull (reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*7));	// 0x0000C000, PA7 Output speed (100MHz)
	GPIOA->PUPDR 	|= (2<<(2*7)); 	// 0x00008000, PA7 Pull-down
	GPIOA->AFR[0] 	|= (5<<(4*7));	// 0x50000000, Connect PA7 to AF5(SPI1)
    
	/*!< SPI1 MISO pin(PA6) configuration : SPI1_MISO */
	GPIOA->MODER 	|= (2<<(2*6));	// 0x00002000, PA6 Alternate function mode
	GPIOA->OTYPER 	&= ~(1<<6);	// 0x0040, PA6 Output type push-pull (reset state)
	GPIOA->OSPEEDR 	|= (3<<(2*6));	// 0x00003000, PA6 Output speed (100MHz)
	GPIOA->PUPDR 	|= (2<<(2*6));	// 0x00002000, PA6 Pull-down
	GPIOA->AFR[0] 	|= (5<<(4*6));	// 0x05000000, Connect PA6 to AF5(SPI1)

	// Init SPI1 Registers 
	SPI1->CR1 |= (1<<2);	// MSTR(Master selection)=1, Master mode
	SPI1->CR1 &= ~(1<<15);	// SPI_Direction_2 Lines_FullDuplex
	SPI1->CR1 &= ~(1<<11);	// SPI_DataSize_8bit
	SPI1->CR1 |= (1<<9);  	// SSM(Software slave management)=1, 
				// NSS 핀 상태가 코딩에 의해 결정
	SPI1->CR1 |= (1<<8);	// SSI(Internal_slave_select)=1,
				// 현재 MCU가 Master이므로 NSS 상태는 'High' 
	SPI1->CR1 &= ~(1<<7);	// LSBFirst=0, MSB transmitted first    
	SPI1->CR1 |= (4<<3);	// BR(BaudRate)=0b100, fPCLK/32 (84MHz/32 = 2.625MHz)
	SPI1->CR1 |= (1<<1);	// CPOL(Clock polarity)=1, CK is 'High' when idle
	SPI1->CR1 |= (1<<0);	// CPHA(Clock phase)=1, 두 번째 edge 에서 데이터가 샘플링
 
	SPI1->CR1 |= (1<<6);	// SPE=1, SPI1 Enable 
}

void TIMER10_Init(void){
     // TIM10 CH1 : PB8 (167번 핀)
     // Clock Enable : GPIOB & TIMER10
        RCC->APB2ENR |= (1<<17);
        RCC->AHB1ENR |= (1<<1);;
        
     // PB8을 출력설정하고 Alternate function(TIM10_CH1)으로 사용 선언 : CC인터럽트
        GPIOB->MODER 	|= (2<<16);	// 0x00020000 PB8 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<16);	// 0x00030000 PB8 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<8);	// PB8 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1<<16);	// 0x00010000 PB8 Pull-up
        GPIOB->AFR[1]   |= (3<<0);      // (AFR[1].(3~0)=0b0011): Connect TIM10 pins(PB8) to AF3(TIM8..11)
                                        // PB.8에서 pin이 8 이기 때문에, AFR[1] 
        
        // TIM10 Channel 1 : CC mode
	// Assign 'Period'
	TIM10->PSC	= 840-1;	// Prescaler 84MHz/840 =  1MHz(10us)  (1~65536)
	TIM10->ARR	= 40000-1;	// Auto reload  (0.1us * 40000 = 400ms : PWM Period)

	// Setting CR1 : 0x0000 (Up counting)
	TIM10->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM10->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM10->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)g events
	TIM10->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM10->CR1 &= ~(1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enable 
	TIM10->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM10->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : use Edge-aligned mode(reset state)
        
        // Update(Clear) the Counter
	TIM10->EGR |= (1<<0);    // UG: Update generation
        
        // Output Compare 설정 <CC 모드를 설정하는 레지스터>
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
        // Ch1을 사용하니까 CCR1
	TIM10->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output 
	TIM10->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM10->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable(CCR1에 언제든지 새로운 값을 loading 가능) 
	TIM10->CCMR1 |= (3<<4);	 // OC1M=0b011 (Output Compare 1 Mode : toggle)
				 // OC1REF toggles when CNT = CCR1
        
        // CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM10->CCER &= ~(1<<0);	// CC1E=1: CC1 channel Output Enable
				// OC1(TIM14_CH1) Active: 해당핀(27번)을 통해 신호출력
	TIM10->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1으로 반전없이 출력)  

	// CC1I(CC 인터럽트) 인터럽트 발생시각 또는 신호변화(토글)시기 결정: 신호의 위상(phase) 결정
	TIM10->CCR1 = 10;	// TIM4 CCR1 TIM4_Pulse,  펄스의 시작시간이 달라진다.
        
        TIM10->DIER |= (1<<1);
        NVIC->ISER[0] |= (1<<25);
        
	TIM10->CR1 |= (1<<0);	// CEN: Enable the Tim4 Counter		
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
	USART1->CR1	&= ~(1<<10);	// NO USART_Parity

	USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
	USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37번)
	USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
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

//* MFC에서 가속도 표시할때 빈칸을 추가하여 엔터의 역할을 하는 함수 *//
void make_space(unsigned int *N){  // make_space의 변수가 포인터로 들어간다
  if (*N != 0){                    // 0x00이 아니면 (가속도측정이 1개라도 있으면 if실행)
      for (int i = 0; i < (74 - 12*(*N)); i++){   // 전체 공간에서 요청받은 가속도 개수를 가지고 빈칸을 계산
                                                  // 전체칸 : 74
            SerialSendChar(' ');                  // 빈칸 출력
      }
  }
      *N = 0;                                     // name space 초기화
}

// Baud rate  
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Determine the integer part 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
	{                                         // USART1->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));    
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Determine the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
	{
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else 			// 16 oversampling
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

	GPIOG->ODR &= 0x00;	// LED0~7 Off 
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;

	for (i=0; i<wMS; i++)
		DelayUS(1000);		//1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
        volatile int Dly = (int)wUS*17;
         for(; Dly; Dly--);
}

void DisplayTitle(void)
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim8);		//폰트 
	LCD_SetBackColor(RGB_YELLOW);
	LCD_SetTextColor(RGB_BLACK);    //글자색
	LCD_DisplayText(0,0,"ACC sensor(SPI):PSY");  // Title

	LCD_SetBackColor(RGB_WHITE);    //글자배경색

	LCD_DisplayText(1,1,"Ax:");	//X AXIS
	LCD_DisplayText(2,1,"Ay:");	//Y AXIS
	LCD_DisplayText(3,1,"Az:");	//Z AXIS
        
        // 막대그래프 기준 축 생성
        LCD_SetBrushColor(RGB_BLACK); // 막대축 색상
        LCD_DrawFillRect(85,80,3,40);  // 막대축
        LCD_DisplayText(6,1,"Ax:");	//X AXIS
	LCD_DisplayText(7,1,"Ay:");	//Y AXIS
	LCD_DisplayText(8,1,"Az:");	//Z AXIS
        
        
}