/////////////////////////////////////////////////////////////////////////////////////
//***********************************************************************************
//HW4: SPI���ӵ� ������ LCD ��� + MFC�� USART ���
//��������:
//       SPI ����� �̿��� ���ӵ����� ����
//       NSS pin:  PA8 (PA4(SPI1_CS) ��ſ� ���)
//       SCK pin:  PA5 (SPI1_SCK)
//       MOSI pin: PA7 (SPI1_MOSI)
//       MISO pin: PA6 (SPI1_MISO)
//       SPI mode: MASTER
//       SPI������� ���ӵ�����(LIS2HH12, Slave mode) X,Y,Z ���� 400ms���� ����
//       USART1������� MFC�� ����� ������ ���ӵ� �� ���� + 800ms���� ������ ���ӵ� ���
//       TIMER.10 CC���ͷ��带 �̿��Ͽ� 400ms�� 800ms �ֱ���� 
//       main.c�� ���� + Fram(400)�� ����� �������� & �������� ������ �и�(0x01, 0x02, 0x04)
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
void make_space(unsigned int *N);  // PC��ſ��� ���Ϳ����� �ϴ� �Լ�
void USART_BRR_Configuration(uint32_t USART_BaudRate);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

//// void SPI1_Process(UINT16 *pBuf);  // ACC.c (ACC.h) 
//// void ACC_Init(void) // ACC.c (ACC.h)
//// void LCD_Init(void); // GLCD.c (GLCD.h)

UINT8 bControl, b2Control = 0; // bControl : 400ms ����, b2Control : 800ms ����
char ch, Old_ch;   //  ������ ����
int16 buffer[3];   // = { X, Y, Z } �� ���ӵ��� ����Ǵ� ����
uint8_t str_x[8];  // x�� string �迭
uint8_t str_y[8];  // y�� string �迭
uint8_t str_z[8];  // z�� string �迭

int main(void)
{
        unsigned int space = 0; // PC�� str_x ~ str_z���� ����� �ϰ� ���͸� ��Ű�� ���� ����
        
	LCD_Init();		// LCD ���� �Լ�
	DelayMS(10);		// LCD���� ������
	DisplayTitle();		// LCD �ʱ�ȭ�鱸�� �Լ�
        Fram_Init();            // FRAM �ʱ�ȭ H/W �ʱ�ȭ
	Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
    
	_GPIO_Init();		// LED, SW �ʱ�ȭ
        TIMER10_Init();         // ���ӵ����� ��ĵ �ֱ�(400ms)����
	SPI1_Init();        	// SPI1 �ʱ�ȭ
        USART1_Init();          // USART1 �ʱ�ȭ
	ACC_Init();		// ���ӵ����� �ʱ�ȭ
   
	while(1)
	{
          if(bControl) // 400ms ���� ����
	  {
              bControl = FALSE;     
              SPI1_Process(&buffer[0]);	// SPI����� �̿��Ͽ� ���ӵ����� ����
              Display_Process(&buffer[0], Fram_Read(400));	// �������� LCD�� ǥ��
	  }
          
          if(b2Control == 2)  // 800ms ���� ����
              {
                  b2Control = 0;  // 0���� �ʱ�ȭ (800ms�� �Ǹ� �ʱ�ȭ)
                  
                  if (Fram_Read(400) & 0x01){    // Fram(400)�� ����  0x01�� and �Ͽ� bit.0�� ���� TRUE�� if�� ����
                        SerialSendString(str_x); // X�� ���ӵ� ���
                        SerialSendString("   "); // ��� ���� ����
                        space++;
                  }
                  
                  if (Fram_Read(400) & 0x02){    // Fram(400)�� ����  0x02�� and �Ͽ� bit.1�� ���� TRUE�� if�� ����
                        SerialSendString(str_y); // Y�� ���ӵ� ���
                        SerialSendString("   "); // ��� ���� ����
                        space++;
                  }
                  
                  if (Fram_Read(400) & 0x04){    // Fram(400)�� ����  0x04�� and �Ͽ� bit.2�� ���� TRUE�� if�� ����
                        SerialSendString(str_z); // Z�� ���ӵ� ���
                        SerialSendString("   "); // ��� ���� ����
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
		
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
                Fram_Write(400,ch);
		// DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
	} 
}

void TIM1_UP_TIM10_IRQHandler(void)      //RESET: 0
{
	TIM10->SR &= ~(1<<1);	// CC1 Interrupt Claer
        bControl = TRUE;	// 400ms���� ���� ����
        b2Control++;            // 800ms���� PC�� ����
}

void Display_Process(int16 *pBuf, char Axis)
{
        UINT16 G_VALUE;
        
        // LCD���� ���õ� ���� '*'ǥ�ø� ����� ����
        for(int i=0;i<3;i++){
            LCD_DisplayChar(i+1,0,' ');
        }
        
        // (X,Y,Z)���� �������� ǥ��
        if (Axis & 0x01){    // Fram(400)�� ����  0x01�� and �Ͽ� bit.0�� ���� TRUE�� if�� ����
              LCD_DisplayChar(1,0,'*');
              
              // X �� ���ӵ� ǥ��		
              if (pBuf[0] < 0)  //����
              {
                      G_VALUE = abs(pBuf[0]);
                      LCD_DisplayChar(1,4,'-'); // g ��ȣ ǥ��
                      str_x[3] = '-';           // PCȭ�� ��ȣ ǥ��
                      
                      // X�� ����׷��� ����
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(88, 80, 125, 10); // + ���⿡�� �׷����� �������� �� ����� ����
                      
                      LCD_SetBrushColor(RGB_RED);   // Y�� �� ����
                      LCD_DrawFillRect(85 - 0.3*(100 * G_VALUE / 0x4009),80,0.3*(100 * G_VALUE / 0x4009)+1,10);  // - ������ ���� ����
                      LCD_SetBrushColor(RGB_WHITE); // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ���� ��
                      LCD_DrawFillRect(0, 80, 85 - 0.3*(100 * G_VALUE / 0x4009), 10); // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ����
              }
              else				// ���
              {
                      G_VALUE = pBuf[0];
                      LCD_DisplayChar(1,4,'+'); // g ��ȣ ǥ��
                      str_x[3] = '+';           // PCȭ�� ��ȣ ǥ��

                      // X�� ����׷��� ����
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(40,80,83,10);
                      
                      LCD_SetBrushColor(RGB_RED);   // X�� �� ����
                      LCD_DrawFillRect(88,80,0.3*(100 * G_VALUE / 0x4009),10);  // + ������ ���� ����
                      LCD_SetBrushColor(RGB_WHITE); // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ���� ��
                      LCD_DrawFillRect(88 + 0.3*(100 * G_VALUE / 0x4009), 80, 125 - 0.3*(100 * G_VALUE / 0x4009), 10);  // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ����
                      
              }
              G_VALUE = 10 * G_VALUE / 0x4009; // ���ӵ� --> g ��ȯ
              
              // X�� ���ӵ� �� ǥ��
              LCD_DisplayChar(1,5, G_VALUE/10 +0x30);
              LCD_DisplayChar(1,6,'.');
              LCD_DisplayChar(1,7, G_VALUE%10 +0x30);
              
              // PC���ۿ� string ���� (X:G_VALUE)
              str_x[0] = 'A';
              str_x[1] = 'x';
              str_x[2] = ':';
              str_x[4] = 0x30 + G_VALUE/10;
              str_x[5] = '.';
              str_x[6] = 0x30 + G_VALUE%10;
        }
        
        
        if (Axis & 0x02){     // Fram(400)�� ����  0x02�� and �Ͽ� bit.1�� ���� TEUE�� if�� ����
              LCD_DisplayChar(2,0,'*');
              
              // Y �� ���ӵ� ǥ��	
              if (pBuf[1] < 0)  //����
              {
                      G_VALUE = abs(pBuf[1]);
                      LCD_DisplayChar(2,4,'-'); // g ��ȣ ǥ��
                      str_y[3] = '-';           // PCȭ�� ��ȣ ǥ��
                      
                      // Y�� ����׷��� ����
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(88, 95, 125, 10); // + ���⿡�� �׷����� �������� �� ����� ����
                      
                      LCD_SetBrushColor(RGB_GREEN);   // Y�� �� ����
                      LCD_DrawFillRect(85 - 0.3*(100 * G_VALUE / 0x4009),95,0.3*(100 * G_VALUE / 0x4009)+1,10);  // - ������ ���� ����
                      LCD_SetBrushColor(RGB_WHITE); // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ���� ��
                      LCD_DrawFillRect(0, 95, 85 - 0.3*(100 * G_VALUE / 0x4009), 10); // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ����
              }
              else				// ���
              {
                      G_VALUE = pBuf[1];
                      LCD_DisplayChar(2,4,'+'); // g ��ȣ ǥ��
                      str_y[3] = '+';           // PCȭ�� ��ȣ ǥ��
                      
                       // Y�� ����׷��� ����
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(40,95,83,10);
                      
                      LCD_SetBrushColor(RGB_GREEN);   // Y�� �� ����
                      LCD_DrawFillRect(88,95,0.3*(100 * G_VALUE / 0x4009),10);  // + ������ ���� ����
                      LCD_SetBrushColor(RGB_WHITE); // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ���� ��
                      LCD_DrawFillRect(88 + 0.3*(100 * G_VALUE / 0x4009), 95, 125 - 0.3*(100 * G_VALUE / 0x4009), 10);  // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ����
              }
              G_VALUE = 10 * G_VALUE / 0x4009; 
              
              // Y�� ���ӵ� �� ǥ��
              LCD_DisplayChar(2,5, G_VALUE/10 +0x30);
              LCD_DisplayChar(2,6,'.');
              LCD_DisplayChar(2,7, G_VALUE%10 +0x30);
              
              // PC���ۿ� string ����  (Y:G_VALUE)
              str_y[0] = 'A';
              str_y[1] = 'y';
              str_y[2] = ':';
              str_y[4] = 0x30 + G_VALUE/10;
              str_y[5] = '.';
              str_y[6] = 0x30 + G_VALUE%10;
        }
        
        
        if (Axis & 0x04){      // Fram(400)�� ����  0x04�� and �Ͽ� bit.2�� ���� TEUE�� if�� ����
              LCD_DisplayChar(3,0,'*');
              
              // Z �� ���ӵ� ǥ��	
              if (pBuf[2] < 0)  //����
              {
                      G_VALUE = abs(pBuf[2]);
                      LCD_DisplayChar(3,4,'-'); // g ��ȣ ǥ��
                      str_z[3] = '-';           // PCȭ�� ��ȣ ǥ��
                      
                      // Z�� ����׷��� ����
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(88, 110, 125, 10); // + ���⿡�� �׷����� �������� �� ����� ����
                      
                      LCD_SetBrushColor(RGB_BLUE);   // Z�� �� ����
                      LCD_DrawFillRect(85 - 0.3*(100 * G_VALUE / 0x4009),110,0.3*(100 * G_VALUE / 0x4009)+1,10);  // - ������ ���� ����
                      LCD_SetBrushColor(RGB_WHITE); // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ���� ��
                      LCD_DrawFillRect(0, 80, 85 - 0.3*(100 * G_VALUE / 0x4009), 10); // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ����
              }
              else				// ���
              {
                      G_VALUE = pBuf[2];
                      LCD_DisplayChar(3,4,'+'); // g ��ȣ ǥ��
                      str_z[3] = '+';           // PCȭ�� ��ȣ ǥ��
                      
                       // Z�� ����׷��� ����
                      LCD_SetBrushColor(RGB_WHITE);
                      LCD_DrawFillRect(40,110,83,10);
                      
                      LCD_SetBrushColor(RGB_BLUE);   // Z�� �� ����
                      LCD_DrawFillRect(88,110,0.3*(100 * G_VALUE / 0x4009),10);  // + ������ ���� ����
                      LCD_SetBrushColor(RGB_WHITE); // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ���� ��
                      LCD_DrawFillRect(88 + 0.3*(100 * G_VALUE / 0x4009), 110, 125 - 0.3*(100 * G_VALUE / 0x4009), 10);  // ���밡 �پ�鶧, �پ�� �κ��� ������� ä��� ����
              }
              G_VALUE = 10 * G_VALUE / 0x4009; 
              
              // Z�� ���ӵ� �� ǥ��
              LCD_DisplayChar(3,5, G_VALUE/10 +0x30);
              LCD_DisplayChar(3,6,'.');
              LCD_DisplayChar(3,7, G_VALUE%10 +0x30);

              // PC���ۿ� string ����  (Z:G_VALUE)
              str_z[0] = 'A';
              str_z[1] = 'z';
              str_z[2] = ':';
              str_z[4] = 0x30 + G_VALUE/10;
              str_z[5] = '.';
              str_z[6] = 0x30 + G_VALUE%10;
        }
        
        // ����׷��� ���� �� �����
        LCD_SetBrushColor(RGB_BLACK); // ������ ����
        LCD_DrawFillRect(85,80,3,40);  // ������
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
	
	/*!< SPI1 NSS pin(PA8) configuration : GPIO ��  */
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
				// NSS �� ���°� �ڵ��� ���� ����
	SPI1->CR1 |= (1<<8);	// SSI(Internal_slave_select)=1,
				// ���� MCU�� Master�̹Ƿ� NSS ���´� 'High' 
	SPI1->CR1 &= ~(1<<7);	// LSBFirst=0, MSB transmitted first    
	SPI1->CR1 |= (4<<3);	// BR(BaudRate)=0b100, fPCLK/32 (84MHz/32 = 2.625MHz)
	SPI1->CR1 |= (1<<1);	// CPOL(Clock polarity)=1, CK is 'High' when idle
	SPI1->CR1 |= (1<<0);	// CPHA(Clock phase)=1, �� ��° edge ���� �����Ͱ� ���ø�
 
	SPI1->CR1 |= (1<<6);	// SPE=1, SPI1 Enable 
}

void TIMER10_Init(void){
     // TIM10 CH1 : PB8 (167�� ��)
     // Clock Enable : GPIOB & TIMER10
        RCC->APB2ENR |= (1<<17);
        RCC->AHB1ENR |= (1<<1);;
        
     // PB8�� ��¼����ϰ� Alternate function(TIM10_CH1)���� ��� ���� : CC���ͷ�Ʈ
        GPIOB->MODER 	|= (2<<16);	// 0x00020000 PB8 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<16);	// 0x00030000 PB8 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<8);	// PB8 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1<<16);	// 0x00010000 PB8 Pull-up
        GPIOB->AFR[1]   |= (3<<0);      // (AFR[1].(3~0)=0b0011): Connect TIM10 pins(PB8) to AF3(TIM8..11)
                                        // PB.8���� pin�� 8 �̱� ������, AFR[1] 
        
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
        
        // Output Compare ���� <CC ��带 �����ϴ� ��������>
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
        // Ch1�� ����ϴϱ� CCR1
	TIM10->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output 
	TIM10->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM10->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable(CCR1�� �������� ���ο� ���� loading ����) 
	TIM10->CCMR1 |= (3<<4);	 // OC1M=0b011 (Output Compare 1 Mode : toggle)
				 // OC1REF toggles when CNT = CCR1
        
        // CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM10->CCER &= ~(1<<0);	// CC1E=1: CC1 channel Output Enable
				// OC1(TIM14_CH1) Active: �ش���(27��)�� ���� ��ȣ���
	TIM10->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1���� �������� ���)  

	// CC1I(CC ���ͷ�Ʈ) ���ͷ�Ʈ �߻��ð� �Ǵ� ��ȣ��ȭ(���)�ñ� ����: ��ȣ�� ����(phase) ����
	TIM10->CCR1 = 10;	// TIM4 CCR1 TIM4_Pulse,  �޽��� ���۽ð��� �޶�����.
        
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
	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37��)
	USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
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

//* MFC���� ���ӵ� ǥ���Ҷ� ��ĭ�� �߰��Ͽ� ������ ������ �ϴ� �Լ� *//
void make_space(unsigned int *N){  // make_space�� ������ �����ͷ� ����
  if (*N != 0){                    // 0x00�� �ƴϸ� (���ӵ������� 1���� ������ if����)
      for (int i = 0; i < (74 - 12*(*N)); i++){   // ��ü �������� ��û���� ���ӵ� ������ ������ ��ĭ�� ���
                                                  // ��üĭ : 74
            SerialSendChar(' ');                  // ��ĭ ���
      }
  }
      *N = 0;                                     // name space �ʱ�ȭ
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
	LCD_SetFont(&Gulim8);		//��Ʈ 
	LCD_SetBackColor(RGB_YELLOW);
	LCD_SetTextColor(RGB_BLACK);    //���ڻ�
	LCD_DisplayText(0,0,"ACC sensor(SPI):PSY");  // Title

	LCD_SetBackColor(RGB_WHITE);    //���ڹ���

	LCD_DisplayText(1,1,"Ax:");	//X AXIS
	LCD_DisplayText(2,1,"Ay:");	//Y AXIS
	LCD_DisplayText(3,1,"Az:");	//Z AXIS
        
        // ����׷��� ���� �� ����
        LCD_SetBrushColor(RGB_BLACK); // ������ ����
        LCD_DrawFillRect(85,80,3,40);  // ������
        LCD_DisplayText(6,1,"Ax:");	//X AXIS
	LCD_DisplayText(7,1,"Ay:");	//Y AXIS
	LCD_DisplayText(8,1,"Az:");	//Z AXIS
        
        
}