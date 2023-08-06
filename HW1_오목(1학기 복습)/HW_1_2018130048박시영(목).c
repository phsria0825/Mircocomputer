///////////////////////////////////////////////////////////////
// ������: HW.1 Make O-MOK Game
// ��������: ��������� ����� ����
// ����� �ϵ����(���): GPIO, Joy-stick, EXTI, GLCD, FRAM
// ������: 2022. 9. 
// ������ Ŭ����:  ����Ϲ�
// �й�: 2018130048
// �̸�: �ڽÿ�
///////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"

// NO Key-Button : 0xFF00 : 1111 1111 0000 0000
                        // Bit No       FEDC BA98 7654 3210
#define SW0_PUSH        0xFE00  //PH8   1111 1110 0000 0000
#define SW1_PUSH        0xFD00  //PH9   1111 1101 0000 0000
#define SW2_PUSH        0xFB00  //PH10  1111 1011 0000 0000
#define SW3_PUSH        0xF700  //PH11  1111 0111 0000 0000
#define SW4_PUSH        0xEF00  //PH12  1110 1111 0000 0000
#define SW5_PUSH        0xDF00  //PH13  1101 1111 0000 0000
#define SW6_PUSH        0xBF00  //PH14  1011 1111 0000 0000
#define SW7_PUSH        0x7F00  //PH15  0111 1111 0000 0000

// NO Joy-Button : 0x03E0 : 0000 0011 1110 0000 
		        //Bit No      FEDC BA98 7654 3210
#define NAVI_PUSH	0x03C0  //PI5 0000 0011 1100 0000 
#define NAVI_UP		0x03A0  //PI6 0000 0011 1010 0000 
#define NAVI_DOWN       0x0360  //PI7 0000 0011 0110 0000 
#define NAVI_RIGHT	0x02E0  //PI8 0000 0010 1110 0000 
#define NAVI_LEFT	0x01E0  //PI9 0000 0001 1110 0000 

void _GPIO_Init(void);          // GPIO �ʱ�ȭ �Լ�
uint16_t KEY_Scan(void);	// Switch �Է� �Լ�
uint16_t JOY_Scan(void);	// Joystick �Է� �Լ�
void BEEP(void);                // Buzzer �Լ�
void DisplayInitScreen(void);   // LCD �ʱ�ȭ �Լ�(�ٵ��� ����)
void Display_Grid_Point(void);  // ������ ��ǥ�� ǥ���ϴ� �Լ�
void Return_Grid(void);         // ���ڸ���� ���������ִ� �Լ�
void Check_OMOK(void);          // �迭�� ������ �̹� �����ϴ��� üũ�ϴ� �Լ�
void Game_end(void);            // �¸����� �Լ�
void Clear(void);               // �¸��� �ʱ�ȭ �Լ�
void _EXTI_Init(void);          // Interrupt �ʱ�ȭ �Լ�
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

uint8_t	SW0_Flag, SW1_Flag, SW2_Flag, SW3_Flag, SW4_Flag;	// SW0_Flag: Joystick UP Interrupt Flag
                                                                // SW1_Flag: Joystick DOWN Interrupt Flag
                                                                // SW2_Flag: Joystick RIGHT Interrupt Flag
                                                                // SW3_Flag: Joystick LEFT Interrupt Flag
                                                                // SW4_Flag: Joystick PUSH Interrupt Flag

uint8_t RED_Flag, BLUE_Flag;  // Interrupt Flag : RED_Flag(SW0) <EXTI.8>, BLUE_Flag(SW15) <EXTI.15>
unsigned int RED_score;   // RED�� ������ ��Ÿ���� ����
unsigned int BLUE_score;  // BLUE�� ������ ��Ÿ���� ����

int cell[10][10] = {0};  // ������ ���� ��ġ�� ����ϱ� ���� 2D �迭
int Red_X = 5;      // �������� X��ǥ
int Red_Y =5;       // �������� Y��ǥ
int Blue_X =5;      // �Ķ����� X��ǥ
int Blue_Y = 5;     // �Ķ����� Y��ǥ
int Old_Red_X, Old_Red_Y, Old_Blue_X, Old_Blue_Y; // ���� ��ǥ�� �����ϴ� ����
uint8_t check_Flag, red_end, blue_end;      // ���񿩺� üũ�ϴ� Flag &  ������ ������ Flag
uint8_t win_Flag;   // �¸��� �����ϴ� Flag

int main(void)
{
	_GPIO_Init(); 	// GPIO (LED,SW,Buzzer,Joy stick) �ʱ�ȭ
	LCD_Init();		// LCD ��� �ʱ�ȭ
        DelayMS(10);
        Fram_Init();            // FRAM �ʱ�ȭ H/W �ʱ�ȭ
	Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
        _EXTI_Init();		// �ܺ����ͷ�Ʈ �ʱ�ȭ
	BEEP();
	DisplayInitScreen();	// LCD �ʱ�ȭ��
	GPIOG->ODR &= ~0x00FF;	// LED �ʱⰪ: LED0~7 Off
        
	while(1)
	{
		switch(JOY_Scan())	          // �Էµ� Joystick ���� �з�
		{
                  case NAVI_UP: 	          // Joystick UP
                    if(RED_Flag == 1){            // RED �����̸�
                      BEEP();
                      Return_Grid();              // ���ڸ���� ���������ִ� �Լ�
                      Red_Y--;                    // ���� ��ĭ
                      Display_Grid_Point();       // ������ ��ǥ ǥ���Լ�
                    }
                    if(BLUE_Flag == 1){           // BLUE �����̸�
                      BEEP();
                      Return_Grid();              // ���ڸ���� ���������ִ� �Լ�
                      Blue_Y--;                   // ������ĭ
                      Display_Grid_Point();       // ������ ��ǥ ǥ���Լ�
                    }
                  break;
                  case NAVI_DOWN:	          // Joystick DOWN	
                    if(RED_Flag == 1){            // RED �����̸�
                      BEEP();
                      Return_Grid();              // ���ڸ���� ���������ִ� �Լ�
                      Red_Y++;                    // �Ʒ��� ��ĭ
                      Display_Grid_Point();       // ������ ��ǥ ǥ���Լ�
                    }
                    if(BLUE_Flag == 1){           // BLUE �����̸�
                      BEEP();
                      Return_Grid();              // ���ڸ���� ���������ִ� �Լ�
                      Blue_Y++;                   // �Ʒ��� ��ĭ
                      Display_Grid_Point();       // ������ ��ǥ ǥ���Լ�
                    }
                  break;
                  case NAVI_RIGHT:                // Joystick RIGHT
                    if(RED_Flag == 1){            // RED �����̸�
                      BEEP();
                      Return_Grid();              // ���ڸ���� ���������ִ� �Լ�
                      Red_X++;                    // �����ʷ� ��ĭ
                      Display_Grid_Point();       // ������ ��ǥ ǥ���Լ�
                    }
                    if(BLUE_Flag == 1){           // BLUE �����̸�
                      BEEP();
                      Return_Grid();              // ���ڸ���� ���������ִ� �Լ�
                      Blue_X++;                   // �����ʷ� ��ĭ
                      Display_Grid_Point();       // ������ ��ǥ ǥ���Լ�
                    }
                  break;
                  case NAVI_LEFT:                 // Joystick RIGHT
                    if(RED_Flag == 1){            // RED �����̸�
                      BEEP();
                      Return_Grid();              // ���ڸ���� ���������ִ� �Լ�
                      Red_X--;                    // ���ʷ� ��ĭ
                      Display_Grid_Point();       // ������ ��ǥ ǥ���Լ�
                    }
                    if(BLUE_Flag == 1){           // BLUE �����̸�
                      BEEP();
                      Return_Grid();              // ���ڸ���� ���������ִ� �Լ�
                      Blue_X--;                   // ���ʷ� ��ĭ
                      Display_Grid_Point();       // ������ ��ǥ ǥ���Լ�
                    }
                  break;
        	}  // switch(JOY_Scan())

	}  // while(1)
}

/* GPIO (GPIOG(LED), GPIOH(Switch), GPIOF(Buzzer), GPIOI(Joy stick)) �ʱ� ����	*/
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

	//Joy Stick SW(PORT I) ����
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	&= ~0x000FFC00;	// GPIOI 5~9 : Input mode (reset state)
	GPIOI->PUPDR    &= ~0x000FFC00;	// GPIOI 5~9 : Floating input (No Pull-up, pull-down) (reset state)
}	

/* GLCD �ʱ�ȭ�� ���� */
void DisplayInitScreen(void)
{
	LCD_Clear(RGB_YELLOW);		// ���ȭ��
	LCD_SetFont(&Gulim8);		// ���� 8
	LCD_SetBackColor(RGB_YELLOW);	// ���ڹ�� : WHITE
	LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : Black
        LCD_SetPenColor(RGB_BLACK);     // �׸����
         
        /*make O-Mok Grid*/
        // Grid�� ���� ǥ��
        // ������
	LCD_DisplayText(0,0,"Mecha-OMOK(PSY)");	// subtitle
        LCD_DisplayText(1,9,"5" );	// Grid mark
        LCD_DisplayText(1,13,"9" );	// Grid mark
        // ������
        LCD_DisplayText(1,2,"0" );	// Grid mark
        LCD_DisplayText(5,2,"5" );	// Grid mark
        LCD_DisplayText(8,2,"9" );	// Grid mark
       
        //Grid ���� (start-point<24,24> : end-point<114, 114> : distance between line,10)
        for (int i=0; i<10; i++){
          LCD_DrawHorLine(24, 24 + (10*i), 90);
          LCD_DrawVerLine(24 + (10*i), 24, 90);
        }//end for loop
        
        
        //������ ����
        LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : BLACK
        LCD_DisplayText(9,8,"VS");
        
        // Fram���� ���� ��������
        LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
        LCD_DisplayChar(9,6,Fram_Read(300)+0x30); // Fram.300 <red score>
        LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
        LCD_DisplayChar(9,11,Fram_Read(301)+0x30); // Fram.301 <blue score>
        
        //�ʱ� ������ǥ ǥ��
        //RED ��ǥ
        LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
        LCD_DisplayChar(9,1,'(');
        LCD_DisplayChar(9,2,Red_X + 0x30);
        LCD_DisplayChar(9,3,',');
        LCD_DisplayChar(9,4,Red_Y + 0x30);
        LCD_DisplayChar(9,5,')');
        
        //BLUE ��ǥ
        LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
        LCD_DisplayChar(9,13,'(');
        LCD_DisplayChar(9,14,Blue_X + 0x30);
        LCD_DisplayChar(9,15,',');
        LCD_DisplayChar(9,16,Blue_Y + 0x30);
        LCD_DisplayChar(9,17,')');
        
        // <5,5>���� 
        LCD_SetBrushColor(RGB_BLACK);
        LCD_DrawFillRect(24+(10*5)-2, 24+(10*5)-2, 5, 5);
        
        
}

/* ������ ������ ��ġ�� ���� Grid ���·� �ٲٴ� �Լ� */
// Joy_stick���� ������ �����̰��� ����� �������� �����
void Return_Grid(void)
{
  // ���� ��
  if(RED_Flag == 1)
  {
    // ���� ���� ��ǥ��  ���� ��ǥ�� ����
    Old_Red_X = Red_X;
    Old_Red_Y = Red_Y;
    
    // ���� ��ǥ�� ������ ũ���� ������ ������ ���� �����.
    LCD_SetBrushColor(RGB_YELLOW);       // �׸���� : YELLOW
    LCD_DrawFillRect(24+(10*Old_Red_X)-4,24+(10*Old_Red_Y)-4,8,8);  
  }
  
  if(BLUE_Flag == 1)
  {
    // ���� ���� ��ǥ��  ���� ��ǥ�� ����
    Old_Blue_X = Blue_X;
    Old_Blue_Y = Blue_Y;
    
    // ���� ��ǥ�� ������ ũ���� ������ ������ ���� �����.
    LCD_SetBrushColor(RGB_YELLOW);       // �׸���� : YELLOW
    LCD_DrawFillRect(24+(10*Old_Blue_X)-4,24+(10*Old_Blue_Y)-4,8,8);  
  }
  
  // ���� ���� ����
  for(int i=0; i<10; i++){
     LCD_DrawHorLine(24, 24 + (10*i), 90);
     LCD_DrawVerLine(24 + (10*i), 24, 90);
  }//end for loop
  
  // <5,5>���� 
  LCD_SetBrushColor(RGB_BLACK);
  LCD_DrawFillRect(24+(10*5)-2, 24+(10*5)-2, 5, 5);
  
  // ���������� ǥ���ϴ� �κ�
  for(int i=0; i<10; i++){   // 2D �迭�� ��
    for(int j=0; j<10; j++){ // 2D �迭�� ��
      if(cell[i][j]==1){     // 2D �迭�� ���������� ������,
        // �������� ǥ��
        LCD_SetBrushColor(RGB_RED);       // �׸���� : RED
        LCD_DrawFillRect(24+(10*i)-4,24+(10*j)-4,8,8);
      }
      
      if(cell[i][j]==2){     // 2D �迭�� �Ķ������� ������,
        // �Ķ����� ǥ��
        LCD_SetBrushColor(RGB_BLUE);       // �׸���� : BLUE
        LCD_DrawFillRect(24+(10*i)-4,24+(10*j)-4,8,8);
      }
    }
  }
  
}// end void Return_Grid(void) function

/* ���� ������ ��ġ�� ��ǥ�� LCD�� ǥ���ϴ� �Լ� */
void Display_Grid_Point(void)
{
  /* 9x9 ������ �ִ��� �Ǵ��� ���Ѿ�� �ϴ� ���ǹ� */
  if((Red_X > 9) || (Red_Y > 9) || (Blue_X > 9) || (Blue_Y > 9)){   // ���� ������ �Ѿ��,
    // ������ ��ǥ�� ���� ��ǥ�� �����Ѵ�
    Red_X = Old_Red_X;
    Red_Y = Old_Red_Y;
    Blue_X = Old_Blue_X;
    Blue_Y = Old_Blue_Y;
  }
  if((Red_X < 0) || (Red_Y < 0) || (Blue_X < 0) || (Blue_Y < 0)){   // ���� ������ �Ѿ��,
    // ������ ��ǥ�� ���� ��ǥ�� �����Ѵ�
    Red_X = Old_Red_X;
    Red_Y = Old_Red_Y;
    Blue_X = Old_Blue_X;
    Blue_Y = Old_Blue_Y;
  } // 9x9 ������ �ִ��� �Ǵ� ���ǹ� ����
  
  if(RED_Flag)
  {
    //RED ��ǥ ǥ��
    LCD_DisplayChar(9,1,'(');
    LCD_DisplayChar(9,2,Red_X + 0x30);
    LCD_DisplayChar(9,3,',');
    LCD_DisplayChar(9,4,Red_Y + 0x30);
    LCD_DisplayChar(9,5,')');
    
    //���� �� ��ġ�� ��ǥ�� �׸���
    LCD_SetBrushColor(RGB_RED);       // �׸���� : RED
    LCD_DrawFillRect(24+(10*Red_X)-4,24+(10*Red_Y)-4,8,8);
  }
  if(BLUE_Flag)
  {
    LCD_DisplayChar(9,13,'(');
    LCD_DisplayChar(9,14,Blue_X + 0x30);
    LCD_DisplayChar(9,15,',');
    LCD_DisplayChar(9,16,Blue_Y + 0x30);
    LCD_DisplayChar(9,17,')');
    
    //���� �� ��ġ�� ��ǥ�� �׸���
    LCD_SetBrushColor(RGB_BLUE);       // �׸���� : BLUE
    LCD_DrawFillRect(24+(10*Blue_X)-4,24+(10*Blue_Y)-4,8,8);
  }
}

/* ���� ���翩�� Ȯ�� �Լ� */ 
void Check_OMOK(void){
  if((cell[Red_X][Red_Y]!= 0) && (cell[Blue_X][Blue_Y]!= 0)){   // 2���� �迭�� ������ �����ϸ�
    check_Flag = 1;           // üũ�÷��� 1 : �̹� ������ �����Ѵٴ� �ǹ�
  }
}

/* ���� �¸����� �˰��� */
void Game_end(void){
  // Fram���� ���� ��������
  RED_score = Fram_Read(300);
  BLUE_score = Fram_Read(301);
  
// ���� �¸� üũ
  if(red_end == 1){
   for(int i=0; i < 10; i++) { 
    for(int j=0; j < 10; j++) {
       // ����(��) ���� üũ,
       if(cell[i][j]==1 && cell[i][j+1]==1 && 
          cell[i][j+2]==1 && cell[i][j+3]==1 && 
          cell[i][j+4]==1){
          RED_score++;       // ������ �����ϸ� +1
          win_Flag = 1;      // �¸� Flag = 1
       }
       // ����(��) ���� üũ,
       else if(cell[i][j]==1 && cell[i+1][j]==1 && 
          cell[i+2][j]==1 && cell[i+3][j]==1 && 
          cell[i+4][j]==1){
          RED_score++;      // ������ �����ϸ� +1
          win_Flag = 1;     // �¸� Flag = 1
       }
       // �밢��(��) ���� üũ,
       else if(cell[i][j]==1 && cell[i+1][j+1]==1 && 
          cell[i+2][j+2]==1 && cell[i+3][j+3]==1 && 
          cell[i+4][j+4]==1){
          RED_score++;     // ������ �����ϸ� +1
          win_Flag = 1;    // �¸� Flag = 1
       }
       // �밢��(��) ���� üũ,
       else if(cell[i][j]==1 && cell[i-1][j+1]==1 && 
          cell[i-2][j+2]==1 && cell[i-3][j+3]==1 && 
          cell[i-4][j+4]==1){
          RED_score++;     // ������ �����ϸ� +1
          win_Flag = 1;    // �¸� Flag = 1
       }
      } 
    }
  }

// �Ķ� �¸�üũ
  if(blue_end == 1){
   for(int i=0; i < 10; i++) { 
    for(int j=0; j < 10; j++) {
       // ����(��) ���� üũ,
       if(cell[i][j]==2 && cell[i][j+1]==2 && 
          cell[i][j+2]==2 && cell[i][j+3]==2 && 
          cell[i][j+4]==2){
          BLUE_score++;     // ������ �����ϸ� +1
          win_Flag = 1;     // �¸� Flag = 1
       }
       // ����(��) ���� üũ,
       else if(cell[i][j]==2 && cell[i+1][j]==2 && 
          cell[i+2][j]==2 && cell[i+3][j]==2 && 
          cell[i+4][j]==2){
          BLUE_score++;     // ������ �����ϸ� +1
          win_Flag = 1;     // �¸� Flag = 1
       }
       // �밢��(��) ���� üũ,
       else if(cell[i][j]==2 && cell[i+1][j+1]==2 && 
          cell[i+2][j+2]==2 && cell[i+3][j+3]==2 && 
          cell[i+4][j+4]==2){
          BLUE_score++;     // ������ �����ϸ� +1
          win_Flag = 1;     // �¸� Flag = 1
       }
       // �밢��(��) ���� üũ,
       else if(cell[i][j]==2 && cell[i-1][j+1]==2 && 
          cell[i-2][j+2]==2 && cell[i-3][j+3]==2 && 
          cell[i-4][j+4]==2){
          BLUE_score++;     // ������ �����ϸ� +1
          win_Flag = 1;     // �¸� Flag = 1
       }
      } 
    }
  }
  
  // Fram�� ���� ����
  if(RED_score <= 9){           // ������ 9���� ������
    Fram_Write(300,RED_score);  // �׳�����
  }
  else{                         // RED_score > 9
    RED_score = 0;              // RED_score�� 0���� �ϰ�
    Fram_Write(300,RED_score);  // Fram�� ����
  }
  
  if(BLUE_score <= 9){          // ������ 9���� ������
    Fram_Write(301,BLUE_score); // �׳�����
  }
  else{                         // BLUE_score > 9
    BLUE_score = 0;             // BLUE_score�� 0���� �ϰ�
    Fram_Write(301,BLUE_score); // Fram�� ����
  }
} // function end

/* Clear Function */
void Clear(void){
  // ��� ǥ��
          
  // �¸��� ������ �ʱ�ȭ
  if(win_Flag == 1){        // �¸��� �����Ǹ�
    GPIOG->ODR &= ~0x00FF;  // ���ʸ� ǥ���ϴ� LED ���� OFF
    
    for(int i=0; i<5; i++){ // �¸� �˸�
      BEEP();
      DelayMS(500);
    }
    DelayMS(5000);          // 5�� ����
    
    // 2D�迭 ���� �ʱ�ȭ
    for(int i=0; i<10; i++){
      for(int j=0; j<10; j++){
        cell[i][j] = 0;
      }
    }
    
    // ��ǥ �ʱ�ȭ
    Red_X = 5;      // �������� X��ǥ
    Red_Y =5;       // �������� Y��ǥ
    Blue_X =5;      // �Ķ����� X��ǥ
    Blue_Y = 5;     // �Ķ����� Y��ǥ
    
    // ��� FLAG ���� �ʱ�ȭ
    check_Flag = 0;
    red_end = 0;
    blue_end = 0;
    win_Flag = 0;
    
    DisplayInitScreen(); // ȭ�� �ʱ�ȭ
  }
}

/* EXTI (EXTI8(GPIOH.8, SW0), EXTI15(GPIOH.15, SW7), EXTI5(GPIOI.5, NAVI_PUSH)) �ʱ� ���� */
void _EXTI_Init(void)
{
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
        RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
	
	GPIOI->MODER 	&= ~0x000FFC00;	// GPIOI 5~9 : Input mode (reset state)
        GPIOH->MODER    &= ~0xFFFF0000; // GPIOH 8~15 : Input mode (reset state)
        
        SYSCFG->EXTICR[1] |= 0x0080;	// EXTI5�� ���� �ҽ� �Է��� GPIOI�� ����
					// EXTI5 <- PI5(NAVI_PUSH)
					// EXTICR2(EXTICR[1])�� �̿� 
					// reset value: 0x0000	
        
	SYSCFG->EXTICR[2] |= 0x0007;	// EXTI8�� ���� �ҽ� �Է��� GPIOH�� ����
					// EXTI8 <- PH8(SW0)
					// EXTICR3(EXTICR[2])�� �̿� 
					// reset value: 0x0000	

        SYSCFG->EXTICR[3] |= 0x7000;	// EXTI15�� ���� �ҽ� �Է��� GPIOH�� ����
					// EXTI15 <- PH15(SW7)
					// EXTICR4(EXTICR[3])�� �̿� 
					// reset value: 0x0000	

	EXTI->RTSR |= 0x009120;		// EXTI8, 15: Rising Trigger  Enable
	EXTI->IMR  |= 0x009120;		// EXTI8, 15 ���ͷ�Ʈ mask (Interrupt Enable) ����
		
	NVIC->ISER[0] |= (1 << 23);	// 0x00800000
					// Enable 'Global Interrupt EXTI8'
					// Vector table Position ����
        
        NVIC->ISER[1] |= (1 << (40-32));   // 0x00000100
                                           // Enable 'Global Interrupt EXTI15'
                                           // Vector table Position ����
}

/* EXTI5~9 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void)		
{
  if(EXTI->PR & 0x0100){                // EXTI.8 Interrupt Pending(�߻�) ����?

    EXTI->PR |= 0x0100; 		// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
    if((BLUE_Flag == 1)||(red_end == 1)){   // ���� BLUE���� �̰ų� �������� ���ʰ� ��������,
      RED_Flag = 0;                     // RED�� ������ ���ΰ� �Ѵ�
    }
    else{
      RED_Flag = 1;                     // RED ����
      blue_end = 0;                     // �Ķ����� �������� �ʱ�ȭ
      LCD_SetTextColor(RGB_YELLOW);	// ���ڻ� : YELLOW
      LCD_DisplayText(9,12,"*" );	// Grid mark
      
      LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
      LCD_DisplayText(9,0,"*" );	// Grid mark
      GPIOG->ODR &= ~0x0080;            // �Ķ� LED OFF
      GPIOG->ODR |= 0x0001;             // ���� LED ON
    }
  }
  
  if(EXTI->PR & 0x0020){                // EXTI.5 Interrupt Pending(�߻�) ����?
    
    EXTI->PR |= 0x0020;                 //Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
    
    if((RED_Flag || BLUE_Flag) == 1){   //���� �ѻ���� ������ �����ʸ�,
      if(RED_Flag == 1){                //������ �����ʸ�,
        // ���� üũ�Լ� ����
        Check_OMOK();
        
        if(check_Flag == 1){            // �̹� ������ �����ϸ�,
          // ����� 3��
          for(int k=0; k<3; k++){
            BEEP();
            DelayMS(50);
          } // BEEP()
        }// check_Flag == 1
        
        else{                           // ������ ���� ���� ������,
          BEEP();
          cell[Red_X][Red_Y] = 1;       //2���� �迭�� 1�� �����Ͽ� ������ġ�� ���
          RED_Flag = 0;                 // ���� ���� ���� �ʱ�ȭ
          red_end = 1;                  // �������� ��������
          Game_end();                   // �¸�üũ
          Clear();                      // �ʱ�ȭ (�¸� �����ÿ��� ����ȴ�)
        } // check_Flag == 0
        check_Flag = 0;                // check_Flag �ʱ�ȭ
      } //���� ���ʸ� if
      
      else{                            // �Ķ��� �������̸�,
        // ���� üũ�Լ� ����
        Check_OMOK();
        
        if(check_Flag == 1){           // �̹� ������ �����ϸ�,
          // ����� 3��
          for(int k=0; k<3; k++){
            BEEP();
            DelayMS(50);
          } // BEEP()
        } // check_Flag == 1
        
        else{                          // ������ ���� ���� ������,
          BEEP();
          cell[Blue_X][Blue_Y] = 2;    // 2���� �迭�� 2�� �����Ͽ� ������ġ�� ���
          BLUE_Flag = 0;               // �Ķ����� ���� �ʱ�ȭ
          blue_end = 1;                // �Ķ����� ��������
          Game_end();                  // �¸� üũ
          Clear();                      // �ʱ�ȭ (�¸� �����ÿ��� ����ȴ�)
        } // check_Flag == 0
        check_Flag = 0;                // check_Flag �ʱ�ȭ
      } // �Ķ��� �����ʸ� else
      
    } //���� �ѻ���� ������ �����ʸ� if 
    
  } // EXTI.5 Interrupt Pending?
  
} // void EXTI9_5_IRQHandler(void)

/* EXTI10~15 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void){
  if(EXTI->PR & 0x8000){                // EXTI.15 Interrupt Pending(�߻�) ����?
    
    EXTI->PR |= 0x8000;                 //Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
    
    if((RED_Flag == 1)||(blue_end==1)){ // ���� RED���� �̰ų� �Ķ����� ���ʰ� ��������
      BLUE_Flag = 0;                    // BLUE�� ������ ���ΰ� �Ѵ�
    }
    else{
      BLUE_Flag = 1;                    // �Ķ����� ����
      red_end = 0;                      // �������� �������� �ʱ�ȭ
      LCD_SetTextColor(RGB_YELLOW);	// ���ڻ� : YELLOW
      LCD_DisplayText(9,0,"*" );	// Grid mark
      
      LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
      LCD_DisplayText(9,12,"*" );	// Grid mark
      GPIOG->ODR &= ~0x0001;            // ���� LED OFF
      GPIOG->ODR |= 0x0080;             // �Ķ� LED ON
    }
  }
}	

/* Joystick switch�� �ԷµǾ������� ���ο� � Joystick switch�� �ԷµǾ������� ������ return�ϴ� �Լ�  */ 
uint8_t joy_flag = 0;
uint16_t JOY_Scan(void)	// input joy stick NAVI_* 
{ 
	uint16_t key;
	key = GPIOI->IDR & 0x03E0;	// any key pressed ?
	if(key == 0x03E0)		// if no key, check key off
	{  	if(joy_flag == 0)
        		return key;
      		else
		{	DelayMS(10);
        		joy_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(joy_flag != 0)	// if continuous key, treat as no key input
        		return 0x03E0;
      		else			// if new key,delay for debounce
		{	joy_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
}

/* Switch�� �ԷµǾ������� ���ο� � switch�� �ԷµǾ������� ������ return�ϴ� �Լ�  */ 
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

/* Buzzer: Beep for 30 ms */
void BEEP(void)			
{ 	
	GPIOF->ODR |=  0x0200;	// PF9 'H' Buzzer on
	DelayMS(50);		// Delay 50 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);	// 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
	for(; Dly; Dly--);
}