///////////////////////////////////////////////////////////////
// 과제명: HW.1 Make O-MOK Game
// 과제개요: 오목게임을 만드는 과제
// 사용한 하드웨어(기능): GPIO, Joy-stick, EXTI, GLCD, FRAM
// 제출일: 2022. 9. 
// 제출자 클래스:  목요일반
// 학번: 2018130048
// 이름: 박시영
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

void _GPIO_Init(void);          // GPIO 초기화 함수
uint16_t KEY_Scan(void);	// Switch 입력 함수
uint16_t JOY_Scan(void);	// Joystick 입력 함수
void BEEP(void);                // Buzzer 함수
void DisplayInitScreen(void);   // LCD 초기화 함수(바둑판 생성)
void Display_Grid_Point(void);  // 오목의 좌표를 표시하는 함수
void Return_Grid(void);         // 격자모양을 유지시켜주는 함수
void Check_OMOK(void);          // 배열에 오목이 이미 존재하는지 체크하는 함수
void Game_end(void);            // 승리판정 함수
void Clear(void);               // 승리후 초기화 함수
void _EXTI_Init(void);          // Interrupt 초기화 함수
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

uint8_t	SW0_Flag, SW1_Flag, SW2_Flag, SW3_Flag, SW4_Flag;	// SW0_Flag: Joystick UP Interrupt Flag
                                                                // SW1_Flag: Joystick DOWN Interrupt Flag
                                                                // SW2_Flag: Joystick RIGHT Interrupt Flag
                                                                // SW3_Flag: Joystick LEFT Interrupt Flag
                                                                // SW4_Flag: Joystick PUSH Interrupt Flag

uint8_t RED_Flag, BLUE_Flag;  // Interrupt Flag : RED_Flag(SW0) <EXTI.8>, BLUE_Flag(SW15) <EXTI.15>
unsigned int RED_score;   // RED의 점수를 나타내는 변수
unsigned int BLUE_score;  // BLUE의 점수를 나타내는 변수

int cell[10][10] = {0};  // 오목이 놓인 위치를 기억하기 위한 2D 배열
int Red_X = 5;      // 빨간돌의 X좌표
int Red_Y =5;       // 빨간돌의 Y좌표
int Blue_X =5;      // 파란돌의 X좌표
int Blue_Y = 5;     // 파란돌의 Y좌표
int Old_Red_X, Old_Red_Y, Old_Blue_X, Old_Blue_Y; // 이전 좌표를 저장하는 변수
uint8_t check_Flag, red_end, blue_end;      // 오목여부 체크하는 Flag &  착돌후 턴종료 Flag
uint8_t win_Flag;   // 승리를 감지하는 Flag

int main(void)
{
	_GPIO_Init(); 	// GPIO (LED,SW,Buzzer,Joy stick) 초기화
	LCD_Init();		// LCD 모듈 초기화
        DelayMS(10);
        Fram_Init();            // FRAM 초기화 H/W 초기화
	Fram_Status_Config();   // FRAM 초기화 S/W 초기화
        _EXTI_Init();		// 외부인터럽트 초기화
	BEEP();
	DisplayInitScreen();	// LCD 초기화면
	GPIOG->ODR &= ~0x00FF;	// LED 초기값: LED0~7 Off
        
	while(1)
	{
		switch(JOY_Scan())	          // 입력된 Joystick 정보 분류
		{
                  case NAVI_UP: 	          // Joystick UP
                    if(RED_Flag == 1){            // RED 차례이면
                      BEEP();
                      Return_Grid();              // 격자모양을 유지시켜주는 함수
                      Red_Y--;                    // 위로 한칸
                      Display_Grid_Point();       // 움직인 좌표 표시함수
                    }
                    if(BLUE_Flag == 1){           // BLUE 차례이면
                      BEEP();
                      Return_Grid();              // 격자모양을 유지시켜주는 함수
                      Blue_Y--;                   // 위로한칸
                      Display_Grid_Point();       // 움직인 좌표 표시함수
                    }
                  break;
                  case NAVI_DOWN:	          // Joystick DOWN	
                    if(RED_Flag == 1){            // RED 차례이면
                      BEEP();
                      Return_Grid();              // 격자모양을 유지시켜주는 함수
                      Red_Y++;                    // 아래로 한칸
                      Display_Grid_Point();       // 움직인 좌표 표시함수
                    }
                    if(BLUE_Flag == 1){           // BLUE 차례이면
                      BEEP();
                      Return_Grid();              // 격자모양을 유지시켜주는 함수
                      Blue_Y++;                   // 아래로 한칸
                      Display_Grid_Point();       // 움직인 좌표 표시함수
                    }
                  break;
                  case NAVI_RIGHT:                // Joystick RIGHT
                    if(RED_Flag == 1){            // RED 차례이면
                      BEEP();
                      Return_Grid();              // 격자모양을 유지시켜주는 함수
                      Red_X++;                    // 오른쪽로 한칸
                      Display_Grid_Point();       // 움직인 좌표 표시함수
                    }
                    if(BLUE_Flag == 1){           // BLUE 차례이면
                      BEEP();
                      Return_Grid();              // 격자모양을 유지시켜주는 함수
                      Blue_X++;                   // 오른쪽로 한칸
                      Display_Grid_Point();       // 움직인 좌표 표시함수
                    }
                  break;
                  case NAVI_LEFT:                 // Joystick RIGHT
                    if(RED_Flag == 1){            // RED 차례이면
                      BEEP();
                      Return_Grid();              // 격자모양을 유지시켜주는 함수
                      Red_X--;                    // 왼쪽로 한칸
                      Display_Grid_Point();       // 움직인 좌표 표시함수
                    }
                    if(BLUE_Flag == 1){           // BLUE 차례이면
                      BEEP();
                      Return_Grid();              // 격자모양을 유지시켜주는 함수
                      Blue_X--;                   // 왼쪽로 한칸
                      Display_Grid_Point();       // 움직인 좌표 표시함수
                    }
                  break;
        	}  // switch(JOY_Scan())

	}  // while(1)
}

/* GPIO (GPIOG(LED), GPIOH(Switch), GPIOF(Buzzer), GPIOI(Joy stick)) 초기 설정	*/
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

	//Joy Stick SW(PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	&= ~0x000FFC00;	// GPIOI 5~9 : Input mode (reset state)
	GPIOI->PUPDR    &= ~0x000FFC00;	// GPIOI 5~9 : Floating input (No Pull-up, pull-down) (reset state)
}	

/* GLCD 초기화면 설정 */
void DisplayInitScreen(void)
{
	LCD_Clear(RGB_YELLOW);		// 배경화면
	LCD_SetFont(&Gulim8);		// 굴림 8
	LCD_SetBackColor(RGB_YELLOW);	// 글자배경 : WHITE
	LCD_SetTextColor(RGB_BLACK);	// 글자색 : Black
        LCD_SetPenColor(RGB_BLACK);     // 그리기색
         
        /*make O-Mok Grid*/
        // Grid의 숫자 표시
        // 가로축
	LCD_DisplayText(0,0,"Mecha-OMOK(PSY)");	// subtitle
        LCD_DisplayText(1,9,"5" );	// Grid mark
        LCD_DisplayText(1,13,"9" );	// Grid mark
        // 세로축
        LCD_DisplayText(1,2,"0" );	// Grid mark
        LCD_DisplayText(5,2,"5" );	// Grid mark
        LCD_DisplayText(8,2,"9" );	// Grid mark
       
        //Grid 생성 (start-point<24,24> : end-point<114, 114> : distance between line,10)
        for (int i=0; i<10; i++){
          LCD_DrawHorLine(24, 24 + (10*i), 90);
          LCD_DrawVerLine(24 + (10*i), 24, 90);
        }//end for loop
        
        
        //점수판 생성
        LCD_SetTextColor(RGB_BLACK);	// 글자색 : BLACK
        LCD_DisplayText(9,8,"VS");
        
        // Fram에서 점수 가져오기
        LCD_SetTextColor(RGB_RED);	// 글자색 : RED
        LCD_DisplayChar(9,6,Fram_Read(300)+0x30); // Fram.300 <red score>
        LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
        LCD_DisplayChar(9,11,Fram_Read(301)+0x30); // Fram.301 <blue score>
        
        //초기 오목좌표 표시
        //RED 좌표
        LCD_SetTextColor(RGB_RED);	// 글자색 : RED
        LCD_DisplayChar(9,1,'(');
        LCD_DisplayChar(9,2,Red_X + 0x30);
        LCD_DisplayChar(9,3,',');
        LCD_DisplayChar(9,4,Red_Y + 0x30);
        LCD_DisplayChar(9,5,')');
        
        //BLUE 좌표
        LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
        LCD_DisplayChar(9,13,'(');
        LCD_DisplayChar(9,14,Blue_X + 0x30);
        LCD_DisplayChar(9,15,',');
        LCD_DisplayChar(9,16,Blue_Y + 0x30);
        LCD_DisplayChar(9,17,')');
        
        // <5,5>지점 
        LCD_SetBrushColor(RGB_BLACK);
        LCD_DrawFillRect(24+(10*5)-2, 24+(10*5)-2, 5, 5);
        
        
}

/* 오목이 지나간 위치를 원래 Grid 형태로 바꾸는 함수 */
// Joy_stick으로 오목을 움직이고나면 생기는 오목모양을 지운다
void Return_Grid(void)
{
  // 빨강 돌
  if(RED_Flag == 1)
  {
    // 현재 오목 좌표를  이전 좌표에 대입
    Old_Red_X = Red_X;
    Old_Red_Y = Red_Y;
    
    // 이전 좌표로 동일한 크기의 오목판 배경색의 돌을 만든다.
    LCD_SetBrushColor(RGB_YELLOW);       // 그리기색 : YELLOW
    LCD_DrawFillRect(24+(10*Old_Red_X)-4,24+(10*Old_Red_Y)-4,8,8);  
  }
  
  if(BLUE_Flag == 1)
  {
    // 현재 오목 좌표를  이전 좌표에 대입
    Old_Blue_X = Blue_X;
    Old_Blue_Y = Blue_Y;
    
    // 이전 좌표로 동일한 크기의 오목판 배경색의 돌을 만든다.
    LCD_SetBrushColor(RGB_YELLOW);       // 그리기색 : YELLOW
    LCD_DrawFillRect(24+(10*Old_Blue_X)-4,24+(10*Old_Blue_Y)-4,8,8);  
  }
  
  // 격자 선을 생성
  for(int i=0; i<10; i++){
     LCD_DrawHorLine(24, 24 + (10*i), 90);
     LCD_DrawVerLine(24 + (10*i), 24, 90);
  }//end for loop
  
  // <5,5>지점 
  LCD_SetBrushColor(RGB_BLACK);
  LCD_DrawFillRect(24+(10*5)-2, 24+(10*5)-2, 5, 5);
  
  // 착돌오목을 표시하는 부분
  for(int i=0; i<10; i++){   // 2D 배열의 행
    for(int j=0; j<10; j++){ // 2D 배열의 열
      if(cell[i][j]==1){     // 2D 배열에 빨강오목이 있으면,
        // 빨강오목 표시
        LCD_SetBrushColor(RGB_RED);       // 그리기색 : RED
        LCD_DrawFillRect(24+(10*i)-4,24+(10*j)-4,8,8);
      }
      
      if(cell[i][j]==2){     // 2D 배열에 파랑오목이 있으면,
        // 파랑오목 표시
        LCD_SetBrushColor(RGB_BLUE);       // 그리기색 : BLUE
        LCD_DrawFillRect(24+(10*i)-4,24+(10*j)-4,8,8);
      }
    }
  }
  
}// end void Return_Grid(void) function

/* 현재 오목이 위치한 좌표를 LCD에 표시하는 함수 */
void Display_Grid_Point(void)
{
  /* 9x9 영역에 있는지 판단후 못넘어가게 하는 조건문 */
  if((Red_X > 9) || (Red_Y > 9) || (Blue_X > 9) || (Blue_Y > 9)){   // 영역 밖으로 넘어가면,
    // 이전의 좌표를 현재 좌표로 대입한다
    Red_X = Old_Red_X;
    Red_Y = Old_Red_Y;
    Blue_X = Old_Blue_X;
    Blue_Y = Old_Blue_Y;
  }
  if((Red_X < 0) || (Red_Y < 0) || (Blue_X < 0) || (Blue_Y < 0)){   // 영역 밖으로 넘어가면,
    // 이전의 좌표를 현재 좌표로 대입한다
    Red_X = Old_Red_X;
    Red_Y = Old_Red_Y;
    Blue_X = Old_Blue_X;
    Blue_Y = Old_Blue_Y;
  } // 9x9 영역에 있는지 판단 조건문 종료
  
  if(RED_Flag)
  {
    //RED 좌표 표시
    LCD_DisplayChar(9,1,'(');
    LCD_DisplayChar(9,2,Red_X + 0x30);
    LCD_DisplayChar(9,3,',');
    LCD_DisplayChar(9,4,Red_Y + 0x30);
    LCD_DisplayChar(9,5,')');
    
    //현재 돌 위치를 좌표상에 그리기
    LCD_SetBrushColor(RGB_RED);       // 그리기색 : RED
    LCD_DrawFillRect(24+(10*Red_X)-4,24+(10*Red_Y)-4,8,8);
  }
  if(BLUE_Flag)
  {
    LCD_DisplayChar(9,13,'(');
    LCD_DisplayChar(9,14,Blue_X + 0x30);
    LCD_DisplayChar(9,15,',');
    LCD_DisplayChar(9,16,Blue_Y + 0x30);
    LCD_DisplayChar(9,17,')');
    
    //현재 돌 위치를 좌표상에 그리기
    LCD_SetBrushColor(RGB_BLUE);       // 그리기색 : BLUE
    LCD_DrawFillRect(24+(10*Blue_X)-4,24+(10*Blue_Y)-4,8,8);
  }
}

/* 오목 존재여부 확인 함수 */ 
void Check_OMOK(void){
  if((cell[Red_X][Red_Y]!= 0) && (cell[Blue_X][Blue_Y]!= 0)){   // 2차원 배열에 오목이 존재하면
    check_Flag = 1;           // 체크플래그 1 : 이미 오목이 존재한다는 의미
  }
}

/* 오목 승리판정 알고리즘 */
void Game_end(void){
  // Fram에서 점수 가져오기
  RED_score = Fram_Read(300);
  BLUE_score = Fram_Read(301);
  
// 빨강 승리 체크
  if(red_end == 1){
   for(int i=0; i < 10; i++) { 
    for(int j=0; j < 10; j++) {
       // 가로(→) 방향 체크,
       if(cell[i][j]==1 && cell[i][j+1]==1 && 
          cell[i][j+2]==1 && cell[i][j+3]==1 && 
          cell[i][j+4]==1){
          RED_score++;       // 조건을 만족하면 +1
          win_Flag = 1;      // 승리 Flag = 1
       }
       // 세로(↓) 방향 체크,
       else if(cell[i][j]==1 && cell[i+1][j]==1 && 
          cell[i+2][j]==1 && cell[i+3][j]==1 && 
          cell[i+4][j]==1){
          RED_score++;      // 조건을 만족하면 +1
          win_Flag = 1;     // 승리 Flag = 1
       }
       // 대각선(↘) 방향 체크,
       else if(cell[i][j]==1 && cell[i+1][j+1]==1 && 
          cell[i+2][j+2]==1 && cell[i+3][j+3]==1 && 
          cell[i+4][j+4]==1){
          RED_score++;     // 조건을 만족하면 +1
          win_Flag = 1;    // 승리 Flag = 1
       }
       // 대각선(↙) 방향 체크,
       else if(cell[i][j]==1 && cell[i-1][j+1]==1 && 
          cell[i-2][j+2]==1 && cell[i-3][j+3]==1 && 
          cell[i-4][j+4]==1){
          RED_score++;     // 조건을 만족하면 +1
          win_Flag = 1;    // 승리 Flag = 1
       }
      } 
    }
  }

// 파랑 승리체크
  if(blue_end == 1){
   for(int i=0; i < 10; i++) { 
    for(int j=0; j < 10; j++) {
       // 가로(→) 방향 체크,
       if(cell[i][j]==2 && cell[i][j+1]==2 && 
          cell[i][j+2]==2 && cell[i][j+3]==2 && 
          cell[i][j+4]==2){
          BLUE_score++;     // 조건을 만족하면 +1
          win_Flag = 1;     // 승리 Flag = 1
       }
       // 세로(↓) 방향 체크,
       else if(cell[i][j]==2 && cell[i+1][j]==2 && 
          cell[i+2][j]==2 && cell[i+3][j]==2 && 
          cell[i+4][j]==2){
          BLUE_score++;     // 조건을 만족하면 +1
          win_Flag = 1;     // 승리 Flag = 1
       }
       // 대각선(↘) 방향 체크,
       else if(cell[i][j]==2 && cell[i+1][j+1]==2 && 
          cell[i+2][j+2]==2 && cell[i+3][j+3]==2 && 
          cell[i+4][j+4]==2){
          BLUE_score++;     // 조건을 만족하면 +1
          win_Flag = 1;     // 승리 Flag = 1
       }
       // 대각선(↙) 방향 체크,
       else if(cell[i][j]==2 && cell[i-1][j+1]==2 && 
          cell[i-2][j+2]==2 && cell[i-3][j+3]==2 && 
          cell[i-4][j+4]==2){
          BLUE_score++;     // 조건을 만족하면 +1
          win_Flag = 1;     // 승리 Flag = 1
       }
      } 
    }
  }
  
  // Fram에 점수 저장
  if(RED_score <= 9){           // 점수가 9보다 작으면
    Fram_Write(300,RED_score);  // 그냥저장
  }
  else{                         // RED_score > 9
    RED_score = 0;              // RED_score를 0으로 하고
    Fram_Write(300,RED_score);  // Fram에 쓰기
  }
  
  if(BLUE_score <= 9){          // 점수가 9보다 작으면
    Fram_Write(301,BLUE_score); // 그냥저장
  }
  else{                         // BLUE_score > 9
    BLUE_score = 0;             // BLUE_score를 0으로 하고
    Fram_Write(301,BLUE_score); // Fram에 쓰기
  }
} // function end

/* Clear Function */
void Clear(void){
  // 결과 표시
          
  // 승리후 오목판 초기화
  if(win_Flag == 1){        // 승리로 판정되면
    GPIOG->ODR &= ~0x00FF;  // 차례를 표시하는 LED 전부 OFF
    
    for(int i=0; i<5; i++){ // 승리 알림
      BEEP();
      DelayMS(500);
    }
    DelayMS(5000);          // 5초 지연
    
    // 2D배열 전부 초기화
    for(int i=0; i<10; i++){
      for(int j=0; j<10; j++){
        cell[i][j] = 0;
      }
    }
    
    // 좌표 초기화
    Red_X = 5;      // 빨간돌의 X좌표
    Red_Y =5;       // 빨간돌의 Y좌표
    Blue_X =5;      // 파란돌의 X좌표
    Blue_Y = 5;     // 파란돌의 Y좌표
    
    // 모든 FLAG 변수 초기화
    check_Flag = 0;
    red_end = 0;
    blue_end = 0;
    win_Flag = 0;
    
    DisplayInitScreen(); // 화면 초기화
  }
}

/* EXTI (EXTI8(GPIOH.8, SW0), EXTI15(GPIOH.15, SW7), EXTI5(GPIOI.5, NAVI_PUSH)) 초기 설정 */
void _EXTI_Init(void)
{
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
        RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
	
	GPIOI->MODER 	&= ~0x000FFC00;	// GPIOI 5~9 : Input mode (reset state)
        GPIOH->MODER    &= ~0xFFFF0000; // GPIOH 8~15 : Input mode (reset state)
        
        SYSCFG->EXTICR[1] |= 0x0080;	// EXTI5에 대한 소스 입력은 GPIOI로 설정
					// EXTI5 <- PI5(NAVI_PUSH)
					// EXTICR2(EXTICR[1])를 이용 
					// reset value: 0x0000	
        
	SYSCFG->EXTICR[2] |= 0x0007;	// EXTI8에 대한 소스 입력은 GPIOH로 설정
					// EXTI8 <- PH8(SW0)
					// EXTICR3(EXTICR[2])를 이용 
					// reset value: 0x0000	

        SYSCFG->EXTICR[3] |= 0x7000;	// EXTI15에 대한 소스 입력은 GPIOH로 설정
					// EXTI15 <- PH15(SW7)
					// EXTICR4(EXTICR[3])를 이용 
					// reset value: 0x0000	

	EXTI->RTSR |= 0x009120;		// EXTI8, 15: Rising Trigger  Enable
	EXTI->IMR  |= 0x009120;		// EXTI8, 15 인터럽트 mask (Interrupt Enable) 설정
		
	NVIC->ISER[0] |= (1 << 23);	// 0x00800000
					// Enable 'Global Interrupt EXTI8'
					// Vector table Position 참조
        
        NVIC->ISER[1] |= (1 << (40-32));   // 0x00000100
                                           // Enable 'Global Interrupt EXTI15'
                                           // Vector table Position 참조
}

/* EXTI5~9 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void)		
{
  if(EXTI->PR & 0x0100){                // EXTI.8 Interrupt Pending(발생) 여부?

    EXTI->PR |= 0x0100; 		// Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    if((BLUE_Flag == 1)||(red_end == 1)){   // 만약 BLUE차례 이거나 빨강오목 차례가 끝났으면,
      RED_Flag = 0;                     // RED는 오목을 못두게 한다
    }
    else{
      RED_Flag = 1;                     // RED 차례
      blue_end = 0;                     // 파랑오목 종료차례 초기화
      LCD_SetTextColor(RGB_YELLOW);	// 글자색 : YELLOW
      LCD_DisplayText(9,12,"*" );	// Grid mark
      
      LCD_SetTextColor(RGB_RED);	// 글자색 : RED
      LCD_DisplayText(9,0,"*" );	// Grid mark
      GPIOG->ODR &= ~0x0080;            // 파랑 LED OFF
      GPIOG->ODR |= 0x0001;             // 빨강 LED ON
    }
  }
  
  if(EXTI->PR & 0x0020){                // EXTI.5 Interrupt Pending(발생) 여부?
    
    EXTI->PR |= 0x0020;                 //Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    
    if((RED_Flag || BLUE_Flag) == 1){   //둘중 한사람이 오목을 둘차례면,
      if(RED_Flag == 1){                //빨강이 둘차례면,
        // 오목 체크함수 실행
        Check_OMOK();
        
        if(check_Flag == 1){            // 이미 오목이 존재하면,
          // 경고음 3번
          for(int k=0; k<3; k++){
            BEEP();
            DelayMS(50);
          } // BEEP()
        }// check_Flag == 1
        
        else{                           // 오목이 존재 하지 않으면,
          BEEP();
          cell[Red_X][Red_Y] = 1;       //2차원 배열에 1을 저장하여 오목위치를 기억
          RED_Flag = 0;                 // 빨강 오목 차례 초기화
          red_end = 1;                  // 빨강오목 차례종료
          Game_end();                   // 승리체크
          Clear();                      // 초기화 (승리 했을시에만 실행된다)
        } // check_Flag == 0
        check_Flag = 0;                // check_Flag 초기화
      } //빨강 차례면 if
      
      else{                            // 파랑이 둘차례이면,
        // 오목 체크함수 실행
        Check_OMOK();
        
        if(check_Flag == 1){           // 이미 오목이 존재하면,
          // 경고음 3번
          for(int k=0; k<3; k++){
            BEEP();
            DelayMS(50);
          } // BEEP()
        } // check_Flag == 1
        
        else{                          // 오목이 존재 하지 않으면,
          BEEP();
          cell[Blue_X][Blue_Y] = 2;    // 2차원 배열에 2을 저장하여 오목위치를 기억
          BLUE_Flag = 0;               // 파랑오목 차례 초기화
          blue_end = 1;                // 파랑오목 차례종료
          Game_end();                  // 승리 체크
          Clear();                      // 초기화 (승리 했을시에만 실행된다)
        } // check_Flag == 0
        check_Flag = 0;                // check_Flag 초기화
      } // 파랑이 둘차례면 else
      
    } //둘중 한사람이 오목을 둘차례면 if 
    
  } // EXTI.5 Interrupt Pending?
  
} // void EXTI9_5_IRQHandler(void)

/* EXTI10~15 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void){
  if(EXTI->PR & 0x8000){                // EXTI.15 Interrupt Pending(발생) 여부?
    
    EXTI->PR |= 0x8000;                 //Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
    
    if((RED_Flag == 1)||(blue_end==1)){ // 만약 RED차례 이거나 파랑오목 차례가 끝났으면
      BLUE_Flag = 0;                    // BLUE는 오목을 못두게 한다
    }
    else{
      BLUE_Flag = 1;                    // 파랑오목 차례
      red_end = 0;                      // 빨강오목 종료차례 초기화
      LCD_SetTextColor(RGB_YELLOW);	// 글자색 : YELLOW
      LCD_DisplayText(9,0,"*" );	// Grid mark
      
      LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
      LCD_DisplayText(9,12,"*" );	// Grid mark
      GPIOG->ODR &= ~0x0001;            // 빨강 LED OFF
      GPIOG->ODR |= 0x0080;             // 파랑 LED ON
    }
  }
}	

/* Joystick switch가 입력되었는지를 여부와 어떤 Joystick switch가 입력되었는지의 정보를 return하는 함수  */ 
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

/* Switch가 입력되었는지를 여부와 어떤 switch가 입력되었는지의 정보를 return하는 함수  */ 
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