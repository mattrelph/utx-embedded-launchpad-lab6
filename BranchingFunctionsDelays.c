// BranchingFunctionsDelays.c Lab 6
// Runs on LM4F120/TM4C123
// Use simple programming structures in C to 
// toggle an LED while a button is pressed and 
// turn the LED on when the button is released.  
// This lab will use the hardware already built into the LaunchPad.
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// built-in connection: PF0 connected to negative logic momentary switch, SW2
// built-in connection: PF1 connected to red LED
// built-in connection: PF2 connected to blue LED
// built-in connection: PF3 connected to green LED
// built-in connection: PF4 connected to negative logic momentary switch, SW1

#include "TExaS.h"


#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control


#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

//   Function Prototypes
void PortF_Init(void);
void Delay(void);

void SysTick_Init(void);
//void SysTick_Wait(unsigned long delay);
//void SysTick_Wait10ms(unsigned long delay);

//   Global Variables
unsigned long SW1,SW2;  // input from PF4,PF0
unsigned long Out;      // outputs to PF3,PF2,PF1 (multicolor LED)
int ledOn = 1;


int main(void){ unsigned long volatile delay;
  TExaS_Init(SW_PIN_PF4, LED_PIN_PF2);  // activate grader and set system clock to 80 MHz
  // initialization goes here
//	SysTick_Init();		//Initialize system clock
  PortF_Init();        // Call initialization of port PF4, PF3, PF2, PF1, PF0  
  EnableInterrupts();           // enable interrupts for the grader
	
	GPIO_PORTF_DATA_R = 0x04;       // LED is blue
	ledOn = 1;
	
  while(1){
    // body goes here
		SW1 = GPIO_PORTF_DATA_R&0x10;     // read PF4 into SW1
		if (!SW1)
		{	
			if (ledOn)
			{
				GPIO_PORTF_DATA_R = 0x00;   // LED is off
				ledOn = 0;
			}
			else
			{
				GPIO_PORTF_DATA_R = 0x04;       // LED is blue
				ledOn = 1;
			}
			
	//		SysTick_Wait10ms(10); //Wait 100 ms to change state
		}
		else
		{
			GPIO_PORTF_DATA_R = 0x04;       // LED is blue
			ledOn = 1;
		}
  }
}

// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Inputs: None
// Outputs: None
// Notes: These five pins are connected to hardware on the LaunchPad
void PortF_Init(void)
{ 
		volatile unsigned long delay;
		SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
		delay = SYSCTL_RCGC2_R;           // delay   
		GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
		GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
		GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
		GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
		GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
		GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
		GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0       
		GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0        
}
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06





// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = 0x00FFFFFF;        // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it             
  NVIC_ST_CTRL_R = 0x00000005;          // enable SysTick with core clock
}

// Time delay using busy wait.
// The delay parameter is in units of the core clock. 
void SysTick_Wait(unsigned long delay){
  volatile unsigned long elapsedTime;
  unsigned long startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
}

// Time delay using busy wait.
// This assumes 16 MHz system clock.
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(1600000);  // wait 10ms (assumes 16 MHz clock)
  }
}
