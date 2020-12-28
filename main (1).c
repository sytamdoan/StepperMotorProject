// Documentation
// CECS 346 
// Description: 
// Student Name: Tam Doan


// Preprocessor Directives
#include <stdint.h>
#include "SysTick.h"
#include "stepper.h"


// PORT E register definitions
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
	
#define GPIO_PORTE_IS_R         (*((volatile unsigned long *)0x40024404))
#define GPIO_PORTE_IBE_R        (*((volatile unsigned long *)0x40024408))
#define GPIO_PORTE_IEV_R        (*((volatile unsigned long *)0x4002440C))
#define GPIO_PORTE_IM_R         (*((volatile unsigned long *)0x40024410))
#define GPIO_PORTE_RIS_R        (*((volatile unsigned long *)0x40024414))
#define GPIO_PORTE_ICR_R        (*((volatile unsigned long *)0x4002441C))

// PORT F register definitions
#define GPIO_PORTF_DATA_R 			(*((volatile unsigned long *)0x400250FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
	
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))
#define NVIC_PRI1_R             (*((volatile unsigned long *)0xE000E404))
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))
	
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))

#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))


// system control register RCGC2 definition
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

// define constants and alies
#define Light 									(*((volatile unsigned long *)0x40025038))
#define Sensor									(*((volatile unsigned long *)0x400243FC))


// Function Prototypes - Each subroutine defined
void Delay(uint8_t n_50ms);
void PortE_Init(void);
void PortF_Init(void);
extern void EnableInterrupts(void);
extern void WaitForInterrupt(void);

unsigned char pressed = 0;
unsigned char flag6 = 0;
unsigned char SensorDetect = 0;
unsigned char carInside = 0;


#define T1ms 16000    // assumes using 16 MHz PIOSC (default setting for clock source) f=16Mhz->Tmc=1/f=1/16MHz, t=1ms, t/Tmc->number of machines cycles to generate 1ms,

int main(void) {
	// Initialize GPIO Ports E, F
	
	unsigned int i=0;
	PortE_Init();
	PortF_Init();
  EnableInterrupts();
	Stepper_Init();
	Light = 0x08;


	while(1) {

		
		if((Light == 0x08) & (pressed == 2)){
			Light = 0x02;
			for (i=0;i<1000; i++) {
				Stepper_CW(10*T1ms);   // output every 10ms, frequency for the stepper motor is 100Hz. f=100Hz, t=1/f=1/100=0.01s
					if(i % 125 == 0) {
						Light ^= 0x02;
					}
			}
			Light = 0x04;
			pressed = 0;
		}
		
		if((Light == 0x04) & (pressed == 2)){ //If light is at blue and onboard button 2 is pressed
			Light = 0x02;
			for (i=0;i<1000; i++) {
				Stepper_CCW(10*T1ms);   // output every 10ms, frequency for the stepper motor is 100Hz.
					if(i % 125 == 0) {
						Light ^= 0x02;
					}
			}
			Light = 0x08;
			pressed = 0;
			carInside = 1;
		}
		
		if(SensorDetect == 2) {
			carInside = 0;
		}
		
		if(SensorDetect == 1){ 
			SysTick_Wait10ms(50);
			
			if (Light == 0x08 & SensorDetect == 1 & carInside == 0) { //If light is at green and Sensor Detects something
				Light = 0x02;
				for (i=0;i<1000; i++) {
					Stepper_CW(10*T1ms);   // output every 10ms, frequency for the stepper motor is 100Hz.
					if(i % 125 == 0) {
						Light ^= 0x02;
					}
				}
			Light = 0x04;
			carInside = 1;
			}
			
			
			if(Light == 0x04 & SensorDetect == 2){ //If light is at blue and Sensor No Longer Detects something					
				Light = 0x02;
				for (i=0;i<1000; i++) {
					Stepper_CCW(10*T1ms);   // output every 10ms, frequency for the stepper motor is 100Hz.
					if(i % 125 == 0) {
						Light ^= 0x02;
					}
				}
				Light = 0x08;
				carInside = 0;
			}
	}
}
}



// Subroutine to initialize port E pins PE0 for input
// Inputs: None
// Outputs: None
void PortE_Init(void){
	
	SYSCTL_RCGC2_R |= 0x00000010;
	while ((SYSCTL_RCGC2_R&0x00000010)!=0x00000010){}
		
  GPIO_PORTE_AMSEL_R &= ~0x01; 
  GPIO_PORTE_PCTL_R &= ~0x00000FFF; 
  GPIO_PORTE_DIR_R &= ~0x01;   
  GPIO_PORTE_AFSEL_R &= ~0x01; 
  GPIO_PORTE_DEN_R |= 0x01;    
		
	GPIO_PORTE_IS_R &= ~0x01;
	GPIO_PORTE_IBE_R |= 0x01;
	GPIO_PORTE_IEV_R |= 0x01;
	GPIO_PORTE_ICR_R |= 0x01;
	GPIO_PORTE_IM_R |= 0x01;
	NVIC_PRI1_R = (NVIC_PRI1_R&0xFFFFFF1F)|0x000000A0; // pri1, bit 7-5: 7654 3210->11100000->0x000000E0-> ~0x000000E0->0xFFFFFF1F
	NVIC_EN0_R |= 0x00000010;  
		
}

void GPIOPortF_Handler(void){
	// if () {}: poll to find out which pin is pressed in case that multiple pins can generate interrupts.
	if (GPIO_PORTF_RIS_R & 0x10) { // switch 4 is pressed: sw2, 3 leds, sw1: 10000
		GPIO_PORTF_ICR_R |= 0x10;      // acknowledge flag 0
		pressed = 2;
	}
	
}

void GPIOPortE_Handler(void){
	// if () {}: poll to find out which pin is pressed in case that multiple pins can generate interrupts.
	//if (GPIO_PORTE_RIS_R & 0x01) { // switch 2 is pressed: sw2, 3 leds, sw1: 10000
	//	GPIO_PORTE_ICR_R |= 0x01;      // acknowledge flag 0 
	//}
	if(Sensor == 0x00){
		GPIO_PORTE_ICR_R |= 0x01;
		SensorDetect = 1;
	}
	
	if(Sensor == 0x01){
		GPIO_PORTE_ICR_R |= 0x01;
		SensorDetect = 2;
	}
}


// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED

void PortF_Init(void){
	
  SYSCTL_RCGC2_R |= 0x00000020; 	
	while ((SYSCTL_RCGC2_R&0x00000020)!=0x00000020){}
		
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   	// unlock PortF PF0  
	GPIO_PORTF_CR_R |= 0x1F;         		// allow changes to PF4-0 :11111->0x1F     
  GPIO_PORTF_AMSEL_R &= ~0x1F;        // disable analog function
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; 	// GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R &= ~0x11;          // PF4,PF0 input   
  GPIO_PORTF_DIR_R |= 0x0E;          	// PF3,PF2,PF1 output   
	GPIO_PORTF_AFSEL_R &= ~0x1F;        // no alternate function
  GPIO_PORTF_PUR_R |= 0x11;          	// enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R |= 0x1F;          	// enable digital pins PF4-PF0
		
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,0 falling edge event
  GPIO_PORTF_ICR_R |= 0x11;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC   
}


