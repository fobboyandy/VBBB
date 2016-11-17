#include <stdio.h>
#include <rt_misc.h>
#include "LPC11xx.h"

#define PERIOD 				9889
#define MAX_ANGLE			12
#define CENTER_ANGLE	7.5
#define MIN_ANGLE			3
#define INIT_WORD 		0xF0

#define SERVO_N				0xF0
#define SERVO_W				0x0F

typedef enum
{
	UP = 0x8,
	DOWN = 0x4,	
	LEFT = 0x2,
	RIGHT = 0x1	
}dir_t;

extern void SER_init (void);

void configGPIO()
{
	LPC_SYSCON->SYSAHBCLKCTRL |= (1UL <<  6);		// Enables GPIO
	LPC_SYSCON->SYSAHBCLKCTRL |= (1UL <<  16);	// Enables IOCON
	
	//set port 0_7 to output (high current drain in LPC1114)
	LPC_GPIO0->DIR |= (1<<7);
}

void ledOn()
{
	LPC_GPIO0->DATA &= ~(1<<7);
}

void ledOff()
{						 
	LPC_GPIO0->DATA |= (1<<7);
}



void configPWM()
{
	// Configuration for the first PWM signal
	LPC_IOCON->PIO0_8 = (LPC_IOCON->PIO0_8 & ~(0x3FF)) | 0x2;    //set up pin for PWM use (sec 7.4.23)
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);                         //enable clock signal to 16 bit timer0 (sec 3.5.14)
	LPC_TMR16B0->PR = 0x60;																			 //set prescaler max value (sec 18.7.4)
	LPC_TMR16B0->MCR = 0x10;                                     //set for reset on counter match (sec 18.7.6)
	LPC_TMR16B0->EMR |= 0x20;                                    //set pin 27 to 1 on match (sec 18.7.10)
	LPC_TMR16B0->CCR = 0;                                        //set to timer mode (sec 18.7.11)
	LPC_TMR16B0->PWMC = 0x1;                                     //set channel zero to PWM mode (sec 18.7.12)
	LPC_TMR16B0->MR1 = PERIOD;                                   //set value for period (sec 18.7.7)
	LPC_TMR16B0->MR0 = PERIOD/10;                                //set value for duty cycle (sec 18.7.7)
	LPC_TMR16B0->TCR |= 0x3;                                     //enable and reset counter (sec 18.7.2)
	LPC_TMR16B0->TCR &= ~(0x2);                                  //clear reset bit (sec 18.7.2)
	
	// Configuration for the second PWM signal
	LPC_IOCON->PIO1_9 = (LPC_IOCON->PIO1_9 & ~(0x3FF)) | 0x1;		 //set up pin for PWM use (sec 7.4.23)
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8);                         //enable clock signal to 16 bit timer1 (sec 3.5.14)
	LPC_TMR16B1->PR = 0x60;																			 //set prescaler max value (sec 18.7.4)
	LPC_TMR16B1->MCR = 0x10;                                     //set for reset on counter match (sec 18.7.6)
	LPC_TMR16B1->EMR |= 0x20;                                    //set pin 27 to 1 on match (sec 18.7.10)
	LPC_TMR16B1->CCR = 0;                                        //set to timer mode (sec 18.7.11)
	LPC_TMR16B1->PWMC = 0x1;                                     //set channel zero to PWM mode (sec 18.7.12)
	LPC_TMR16B1->MR1 = PERIOD;                                   //set value for period (sec 18.7.7)
	LPC_TMR16B1->MR0 = PERIOD/10;                                //set value for duty cycle (sec 18.7.7)
	LPC_TMR16B1->TCR |= 0x3;                                     //enable and reset counter (sec 18.7.2)
	LPC_TMR16B1->TCR &= ~(0x2);                                  //clear reset bit (sec 18.7.2)
	
}

void setPeriod(uint16_t period)
{
	LPC_TMR16B0->MR1 = period; 
}

// Controls north servo
void setDuty1(uint16_t duty)
{
	LPC_TMR16B0->MR0 = duty;
}

// Controls west servo
void setDuty2(uint16_t duty)
{
	LPC_TMR16B1->MR0 = duty;
}

//sets the duty cycle on the north servo to roll ball up
//input : upper half determines direction and lower half determines speed
void servo_turn(uint8_t dir, uint8_t speed)
{
	//static global variables to keep track servos' angles
	static float n_servo_angle = CENTER_ANGLE;
	static float w_servo_angle = CENTER_ANGLE;

	
	void (*servoFunction)(uint16_t);
	float angle;
	//MCU uses units of 0.1
	float inc = (float)speed/100;
	switch(dir)
	{
		case(UP):
		if((n_servo_angle += inc) > MAX_ANGLE)
				n_servo_angle = MAX_ANGLE;
			angle = n_servo_angle;
			servoFunction = setDuty1;
			break;
		
		case(DOWN):
		if((n_servo_angle -= inc) < MIN_ANGLE)
				n_servo_angle = MIN_ANGLE;
			angle = n_servo_angle;
			servoFunction = setDuty1;
			break;
		
		case(RIGHT):
		if((w_servo_angle += inc) > MAX_ANGLE)
				w_servo_angle = MAX_ANGLE;
			angle = w_servo_angle;
			servoFunction = setDuty2;
			break;
		
		case(LEFT):
		if((w_servo_angle -= inc) < MIN_ANGLE)
				w_servo_angle = MIN_ANGLE;
			angle = w_servo_angle;
			servoFunction = setDuty2;
			break;
		
		default:
			n_servo_angle = CENTER_ANGLE;
			w_servo_angle = CENTER_ANGLE;
			setDuty1((uint16_t)(PERIOD * (1 - (n_servo_angle/100))));
			setDuty2((uint16_t)(PERIOD * (1 - (w_servo_angle/100))));
			return;
	}
	//output to the select servo
	(*servoFunction)((uint16_t)(PERIOD * (1 - (angle/100))));
}


void move_board(uint8_t servo, int8_t angle)
{
	float duty = (PERIOD * (1 - ((float)(( angle * 0.05) + 7.5)/100)));
	if(servo & SERVO_W)
		setDuty1((uint16_t)(duty));
	else
		setDuty2((uint16_t)(duty));
}


int main()
{
	//configure IO ports
	configGPIO();	
	configPWM();
	SER_init();
	//resets the servos' positions
	setDuty1((uint16_t)(PERIOD * (1 - ((float)CENTER_ANGLE/100))));
	setDuty2((uint16_t)(PERIOD * (1 - ((float)CENTER_ANGLE/100))));
	
	while(1)
	{
		static uint8_t rec[4];
		scanf("%s", rec);		// Scanf is a blocking statement
		if(rec[0] == INIT_WORD)
			move_board(rec[1], rec[2]);
	}
}











