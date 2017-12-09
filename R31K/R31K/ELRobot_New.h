//***********************************************************************************
// elRobot.h W.Kuran
// jan. 2014
//***********************************************************************************

#ifndef __OPTIMIZE__
# warning " Compiler optimizations disabled; see PROJECT > BUILD OPTIONS .... use -Os"
#endif

#include <avr/interrupt.h>

#define SPEEDY_VERSION

#define U6_8 197
#define U6_4 186

//*************************************************************************************************
//*** MOTOR ***
//*************************************************************************************************

// MotorPins

#define MOT_L_V			5			// PF5
#define MOT_L_B			6			// PF6
#define MOT_R_V			7			// PF7
#define MOT_R_B			6			// PC6

#define MOTOR_R			6			// PB6
#define MOTOR_L			7			// PD7

// Motor Funktion

#ifdef  SPEEDY_VERSION
#define MOT_FORWARD		1
#define MOT_BACKWARD	0
#elif
#define MOT_FORWARD		0
#define MOT_BACKWARD	1
#endif
#define MOT_FAST_STOPP	2
#define PWM_STOPP		0

//*************************************************************************************************
//*** LED *****
//*************************************************************************************************

// LED Pins
#define LED_RV			0			// PB0
#define LED_RH			1			// PB1
#define LED_LV			2			// PB2
#define LED_LH			3			// PB3
// Duo LED
#define LED_RED			2			// PD2
#define LED_GREEN		3			// PD3


//*************************************************************************************************
//*** INFRAROT ***
//*************************************************************************************************

#define IR_SENDER		6			// PD6
#define IR_EMPF_R		1			// ADC1
#define IR_EMPF_L		0			// ADC0

//*************************************************************************************************
//*** IMPULSGEBER ***
//*************************************************************************************************

#define WHEEL_RIGHT		4			// PD4
#define WHEEL_LEFT		7			// PC7

//*************************************************************************************************
//*** BEEPER ***
//*************************************************************************************************

// Beeper Pins
#define BEEPER			7			// PB7
// Toene
#define NONE			0
#define BEEP			8

#define A1              71         //18182

//*************************************************************************************************
//*** TIMER 0 ***
//*************************************************************************************************

#define TICK_ONE_SEC      62

//*************************************************************************************************
//*** FUNKTIONSPROTOTYPEN ***
//*************************************************************************************************

void set_fuses(void);
void init(void);
void pwm_timer_init(void);
void pwm_timer_start(void);
void pwm_timer_stop(void);
void odometer_timer_init(void);
void odometer_links_start(void);
void odometer_rechts_start(void);
void odometer_links_stop(void);
void odometer_rechts_stop(void);
//#  void timer_beeper_init(void);
void timer_beep_tone(unsigned int freq);
//#  void beep(unsigned char);
//void drive_Robot(unsigned char, unsigned char, unsigned char, unsigned char, unsigned int, unsigned int);

void start(void);

void ledGreenOn(void);
void ledRedOn(void);
void ledGreenOff(void);
void ledRedOff(void);

void led(uint8_t x);
void drive(int8_t l, int8_t r);

void low_voltage(void);


volatile uint16_t ms, tick, tickbeep;
volatile uint16_t speedR[11], speedL[11];
volatile uint16_t curfreq, frequenz;


//*************************************************************************************************
//  Funktionen
//*************************************************************************************************

void set_fuses(void)
{
	MCUCR |= (1<<JTD);
	MCUCR |= (1<<JTD);
	CLKPR = 0b10000000;
	CLKPR = 1;
}

void init(void)
{
	DDRB |= (1 << BEEPER)  |
	        (1 << MOTOR_R) |
 			(1 << LED_LH)  |
			(1 << LED_LV)  |
			(1 << LED_RH)  |
			(1 << LED_RV);

	DDRC |= (1 << MOT_R_B);

	DDRF |= (1 << MOT_L_V) |
	        (1 << MOT_L_B) |
			(1 << MOT_R_V);

	DDRD |= (1 << MOTOR_L)   |
	        (1 << IR_SENDER) |
			(1 << LED_GREEN) |
			(1 << LED_RED);

	PORTB = 0;
	//       |= (1 << LED_LH) |
	//         (1 << LED_LV) |
    //			 (1 << LED_RH) |
	//		 (1 << LED_RV);

	PORTF = 0;

	PORTD = 0;


	speedR[10] = speedR[0] = 250;
	speedR[ 9] = speedR[1] = 230;
	speedR[ 8] = speedR[2] = 210;
	speedR[ 7] = speedR[3] = 190;
	speedR[ 6] = speedR[4] = 170;
    speedR[ 5] = 0;


	speedL[10] = speedL[0] = 250;
	speedL[ 9] = speedL[1] = 230;
	speedL[ 8] = speedL[2] = 210;
	speedL[ 7] = speedL[3] = 190;
	speedL[ 6] = speedL[4] = 170;
    speedL[ 5] = 0;


	pwm_timer_init();

	odometer_timer_init();
//#  	timer_beeper_init();


//  Timer 0:
    TIMSK0 = (1 << TOIE0);
	TCNT0 = 206;
	TCCR0B = (1 << CS00);

//  ADC:
    ADCSRA |= (1<<ADEN)|(1<<ADPS0)|(1<<ADPS2);
	ADMUX |= (1<<ADLAR)|4;


    ms = tick = tickbeep = curfreq = frequenz = 0;

	sei();
}

void ledGreenOn(void)
{
    PORTD |= (1 << LED_GREEN);
}

void ledGreenOff(void)
{
    PORTD &= ~(1 << LED_GREEN);
}

void ledRedOn(void)
{
    PORTD |= (1 << LED_RED);
}

void ledRedOff(void)
{
    PORTD &= ~(1 << LED_RED);
}

void led(uint8_t x)
{
    if ((x & 0x1) == 0x1) PORTB |= (1 << LED_LH); else PORTB &= ~(1 << LED_LH);
    if ((x & 0x2) == 0x2) PORTB |= (1 << LED_LV); else PORTB &= ~(1 << LED_LV);
    if ((x & 0x4) == 0x4) PORTB |= (1 << LED_RH); else PORTB &= ~(1 << LED_RH);
    if ((x & 0x8) == 0x8) PORTB |= (1 << LED_RV); else PORTB &= ~(1 << LED_RV);
}



void pwm_timer_init(void)
{
	// Timer 4:

	//MOTOR RECHTS

	TCCR4A |= (1 << COM4B1) |
	          (1 << PWM4B);
	// OC4B aktiviert bei Compare Match, PWM4B aktiviert

	//MOTOR LINKS

	TCCR4C |= (1 << COM4D1) |
	          (1 << PWM4D);

    // OC4D aktiviert bei Compare Match, PWM4D aktiviert
}

void odometer_timer_init(void)
{
	TCCR1A = 0;
	TCCR1B = (1 << ICNC1);					// Input Capture Mode aktiviert
	TCCR1C = 0;
	TCCR3A = 0;
	TCCR3B = (1 << ICNC3);					// Input Capture Mode aktiviert
	TCCR3C = 0;
}

//#  void timer_beeper_init(void)
//#  {
//#  	//Timer0
//#  	TCNT0 = 0;
//#  	OCR0A = 0;
//#
//#  	TCCR0A |= (1 << COM0A0) |
//#  	          (1 << WGM01)  |
//#  			  (1 << WGM00);
//#  	TCCR0B |= (1 << WGM02);
//#
//#  }


void timer_beep_tone(unsigned int freq)
  {
  	OCR0A = 255;
  	frequenz=freq;
  	//if(TCNT0==255)
  	//curfreq++;

  	//TCCR0B |= (1<<CS02);
  }



void start(void)
{
    pwm_timer_start();
    odometer_links_start();
	odometer_rechts_start();
}

void pwm_timer_start(void)
{
	// Timer 4
	//MOTOR RECHTS
	TCCR4A |= (1 << COM4B1)
	        | (1 << PWM4B);		// OC4B aktiviert bei Compare Match, PWM4B aktiviert
	//MOTOR LINKS
	TCCR4C |= (1 << COM4D1) |
	          (1 << PWM4D);		// OC4D aktiviert bei Compare Match, PWM4D aktiviert

	TCCR4B = 0;
	TCCR4D = 0;
	TCCR4E = 0;

	TCNT4 = 0;
	DT4 = 0;								// Dead Time Generator aus

	OCR4B = PWM_STOPP;						// PWM 0%
	OCR4D = PWM_STOPP;						// PWM 0%

	TCCR4B |= (1 << CS40);					// ca. 64kHz

}

void pwm_timer_stop(void)
{
	TCCR4B = 0;
	TCCR4A = 0;
	TCCR4C = 0;
//	status &= ~(1<<STATUS_PWM_TIMER);
}

void odometer_links_start(void)
{
//	motor_left_count = 0;
//	check_impuls_left = 0;
	TIMSK3 |= (1 << ICIE3);
	ICR3    = 0;
	TCNT3   = 0;
	TCCR3B |= (1 << CS32);
	TIFR3  |= (1 << ICF3);
}

void odometer_rechts_start(void)
{
//	motor_right_count = 0;
//	check_impuls_right = 0;
	TIMSK1 |= (1 << ICIE1);
	ICR1    = 0;
	TCNT1   = 0;
	TCCR1B |= (1 << CS12);
	TIFR1  |= (1 << ICF1);
}

void odometer_links_stop(void)
{
	TCCR3B &= ~((1<<CS32) | (1<<CS31) | (1<<CS30));
}

void odometer_rechtscurfreq_stop(void)
{
	TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
}


void drive(int8_t l, int8_t r)
{

    // left:

    if (l > 0)       // forward
	{
	    PORTD &= ~(1 << MOTOR_L); // Motor ein
        PORTF |=  (1 << MOT_L_V); PORTF &= ~(1 << MOT_L_B); //L->
    }
	else if (l < 0)  // backward
	{
	    PORTD &= ~(1 << MOTOR_L); // Motor ein
        PORTF &= ~(1 << MOT_L_V); PORTF |=  (1 << MOT_L_B); //L->

	}
	else             // stop
	{
		PORTD |=  (1 << MOTOR_L); // Motor aus
        PORTF |=  (1 << MOT_L_V); PORTF |=  (1 << MOT_L_B); //L->
	}

    // right:

	if (r > 0)       // forward
	{
	    PORTB &= ~(1 << MOTOR_R);
		PORTF |=  (1 << MOT_R_V); PORTC &= ~(1 << MOT_R_B); //R ->
    }
	else if (r < 0)  // backward
	{
	    PORTB &= ~(1 << MOTOR_R); // Motor ein
	    PORTF &= ~(1 << MOT_R_V); PORTC |=  (1 << MOT_R_B); //R<-

	}
	else             // stop
	{
	    PORTB |=  (1 << MOTOR_R); // Motor aus
	    PORTF |=  (1 << MOT_R_V); PORTC |=  (1 << MOT_R_B); //R ||
	}

	OCR4D = (unsigned int)speedL[l + 5]; // Left
	OCR4B = (unsigned int)speedR[r + 5]; // Right
}



unsigned char check_impulse(void)
{
		return 1;
}

void low_voltage(void)
{
	int adc_result;

	ADCSRA |= (1<<ADSC);
	while(ADCSRA&(1<<ADSC));

	adc_result = ADCH;

	if(adc_result<=U6_4)
	{
		PORTD &= ~(1<<LED_GREEN);
		PORTD |= (1<<LED_RED);
		while(1)
		{
		}
	}
	else if(adc_result>U6_4&&adc_result<=U6_8)
	{
		PORTD |= (1<<LED_RED);
		PORTD |= (1<<LED_GREEN);
	}
	else
	{
		PORTD |= (1<<LED_GREEN);
		PORTD &= ~(1<<LED_RED);
	}
}



// rechter Motorgeber

ISR(TIMER1_CAPT_vect)
{
	TIFR1 |= (1<<ICF1);

}

// linker Motorgeber

ISR(TIMER3_CAPT_vect)
{
	TIFR3 |= (1<<ICF3);

}


ISR(TIMER0_OVF_vect)
{
    TCNT0 = 206;

    tick++;
    tickbeep++;

    if(frequenz==0)
    {
       TCCR0A &= ~(1<<COM0A0);
    }
    else if(tickbeep > frequenz)
    {
       TCCR0A |= (1<<COM0A0);
    }
    else if(tickbeep > frequenz+1)
    {
        TCCR0A &= ~(1<<COM0A0);
        tickbeep=1;
    }

	if (tick > TICK_ONE_SEC)
	{
        tick = 0;
		ms++;
	}
}
