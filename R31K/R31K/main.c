/*
 * Roboter R13K
 *
 * Created: 20.10.2017
 *  Author: Philipp Leeb
 */

#define F_CPU 8000000UL

/*volatile unsigned int status = 0;

#define STATUS_READY		0
#define STATUS_BUSY			1
#define STATUS_END			2
//#define STATUS_PROGRAMM		3
#define STATUS_MOT_R_READY	4
#define STATUS_MOT_L_READY	5
#define STATUS_PWM_TIMER	6

volatile unsigned int impulstime_right=0, impulstime_left=0;
volatile unsigned char check_impuls_right=0, check_impuls_left=0,impuls_right_set=0, impuls_left_set=0, motor_right_count=0,motor_left_count=0;
volatile unsigned char pwm_left=0, pwm_right=0;
//volatile unsigned char check_lock_left=0, check_lock_right=0;*/

#include <avr/io.h>
#include <util/delay.h>
#include "ELRobot_New.h"

int main(void)
{
	set_fuses();
	init();

	ms=0;
    while(ms<1000);
    ms=0;

    while(1)
    {
        if(ms>1000&&ms<1500)
        {
            pwm_timer_stop();
            pwm_timer_start();
            drive(-2,2);
        }
        else if(ms<1000)
        {
            pwm_timer_stop();
            pwm_timer_start();
            drive(2,2);
        }
        else ms=0;

        low_voltage();
    }

    return 0;
}
