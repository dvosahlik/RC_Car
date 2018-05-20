#include <math.h>
#include "Types.h"


#define  ROZVOR 0.5
#define ROZCHOD 0.2


double get_angle(int pwm)
{
	return (pwm - 1500)/11.1111;
}


double get_inner_radius(double angle)
{
	return 1/tan(angle/180*M_PI)*ROZVOR - ROZCHOD/2;
}

static int allocated = 0;
static torque_vector* vektor;



torque_vector* get_torque_vector(int servo_pwm, int engine_pwm)
{
	if (allocated == 0)
	{
		vektor = (torque_vector*)malloc(sizeof(torque_vector));
		allocated = 1;
	}
	vektor->left_wheel = engine_pwm;
	vektor->right_wheel = engine_pwm;
	if (engine_pwm > 1500)
	{
		double angle = get_angle(servo_pwm);
		if (angle != 0)
		{
			double radius = get_inner_radius(angle);
			double outer_speed = engine_pwm;
			double inner_speed = 1500 + abs(outer_speed - 1500)*abs(radius)/(abs(radius) + ROZCHOD);
			if (angle > 0)
			{
				vektor->right_wheel = inner_speed;
				vektor->left_wheel = outer_speed;
			}
			else
			{
			
				vektor->left_wheel = inner_speed;
				vektor->right_wheel = outer_speed;
			}
		}
		else
		{
			
			
		}
	}
	return vektor;
}


