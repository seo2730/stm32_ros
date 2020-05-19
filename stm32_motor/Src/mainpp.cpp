/*
 * mainpp.cpp
 *
 *  Created on: May 14, 2020
 *      Author: seo27
 */

#define P_deg		150
#define D_deg		0

#define P_vel		150
#define I_vel		0

#define P_cur		0
#define I_cur		0

#define PI 			3.14159265
#define GEAR_RATIO 	21.0
#define CPT			500.0

#define DEG2RAD 	PI/180
#define RAD2DEG 	180/PI

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <stm32_ros_encoder/motor.h>
#include <std_msgs/String.h>

extern TIM_HandleTypeDef htim4;

ros::NodeHandle nh;

uint32_t nowTick = 0;
uint32_t pastTick[2] = {0,};

//// Encoder ////
std_msgs::UInt32 encoder_msg;
ros::Publisher encoder_pub("encoder", &encoder_msg);
uint32_t encoder=0;

//// Motor data ////
//stm32_ros_encoder::motor motor_msg;
std_msgs::Float32 deg_msg;
std_msgs::Float32 vel_msg;
//std_msgs::Float32 cur_msg;

ros::Publisher deg_pub("degree", &deg_msg);
ros::Publisher vel_pub("velocity", &vel_msg);
//ros::Publisher cur_pub("current", &cur_msg);

float T_deg=0, T_vel=0, T_cur=0;

float cur_cur=0, past_cur=0, error_cur=0, error_cur_sum=0;
float cur_vel=0, past_vel=0, error_vel=0, error_vel_sum=0;
float cur_deg=0, past_deg=0, error_deg=0, past_error_deg=0;

float position=0, velocity=0, current=0;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void current_sensor();

void postion_control(float target);

void speed_control(float target, float limit);

void current_control(float target, float limit);

void setup(void)
{
 nh.initNode();
 nh.advertise(encoder_pub);
 //nh.advertise(motor_pub);
 nh.advertise(deg_pub);
 nh.advertise(vel_pub);
 //nh.advertise(cur_pub);
}

void loop(void)
{
	nowTick = HAL_GetTick();
	encoder = TIM3->CNT;
	cur_deg = encoder * 2.0 * PI /(GEAR_RATIO*CPT*2.0);
	cur_vel = (cur_deg - past_deg) * 10;
	//current_sensor();

	if(nowTick - pastTick[0] > 10)
	{
		//cur_vel = (cur_vel - past_deg) ;
		//cur_cur = 10.0*(GetADC(0)*5.0/1024.0-2.5);

		speed_control(position, 45*DEG2RAD);

		vel_msg.data = cur_vel;
		vel_pub.publish(&vel_msg);

		pastTick[0] = nowTick;
	}

	if(nowTick - pastTick[1] > 100)
	{
		postion_control(360 * DEG2RAD);

		encoder_msg.data = encoder;
		encoder_pub.publish(&encoder_msg);

		deg_msg.data = cur_deg;
		deg_pub.publish(&deg_msg);


		pastTick[1] = nowTick;
		past_deg = cur_deg;
	}


	htim4.Instance->CCR1 = velocity;
	nh.spinOnce();

}

void current_sensor()
{
	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	//cur_cur = 10.0*(HAL_ADC_GetValue(&hadc1)*5.0/1024.0 - 2.5);
	//HAL_ADC_Stop(&hadc1);
	cur_cur=10;
}

/*
void SetDutyCW(double v)
{
	int ocr = v * (500.0 / 12.0) + 500;
}
*/

void postion_control(float target)
{
	error_deg = target - cur_deg;

	position = P_deg * error_deg + D_deg * (error_deg - past_error_deg) * 100;

	past_error_deg = error_deg;
}

void speed_control(float target, float limit)
{
	if(target > limit)
		target = limit;
	else if(target < -limit)
		target = -limit;

	error_vel = target - cur_vel;
	error_vel_sum += error_vel;

	if(error_vel_sum>600.0)
		error_vel_sum=600.0;
	else if(error_vel_sum<-600.0)
		error_vel_sum = -600.0;

	velocity = P_vel * error_vel + I_vel * error_vel_sum * 0.01;

	if(velocity > 600.0)
		velocity = 600.0;
	else if(velocity < -600)
		velocity = -600.0;


}

void current_control(float target, float limit)
{
	if(target > limit)
		target = limit;
	else if(target < -limit)
		target = -limit;

	error_cur = target - cur_cur;
	error_cur_sum += error_cur;

	if(error_cur_sum>12.0)
		error_cur_sum=12.0;
	else if(error_cur_sum<-12.0)
		error_cur_sum = -12.0;

	current = P_cur * error_cur + I_cur * error_cur_sum * 0.01;
}
