/*
 * mainpp.cpp
 *
 *  Created on: May 20, 2020
 *      Author: seodeoghyeon
 */


#define P_deg		80
#define D_deg		0.2

#define P_vel		150
#define I_vel		0

#define P_cur		1
#define I_cur		0

#define PI 			3.14159265
#define GEAR_RATIO 	21.0
#define CPT			500.0

#define DEG2RAD 	PI/180
#define RAD2DEG 	180/PI

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "mainpp.h"
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <stm32_ros_encoder/motor.h>
#include <std_msgs/String.h>

extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;

ros::NodeHandle nh;

uint32_t nowTick = 0;
uint32_t pastTick[3] = {0,};

//// Encoder ////
std_msgs::UInt32 encoder_msg;
ros::Publisher encoder_pub("encoder", &encoder_msg);
int32_t encoder=0;

//// Motor data ////
stm32_ros_encoder::motor motor_msg;
std_msgs::Float32 deg_msg;
std_msgs::Float32 vel_msg;
std_msgs::Float32 cur_msg;

ros::Publisher deg_pub("degree", &deg_msg);
ros::Publisher vel_pub("velocity", &vel_msg);
ros::Publisher cur_pub("current", &cur_msg);

float T_deg=0, T_vel=0, T_cur=0;

float cur_cur=0, past_cur=0, error_cur=0, error_cur_sum=0, lpf_cur=0 ,value=0;
float cur_vel=0, past_vel=0, error_vel=0, error_vel_sum=0;
float cur_deg=0, past_deg=0, error_deg=0, past_error_deg=0;

float position=0, velocity=0, current=0;

float target_deg=0;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void current_sensor();

void postion_control(float target);

void speed_control(float target, float limit);

//void current_control(float target, float limit);
void current_control(float target);

void setup(void)
{
 nh.initNode();
 nh.advertise(encoder_pub);
 //nh.advertise(motor_pub);
 nh.advertise(deg_pub);
 nh.advertise(vel_pub);
 nh.advertise(cur_pub);
}

void loop(void)
{
	nowTick = HAL_GetTick();
	target_deg = 180 * DEG2RAD;
	encoder = TIM3->CNT;
	cur_deg = encoder * 2.0 * PI /(GEAR_RATIO*CPT*2.0);
	cur_vel = (cur_deg - past_deg) * 4000;
	//current_sensor();

	error_deg = target_deg - cur_deg;
	if(error_deg>=0)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // RESET: CCW, SET: CW

	else if(error_deg<0)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	if(nowTick - pastTick[0] > 5)
	{
		current_sensor();
		current_control(0.1);//current_control(velocity,5);

		cur_msg.data = value;
		cur_pub.publish(&cur_msg);


		pastTick[0] = nowTick;
	}

	if(nowTick - pastTick[1] > 50)
	{
		speed_control(position,180*DEG2RAD);//speed_control(position, 270*DEG2RAD);

		vel_msg.data = cur_vel;
		vel_pub.publish(&vel_msg);

		pastTick[1] = nowTick;
	}

	if(nowTick - pastTick[2]>500)
	{
		postion_control(target_deg);

		encoder_msg.data = encoder;
		encoder_pub.publish(&encoder_msg);

		deg_msg.data = cur_deg;
		deg_pub.publish(&deg_msg);


		pastTick[2] = nowTick;
	}

	htim4.Instance->CCR1 = 900;
	past_deg = cur_deg;
	past_error_deg = error_deg;

	nh.spinOnce();

}

void current_sensor()
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	cur_cur = HAL_ADC_GetValue(&hadc1);//20*(HAL_ADC_GetValue(&hadc1)*5.0/1024.0 - 2.5); //HAL_ADC_GetValue(&hadc1);

	lpf_cur= 0.7*past_cur + 0.3*cur_cur;

	past_cur=lpf_cur;

	value = 5*(lpf_cur*5.0/1540.0 - 2.5);

}


void postion_control(float target)
{
	position = P_deg * error_deg + D_deg * (error_deg - past_error_deg) * 100;
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

	/*
	if(velocity > 600.0)
		velocity = 600.0;
	else if(velocity < -600)
		velocity = -600.0;
	*/
}

void current_control(float target)
{
	error_cur = target - cur_cur;
	error_cur_sum += error_cur;

	current = P_cur * error_cur + I_cur * error_cur_sum * 0.005;
}

/*
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
*/
