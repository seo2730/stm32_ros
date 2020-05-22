/*
 * mainpp.cpp
 *
 *  Created on: May 20, 2020
 *      Author: seodeoghyeon
 */
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
uint32_t pastTick = 0;

//// Encoder ////
std_msgs::UInt32 encoder_msg;
ros::Publisher encoder_pub("encoder", &encoder_msg);

//// Motor data ////
stm32_ros_encoder::motor motor_msg;
std_msgs::Float32 deg_msg;
std_msgs::Float32 vel_msg;
std_msgs::Float32 cur_msg;

ros::Publisher deg_pub("degree", &deg_msg);
ros::Publisher vel_pub("velocity", &vel_msg);
ros::Publisher cur_pub("current", &cur_msg);

int32_t encoder=0;

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

// 값 보내는 용도로 쓰임
void loop(void)
{
	nowTick = HAL_GetTick();

	if(nowTick - pastTick > 10)
	{
		cur_msg.data =cur_cur;
		cur_pub.publish(&cur_msg);

		vel_msg.data = cur_vel;
		vel_pub.publish(&vel_msg);

		encoder_msg.data = encoder;
		encoder_pub.publish(&encoder_msg);

		deg_msg.data = cur_deg;
		deg_pub.publish(&deg_msg);

		pastTick = nowTick;
		nowTick=0;
	}

	nh.spinOnce();

}

void current_sensor()
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	cur_cur = 2*(HAL_ADC_GetValue(&hadc1)*5.0/1360.0 - 2.5); //HAL_ADC_GetValue(&hadc1);
}


void postion_control(float target)
{
	position = P_deg * error_deg + D_deg * (error_deg - past_error_deg) * 2;
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

	velocity = P_vel * error_vel + I_vel * error_vel_sum * 0.005;

	/*
	if(velocity > 600.0)
		velocity = 600.0;
	else if(velocity < -600)
		velocity = -600.0;
	*/
}


void current_control(float target, float limit)
{
	if(target > limit)
		target = limit;
	else if(target < -limit)
		target = -limit;

	error_cur = target - cur_cur;
	error_cur_sum += error_cur;

	if(error_cur_sum>500)
		error_cur_sum=500;
	else if(error_cur_sum<-500)
		error_cur_sum = -500;

	current = P_cur * error_cur + I_cur * error_cur_sum * 0.0005;
}
