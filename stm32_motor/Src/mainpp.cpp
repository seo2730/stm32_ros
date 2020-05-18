/*
 * mainpp.cpp
 *
 *  Created on: May 14, 2020
 *      Author: seo27
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

uint32_t nowTick = 0;
uint32_t pastTick = 0;

uint8_t led0=0;
void led0_cb(const std_msgs::UInt8& msg);
ros::Subscriber<std_msgs::UInt8> led0_sub("led0",&led0_cb);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

std_msgs::UInt32 encoder_msg;
ros::Publisher encoder_pub("encoder_pub", &encoder_msg);
uint32_t encoder=0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
 nh.initNode();
 nh.advertise(chatter);
 nh.advertise(encoder_pub);
 nh.subscribe(led0_sub);
}

void loop(void)
{
	nowTick = HAL_GetTick();
	encoder = TIM3->CNT;
	if(nowTick - pastTick > 10)
	{
		if(led0 == 1)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

		else if(led0 == 0)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

		str_msg.data = hello;
		chatter.publish(&str_msg);

		encoder_msg.data = encoder;
		encoder_pub.publish(&encoder_msg);
		pastTick = nowTick;
	}

	nh.spinOnce();

}

void led0_cb(const std_msgs::UInt8& msg)
{
	if(msg.data == 1)
		led0=1;

	else if(msg.data == 0)
		led0=0;
}
