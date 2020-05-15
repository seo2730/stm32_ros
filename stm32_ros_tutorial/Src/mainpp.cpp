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
#include <std_msgs/String.h>

ros::NodeHandle nh;

uint32_t nowTick = 0;
uint32_t pastTick = 0;

uint8_t led0=0;
char hello[] = "Hello world!";

void led0_cb(const std_msgs::UInt8& msg);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

ros::Subscriber<std_msgs::UInt8> led0_sub("led0",&led0_cb);

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
 nh.subscribe(led0_sub);
}

void loop(void)
{
	nowTick = HAL_GetTick();

	if(nowTick - pastTick > 10)
	{
		if(led0 == 1)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

		else if(led0 == 0)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

		str_msg.data = hello;
		chatter.publish(&str_msg);
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
