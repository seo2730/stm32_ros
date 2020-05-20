/*
 * mainpp.h
 *
 *  Created on: May 20, 2020
 *      Author: seodeoghyeon
 */

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

#define P_deg		5
#define D_deg		1

#define P_vel		1.35
#define I_vel		0.01

#define P_cur		0.25
#define I_cur		100

#define PI 			3.14159265
#define GEAR_RATIO 	21.0
#define CPT			500.0

#define DEG2RAD 	PI/180
#define RAD2DEG 	180/PI

#ifdef __cplusplus
 extern "C" {
#endif

void setup(void);
void loop(void);
void current_sensor();
void postion_control(float target);
void speed_control(float target, float limit);
void current_control(float target, float limit);

#ifdef __cplusplus
}
#endif



#endif /* INC_MAINPP_H_ */
