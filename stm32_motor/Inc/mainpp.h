/*
* mainpp.h
*
*  Created on: May 20, 2020
*      Author: seodeoghyeon
*/

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

#define P_deg	    100
#define D_deg		0.5

#define P_vel		100
#define I_vel		0.1

#define P_cur		0.798
#define I_cur		4216

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
