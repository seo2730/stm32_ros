#include "ros/ros.h"
#include "stm32_ros_encoder/motor.h"

void motorCallback(const stm32_ros_encoder::motor::ConstPtr& msg)
{
    ROS_INFO("Vel : %f,  Deg : %f", msg->velocity, msg->degree );
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"motor data");
    ros::NodeHandle nh;

    ros::Subscriber motor_sub = nh.subscribe("motor_data",100, motorCallback);

    ros::spin();

    return 0;
}