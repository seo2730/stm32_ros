#include "ros/ros.h"
#include "stm32_ros_encoder/motor.h"
#include "std_msgs/Float32.h"

void motorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    //ROS_INFO("Deg : %f", (msg->data)*180/3.14159265); //degree
    ROS_INFO("vel : %f", (msg->data)*180/3.14159265); //velocity
    //ROS_INFO("Cur : %f", msg->data); //current
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"deg_data");
    ros::NodeHandle nh;

    //ros::Subscriber motor_sub = nh.subscribe("degree",100, motorCallback);
    ros::Subscriber motor_sub = nh.subscribe("velocity",100, motorCallback);
    //ros::Subscriber motor_sub = nh.subscribe("current",100, motorCallback);

    ros::spin();

    return 0;
}