#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char** argv){
    //ros initial
    ros::init(argc, argv, "test_ultra_node");
    ros::NodeHandle h;
    ros::Rate rate(10);

    ros::Publisher pub_ultra = h.advertise<std_msgs::Float32MultiArray>("ultra_sensors", 1);
    std_msgs::Float32MultiArray ultra_msg;

    ultra_msg.data = {0, 0, 0, 0, 0};

    while(ros::ok())
    {
        ultra_msg.data[0] = 1000.;
        ultra_msg.data[1] = 1000.;
        ultra_msg.data[2] = 1000.;
        ultra_msg.data[3] = 1000.;
        ultra_msg.data[4] = 1000.;
        ROS_INFO("Pub ultra");
        pub_ultra.publish(ultra_msg);
        rate.sleep();
    }
    return 0;
}