#include <ros/ros.h>
// #include "boids/Boid.h"
#include "boids/Ultrasonic.h"
#include "std_msgs/Float32MultiArray.h"
#include <algorithm>

Ultrasonic::Ultrasonic(){
    sub_ultra = n.subscribe("ultra_sensors", 1, &Ultrasonic::UltraFeedback, this);
    std::fill(sonicDistance, sonicDistance+5, 0);
    std::fill(oldDistance, oldDistance+5, 0);
}

//(degree) [0]=-22.5~22.5, [1]=22.5~67.5, [2]=67.5~112.5, [3]=112.5~157.5, [4]=157.5~202.5
void Ultrasonic::UltraFeedback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for(int i = 0 ; i < 5 ; i++)
        oldDistance[i] = sonicDistance[i];
    for(int i = 0 ; i < 5 ; i++)
        sonicDistance[i] = msg->data[i];
    //if distance > 1000 -> not see or error
}

void Ultrasonic::getUltra(){
    ros::spinOnce();
    //ROS_INFO("Get ultrsonic");
    //for(int i = 0 ; i < 5 ; i++)
        //ROS_INFO("Ultra[%d] : %f", i, sonicDistance[i]);
}

float Ultrasonic::getDistance(int i) const{
    if(sonicDistance[i] == 404)
        return this->oldDistance[i];
    else
        return this->sonicDistance[i];
}
