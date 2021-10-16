#ifndef ALTRASONIC_H_
#define ALTRASONIC_H_

#include <ros/ros.h>
// #include "boids/Boid.h"
#include "std_msgs/Float32MultiArray.h"


class Ultrasonic {
private:
    float sonicDistance[5]; //five ultrasonic distance
    float oldDistance[5]; //previous five ultrasonic distance
    //ros
    ros::NodeHandle n;
    ros::Subscriber sub_ultra; 
public:
    Ultrasonic();
    void UltraFeedback(const std_msgs::Float32MultiArray::ConstPtr&); 
    void getUltra();
    float getDistance(int i) const;
};

#endif