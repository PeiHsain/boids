#include <ros/ros.h>
#include "boids/Robot.h"
#include "boids/RobotArray.h"
#include "boids/Robot_Velocity.h"
#include "boids/Robot_Velocity_Array.h"

int main(int argc, char** argv){
    //ros initial
    ros::init(argc, argv, "test_camera_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher pub_velocity = nh.advertise<boids::Robot_Velocity_Array>("velocity_robot", 1);
    boids::Robot_Velocity_Array velocity_msg;
    ros::Publisher pub_pose = nh.advertise<boids::RobotArray>("pose_robot", 1);
    boids::RobotArray pose_msg;

    velocity_msg.robots.resize(5);
    velocity_msg.robots[0].id = -1;
    velocity_msg.robots[0].velocity_x = 0;
    velocity_msg.robots[0].velocity_y = 0;
    velocity_msg.robots[1].id = -1;
    velocity_msg.robots[1].velocity_x = 0;
    velocity_msg.robots[1].velocity_y = 0;
    velocity_msg.robots[2].id = -1;
    velocity_msg.robots[2].velocity_x = 0;
    velocity_msg.robots[2].velocity_y = 0;
    velocity_msg.robots[3].id = -1;
    velocity_msg.robots[3].velocity_x = 0;
    velocity_msg.robots[3].velocity_y = 0;
    velocity_msg.robots[4].id = -1;
    velocity_msg.robots[4].velocity_x = 0;
    velocity_msg.robots[4].velocity_y = 0;

    pose_msg.robots.resize(0);
    // pose_msg.robots[0].id = 1;
    // pose_msg.robots[0].pose.position.x = 32;
    // pose_msg.robots[0].pose.position.z = 55;
    // pose_msg.robots[1].id = 0;
    // pose_msg.robots[1].pose.position.x = 34;
    // pose_msg.robots[1].pose.position.z = 60;


    while (ros::ok())
    {
        ROS_INFO("Pub v");
        pub_velocity.publish(velocity_msg);
        ROS_INFO("Pub p");
        pub_pose.publish(pose_msg);
        rate.sleep();
    }
    return 0;
}