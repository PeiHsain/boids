#include <ros/ros.h>
#include "boids/Flock.h"
#include "boids/Boid.h"
#include "boids/Pvector.h"
#include "boids/Leader.h" 
#include "boids/Ultrasonic.h"
#include "boids/Camera.h" 
#include "std_msgs/Float32MultiArray.h"

using namespace std;

int main(int argc, char** argv)
{
    //ros initial
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher pub_robot = nh.advertise<std_msgs::Float32MultiArray>("RobotData", 1);
    std_msgs::Float32MultiArray robot_msg;

    //RobotData topic initial
    robot_msg.data = {0, 0, 0, 0};

    //RobotData variables
    bool firstStop = true;
    float distance = 0., angle = 0., stop = 0., inCircle = 0.;
    Flock flock;
    Boid me(0., 0.);
    Leader leader(0., 0.);
    Ultrasonic ultra;
    Camera camera;

    while(ros::ok()){
        double begin = ros::Time::now().toSec();
        //call altrasonic
        ultra.getUltra();
        //call camera
        camera.CallCamera(flock, leader);
        if(!camera.HadRobots()){//camera don't see
            ROS_INFO("Don't see robots");
            me.v_update(0., 0.);
            distance = 0.;
            angle = 0.;              
            if(firstStop == true){//stop to see more once
                ROS_INFO("First don't see");
                stop = 1.;
                inCircle = 0.;
                //first time nothing = false
                firstStop = false;                
            }
            else{         
                ROS_INFO("In Circle");
                stop = 0.;
                inCircle = 1.;
                //first time nothing = false
                firstStop = false;
            }
        }
        else{//boids
            ROS_INFO("Boids");
            //first time nothing = true
            firstStop = true;
            //boids rules
            me.run(flock.flock, leader, ultra);
            //robot stop or not
            if(me.beBlocked(ultra)){//be blocked
                ROS_INFO("Blocked");
                me.v_update(0., 0.);
                stop = 1.;//true
            }
            else stop = 0.;//false
            //d = distance
            distance = me.v_get().magnitude();
            //a = angle
            angle = me.Angletf();
            inCircle = 0.;        
        }
        ROS_INFO("location: x=%f, y=%f", me.l_get().x, me.l_get().y);
        ROS_INFO("Velocity: x=%f, y=%f", me.v_get().x, me.v_get().y);
        //give robot control
        robot_msg.data[0] = distance;
        robot_msg.data[1] = angle; //-5~5(left to right)
        robot_msg.data[2] = stop; //stop=1, no stop=0
        robot_msg.data[3] = inCircle; //
        ROS_INFO("Pub robot");
        pub_robot.publish(robot_msg);

        double end = ros::Time::now().toSec();
        double duringTime = end - begin;
        ROS_INFO("procces time = %f s", duringTime);

        rate.sleep();
    }
    return 0;
}
