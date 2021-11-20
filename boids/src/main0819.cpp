
#include <ros/ros.h>
#include "boids/Flock.h"
#include "boids/Boid.h"
#include "boids/Pvector.h"
#include "boids/Leader.h"
#include "boids/Ultrasonic.h"
#include "boids/Camera.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;
enum Status{NotSee = 1, JustSeeRobot, SeeLeader, Catch};

int main(int argc, char** argv)
{
    //ros initial
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher pub_robot = nh.advertise<std_msgs::Float32MultiArray>("RobotData", 1);
    std_msgs::Float32MultiArray robot_msg;

    //RobotData topic initial
    robot_msg.data = {0, 0, 0, 0, 0, 0};

    //RobotData variables
    int status = 0, firstStop = 0, block = 0;
    float addAngle = 0.5;
    float speed = 0., angle = 0., stop = 0., inCircle = 0.;//inCircle: no circle=0, turn right=1, turn left=2
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

        status = camera.WhichState();
        switch(status){
            case NotSee:
                ROS_INFO("Don't see robots!");
                if (firstStop == 0){//first see nothing
			        firstStop ++;                        
                    break;
                }    
                else if(firstStop > 0 && firstStop < 5){//stop to see more once
                    ROS_INFO("Stop");
                    stop = 1.;
                    inCircle = 0.;
                    //first time nothing = false
                    firstStop ++;
                }
                else{
                    ROS_INFO("In Circle");
                    //if(firstStop <= 1){
                        if(angle >= 0) //turn right
                            inCircle = 1.;
                        else //turn left
                            inCircle = 2.;
                    //}
                    me.v_update(0., 0.);
                    speed = 0.;
                    //angle = 0.;
                    stop = 0.;
                    //first time nothing = false
                    firstStop ++;
                    if(firstStop >= 8)
                        firstStop = 0;
                }                         
                break;
            case JustSeeRobot:
                ROS_INFO("Boids!");
                //first time nothing = true
                firstStop = 0;
                //boids rules
                me.run(flock.flock, leader, ultra);
                //robot stop or not
                block = me.beBlocked(ultra);
                if(block == 1){//be blocked
                    ROS_INFO("Blocked");
                    me.v_update(0., 0.);
                    stop = 1.;//true
                    angle = 0;
                }
                else if(block == 2){//add degree, move to left
                    ROS_INFO("Blocked ADD");
                    stop = 0.;
                    angle = -0.5;
                    // angle = me.Angletf() + addAngle;
                }
                else if(block == 3){//sub degree, move to right
                    ROS_INFO("Blocked SUB");
                    stop = 0.;
                    angle = 0.5;
                    // angle = me.Angletf() - addAngle;
                }
                else{//no blocked
                    stop = 0.;//false
                    angle = me.Angletf();
                }
                //d = speed
                speed = me.v_get().magnitude();
	            inCircle = 0.;
                break;
            case SeeLeader:
                ROS_INFO("Leader boids!");
                //first time nothing = true
                firstStop = 0;
                //boids rules
                me.runL(flock.flock, leader, ultra);
                //robot stop or not
                block = me.beBlocked(ultra);
                if(block == 1){//be blocked
                    ROS_INFO("Blocked");
                    me.v_update(0., 0.);
                    stop = 1.;//true
                    angle = 0;
                }
                else if(block == 2){//add degree, move more to left
                    ROS_INFO("Blocked ADD");
                    stop = 0.;
                    angle = -0.5;//me.Angletf() + addAngle;
                }
                else if(block == 3){//sub degree, move more to right
                    ROS_INFO("Blocked SUB");
                    stop = 0.;
                    angle = 0.5;//me.Angletf() - addAngle;
                }
                else{//no blocked
                    stop = 0.;//false
                    angle = me.Angletf();
                }
                //d = speed
                speed = me.v_get().magnitude();
	            inCircle = 0.;
                break;
            case Catch:
                ROS_INFO("Catch! %f", leader.l_get().magnitude());
		ROS_INFO("leader x = %f, y = %f", leader.l_get().x, leader.l_get().y);
                firstStop = true;
                me.v_update(0., 0.);
                speed = 0.;
                angle = 0.;
                stop = 1.;
	            inCircle = 0.;
                break;
        }

        ROS_INFO("location: x=%f, y=%f", me.l_get().x, me.l_get().y);
        ROS_INFO("Velocity: x=%f, y=%f", me.v_get().x, me.v_get().y);
        //give robot control
	    ROS_INFO("speed: %f", speed);
	    ROS_INFO("angle: %f", angle);
        if(speed > 100)
            speed = 100;
	else if(speed > 15 && speed < 30)
	    speed *= 0.9;
        else if(speed < 0)
            speed = 0;
        robot_msg.data[0] = speed;
        robot_msg.data[1] = angle; //-5~5(left to right)
        robot_msg.data[2] = stop; //stop=1, no stop=0
        robot_msg.data[3] = inCircle; //no circle=0, turn right circle=1, turn left circle=2
        //ROS_INFO("Pub robot");
        pub_robot.publish(robot_msg);

        double end = ros::Time::now().toSec();
        double duringTime = end - begin;
        ROS_INFO("procces time = %f s", duringTime);
        ROS_INFO("------------------------------");
        rate.sleep();
    }
    return 0;
}
