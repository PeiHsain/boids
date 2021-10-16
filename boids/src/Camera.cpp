#include <ros/ros.h>
#include "boids/Camera.h"
#include "boids/Robot.h"
#include "boids/RobotArray.h"
#include "boids/Robot_Velocity.h"
#include "boids/Robot_Velocity_Array.h"
#include "boids/Flock.h"
#include "boids/Leader.h"
#include "boids/Boid.h"
#include <algorithm>
#include <vector>

#define safeDistance 20 //cm

Camera::Camera(){
    sub_pose = n.subscribe("pose_robot", 1, &Camera::PoseFeedback, this);
    sub_v = n.subscribe("velocity_robot", 1, &Camera::VelocityFeedback, this);
    see = false;
    robotSize = 0;
    notSeeDistance = 70; //cm
    robotCloseDistance = 50000; //init very long distance
}

//(cm) msg->x = pose x, msg->z = pose y, msg->orientation = theta(-pi~0~pi)
void Camera::PoseFeedback(const boids::RobotArray::ConstPtr& msg){
    robotSize = msg->robots.size();
    if(robotSize == 0) see = false;
    else{
        see = true;
        leader.l_update(0., 0.);
        for(int i = 0 ; i < 4 ; i++)
            robot[i].l_update(0., 0.);
        for(int i = 0 ; i < robotSize ; i++){
            float leaderDis = 0, robotDis = 0;
            robotCloseDistance = 50000;
            switch (msg->robots[i].id)
            {
            case 0:
                leader.l_update(msg->robots[i].pose.position.x, msg->robots[i].pose.position.z);
                leaderDis = leader.l_get().magnitude();
                if(leaderDis < robotCloseDistance)
                    robotCloseDistance = leaderDis;
                break;
            case 1:
                robot[0].l_update(msg->robots[i].pose.position.x, msg->robots[i].pose.position.z);
                robotDis = robot[0].l_get().magnitude();
                if(robotDis < robotCloseDistance)
                    robotCloseDistance = robotDis;
                break;
            case 2:
                robot[1].l_update(msg->robots[i].pose.position.x, msg->robots[i].pose.position.z);
                robotDis = robot[1].l_get().magnitude();
                if(robotDis < robotCloseDistance)
                    robotCloseDistance = robotDis;
                break;
            case 3:
                robot[2].l_update(msg->robots[i].pose.position.x, msg->robots[i].pose.position.z);
                robotDis = robot[2].l_get().magnitude();
                if(robotDis < robotCloseDistance)
                    robotCloseDistance = robotDis;
                break;
            case 4:
                robot[3].l_update(msg->robots[i].pose.position.x, msg->robots[i].pose.position.z);
                robotDis = robot[3].l_get().magnitude();
                if(robotDis < robotCloseDistance)
                    robotCloseDistance = robotDis;
                break;
            default:
                break;
            }
        }
    }
}

//no see robot -> id = -1, five robots -> id=0 is leader
void Camera::VelocityFeedback(const boids::Robot_Velocity_Array::ConstPtr& msg){
    int i = 0;
    leader.v_update(0., 0.);
    for(int i = 0 ; i < 4 ; i++)
        robot[i].v_update(0., 0.);
    while(i < msg->robots.size())
    {
        switch(msg->robots[i].id)
        {
        case 0:
            leader.v_update(msg->robots[i].velocity_x, msg->robots[i].velocity_y);
            break;
        case 1:
            robot[0].v_update(msg->robots[i].velocity_x, msg->robots[i].velocity_y);
            break;
        case 2:
            robot[1].v_update(msg->robots[i].velocity_x, msg->robots[i].velocity_y);
            break;
        case 3:
            robot[2].v_update(msg->robots[i].velocity_x, msg->robots[i].velocity_y);
            break;
        case 4:
            robot[3].v_update(msg->robots[i].velocity_x, msg->robots[i].velocity_y);
            break;
        }
        i++;
    }
}

void Camera::CallCamera(Flock& f, Leader& l){
    ros::spinOnce();
    //ROS_INFO("Get pose");
    //ROS_INFO("Get velocity");
    leader.See();
    if(leader.seeOrNot()){
        l.appear_update(leader.seeOrNot());
        l.l_update(leader.l_get().x, leader.l_get().y);
        l.v_update(leader.v_get().x, leader.v_get().y);
        //ROS_INFO("leader : vx=%f, vy=%f, px=%f, py=%f", leader.v_get().x, leader.v_get().y, leader.l_get().x, leader.l_get().y);
    }
    else{
        l.l_update(0., 0.);
        l.v_update(0., 0.);
    }
    //flock clean
    f.flock.clear();
    for(int i = 0 ; i < 4 ; i++)
        if(robot[i].See())
            f.addBoid(robot[i]);
    //for(int i = 0 ; i < 4 ; i++)
        //ROS_INFO("robot[%d] : vx=%f, vy=%f, px=%f, py=%f", i, robot[i].v_get().x, robot[i].v_get().y, robot[i].l_get().x, robot[i].l_get().y);
}

//camera had see robots return true, else return false
bool Camera::HadRobots(){ return see;}

//identify the robot status
int Camera::WhichState(){
    if(see == false) //see nothing
        return 1;
    else{
        if(leader.seeOrNot()){
            if(leader.l_get().magnitude() < safeDistance) //catch leader
                return 4;
            else return 3; //see leader
        }
        else{ //don't see leader, but see robots
            for(int i = 0 ; i < 4 ; i++){
                if(robot[i].See())
                    if(robot[i].l_get().magnitude() < safeDistance) //catch robots
                        return 4;
            }
            return 2; //see robots
        }
    }
}

//if the camera doesn't see anything, check how far the closest robot with me last time
//if the distance bigger than notSeeDistance, remain same vector(11), otherwise stop(12).
int Camera::IsReallyUnvisible(){
    if(robotCloseDistance >= notSeeDistance)
        return 11;
    else return 12;
}
