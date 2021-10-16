#include <ros/ros.h>
#include "boids/Boid.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
// #include "SFML/Graphics.hpp"

// Global Variables for borders()
// desktopTemp gets screen resolution of PC running the program
// extern sf::VideoMode desktopTemp;// = sf::VideoMode::getDesktopMode();
// const int window_height = desktopTemp.height;
// const int window_width = desktopTemp.width;

// #define w_height window_height
// #define w_width window_width
#define PI 3.141592635

// =============================================== //
// ======== Boid Functions from Boid.h =========== //
// =============================================== //

Boid::Boid(float x, float y)
{
    acceleration = Pvector(0, 0);
    velocity = Pvector(0, 0);
    location = Pvector(x, y);
    maxSpeed = 50;
    maxForce = 1.5;
    maxRange = 10;
    stop = false;
    blockDistance = 20;//cm, for stop
    desiredseparation = 40;//cm, for separation
    Ultraseparation = 40;//cm, for separation
    slowDownDistance = 30;//cm, for separation slow down
    angle = 0;
    angletf = 0;
}

// Adds force Pvector to current force Pvector
void Boid::applyForce(const Pvector& force)
{
    acceleration.addVector(force);
}

//update relative location
void Boid::l_update(const float x, const float y)
{
    location.set(x, y);
}

//update relative velocity
void Boid::v_update(const float x, const float y)
{
    velocity.set(x, y);
}

Pvector Boid::l_get() const{ return this->location;}

Pvector Boid::v_get() const{ return this->velocity;}

Pvector Boid::a_get() const{ return this->acceleration;}

// Separation
// Keeps boids from getting too close to one another
Pvector Boid::Separation(const vector<Boid>& boids, const Leader& lead, const Ultrasonic& ultra)
{
    // Distance of field of vision for separation between boids
    Pvector steer(0, 0);
    Pvector steerUltra(0, 0);
    int count = 0, countUltra = 0;
    // For every boid in the system, check if it's too close
    for (int i = 0; i < boids.size(); i++) {
        // Calculate distance from current boid to boid we're looking at
        float d = boids[i].l_get().magnitude();
        // If this is a fellow boid and it's too close, move away from it
        if ((d < desiredseparation)) {
            Pvector diff(0,0);
            diff.subTwoVector(location, boids[i].l_get());
            if(d <= slowDownDistance)
                diff.mulScalar(1.1);
            // diff.normalize();
            // diff.divScalar(d);      // Weight by distance
            steer.addVector(diff);
            count++;
        }
    }
    // Leader
    if(lead.seeOrNot()){
        // Calculate distance from leader to boid we're looking at
        float d = lead.l_get().magnitude();
        if (d < desiredseparation) {
            Pvector diff(0,0);
            // diff = diff.subTwoVector(location, boids[i].location);
            diff.subTwoVector(location, lead.l_get());
            if(d <= slowDownDistance)
                diff.mulScalar(1.1);
            // diff.normalize();
            // diff.divScalar(d);      // Weight by distance
            steer.addVector(diff);
            count++;
        }
    }
    //ultrasonic
    for (int i = 0; i < 5; i++) {
        // Calculate distance from the blocked to boid we're looking at
        float dis = ultra.getDistance(i);
        if (dis < Ultraseparation) {
            Pvector diff(0,0), u(0,0);
            float x = 0, y = 0;
            switch (i)
            {
            case 0:
                u.set(dis, 0);
                break;
           case 1:
                x = dis * std::cos(PI/4);
                y = dis * std::sin(PI/4);
                u.set(x, y);
                break;
            case 2:
                u.set(0, dis);
                break;
            case 3:
                x = -(dis * std::cos(PI/4));
               y = dis * std::sin(PI/4);
                u.set(x, y);
                break;
            case 4:
                u.set(-dis, 0);
                break;
            }
            diff.subTwoVector(location, u);
            // diff.normalize();
            // diff.divScalar(d);      // Weight by distance
            steerUltra.addVector(diff);
            countUltra++;
        }
    }
    // Adds average difference of location to acceleration
    if (count > 0 && countUltra > 0){ //see robots and ultra too clse(6:4)
        steer.divScalar((float)count);
        steer.mulScalar(0.6);
        steerUltra.divScalar((float)count);
        steerUltra.mulScalar(0.4);
        steer.addVector(steerUltra);
    }
    else if(count > 0 && countUltra <= 0){ //see robots and ultra not close
        steer.divScalar((float)count);
    }
    if (steer.magnitude() > 0) {
        // Steering = Desired - Velocity
        steer.normalize();
        steer.mulScalar(maxSpeed);
        steer.subVector(velocity);
        steer.limit(maxForce);
    }
    //ROS_INFO("S : x=%f, y=%f", steer.x, steer.y);
    return steer;
}

// Alignment
// Calculates the average velocity of boids in the field of vision and
// manipulates the velocity of the current boid in order to match it
Pvector Boid::Alignment(const vector<Boid>& Boids, const Leader& lead)
{
    Pvector sum(0, 0);
    int count = 0;
    for (int i = 0; i < Boids.size(); i++) {
        sum.addVector(Boids[i].v_get());
        count++;
    }
    //leader
    if(lead.seeOrNot()){
        sum.addVector(lead.v_get());
        count++;
    }

    // If there are boids close enough for alignment...
    if (count > 0) {
        sum.divScalar((float)count);// Divide sum by the number of close boids (average of velocity)
        sum.normalize();            // Turn sum into a unit vector, and
        sum.mulScalar(maxSpeed);    // Multiply by maxSpeed
        Pvector steer;
        steer.subTwoVector(sum, velocity);
        steer.limit(maxForce);
        //ROS_INFO("A : x=%f, y=%f", steer.x, steer.y);
        return steer;
    } else {
        Pvector temp(0, 0);
        return temp;
    }
}

// Cohesion
// Finds the average location of nearby boids and manipulates the
// steering force to move in that direction.
Pvector Boid::Cohesion(const vector<Boid>& Boids, const Leader& lead)
{
    // float neighbordist = 100;
    Pvector sum(0, 0);
    int count = 0;
    for (int i = 0; i < Boids.size(); i++) {
        sum.addVector(Boids[i].l_get());
        count++;
    }
    //leader
    if(lead.seeOrNot()){
        sum.addVector(lead.l_get());
        count++;
    }

    if (count > 0) {
        sum.divScalar(count);
        return seek(sum);
    } else {
        Pvector temp(0,0);
        return temp;
    }
}

// FollowLeader
// Finds the average location of nearby boids and manipulates the
// steering force to move in that direction.
Pvector Boid::FollowLeader(const Leader& lead)
{
    //calculate approach point
    int stopDistance = 15; //cm
    Pvector betweenV(0, 0);
    Pvector approachP(0, 0);
    betweenV.subTwoVector(lead.l_get(), this->l_get());
    approachP.subTwoVector(lead.l_get(), this->l_get());
    betweenV.normalize();
    betweenV.mulScalar(stopDistance);
    approachP.subVector(betweenV);
    //ROS_INFO("Leader point : x=%f, y=%f", approachP.x, approachP.y);
    //calculate acceleration to approach point
    Pvector sum(0, 0);
    sum.addVector(approachP);
    return seek(sum);
}

// Limits the maxSpeed, finds necessary steering force and
// normalizes vectors
Pvector Boid::seek(const Pvector& v)
{
    Pvector desired(location.x, location.y);
    desired.subVector(v);  // A vector pointing from the location to the target
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mulScalar(maxSpeed);
    // Steering = Desired minus Velocity
    acceleration.subTwoVector(velocity, desired);//(desired, velocity);
    acceleration.limit(maxForce);  // Limit to maximum steering force
    //ROS_INFO("Seek : x=%f, y=%f", acceleration.x, acceleration.y);
    return acceleration;
}

// Modifies velocity, location, and resets acceleration with values that
// are given by the laws.
void Boid::update(const vector <Boid>& v, const Leader& l)
{
        // ROS_INFO("Up1 : x=%f, y=%f", acceleration.x, acceleration.y);
        // acceleration.mulScalar(0.5);
        //ROS_INFO("UpAcc : x=%f, y=%f", acceleration.x, acceleration.y);
        // Update velocity
        //ROS_INFO("Up1v : x=%f, y=%f", velocity.x, velocity.y);
        velocity.addVector(acceleration);
        //ROS_INFO("Up2v : x=%f, y=%f", velocity.x, velocity.y);
        // Limit speed
        velocity.v_tf(maxSpeed, maxRange);
        //ROS_INFO("Up3v : x=%f, y=%f", velocity.x, velocity.y);
        location.addVector(velocity);
        // Reset accelertion to 0 each cycle
        acceleration.mulScalar(0);
}

// Run flock() on the flock of boids.
// This applies the three rules, modifies velocities accordingly, updates data,
// and corrects boids which are sitting outside of the SFML window
void Boid::run(const vector <Boid>& v, const Leader& l, const Ultrasonic& u)
{
    //start form zero point
    l_update(0, 0);
    v_update(0, 0);
    flock(v, l, u);
    update(v, l);
}
// Run for leader
void Boid::runL(const vector <Boid>& v, const Leader& l, const Ultrasonic& u)
{
    //start form zero point
    l_update(0, 0);
    v_update(0, 0);
    flockL(v, l, u);
    update(v, l);
}

// Applies the three laws to the flock of boids
void Boid::flock(const vector<Boid>& v, const Leader& l, const Ultrasonic& u)
{
    Pvector sep = Separation(v, l, u);
    Pvector ali = Alignment(v, l);
    Pvector coh = Cohesion(v, l);

    // Might need to alter weights for different characteristics
    sep.mulScalar(3.3);
    ali.mulScalar(1.75);
    coh.mulScalar(1.75);

    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
}
// Flock for leader
void Boid::flockL(const vector<Boid>& v, const Leader& l, const Ultrasonic& u)
{
    Pvector sep = Separation(v, l, u);
    Pvector ali = Alignment(v, l);
    Pvector coh = Cohesion(v, l);
    Pvector foll = FollowLeader(l);

    // Might need to alter weights for different characteristics
    sep.mulScalar(2.6);
    ali.mulScalar(1.0);
    coh.mulScalar(1.0);
    foll.mulScalar(0.8);

    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
    applyForce(foll);
}

// Calculates the angle for the velocity of a boid which allows the visual
// image to rotate in the direction that it is going in.(0~pi, 0~-pi)
void Boid::Angle()
{
    // From the definition of the dot product
    angle = (float)(atan2(velocity.y, velocity.x) * 180 / PI);
    ROS_INFO("angle = %f", angle);
}

//forword = 0, trans. angle to -5~5(left to right)
float Boid::Angletf()
{
    Angle();
    if(angle <= 90 && angle >-90) //0~pi
        angletf = (90 - angle)*5/180;
    else if(angle > 90)
        angletf = (90 - angle)*5/180;
    else if(angle <= -90)
        angletf = (-270 - angle)*5/180;
    return angletf;
}

// float Boid::getangle(){
//     return angle;
// }

//if see boid return true(have v and l), else return false
bool Boid::See(){
    if(velocity.x == 0 && velocity.y == 0 && location.x == 0 && location.y == 0)
        return false;
    else return true;
}

//180/8 = 22.5 degree/area ; return = 0:no blocked, = 1:blocked, = 2:add degree, = 3:sub degree
int Boid::beBlocked(const Ultrasonic& u){
    Angle();
    if(angle > 0 && angle <= 22.5){ //first area (<22.5 degree)
        if(u.getDistance(0) <= blockDistance && u.getDistance(1) <= blockDistance)
            return 1;//stop = true;
        else if(u.getDistance(0) <= blockDistance && u.getDistance(1) > blockDistance)
            return 2;//add degree;
        else if(u.getDistance(0) > blockDistance && u.getDistance(1) <= blockDistance)
            return 3;//sub degree;
        else return 0;//stop = false;
    }
    else if(angle > 22.5 && angle <= 45){ //second area (22.5~45 degree)
        if(u.getDistance(0) <= blockDistance && u.getDistance(1) <= blockDistance)// || u.getDistance(2) <= blockDistance)
            return 1;//stop = true;
        else if(u.getDistance(0) <= blockDistance && u.getDistance(1) > blockDistance)
            return 2;//add degree;
        else if(u.getDistance(0) > blockDistance && u.getDistance(1) <= blockDistance)
            return 3;//sub degree;
        else return 0;//stop = false;
    }
    else if(angle > 45 && angle <= 67.5){ //third area (45~67.5 degree)
        if(u.getDistance(1) <= blockDistance || u.getDistance(2) <= blockDistance)// || u.getDistance(2) <= blockDistance)
            return 1;//stop = true;
        else if(u.getDistance(1) <= blockDistance && u.getDistance(2) > blockDistance)
            return 2;//add degree;
        else if(u.getDistance(1) > blockDistance && u.getDistance(2) <= blockDistance)
            return 3;//sub degree;
        else return 0;//stop = false;
    }
    else if(angle > 67.5 && angle <= 90){ //forth area (67.5~90 degree)
        if(u.getDistance(1) <= blockDistance ||u.getDistance(2) <= blockDistance)// || u.getDistance(3) <= blockDistance)
            return 1;//stop = true;
        else if(u.getDistance(1) <= blockDistance && u.getDistance(2) > blockDistance)
            return 2;//add degree;
        else if(u.getDistance(1) > blockDistance && u.getDistance(2) <= blockDistance)
            return 3;//sub degree;
        else return 0;//stop = false;
    }
    else if(angle > 90 && angle <= 112.5){ //third area (90~112.5 degree)
        if(u.getDistance(2) <= blockDistance || u.getDistance(3) <= blockDistance)// || u.getDistance(3) <= blockDistance)
            return 1;//stop = true;
        else if(u.getDistance(2) <= blockDistance && u.getDistance(3) > blockDistance)
            return 2;//add degree;
        else if(u.getDistance(2) > blockDistance && u.getDistance(3) <= blockDistance)
            return 3;//sub degree;
        else return 0;//stop = false;
    }
    else if(angle > 112.5 && angle <= 135){ //forth area (112.5~135 degree)
        if(u.getDistance(2) <= blockDistance || u.getDistance(3) <= blockDistance)// || u.getDistance(4) <= blockDistance)
            return 1;//stop = true;
        else if(u.getDistance(2) <= blockDistance && u.getDistance(3) > blockDistance)
            return 2;//add degree;
        else if(u.getDistance(2) > blockDistance && u.getDistance(3) <= blockDistance)
            return 3;//sub degree;
        else return 0;//stop = false;
    }
    else if(angle > 135 && angle <= 157.5){ //forth area (135~157.5 degree)
        if(u.getDistance(3) <= blockDistance || u.getDistance(4) <= blockDistance)// || u.getDistance(4) <= blockDistance)
            return 1;//stop = true;
        else if(u.getDistance(3) <= blockDistance && u.getDistance(4) > blockDistance)
            return 2;//add degree;
        else if(u.getDistance(3) > blockDistance && u.getDistance(4) <= blockDistance)
            return 3;//sub degree;
        else return 0;//stop = false;
    }
    else if(angle > 157.5){ //first area (>157.5 degree)
        if(u.getDistance(3) <= blockDistance && u.getDistance(4) <= blockDistance)
            return 1;//stop = true;
        else if(u.getDistance(3) <= blockDistance && u.getDistance(4) > blockDistance)
            return 2;//add degree;
        else if(u.getDistance(3) > blockDistance && u.getDistance(4) <= blockDistance)
            return 3;//sub degree;
        else return 0;//stop = false;
    }
    else return 1;//stop = true;
}
