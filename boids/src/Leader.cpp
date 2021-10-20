#include <ros/ros.h>
#include "boids/Leader.h"
// #include <iostream>
// #include <vector>
// #include <string>
// #include <math.h>
// #include "SFML/Graphics.hpp"

// Global Variables for borders()
// desktopTemp gets screen resolution of PC running the program
// sf::VideoMode desktopTemp = sf::VideoMode::getDesktopMode();
// const int window_height = desktopTemp.height;
// const int window_width = desktopTemp.width;

// #define w_height window_height
// #define w_width window_width
// #define PI 3.141592635

// =============================================== //
// ======== Leader Functions from Leader.h =========== //
// =============================================== //

Leader::Leader(float x, float y)
{
    // acceleration = Pvector(0, 0);
    appear = false;
    // velocity = Pvector(rand()%3 - 2, rand()%3 - 2);
    velocity = Pvector(0, 0);
    location = Pvector(x, y);
    // maxSpeed = 3.5;
    // maxForce = 0.5;
    // target = Pvector((int)x, (int)y);
}

//see leader or not
bool Leader::seeOrNot() const{ return this->appear;}

//if see leader return true(have v and l), else return false
void Leader::See(){
    if(velocity.x == 0 && velocity.y == 0 && location.x == 0 && location.y == 0)
        this->appear = false;
    else this->appear = true;
}

//update see leader
void Leader::appear_update(bool see){ this->appear = see;}

//update relative location
void Leader::l_update(const float x, const float y)
{
    location.set(x, y);
}

//update relative velocity
void Leader::v_update(const float x, const float y)
{
    velocity.set(x, y);
}

//update orientation
void Leader::o_update(const float o)
{
    angle = o;
}

Pvector Leader::l_get() const{ return this->location;}

Pvector Leader::v_get() const{ return this->velocity;}

float Leader::o_get() const{ return this->angle;}