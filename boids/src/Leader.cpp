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

Pvector Leader::l_get() const{ return this->location;}

Pvector Leader::v_get() const{ return this->velocity;}

// Adds force Pvector to current force Pvector
// void Leader::applyForce(const Pvector& force)
// {
//     acceleration.addVector(force);
// }

// //approach goal
// Pvector Leader::Approach(const Pvector& tar)
// {
//     //arrived goal, stop
//     if((tar.x == (int)location.x) && (tar.y == (int)location.y)){
//         Pvector temp(0,0);
//         return temp;
//     }
//     else{ //approach goal
//         Pvector sum(0, 0);
//         sum.addVector(tar);
//         return seek(sum);
//     }
// }
// Pvector Leader::Separation(const Pvector& tar)
// {
//     // Distance of field of vision for separation between boids
//     float desiredseparation = 50;
//     Pvector steer(0, 0);
//     // For every boid in the system, check if it's too close
//     float d = location.distance(tar);
//     // If this is a fellow boid and it's too close, move away from it
//     if ((d > 0) && (d < desiredseparation)) {
//         // diff = diff.subTwoVector(location, boids[i].location);
//         steer.subTwoVector(location, tar);
//         steer.normalize();
//         steer.divScalar(d);      // Weight by distance
//     }
//     // Adds average difference of location to acceleration
//     if (steer.magnitude() > 0) {
//         // Steering = Desired - Velocity
//         // steer.normalize();
//         steer.mulScalar(maxSpeed);
//         steer.subVector(velocity);
//         // steer.limit(maxForce);
//     }
//     // cout << steer.x << ", " << steer.y <<endl;
//     return steer;
// }

// // Limits the maxSpeed, finds necessary steering force and
// // normalizes vectors
// Pvector Leader::seek(const Pvector& v)
// {
//     Pvector desired(location.x, location.y);
//     desired.subVector(v);  // A vector pointing from the location to the target
//     // Normalize desired and scale to maximum speed
//     desired.normalize();
//     desired.mulScalar(maxSpeed);
//     // cout << "desired : " << desired.x << ", " << desired.y <<endl;
//     // cout << "v : " << velocity.x << ", " << velocity.y <<endl;
//     // Steering = Desired minus Velocity
//     acceleration.subTwoVector(velocity, desired);
//     // acceleration.limit(maxForce);  // Limit to maximum steering force
//     // cout << acceleration.x << ", " << acceleration.y <<endl;
//     return acceleration;
// }

// // Modifies velocity, location, and resets acceleration with values that
// // are given by the three laws.
// void Leader::update()
// {
//     int dis = location.distance(target);
//     if(dis == 0){
//         velocity.set(0, 0);
//     }
//     else{
//         //To make the slow down not as abrupt
//         acceleration.mulScalar(0.4);
//         // cout << acceleration.x << ", " << acceleration.y <<endl;
//         // Update velocity
//         velocity.addVector(acceleration);
//             // cout << "v : " << velocity.x << ", " << velocity.y <<endl;
//         // Limit speed
//         velocity.limit(maxSpeed);
//             // cout << "v : " << velocity.x << ", " << velocity.y <<endl;
//         location.addVector(velocity);
//         // Reset accelertion to 0 each cycle
//         acceleration.mulScalar(0);        
//     }

// }

// // Run flock() on the flock of boids.
// // This applies the three rules, modifies velocities accordingly, updates data,
// // and corrects boids which are sitting outside of the SFML window
// void Leader::run()//const vector <Boid>& v)
// {
//     flock();
//     update();
//     // borders();
// }

// // Applies the three laws to the flock of boids
// void Leader::flock()//const vector<Boid>& v)
// {
//     Pvector app = Approach(target);
//     Pvector sep = Separation(target);
//     if(app.magnitude() == 0){//arrived
//         // Arbitrarily weight these forces
//         app.mulScalar(1.0);
//         sep.mulScalar(0.5);
//     }
//     // Add the force vectors to acceleration
//     applyForce(app);
//     applyForce(sep);
// }

// // Checks if boids go out of the window and if so, wraps them around to
// // the other side.
// // void Leader::borders()
// // {
// //     // if (location.x < 0)    location.x += w_width;
// //     if (location.x < 0)    location.x = 0;
// //     // if (location.y < 0)    location.y += w_height;
// //     if (location.y < 0)    location.y = 0;
// //     // if (location.x > 1000) location.x -= w_width;
// //     if (location.x > w_width) location.x = w_width;
// //     // if (location.y > 1000) location.y -= w_height;
// //     if (location.y > w_height) location.y = w_height;
// //     // cout << "window w ; " << w_width << ", h ; " << w_height << endl;
// //     // cout << location.x << ", " << location.y <<endl;
// // }

// // Calculates the angle for the velocity of a boid which allows the visual
// // image to rotate in the direction that it is going in.
// float Leader::angle(const Pvector& v)
// {
//     // From the definition of the dot product
//     float angle = (float)(atan2(v.x, -v.y) * 180 / PI);
//     return angle;
// }

// //set new target
// void Leader::setTarget(int x, int y)
// {
//     target.set(x, y);
// }