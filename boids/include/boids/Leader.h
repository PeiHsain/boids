#ifndef LEADER_H_
#define LEADER_H_

#include <ros/ros.h>
#include "boids/Pvector.h"
// #include <vector>
// #include <stdlib.h>
// #include <iostream>
// The Leader Class
//
// Attributes
//  bool predator: flag that specifies whether a given boid is a predator.
//  Pvector location: Vector that specifies a boid's location.
//  Pvector velocity: Vector that specifies a boid's current velocity.
//  Pvector acceleration: Vector that specifies a boid's current acceleration.
//  float maxSpeed: Limits magnitude of velocity vector.
//  float maxForce: Limits magnitude of acceleration vector. (F = m*a!)
//
// Methods
//  applyForce(Pvector force): Adds the given vector to acceleration
//
//  Pvector Separation(vector<Boid> Boids): If any other boids are within a
//      given distance, Separation computes a a vector that distances the
//      current boid from the boids that are too close.
//
//  Pvector Alignment(vector<Boid> Boids): Computes a vector that causes the
//      velocity of the current boid to match that of boids that are nearby.
//
//  Pvector Cohesion(vector<Boid> Boids): Computes a vector that causes the
//      current boid to seek the center of mass of nearby boids.

class Leader {
private:
    // Pvector target;
    bool appear;
    //relative
    Pvector location;
    Pvector velocity;
    float angle;  
public:
    // Pvector acceleration;
    // float maxSpeed;
    // float maxForce;
    Leader() {}
    Leader(float x, float y);
    bool seeOrNot() const;
    void See();
    void appear_update(bool see);
    void l_update(const float x, const float y);
    void v_update(const float x, const float y);
    Pvector l_get() const;
    Pvector v_get() const; 
    //add force
    // void applyForce(const Pvector& force);
    // //approach goal
    // Pvector Approach(const Pvector& tar);
    // Pvector Separation(const Pvector& tar);
    // //Functions involving SFML and visualisation linking
    // Pvector seek(const Pvector& v);
    // void run();//const vector<Boid>& v);
    // void update();
    // void flock();//const vector<Boid>& v);
    // // void borders();
    // float angle(const Pvector& v);
    // void setTarget(int x, int y);
};

#endif
