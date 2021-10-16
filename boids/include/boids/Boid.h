#ifndef BOID_H_
#define BOID_H_

#include <ros/ros.h>
#include "boids/Pvector.h"
#include "boids/Leader.h"
#include "boids/Ultrasonic.h"
#include <vector>
#include <stdlib.h>
#include <iostream>
// The Boid Class
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

class Boid {
private:
    Pvector location;
    Pvector velocity;
    Pvector acceleration;
    float maxSpeed;
    float maxForce;
    float maxRange;
    bool stop;
    int blockDistance;
    int desiredseparation;
    int Ultraseparation;
    int slowDownDistance;
    float angle;
    float angletf;

public:
    Boid() {}
    Boid(float x, float y);
    // Boid(float lx, float ly, float vx, float vy);
    void applyForce(const Pvector& force);
    void l_update(const float x, const float y);
    void v_update(const float x, const float y);
    Pvector l_get() const;
    Pvector v_get() const;
    Pvector a_get() const;
    // Three Laws that boids follow
    Pvector Separation(const vector<Boid>& Boids, const Leader& lead, const Ultrasonic& ultra);
    Pvector Alignment(const vector<Boid>& Boids, const Leader& lead);
    Pvector Cohesion(const vector<Boid>& Boids, const Leader& lead);
    //Leader follower
    Pvector FollowLeader(const Leader& lead);
    //Functions involving SFML and visualisation linking
    Pvector seek(const Pvector& v);
    void run(const vector<Boid>& v, const Leader& l, const Ultrasonic& u);
    void runL(const vector<Boid>& v, const Leader& l, const Ultrasonic& u);
    void update(const vector <Boid>& v, const Leader& l);
    void flock(const vector<Boid>& v, const Leader& l, const Ultrasonic& u);
    void flockL(const vector<Boid>& v, const Leader& l, const Ultrasonic& u);
    void Angle();
    float Angletf();
    // float getangle();
    bool See();
    int beBlocked(const Ultrasonic& u);

};

#endif
