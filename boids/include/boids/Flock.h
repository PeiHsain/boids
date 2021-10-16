#ifndef FLOCK_H_
#define FLOCK_H_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "boids/Boid.h"
#include "boids/Leader.h"
// Brief description of Flock Class:
// This file contains the class needed to create a flock of boids. It utilizes
// the boids class and initializes boid flocks with parameters that can be
// specified. This class will be utilized in main.

class Flock {
public:
    vector<Boid> flock;
    //Constructors
    Flock() {}
    // Accessor functions
    int getSize();
    Boid getBoid(int i);
    // Mutator Functions
    void addBoid(const Boid& b);
    // void flocking(const Leader& l);
};

#endif
