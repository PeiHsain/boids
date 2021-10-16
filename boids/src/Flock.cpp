#include <ros/ros.h>
#include "boids/Flock.h"
#include "boids/Boid.h"

// =============================================== //
// ======== Flock Functions from Flock.h ========= //
// =============================================== //

int Flock::getSize()
{
    return flock.size();
}

Boid Flock::getBoid(int i)
{
    return flock[i];
}

void Flock::addBoid(const Boid& b)
{
    flock.push_back(std::move(b));
}

// Runs the run function for every boid in the flock checking against the flock
// itself. Which in turn applies all the rules to the flock.
// void Flock::flocking(const Leader& l) 
// {
//     for (int i = 0; i < flock.size(); i++)
//         flock[i].run(flock, l);
// }