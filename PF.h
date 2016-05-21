#ifndef PF_H
#define PF_H

#include <iostream>
#include <vector>
#include <fstream>
#include <random>
#include <math.h>

using namespace std;

class PF
{
public:
    PF();
    virtual ~PF();
    void run();

private:

    struct odometry
    {
        double r1, r2, t;
    };

    struct particle
    {
        double x;
        double y;
        double theta;
        double weight;
        //particle *history;

        particle(double posex, double posey, double poset, double w)
        {
            x=posex;
            y=posey;
            theta=poset;
            weight=w;

        }
    };

    double noise_r1 = 0.5;
    double noise_t = 0.0001;
    double noise_r2 = 0.5;

    int num_particles = 100;

    vector<particle> X;
    odometry U;

    bool update_odometry();
    void prediction();
    void print_particle();


    fstream input;
    ofstream output;


};

#endif // PF_H
