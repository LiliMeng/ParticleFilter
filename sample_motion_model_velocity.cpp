#include <iostream>
//#include "motion_model_velocity.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <random>
#include <math.h>

using namespace std;

using namespace std;

struct particle
{
        double x;
        double y;
        double theta;
        double weight;


        particle(double posex, double posey, double poset, double w)
        {
            x=posex;
            y=posey;
            theta=poset;
            weight=w;

        }
};



void motion_model_velocity_update(double linear_vel, double angular_vel, vector<particle> pose, vector<particle> &new_pose, double time_interval, int num_particles)
{

    double alpha1 = 0.000001;
    double alpha2 = 0.000001;
    double alpha3 = 0.000001;
    double alpha4 = 0.000001;
    double alpha5 = 0.000001;
    double alpha6 = 0.000001;

    std::random_device r;
    std::mt19937 gen(r());

    normal_distribution<> sample1(0, alpha1*pow(linear_vel,2)+alpha2*pow(angular_vel,2));
    normal_distribution<> sample2(0, alpha3*pow(linear_vel,2)+alpha4*pow(angular_vel,2));
    normal_distribution<> sample3(0, alpha5*pow(linear_vel,2)+alpha6*pow(angular_vel,2));

    cout<<"no problem with sampling"<<endl;

    double new_linear_vel=linear_vel+sample1(gen);
    double new_angualr_vel=angular_vel+sample2(gen);
    double gamma=sample3(gen);

    for(int i=0; i<num_particles; i++)
    {
        double new_linear_vel=linear_vel+sample1(gen);
        double new_angular_vel=angular_vel+sample2(gen);
        double gamma=sample3(gen);


        new_pose[i].x=pose[i].x - new_linear_vel/new_angular_vel*sin(pose[i].theta) + new_linear_vel/new_angular_vel*sin(pose[i].theta+new_angular_vel*time_interval);
        new_pose[i].y=pose[i].y + new_linear_vel/new_angular_vel*cos(pose[i].theta) + new_linear_vel/new_angular_vel*cos(pose[i].theta+new_angular_vel*time_interval);
        new_pose[i].theta=pose[i].theta + new_angular_vel*time_interval + gamma*time_interval;
        cout<<new_pose[i].x<<" "<<new_pose[i].y<<" "<<new_pose[i].theta<<endl;
    }

}


int main()
{
    ofstream fout("/home/lci/workspace/odometry/data_nn_pwc/output_motion/pose.txt");

    vector<particle> pose, new_pose;

    double num_particles=100;

    for(int i=0; i<num_particles; i++)
    {
        pose.push_back(particle(0,0,0, 1.0/num_particles));
    }

    for(int i=0; i<num_particles; i++)
    {
        new_pose.push_back(particle(0,0,0, 1.0/num_particles));
    }


    double time_interval=1;

    double linear_vel=2;
    double angular_vel=0;

    vector<vector<particle>> new_pose_vec;

    for(int i=0; i<10; i++)
    {
         motion_model_velocity_update(linear_vel, angular_vel, pose, new_pose, time_interval, num_particles);

         new_pose_vec.push_back(new_pose);

        for(int j=0; j<num_particles; j++)
        {
            cout<<new_pose[j].x<<" "<<new_pose[j].y<<" "<<new_pose[j].theta<<endl;
            fout<<new_pose[j].x<<" "<<new_pose[j].y<<" "<<new_pose[j].theta<<endl;

        }

        pose[i+1].x=new_pose[i].x;
        pose[i+1].y=new_pose[i].y;
        pose[i+1].theta=new_pose[i].theta;

        new_pose[i].x=0;
        new_pose[i].y=0;
        new_pose[i].theta=0;

    }

    return 0;
}
