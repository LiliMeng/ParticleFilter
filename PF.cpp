#include "PF.h"

PF::PF()
{
    input.open("/home/lili/Journal/wheelchair/Particle_filter_lili/odometry.dat");
    if(input.fail())
    {
        cout<<"cannot open odometry.dat file"<<endl;
    }

    output.open("/home/lili/Journal/wheelchair/Particle_filter_lili/odometryOutput.csv");
    if(output.fail())
    {
       cout<<"cannot open the output files"<<endl;
    }


    for(int i=0; i<num_particles; i++)
    {
        X.push_back(particle(0,0,0, 1.0/num_particles));
    }

    cout<<"the size of X is "<<X.size()<<endl;
}

PF::~PF()
{
   input.close();
   output.close();
}


void PF::run()
{

    while(update_odometry())
    {
        prediction();
        print_particle();
    }

    return;
}


bool PF::update_odometry()
{
    string tmp;
    if(input>>tmp)
    {
        input>>U.r1;
        input>>U.t;
        input>>U.r2;
        return true;
    }
    else
    {
        return false;
    }

}

void PF::prediction()
{
    std::random_device r;
    std::mt19937 gen(r());

    normal_distribution<> dr1(U.r1, pow(noise_r1, 1));
    normal_distribution<> dt(U.t, pow(noise_t, 1));
    normal_distribution<> dr2(U.r2, pow(noise_r2, 1));

    cout<<"no problem with sampling"<<endl;

    for(int i=0; i<num_particles; i++)
    {
        double delta_rot1=dr1(gen);
        double delta_trans= dt(gen);
        double delta_rot2=dr2(gen);

        X[i].x=X[i].x + delta_trans*cos(X[i].theta+delta_rot1);
        X[i].y=X[i].y + delta_trans*sin(X[i].theta+delta_rot1);
        X[i].theta=X[i].theta + delta_rot1 + delta_rot2;
    }


    cout<<"no problem with update"<<endl;

    return;
}

void PF::print_particle()
{
    for(int i=0; i<num_particles; i++)
    {
        output<<X[i].x<<" "<<X[i].y<<" "<<X[i].theta<<endl;
    }

}
