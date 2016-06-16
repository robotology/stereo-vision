/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <cmath>
#include <limits>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Rand.h>

#include "eyesCalibration.h"

#define DEG2RAD     (M_PI/180.0)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


/**************************************************************************/
struct Parameters
{
    int    numParticles;
    int    maxIter;
    double maxT;
    double omega;
    double phi_p;
    double phi_g;
    Vector cost;
    Matrix lim;

    /**************************************************************************/
    Parameters() : numParticles(20),
                   maxIter(std::numeric_limits<int>::max()),
                   maxT(std::numeric_limits<double>::infinity()),
                   omega(0.8),
                   phi_p(0.1),
                   phi_g(0.1),
                   cost(2,0.0)
    {
        lim.resize(6,2);
        lim(0,0)=-0.01;         lim(0,1)=0.01;
        lim(1,0)=-0.01;         lim(1,1)=0.01;
        lim(2,0)=-0.01;         lim(2,1)=0.01;
        lim(3,0)=-10.0*DEG2RAD; lim(3,1)=10.0*DEG2RAD;
        lim(4,0)=-10.0*DEG2RAD; lim(4,1)=10.0*DEG2RAD;
        lim(5,0)=-10.0*DEG2RAD; lim(5,1)=10.0*DEG2RAD;
    }
};


/**************************************************************************/
struct Particle
{
    Vector pos;
    Vector vel;
    Vector cost;    
    
    Particle() : pos(6,0.0), vel(6,0.0),
                 cost(2,std::numeric_limits<double>::infinity()) { }

    /**************************************************************************/
    void copy_translation(const Particle &p)
    {
        pos.setSubvector(0,p.pos.subVector(0,2));
        vel.setSubvector(0,p.vel.subVector(0,2));
        cost[0]=p.cost[0];
    }

    /**************************************************************************/
    void copy_orientation(const Particle &p)
    {
        pos.setSubvector(3,p.pos.subVector(3,5));
        vel.setSubvector(3,p.vel.subVector(3,5));
        cost[1]=p.cost[1];
    }
};


/**************************************************************************/
class Optimizer
{
    const std::deque<CalibrationData> &data;
    Parameters parameters;

    std::deque<Particle> x,p;
    Particle g;

    Vector rand_min,rand_max;
    int iter;
    double t,t0;

    /**************************************************************************/
    void randomize_translation()
    {
        for (size_t i=0; i<x.size(); i++)
        {
            Particle &particle=x[i];
            for (size_t i=0; i<3; i++)
            {
                particle.pos[i]=Rand::scalar(parameters.lim(i,0),parameters.lim(i,1));
                particle.vel[i]=Rand::scalar(-1e-5,1e-5);
            }
        }
    }

    /**************************************************************************/
    void randomize_orientation()
    {
        for (size_t i=0; i<x.size(); i++)
        {
            Particle &particle=x[i];
            for (size_t i=3; i<particle.pos.length(); i++)
            {
                particle.pos[i]=Rand::scalar(parameters.lim(i,0),parameters.lim(i,1));
                particle.vel[i]=Rand::scalar(-0.1,0.1)*DEG2RAD;
            }
        }
    }

    /**************************************************************************/
    Vector evaluate(Particle &particle)
    {
        Matrix Hl,Hr;
        getExtrinsics(particle.pos,Hl,Hr);

        particle.cost=0.0;
        if (data.size()>0)
        {
            for (size_t i=0; i<data.size(); i++)
            {
                Matrix Hl_=data[i].eye_kin_left*Hl;
                Matrix Hr_=data[i].eye_kin_right*Hr;
                Matrix F=SE3inv(Hr_)*Hl_;

                particle.cost[0]+=norm(data[i].fundamental.getCol(3).subVector(0,2)-F.getCol(3).subVector(0,2));
                particle.cost[1]+=norm(dcm2rpy(data[i].fundamental)-dcm2rpy(F));
            }
            particle.cost/=data.size();
        }
        
        return particle.cost;
    }

    /**************************************************************************/
    void print(const bool randomize_print=false)
    {
        ostringstream str;
        str<<"iter #"<<iter<<" t="<<setprecision(3)<<fixed<<t<<" [s]: ";
        str.unsetf(ios::floatfield);
        str<<"cost=["<<g.cost.toString(5,5)<<"] (["<<parameters.cost.toString(5,5)<<"]); ";
        if (randomize_print)
            str<<"particles scattered away";
        yInfo()<<str.str();
    }

public:
    /**************************************************************************/
    Optimizer(const std::deque<CalibrationData> &data_) : data(data_)
    {
        rand_min.resize(6,0.0);
        rand_max.resize(6,1.0);
    }

    /**************************************************************************/
    Parameters &getParameters()
    {
        return parameters;
    }

    /**************************************************************************/
    bool getExtrinsics(const Vector &x, Matrix &Hl, Matrix &Hr)
    {
        if (x.length()>=6)
        {
            Hr=rpy2dcm(x.subVector(3,5)); 
            Hr.setSubcol(x.subVector(0,2),0,3);

            Vector y=-1.0*x;
            Hl=rpy2dcm(y.subVector(3,5));
            Hl.setSubcol(y.subVector(0,2),0,3);
            return true;
        }
        else
            return false;
    }

    /**************************************************************************/
    void init()
    {
        x.assign(parameters.numParticles,Particle());
        randomize_translation();
        randomize_orientation();
        p=x;
        
        for (size_t i=0; i<x.size(); i++)
        {
            Vector f=evaluate(p[i]);
            if (f[0]<g.cost[0])
                g.copy_translation(p[i]);
            if (f[1]<g.cost[1])
                g.copy_orientation(p[i]);
        }
        
        iter=0;
        t0=Time::now();
        t=0.0;
    }

    /**************************************************************************/
    bool step()
    {
        iter++;
        for (size_t i=0; i<x.size(); i++)
        {
            Vector r1=Rand::vector(rand_min,rand_max);
            Vector r2=Rand::vector(rand_min,rand_max);
            
            x[i].vel=parameters.omega*x[i].vel+
                     parameters.phi_p*r1*(p[i].pos-x[i].pos)+
                     parameters.phi_g*r2*(g.pos-x[i].pos);
            
            x[i].pos+=x[i].vel;
            for (size_t j=0; j<x[i].pos.length(); j++)
                x[i].pos[j]=std::min(std::max(x[i].pos[j],parameters.lim(j,0)),parameters.lim(j,1));
            
            Vector f=evaluate(x[i]);
            if (f[0]<p[i].cost[0])
            {
                p[i].copy_translation(x[i]);
                p[i].cost[0]=f[0];
                if (f[0]<g.cost[0])
                    g.copy_translation(p[i]); 
            }
            if (f[1]<p[i].cost[1])
            {
                p[i].copy_orientation(x[i]);
                p[i].cost[1]=f[1];
                if (f[1]<g.cost[1])
                    g.copy_orientation(p[i]); 
            }
        }
        
        bool randomize_print=false;
        if ((iter%100)==0)
        {
            double mean;

            mean=0.0;
            if (x.size()>0)
            {
                for (size_t i=0; i<x.size(); i++)
                    mean+=norm(g.pos.subVector(0,2)-x[i].pos.subVector(0,2));
                mean/=x.size();
            }
            if (mean<5e-4)
            {
                randomize_translation();
                randomize_print=true;
            }

            mean=0.0;
            if (x.size()>0)
            {
                for (size_t i=0; i<x.size(); i++)
                    mean+=norm(g.pos.subVector(3,5)-x[i].pos.subVector(3,5));
                mean/=x.size();
            }
            if (mean<5e-8)
            {
                randomize_orientation();
                randomize_print=true;
            }
        }
        
        t=Time::now()-t0;
        bool cont=(iter<parameters.maxIter) && (t<parameters.maxT) &&
                  ((g.cost[0]>parameters.cost[0]) || (g.cost[1]>parameters.cost[1]));
        
        if ((iter%10)==0)
            print(randomize_print);
        
        return cont;
    }

    /**************************************************************************/
    const Particle &finalize()
    {
        print();
        return g;
    }
};


/**************************************************************************/
CalibrationData &EyesCalibration::addData()
{
    CalibrationData d;
    data.push_back(d);
    return data.back();
}


/**************************************************************************/
Vector EyesCalibration::calibrate(Matrix &extrinsics_left,
                                  Matrix &extrinsics_right,
                                  const string &logFile)
{
    Rand::init();

    Optimizer swarm(data);
    swarm.getParameters().maxT=10.0;
    swarm.init();

    int cnt=0;
    double t0=Time::now();
    while (swarm.step())
    {
        if (++cnt>=10)
        {
            Time::yield();
            cnt=0;
        }
    }
    double t=Time::now()-t0;

    const Particle &g=swarm.finalize();    
    yInfo()<<"solution: ["<<g.pos.toString(5,5)<<"] "
           <<"found in "<<t<<" [s]";

    swarm.getExtrinsics(g.pos,extrinsics_left,extrinsics_right);

    yInfo()<<"Saving results in "<<logFile;
    ofstream fout;
    fout.open(logFile.c_str());
    if (fout.is_open())
    {
        for (size_t i=0; i<data.size(); i++)
        {
            Matrix Hl_=data[i].eye_kin_left*extrinsics_left;
            Matrix Hr_=data[i].eye_kin_right*extrinsics_right;
            Matrix F=SE3inv(Hr_)*Hl_;

            fout<<"[data_"<<i<<"]"<<endl;
            fout<<"vergence="<<data[i].vergence<<" [deg]"<<endl;
            fout<<"measured fundamental matrix"<<endl;
            fout<<data[i].fundamental.toString(5,5)<<endl;
            fout<<"estimate fundamental matrix"<<endl;
            fout<<F.toString(5,5)<<endl;
            fout<<endl;
        }
        fout.close();
    }

    return g.cost;
}


