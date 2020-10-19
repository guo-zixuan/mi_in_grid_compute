#pragma once

#include <random>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <limits>

#include <ros/ros.h>

namespace MiCompute{

    class BeamSensorModel{
        
    public:
        BeamSensorModel(const double& zMaxRange):
        zHit_(0.7),
        zMax_(0.1),
        zRand_(0.1),
        zShort_(0.1),
        lambda_(5.0),
        sigma_(0.03),
        zMaxRange_(zMaxRange){};
        ~BeamSensorModel()=default; 
        
        double ComputeSensorModelP(const double& zExp,const double& z){
            
            if(z < 0.0 or z > zMaxRange_){
                ROS_ERROR("z out of range");
                return 0.0;
            }
            else{
                
                double p = zHit_ * HitModel(zExp,z)+
                           zShort_ * ShortModel(zExp,z)+
                           zRand_ * RandModel()+
                           zMax_ * MaxModel(z);

                return p;

            }

        };


        double HitModel(const double& zExp,const double& z){

            double sigma = std::max<double>(sigma_*zExp,sigma_);

            //std::normal_distribution<double> norm(zExp,sigma);
            
            double p = GaussRand(zExp,sigma,z);

            return p;
        };

        double ShortModel(const double& zExp,const double& z){

            double p = 0.0;

            if(z >= zExp ){
                return p;
            }

            p = lambda_*std::exp(-lambda_*z);
            return p;
        };

        double RandModel(){
            
            double p = 0.0;

            p = 1.0/zMaxRange_;

            return p;
        };

        double MaxModel(const double& z){

            if(z == zMaxRange_)
            return 1.0/zShort_;
            else
            return 0.0;
        };

        double GaussRand(const double& dExpect,const double& SVariance,const double& z)
        {
            double denominator = 1.0/(std::sqrt(2*M_PI)*SVariance);

            double molecule = std::exp(-(z - dExpect)*(z - dExpect) / (2*SVariance*SVariance));

            return denominator*molecule;
        };

    private:
        
        double zHit_;//hit model
        double zMax_;//max model
        double zRand_;//rand model
        double zShort_;//short model

        double lambda_;//unexpected obstacle  （exponential distribution）
        double sigma_;//hit model (standard deviation)

        double zMaxRange_;

    };



}