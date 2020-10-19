#pragma once
#include <vector>
#include <mi_compute/grid_mi.hpp>

namespace MiCompute{
    class ComputeMI{
        public:
            ComputeMI();
            ~ComputeMI();

            static void ComputeMIEta(const std::vector<double>& rk,std::vector<double>& etaVector){
                
                etaVector.clear();

                double eta = 0.0;
                double last_eta = 0.0;
                
                for(int i = 0;i <rk.size();i++){

                    eta = rk[i]+1;
                    
                    if(i == 0 ){
                        etaVector.push_back(eta);
                    }
                    else if( i != 0){
                        etaVector.push_back( eta*etaVector[i-1]);
                    }
                }
            };
            //function: Alorithm3 ComputeMeasurementPrior 
            //param: *girdMI  get  GetRatioMapValue 
            //param: line get I[j]
            //param: etaVector  get etavector[i]
            //param: z measurement 
            //param: z max  M=0 就意味者 ei = Zmax
            static double ComputeMeasurementPrior(MiCompute::GirdMI*  girdMI,
                                                const std::vector<Eigen::Vector2i>& line,
                                                const std::vector<double>& etaVector,
                                                const double& z,
                                                const double& zMax){
                BeamSensorModel b(zMax);

                //M = 0 的概率分布认为是zMAX_ 姑且认为是6.0
                double p = b.ComputeSensorModelP(zMax,z) / etaVector[etaVector.size()-1];

                for(int i = 0;i < line.size();i++){
                    int x = line[i][0];
                    int y = line[i][1];

                    double r = girdMI->GetRatioMapValue(girdMI->GetIndex(x,y));
                    //(girdMI->GetIndex(x,y));

                    p = p + r * b.ComputeSensorModelP(i+1,z) / etaVector[i];

                }
                return p;
            };


            //计算一束光线生成的MI
            static double ComputeBeamMI(MiCompute::GirdMI*  girdMI,
                                        const std::vector<Eigen::Vector2i>& line,
                                        const std::vector<double>& etaVector,
                                        const double& zMax = 6.0){

                double MI;

                //0到6认为是z观测的格子
                for(int i = 0;i<= zMax;i++){
                    
                    double Pocc = 0.0;
                    double Pemp = 0.0;

                    if(i == 0){
                        Pocc = Pocc + ComputeMeasurementPrior(girdMI,line,etaVector,i,zMax);
                    }
                    else{
                        for(int j =0;j<i;j++){
                            Pemp = Pemp + ComputeMeasurementPrior(girdMI,line,etaVector,j,zMax);
                        }
                        Pocc = Pocc + ComputeMeasurementPrior(girdMI,line,etaVector,i,zMax);
                    }

                    int x = line[i][0];
                    int y = line[i][1];

                    double r = girdMI->GetRatioMapValue(girdMI->GetIndex(x,y));

                    MI = MI + Pocc*InformationGainFunction(true,r) + Pemp*InformationGainFunction(false,r); 
                
                }
                
                return MI;

            };

            //如果r是0
            //tmp_occ = 0
            //tmp_emp = 0
            //如果r是1
            //tmp_occ = 0.0304784
            //tmp_emp = 0.0251001
            //如果r是占据的
            //tmp_occ = 0.000205648
            //tmp_emp = 0.000113435
            static double InformationGainFunction(const bool& occ, const double& r){
                
                double tmp;
                if(occ == true){
                    tmp = std::log2((r+1)/(r+1.0/1.5)) - 0.584963/(1.5*r+1);

                }
                else if(occ == false){
                    tmp = std::log2((r+1)/(r+1.0/0.66)) - (-0.599462)/(0.66*r+1);
                }
                
                //std::cout<<"tmp: "<<tmp<<std::endl;
                return tmp;

            };


            
        private:


    };
}
