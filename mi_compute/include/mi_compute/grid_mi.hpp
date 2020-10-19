#pragma once

#include <vector>
#include <array>
#include <cmath>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <nav_msgs/OccupancyGrid.h>

#include <mi_compute/ray_cast.hpp>

namespace MiCompute{
    //对于栅格地图操作的类
    class GirdMI{
        public:
            GirdMI(const unsigned int& mapWidth,const unsigned int& mapHeight,const std::vector<double>& map):
            mapWidth_(mapWidth),
            mapHeight_(mapHeight),
            occ_th_(0.9),
            free_th_(0.2),
            log_occ_(2),//占据更新为0.9
            log_free_(-1.3),
            p_max_(99),
            p_min_(1),
            rOcc_(1.5),
            rEmp_(0.66)
            {
                map_ = std::vector<double>(mapWidth_*mapHeight_);
                map_ = map;

                ratioMap_ = GridValueConvert();
                
            };

            //将占据栅格值转换为乘积形式
            std::vector<double> GridValueConvert(){
                
                ratioMap_.clear();
                for(int i = 0;i < map_.size();++i){
                    
                    int tmp = map_[i];

                    if(tmp == -1.0){
                        tmp = 50.0;

                    }
                    tmp = tmp - 50.0;

                    if(tmp < -50.0 ||tmp > 50.0){

                        ROS_ERROR("map data error,convert failed");
                        //return;
                    }

                    double value = std::pow(2,tmp/2.5);
                    ratioMap_.push_back(value);
                }
                return ratioMap_;
            };

            std::vector<double> Convert2GridValue(){
                
                std::vector<double> mapData;
                mapData.clear();

                for(int i = 0;i<ratioMap_.size();i++){
                    
                    double tmp = ratioMap_[i];

                    double value = 2.5*std::log2(tmp);
                    value = value + 50.0;

                    
                    mapData.push_back(value);
                }

                return mapData;
            };

            std::vector<double> GetRkInLine(const Eigen::Vector2i& initalPose,const double& theta,const double& range,std::vector<Eigen::Vector2i>& line){
                
                Rk_.clear();

                std::vector<Eigen::Vector2i> tmp_line;

                tmp_line = DrawLine(initalPose,theta,range);

                for(int i = 0;i < tmp_line.size();i++){
                    
                    int x = tmp_line[i][0];
                    int y = tmp_line[i][1];

                    int index = GetIndex(x,y);//如果index在地图外面，会报错。

                    if(index == -1){
                        break;
                    }

                    line.push_back(tmp_line[i]);

                    Rk_.push_back(ratioMap_[index]);

                    //std::cout<<"index"<<index<<std::endl;

                    //std::cout<<"ratioMap_[index]"<<ratioMap_[index]<<std::endl;
                }

                return Rk_;

            };

            //调用raycast函数，求出线经过的栅格
            std::vector<Eigen::Vector2i> DrawLine(const Eigen::Vector2i& initalPose,const double& theta,const double& range){
                
                std::vector<Eigen::Vector2i> line;
                
                line = MiCompute::RayCast::LinearGrid(initalPose,theta,range);

                return line;

            };

            //function： 利用光束对栅格地图进行更新
            //param：line 里面包含了x,y信息
            //output：返回地图的栅格地图值
            std::vector<double> BeamUpdateGridMap(const std::vector<Eigen::Vector2i>& line){
                
                Eigen::Vector2i tmp;

                std::vector<double> newMap;
                newMap = std::vector<double>(mapWidth_*mapHeight_);
                newMap = map_;

                bool occ = false;

                for(int i = 0;i<line.size();i++){
                    tmp = line[i];

                    //得到该点的概率值
                    if(GetIndex(tmp[0],tmp[1]) == -1){
                        break;
                    }

                    double p = map_[GetIndex(tmp[0],tmp[1])];
                    
                    if(i != (line.size() - 1)){occ = false;}
                    else{occ = true;}

                    newMap[GetIndex(tmp[0],tmp[1])] = UpdateOneCell(p,occ);

                }
                return newMap;

            };


            //function： 根据占据与否更新一个栅格的占据值
            //param：p：栅格本身的值 occ：是否占据
            //output：返回一个栅格的占据值
            double UpdateOneCell(const double& p,const bool& occ){
                if(occ == true){
                    
                    if(p >= p_max_) {
                        return p;
                    }

                    return (p+log_occ_);

                }
                else if(occ == false){
                    if(p <= p_min_) return p;
                    return (p+log_free_);
                }
            };

            //function：得到栅格在地图中的索引
            //param：x：栅格地图的x y：栅格地图的y
            //output：返回栅格地图在vector中的索引
            int GetIndex(const int& x,const int& y){
                if(x < 0 or x >= mapHeight_ or y < 0 or y >= mapWidth_){
                    return -1;
                }
                return x*mapWidth_+y;
            };

            void reset(const unsigned int mapWidth,const unsigned int mapHeight,const std::vector<double>& map){
                mapWidth_ = mapWidth;
                mapHeight_ = mapHeight;
                map_.clear();
                map_ = std::vector<double>(mapWidth_*mapHeight_);
                map_ = map;
                ratioMap_.clear();
                Rk_.clear();

                ratioMap_ = GridValueConvert();
            };

            double GetRatioMapValue(int index){

                if(index > ratioMap_.size()){
                    ROS_ERROR("get error ratiomap index");
                    return 0.0;
                }
                double value = ratioMap_[index];

                return value;
            }

        private:

        unsigned int mapWidth_;
        unsigned int mapHeight_;

        std::vector<double> map_;
        std::vector<double> ratioMap_;
        std::vector<double> Rk_;

        double occ_th_;
        double free_th_;

        double log_occ_;
        double log_free_;

        double p_max_;
        double p_min_;

        //论文中给出的更新值
        double rOcc_;
        double rEmp_;
        
    };

}