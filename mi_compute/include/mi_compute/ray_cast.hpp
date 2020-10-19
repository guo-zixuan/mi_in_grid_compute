#pragma once

#include <vector>
#include <array>
#include <cmath>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

namespace MiCompute{
    class RayCast{
        public:
            
            RayCast(){};
            
            ~RayCast();

            RayCast(const std::vector<double>& map,const double& length,const double& theta):
            length_(length),
            map_(map),
            theta_(theta)
            {};
            
            void RayCastCompute(){
                
            };

            //theta以（1,0）为0初始
            static std::vector<Eigen::Vector2i> LinearGrid(const Eigen::Vector2i& initalPoint,const double& theta,const double& range){
                
                if(theta < 0 ||theta > 2*M_PI){
                    ROS_ERROR("input theta ERROR");
                }
                
                int x1 = initalPoint[0];
                int y1 = initalPoint[1];

                double sinTheta = std::sin(theta);
                double cosTheta = std::cos(theta);

                int dx = std::round(range * cosTheta);
                int dy = std::round(range * sinTheta);

                int x2 = x1 + dx;
                int y2 = y1 + dy;

                Eigen::Vector2i finalPoint;
                finalPoint<<x2,y2;

                return LinearGrid(initalPoint,finalPoint);

            }
            
            static std::vector<Eigen::Vector2i> LinearGrid(const Eigen::Vector2i& initalPoint,const Eigen::Vector2i& endPoint){

                std::vector<Eigen::Vector2i> line;

                Eigen::Vector2i tmp;

                int x2 = endPoint[0];
                int y2 = endPoint[1];
                int x1 = initalPoint[0];
                int y1 = initalPoint[1];

                tmp = initalPoint;

                line.push_back(tmp);
                
                int dx = abs( x2 - x1 );
                int dy = abs( y2 - y1 );
                int x = x1;
                int y = y1;
                int sx = x2 > x1 ? 1 : -1;
                int sy = y2 > y1 ? 1 : -1;
            
                if (dx > dy)
                {
                    int e = -dx;
                    for (int i = 0; i < dx ; i ++)
                    {
                        x += sx ;
                        e += 2 * dy ;
                        if (e >= 0)
                        {
                            y += sy ;
                            e -= 2 * dx ;
                        }
                        tmp<<x,y;
                        line.push_back(tmp);
                    }
                }
                else
                {
                    int e = -dy;
                    for (int i = 0; i < dy ; i ++)
                    {
                        y += sy ;
                        e += 2 * dx ;
                        if (e >= 0)
                        {
                            x += sx ;
                            e -= 2 * dy ;
                        }
                        tmp<<x,y;
                        line.push_back(tmp);
                    }
                }
                return line;
            };


        private:
            double length_;//光线投影的最大长度
            double theta_;//光线投影的角度
            Eigen::Vector2d origin_point_;//光线投影的起始点
            Eigen::Vector2d final_point_;//光线投影的终止点
            std::vector<double> map_;//读入的栅格地图值
            std::vector<Eigen::Vector2i> line_;//存储一条直线，保存该线上的x,y信息

    };
}