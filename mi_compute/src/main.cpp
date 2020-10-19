#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>

#include <mi_compute/ray_cast.hpp>
#include <mi_compute/grid_mi.hpp>
#include <mi_compute/beam_sensor_model.hpp>
#include <mi_compute/compute_mi.hpp>

#include <mi_compute/MIGrid.h>

struct MIInMapStruct{
    int x;
    int y;
    double value;
};

class MICompute{
public:

    MICompute():
    beamNumber_(20),
    range_(15.0){
        nh_ = ros::NodeHandle();
        mapSub_ = nh_.subscribe("/map", 1, &MICompute::map_callback, this);
        click_sub_ = nh_.subscribe("/clicked_point", 1, &MICompute::click_callback, this);
        mapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/new_map",1,true);
        MIMapPub_ = nh_.advertise<mi_compute::MIGrid>("/MI_map",1,true);

    }

    ~MICompute(){}

    //主循环
    void map_callback(const nav_msgs::OccupancyGrid& map_msg) {

        mapWidth_ = map_msg.info.width;
        mapHeight_ = map_msg.info.height;
        mapResolution_ = map_msg.info.resolution;

        unsigned int mapSize = map_msg.data.size();
        mapData_ = std::vector<double>(mapSize);
        newMapData_ = std::vector<double>(mapSize);

        std::cout<<"mapWidth_: "<<mapWidth_<<"mapHeight_: "<<mapHeight_<<std::endl;
        //宽为70,长为90,长是x，宽是y
        
        //for test
        if(mapSize != mapWidth_*mapHeight_){
            ROS_WARN("map data is not right");
            return;
        }

        for(int i=0;i < mapSize;i++){
            mapData_[i] = static_cast<double>(map_msg.data[i]);
        }

        girdMI_ = new MiCompute::GirdMI(mapWidth_,mapHeight_,mapData_);

        std::vector<double> Rk;
        std::vector<Eigen::Vector2i> line;

        std::vector<double> etaVector;
        std::vector<double> MIVector;

        MIVector.clear();


        for(int i = 0;i < mapWidth_;i++){//y

            int y = i;

            for(int j = 0;j < mapHeight_;j++){//x

                int x = j;
                
                Eigen::Vector2i startPoint(x,y);

                double MI = 0.0;

                //认为不是free的，跳出
                if(girdMI_->GetRatioMapValue(girdMI_->GetIndex(x,y)) >= 0.5){

                    MIVector.push_back(0.0);
                    ROS_INFO("point: %d,%d  MI: %lf",x,y,MI);
                    
                    continue;
                }

                for(int k=0;k <= beamNumber_;k++){

                    double theta = k*(360.0/beamNumber_)*M_PI/180.0;

                    Rk.clear();
                    etaVector.clear();
                    line.clear();

                    Rk = girdMI_->GetRkInLine(startPoint,theta,range_,line);

                    MiCompute::ComputeMI::ComputeMIEta(Rk,etaVector);

                    MI = MI + MiCompute::ComputeMI::ComputeBeamMI(girdMI_,line,etaVector,range_);
                    // BeamRayCast(startPoint,theta,range_);
                    //std::cout<<"MI_step"<<MI<<std::endl;
                }
                MIVector.push_back(MI);
                
            ROS_INFO("point: %d,%d  MI: %lf",x,y,MI);
            }

        }

        ROS_INFO("get final map");

        PublishMap(map_msg,MIVector);//对地图做了更新

    }

    void click_callback(const geometry_msgs::PointStamped& click_msg) {

    }

    void PublishMap(const nav_msgs::OccupancyGrid& origin_map,
                    const std::vector<double>& MIVector){
        
        mi_compute::MIGrid MImap;

        MImap.header = origin_map.header;
        MImap.height = mapHeight_;
        MImap.width = mapWidth_;

        if(mapHeight_*mapWidth_ != MIVector.size()){
            ROS_ERROR("origin_map.size() != MIVector.size()");
        }


        for(int i = 0;i < mapHeight_*mapWidth_;i++){
            MImap.data.push_back(MIVector[i]);
        }

        MIMapPub_.publish(MImap);
        
    }



    //调用RayCast函数，给（x0,y0）和（x1,y1）;
    // 现在考虑比较简单 ，假设地图原点为（0,0），（x1,y1）为（0，width）和（0,height）之间
    // 发布一个vector,內部有从近到远一条线上穿过的点，最后一个点为（x1,y1）
    void BeamRayCast(const Eigen::Vector2i& initalPose ,const double& theta,const double& range){

        std::vector<Eigen::Vector2i> line;
        
        line = MiCompute::RayCast::LinearGrid(initalPose,theta,range);

        //line = MiCompute::RayCast::LinearGrid(initalPose,finalPose);

        std::vector<Eigen::Vector2i>::iterator itr = line.begin();

        for(;itr != line.end();++itr){
            std::cout<<"x: "<<(*itr)[0]<<" y:"<<(*itr)[1]<<std::endl;
        }
            newMapData_ = girdMI_->BeamUpdateGridMap(line);
            girdMI_->reset(mapWidth_,mapHeight_,newMapData_);

    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber mapSub_,click_sub_;
    ros::Publisher mapPub_,MIMapPub_;

    unsigned int mapWidth_;
    unsigned int mapHeight_;
    double mapResolution_;

    std::vector<double> mapData_;
    std::vector<double> newMapData_;

    MiCompute::GirdMI* girdMI_;

    int beamNumber_;
    double range_;
};





int main(int argc, char ** argv) {
    ros::init(argc, argv, "mi_compute_node");

    MICompute MI_compute;

    //MiCompute::BeamSensorModel b;

    ros::spin();
    return 0;
}
