#include "AstarPlanner.h"
#include <pluginlib/class_list_macros.h>
//register this planner as a BaseAstarPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace astar_planner {

    AstarPlanner::AstarPlanner(){}

    AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }


    void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized) {
            m_costmap_ros = costmap_ros;
            m_costmap = m_costmap_ros->getCostmap();
            height = m_costmap->getSizeInCellsX();
            width = m_costmap->getSizeInCellsY();
            area = height * width;
            OccupancyGridMap.resize(area);
            for(int i = 0; i < height; i++) {
                for(int j = 0; j < width; j++) {
                    int cost = (int)m_costmap->getCost(i,j);
                    if(cost == 0) {
                        OccupancyGridMap[i*width+j] = true;
                    }
                    else {
                        OccupancyGridMap[i*width+j] = false;
                    }
                }
            }
            m_frame_id = m_costmap_ros->getGlobalFrameID();
            ros::NodeHandle private_nh("~/"+name);
            pub = private_nh.advertise<nav_msgs::Path>("plan", 1);
            initialized = true;
        }
        else {

        }
    }

    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        
        return true;
    }
};