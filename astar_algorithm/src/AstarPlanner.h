#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>

using std::string;
using std::vector;
#ifndef ASTAR_PLANNER_CPP
#define ASTAR_PLANNER_CPP

namespace astar_planner {
    class AstarPlanner : public nav_core::BaseGlobalPlanner {
        public:
            bool initialized;
            costmap_2d::Costmap2DROS *m_costmap_ros;
            costmap_2d::Costmap2D *m_costmap;
            int height;
            int width;
            int area;
            vector<bool> OccupancyGridMap;
            string m_frame_id;
            ros::Publisher pub;
            AstarPlanner(); // default constructor
            AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            /** overridden classes from interface nav_core::BaseGlobalPlanner **/
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
    };
};
 #endif
