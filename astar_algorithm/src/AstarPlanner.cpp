#include "AstarPlanner.h"
#include <pluginlib/class_list_macros.h>
//register this planner as a BaseAstarPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

struct cmp {
    bool operator()(Node &fst_node, Node &snd_node) {
        return fst_node.f_cost > snd_node.f_cost;
    }
};

int dx[8] = {0,1,1,1,0,-1,-1,-1};
int dy[8] = {-1,-1,0,1,1,1,0,-1};
namespace astar_planner {
    //Default Constructor
    AstarPlanner::AstarPlanner(){}

    AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }


    void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized) {
            m_costmap_ros = costmap_ros;
            m_costmap = m_costmap_ros->getCostmap();
            cellsY = m_costmap->getSizeInCellsY();
            cellsX = m_costmap->getSizeInCellsX();
            area = cellsY * cellsX;

            OccupancyGridMap.resize(area,0);
            for(unsigned int i = 0; i < cellsY; i++) {
                for(unsigned int j = 0; j < cellsX; j++) {
                    int cost = (int)m_costmap->getCost(j,i);
                    if(cost == 0) {
                        OccupancyGridMap[i*cellsX+j] = 1;
                    }
                }
            }

            m_frame_id = m_costmap_ros->getGlobalFrameID();
            ros::NodeHandle private_nh("~/"+name);
            pub = private_nh.advertise<nav_msgs::Path>("plan", 1);
            initialized = true;
        }
        else {
            ROS_WARN("Already Initialized");

        }
    }

    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        if(!initialized) {
            ROS_WARN("Not Initialized");
            return false;
        }
        //get the start pose of the map from the world
        double world_x = start.pose.position.x;
        double world_y = start.pose.position.y;
        unsigned int start_x,start_y;
        m_costmap->worldToMap(world_x,world_y,start_x,start_y);

        //get the map goal pose of the map from the world
        world_x = goal.pose.position.x;
        world_y = goal.pose.position.y;
        unsigned int goal_x,goal_y;
        m_costmap->worldToMap(world_x,world_y,goal_x,goal_y);

        //making open and cloosed checking vector
        open.resize(area,1000000000);
        closed.resize(area,false);

        //convert to the index from the coordinates
        int start_idx = m_costmap->getIndex(start_x,start_y);
        int goal_idx = m_costmap->getIndex(goal_x,goal_y);
        
        priority_queue<Node,vector<Node>,cmp> pq_wait;

        Node start_node;
        start_node.f_cost = 0 + getHeuristic(start_idx,goal_idx);
        start_node.g_cost = 0;
        start_node.idx = start_idx;
        open[start_idx] = start_node.f_cost;
        pq_wait.push(start_node);
        
        while(!pq_wait.empty()) {
            Node cur = pq_wait.top();
            pq_wait.pop();
            if(closed[cur.idx]== true) {
                continue;
            }
            if(cur.idx == goal_idx) {
                break;
            }
            closed[cur.idx] = true;
            vector<int> adj = getAdjacent(cur.idx);
            for(int i = 0; i < adj.size(); i++) {
                Node nxt;
                nxt.idx = adj[i];
                if(closed[nxt.idx]) {
                    continue;
                }
                nxt.g_cost = cur.g_cost + getGcost(cur.idx,nxt.idx);
                nxt.f_cost = nxt.g_cost + getHeuristic(nxt.idx,goal_idx);
                if(open[nxt.idx] < nxt.f_cost) {
                    continue;
                }
                open[nxt.idx] = nxt.f_cost;
                pq_wait.push(nxt);
            }
        }
        return true;
    }

    vector<int> AstarPlanner::getAdjacent(int cur_idx) {
        vector<int> adj;
        unsigned int cur_x, cur_y;
        m_costmap->indexToCells(cur_idx,cur_x,cur_y);
        for(int dir = 0; dir < 8; dir++) {
            int nx = cur_x+dx[dir];
            int ny = cur_y+dy[dir];
            if(!areaLimit(nx,ny)) {
                continue;
            }
            int nidx = m_costmap->getIndex(nx,ny);
            if(OccupancyGridMap[nidx] == 0) {
                continue;
            }
            adj.push_back(nidx);
        }
        return adj;
    }
    double AstarPlanner::getGcost(int fstIdx, int sndIdx) {
        return 1.4;
    }
    int AstarPlanner::getHeuristic(int nIdx, int goalIdx) {
        unsigned int tmp_x,tmp_y;
        m_costmap->indexToCells(nIdx,tmp_x,tmp_y);
        int nx = tmp_x;
        int ny = tmp_y;

        m_costmap->indexToCells(goalIdx,tmp_x,tmp_y);
        int gx = tmp_x;
        int gy = tmp_y;

        return (abs(nx-gx)+abs(ny-gy));
    }
    bool AstarPlanner::areaLimit(int x, int y) {
        if(x < 0 || y < 0 || x >= cellsX || y >= cellsY) {
            return false;
        }
        return true;
    }
};