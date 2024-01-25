#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include "control_node/ControlService.h"  // Include the service message type

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RobotController {
public:
  RobotController() : ac_("move_base", true) {
    // Initialize the MoveBaseClient
    // Wait for the action server to come up
    while (!ac_.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Create a service server to receive information from the button_node
    service_ = nh_.advertiseService("get_coord_info_service", &RobotController::handleGetCoordInfo, this);

    // Spin을 호출하지 않고, 서비스 콜백에서 로봇을 이동하도록 변경
    // ros::spin();
  }

  bool handleGetCoordInfo(control_node::ControlService::Request &req,
                           control_node::ControlService::Response &res) {
    // Store the received information
    goal_x_ = req.x;
    goal_y_ = req.y;
    angle_ = req.yaw;

    ROS_INFO("Received button information: X=%f, Y=%f, Yaw=%f", goal_x_, goal_y_, angle_);

    // Set the goal based on the received information
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.header.stamp = ros::Time::now();
    goal_.target_pose.pose.position.x = goal_x_;
    goal_.target_pose.pose.position.y = goal_y_;
    goal_.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle_);

    // Log the goal information
    ROS_INFO("Sending goal: X=%f, Y=%f, Yaw=%f", goal_x_, goal_y_, angle_);

    // Send the goal to move_base
    ac_.sendGoal(goal_);

    // Wait for the result
    ac_.waitForResult();

    // Log the result
    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the base moved to the goal");
    } else {
      ROS_INFO("The base failed to move to the goal for some reason");
    }

    // Respond to the client
    res.result = "Received button information successfully";

    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  MoveBaseClient ac_;

  double goal_x_ = -1.1;
  double goal_y_ = 0.9;
  double angle_ = 0;

  move_base_msgs::MoveBaseGoal goal_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_control_node");

  // Create an instance of the RobotController class
  RobotController robot_controller;

  ros::spin();

  return 0;
}
