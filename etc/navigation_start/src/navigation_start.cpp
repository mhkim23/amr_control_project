#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "navigation_start");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);
    ros::Rate loop_rate(10);
    int count = 0;
    while(ros::ok() && count < 10) {
        count++;
        geometry_msgs::PoseWithCovarianceStamped start;
        start.header.frame_id = "map";
        start.header.stamp = ros::Time::now();
        start.pose.pose.position.x = -1.11;
        start.pose.pose.position.y = 0.96;
        start.pose.pose.position.z = 0.0;
        start.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        start.pose.covariance[0] = 0.25;
        start.pose.covariance[7] = 0.25;
        start.pose.covariance[35] = 0.06853892326654787;
        ROS_INFO("Going to the start pose");
        pub.publish(start);
        loop_rate.sleep();
    }
    return 0;
}