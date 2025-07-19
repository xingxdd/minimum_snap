#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_publisher_node");
    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);

    ros::Rate rate(0.5); // 发布频率为Hz
    while (ros::ok())
    {
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "world"; // 设置坐标系
        goal_msg.pose.position.x = 0.0; // 设置目标位置
        goal_msg.pose.position.y = 0.0;
        goal_msg.pose.position.z = 1.0;
        goal_msg.pose.orientation.w = 1.0; // 设置目标姿态
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = 0.0;

        goal_pub.publish(goal_msg);
        ROS_INFO("Published goal: [%f, %f, %f]", goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z);

        rate.sleep();
    }

    return 0;
}