#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_cmd_pub");
    ros::NodeHandle nh;

    ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);

    double x0 = 0.0, y0 = 0.0, z0 = 1.0; // 圆心
    double r = 2.0;                      // 半径
    double omega = 0.3;                  // 角速度(rad/s)
    double hz = 50.0;                    // 发布频率

    ros::Rate rate(hz);
    double t = 0.0;

    while (ros::ok())
    {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";

        // 圆轨迹
        cmd.position.x = x0 + r * cos(omega * t);
        cmd.position.y = y0 + r * sin(omega * t);
        cmd.position.z = z0;

        // 速度
        cmd.velocity.x = -r * omega * sin(omega * t);
        cmd.velocity.y =  r * omega * cos(omega * t);
        cmd.velocity.z = 0.0;

        // 加速度
        cmd.acceleration.x = -r * omega * omega * cos(omega * t);
        cmd.acceleration.y = -r * omega * omega * sin(omega * t);
        cmd.acceleration.z = 0.0;

        // 朝向
        cmd.yaw = omega * t;
        cmd.yaw_dot = omega;

        // 其他参数可按需设置
        cmd.trajectory_id = 0;
        cmd.trajectory_flag = 0;

        cmd_pub.publish(cmd);

        t += 1.0 / hz;
        rate.sleep();
    }
    return 0;
}