#include <ros/ros.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <cmath>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_pub");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_traj_server/traj", 1, true);

    ros::Rate rate(0.1); // 每5秒发布一次（可根据实际调整）

    while (ros::ok())
    {
        quadrotor_msgs::PolynomialTrajectory traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.trajectory_id = 1;
        traj_msg.num_order = 5;      // 5阶多项式
        traj_msg.num_segment = 3;    // 3段

        // 第一段：直线 x: 0->1, y: 0, z: 1->1.5, T1=3s
        traj_msg.coef_x.insert(traj_msg.coef_x.end(), {0.0, 1.0/3.0, 0, 0, 0, 0});
        traj_msg.coef_y.insert(traj_msg.coef_y.end(), {0.0, 0, 0, 0, 0, 0});
        traj_msg.coef_z.insert(traj_msg.coef_z.end(), {1.0, 0.5/3.0, 0, 0, 0, 0});
        traj_msg.time.push_back(3.0);

        // 第二段：抛物线 x: 1->2, y: 0->1, z: 1.5->1, T2=4s
        traj_msg.coef_x.insert(traj_msg.coef_x.end(), {1.0, 1.0/4.0, 0, 0, 0, 0});
        traj_msg.coef_y.insert(traj_msg.coef_y.end(), {0.0, 1.0/4.0, 0, 0, 0, 0});
        traj_msg.coef_z.insert(traj_msg.coef_z.end(), {1.5, -0.5/4.0, 0, 0, 0, 0});
        traj_msg.time.push_back(4.0);

        // 第三段：圆弧近似 x: 2->2+r*cos, y: 1->1+r*sin, z: 1, T3=9s
        double r = 3;
        double T3 = 9.0;
        double w = M_PI / T3; // 半圆
        traj_msg.coef_x.insert(traj_msg.coef_x.end(), {2.0 + r, 0, 0, 0, 0, 0});
        traj_msg.coef_y.insert(traj_msg.coef_y.end(), {1.0, r * w, 0, 0, 0, 0});
        traj_msg.coef_z.insert(traj_msg.coef_z.end(), {1.0, 0, 0, 0, 0, 0});
        traj_msg.time.push_back(T3);

        traj_pub.publish(traj_msg);

        rate.sleep();
    }

    return 0;
}