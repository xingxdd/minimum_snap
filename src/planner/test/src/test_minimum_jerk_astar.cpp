#include <path_searching/rrt_star.h>
#include <traj_optimization/minimum_control.h>
#include <plan_env/grid_map.h>

#include <path_searching/a_star.h>
// 1. 在文件顶部添加消息头文件
#include <quadrotor_msgs/PolynomialTrajectory.h>

// path_searching::RRTStar::Ptr rrt_star_;
path_searching::Astar::Ptr astar_;
traj_optimization::MinimumControl::Ptr optimizer_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher  trajectory_pub,trajectory_pub_optimized,optimal_path_points_pub;
visualization_msgs::Marker trajectory_marker,trajectory_marker_optimized, optimal_path_points;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;
std::vector<Eigen::Vector3d> optimal_path;
Eigen::VectorXd coef_1d;
std::vector<double> x_vec;
std::vector<double> y_vec;
std::vector<double> z_vec;

/// @brief 新增///////////////////start////////////////////
std::vector<Eigen::Vector3d> optimal_path_new;
// 2. 在全局变量区添加发布器
ros::Publisher poly_traj_pub;
/**
 * @brief 打印网格地图信息
 * @param grid_map 智能指针指向的网格地图对象
 * @details 打印网格地图的原点、分辨率、地图大小等信息，并检查地图中是否有障碍物
 * @note 该函数遍历地图的部分区域，统计并打印障碍物点的位置
 * @note 仅打印前10个障碍物点，统计总数
 * @note 该函数假设网格地图的分辨率是均匀的，并且障碍物点的检测是基于占用栅格的
 * @note 该函数可以用于调试和验证网格地图的正确性
 */
// void printGridMapInfo(const GridMap::Ptr& grid_map)
// {
//     ROS_WARN("GridMap Info:")
//     ROS_WARN(grid_map->getOrigin().transpose());
//     std::cout << "  Resolution: " << grid_map->getResolution() << std::endl;
//     std::cout << "  Map Size: " << grid_map->getMapSize().transpose() << std::endl;

//     // 检查地图中是否有障碍物（遍历部分区域示例）
//     int obs_count = 0;
//     for (double x = grid_map->getOrigin()[0]; x < grid_map->getOrigin()[0] + grid_map->getMapSize()[0]; x += grid_map->getResolution())
//     {
//         for (double y = grid_map->getOrigin()[1]; y < grid_map->getOrigin()[1] + grid_map->getMapSize()[1]; y += grid_map->getResolution())
//         {
//             for (double z = grid_map->getOrigin()[2]; z < grid_map->getOrigin()[2] + grid_map->getMapSize()[2]; z += grid_map->getResolution())
//             {
//                 Eigen::Vector3d pt(x, y, z);
//                 if (grid_map->getInflateOccupancy(pt))
//                 {
//                     obs_count++;
//                     if (obs_count < 10) // 只打印前10个障碍物点
//                         std::cout << "  Obstacle at: " << pt.transpose() << std::endl;
//                 }
//             }
//         }
//     }
//     std::cout << "Total obstacle voxels detected: " << obs_count << std::endl;
// }
void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_ = odom;
}

// /**
//   @brief 路径点剪枝
//   @param path 
//   @name  prunePath
//   @brief 对路径点进行剪枝，去除共线的点
// */
// void prunePath(std::vector<Eigen::Vector3d>& path)
// {
//   std::vector<Eigen::Vector3d> pruned_path;
//   pruned_path.push_back(path.front());
//   for (size_t i = 1; i < path.size() - 1; ++i)
//   {
//     Eigen::Vector3d prev = pruned_path.back();
//     Eigen::Vector3d curr = path[i];
//     Eigen::Vector3d next = path[i + 1];
//     // 如果当前点与前后点共线，则跳过
//     if ((next - curr).normalized() != (curr - prev).normalized())
//     {
//       pruned_path.push_back(curr);
//     }
//   }
//   pruned_path.push_back(path.back());
//   path = pruned_path;
// }


/**
 * @brief 计算点到直线的距离
 * @param start 直线起点
 * @param end 直线终点
 * @param point 待计算的点
 * @return 点到直线的距离
 */
double disP2L(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& point)
{
    Eigen::Vector3d line_vec = end - start;
    Eigen::Vector3d point_vec = point - start;
    double area = line_vec.cross(point_vec).norm();
    double base = line_vec.norm();
    return area / base;
}
/**
 * @brief 使用 RDP 算法简化路径
 * @param path 原始路径
 * @param path_resolution 简化路径的距离阈值
 * @return 简化后的路径
 */
std::vector<Eigen::Vector3d> pathSimplify(const std::vector<Eigen::Vector3d>& path, double path_resolution)
{
    std::vector<Eigen::Vector3d> subPath;

    if (path.size() <= 2)
    {
        subPath = path;
    }
    else
    {
        const Eigen::Vector3d& first = path[0];
        const Eigen::Vector3d& last = path[path.size() - 1];

        int index_dis = 0;
        double max_dis = 0;

        // 找到距离直线最远的点
        for (int i = 1; i < (int)path.size() - 1; i++)
        {
            double tmp_dis = disP2L(first, last, path[i]);
            if (tmp_dis > max_dis)
            {
                max_dis = tmp_dis;
                index_dis = i;
            }
        }
        // 如果最大距离小于阈值，则用直线近似路径
        if (max_dis < path_resolution)
        {
            subPath.push_back(first);
            subPath.push_back(last);
        }
        else
        {
            // 递归分割路径
            std::vector<Eigen::Vector3d> recSubPath1 = pathSimplify(std::vector<Eigen::Vector3d>(path.begin(), path.begin() + index_dis + 1), path_resolution);
            std::vector<Eigen::Vector3d> recSubPath2 = pathSimplify(std::vector<Eigen::Vector3d>(path.begin() + index_dis, path.end()), path_resolution);

            subPath.insert(subPath.end(), recSubPath1.begin(), recSubPath1.end() - 1);
            subPath.insert(subPath.end(), recSubPath2.begin(), recSubPath2.end());
        }
    }

    return subPath;
}

void prunePath(std::vector<Eigen::Vector3d>& path, double path_resolution)
{
    if (path.size() <= 2)
    {
        return; // 如果路径点少于等于2个，直接返回
    }

    // 调用 RDP 算法简化路径
    path = pathSimplify(path, path_resolution);
}






/**
  @brief 检查路径点是否与栅格地图发生碰撞
  @param x_vec 
  @param y_vec 
  @param z_vec 
  @param grid_map 
  @return bool
*/
bool checkCollision(const std::vector<double>& x_vec, 
  const std::vector<double>& y_vec, 
  const std::vector<double>& z_vec, 
  const GridMap::Ptr& grid_map)
{
// for (size_t i = 0; i < x_vec.size(); ++i)
// {
// Eigen::Vector3d point(x_vec[i], y_vec[i], z_vec[i]);
// if (grid_map->getInflateOccupancy(point))
// {
// std::cout << "Collision detected at point: " << point.transpose() << std::endl;
// return true; // 碰撞
// }
// }
// return false; // 无碰撞
}

bool checkTrajectoryCollision(const std::vector<double>& x_vec,
  const std::vector<double>& y_vec,
  const std::vector<double>& z_vec,
  const GridMap::Ptr& grid_map)
{
    for (size_t i = 0; i < x_vec.size(); ++i)
    {
            Eigen::Vector3d pt(x_vec[i], y_vec[i], z_vec[i]);
        if (grid_map->getInflateOccupancy(pt))
        {
            std::cout << "Collision detected at: " << pt.transpose() << std::endl;
            return true;
        }
    }
    return false;
}


/// @brief 新增/////////////////stop//////////////////////
void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // start and end condition
  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Vector3d end_vel(0.0, 0.0, 0.0);
  Eigen::Vector3d end_acc(0.0, 0.0, 0.0);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z);
  Eigen::Vector3d start_vel(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y, odom_->twist.twist.linear.z);
//  Eigen::Vector3d start_vel(1.0, 0.0, 0.0);
  Eigen::Vector3d start_acc(0.0, 0.0, 0.0);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;


//   int success = rrt_star_->search(start_pt, end_pt, path);
  int success = astar_->search(start_pt, end_pt, path);
  
  ROS_WARN("path:",path);
//   optimal_path = rrt_star_->getOptimalPath();
  optimal_path.clear();
  optimal_path = path;
  // optimal_path = astar_->getOptimalPath();
  
  if (success == 1)
  {
    std::cout << "Path found, Start optimization" << std::endl;
    for (size_t i = 0; i < path.size(); ++i)
    {
        ROS_WARN("Path.size():%d",path.size());
        std::cout << "Point " << i << ": " << path[i].transpose() << std::endl;
    }

    double path_resolution = 0.3; // 设置距离阈值（单位：米）
    //剪枝操作
    prunePath(optimal_path, path_resolution);

    optimal_path_points.points.clear();
    for (size_t i = 0; i < optimal_path.size(); ++i)
    {
        ROS_WARN("Optimal path.size():%d",optimal_path.size());
        std::cout << "Optimal Point " << i << ": " << optimal_path[i].transpose() << std::endl;
        geometry_msgs::Point pt;
        pt.x = optimal_path[i][0];
        pt.y = optimal_path[i][1];
        pt.z = optimal_path[i][2];
        optimal_path_points.points.push_back(pt);

        optimal_path_points_pub.publish(optimal_path_points);
    }

    if (optimal_path.empty())
    {
        ROS_WARN("Optimal path is empty, skipping optimization!");
        return;
    }
    Eigen::VectorXd pos_x;
    Eigen::VectorXd pos_y;
    Eigen::VectorXd pos_z;
    pos_x.resize(optimal_path.size());
    pos_y.resize(optimal_path.size());
    pos_z.resize(optimal_path.size());
    for (int i = 0; i < optimal_path.size(); i++)
    {
        pos_x(i) = optimal_path[i][0];
        pos_y(i) = optimal_path[i][1];
        pos_z(i) = optimal_path[i][2];
    }
    Eigen::Vector2d bound_vel_x(start_vel[0], end_vel[0]);
    Eigen::Vector2d bound_vel_y(start_vel[1], end_vel[1]);
    Eigen::Vector2d bound_vel_z(start_vel[2], end_vel[2]);

    Eigen::Vector2d bound_acc_x(start_acc[0], end_acc[0]);
    Eigen::Vector2d bound_acc_y(start_acc[1], end_acc[1]);
    Eigen::Vector2d bound_acc_z(start_acc[2], end_acc[2]);

    // time allocate: need refine, average allocate time
    Eigen::VectorXd time_vec;
    time_vec.resize(optimal_path.size() - 1);
    for (int i = 0; i < optimal_path.size() - 1; i++)
    {
        time_vec(i) = 1.0;
    }

    // start optimization, x, y, z separately, serial optimization
    // need to change to parallel optimization
    bool success_x = optimizer_->solve(pos_x, bound_vel_x, bound_acc_x, time_vec);
    if (success_x)
    {
        coef_1d = optimizer_->getCoef1d();
        for (int i = 0; i < optimal_path.size() - 1; i++)
        {
            for (double t = 0; t < time_vec(i); t+= 0.1)
            {
                Eigen::Matrix<double, 1, 6> coef_matrix = Eigen::Matrix<double, 1, 6>::Zero();
                coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2), coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
                Eigen::Matrix<double, 6, 1> t_vector = Eigen::Matrix<double, 6, 1>::Zero();
                for (int i = 0; i < 6; i++)
                {
                    t_vector(i) = pow(t, i);
                }
                x_vec.push_back(coef_matrix * t_vector);
            }
        }
        x_vec.push_back(end_pt[0]);
    }
    else
    {
        std::cout << "optimize faiure!" << std::endl;
    }

    bool success_y = optimizer_->solve(pos_y, bound_vel_y, bound_acc_y, time_vec);
    if (success_y)
    {
        coef_1d = optimizer_->getCoef1d();
        for (int i = 0; i < optimal_path.size() - 1; i++)
        {
            for (double t = 0; t < time_vec(i); t+= 0.1)
            {
                Eigen::Matrix<double, 1, 6> coef_matrix;
                coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2), coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
                Eigen::Matrix<double, 6, 1> t_vector;
                for (int i = 0; i < 6; i++)
                {
                    t_vector(i) = pow(t, i);
                }
                y_vec.push_back(coef_matrix * t_vector);
            }
        }
        y_vec.push_back(end_pt[1]);
    }
    else
    {
        std::cout << "optimize faiure!" << std::endl;
    }

    bool success_z = optimizer_->solve(pos_z, bound_vel_z, bound_acc_z, time_vec);
    if (success_z)
    {
        coef_1d = optimizer_->getCoef1d();
        for (int i = 0; i < optimal_path.size() - 1; i++)
        {
            for (double t = 0; t < time_vec(i); t+= 0.1)
            {
                Eigen::Matrix<double, 1, 6> coef_matrix;
                coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2), coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
                Eigen::Matrix<double, 6, 1> t_vector;
                for (int i = 0; i < 6; i++)
                {
                    t_vector(i) = pow(t, i);
                }
                z_vec.push_back(coef_matrix * t_vector);
            }
        }
        z_vec.push_back(end_pt[2]);
    }
    else
    {
        std::cout << "optimize faiure!" << std::endl;
    }


// 仅在轨迹优化成功后发布
if (success_x && success_y && success_z)
{
    quadrotor_msgs::PolynomialTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.trajectory_id = 0;
    traj_msg.num_order = 5; // 6阶多项式，order=5
    traj_msg.num_segment = optimal_path.size() - 1;

    // 展开系数到一维
    for (int i = 0; i < traj_msg.num_segment; ++i)
    {
        // x
        for (int j = 0; j < 6; ++j)
            traj_msg.coef_x.push_back(optimizer_->getCoef1d()(6 * i + j));
        // y
        for (int j = 0; j < 6; ++j)
            traj_msg.coef_y.push_back(optimizer_->getCoef1d()(6 * i + j));
        // z
        for (int j = 0; j < 6; ++j)
            traj_msg.coef_z.push_back(optimizer_->getCoef1d()(6 * i + j));
        // 时间
        traj_msg.time.push_back(time_vec(i));
    }

    poly_traj_pub.publish(traj_msg);
}







    // visualize th trajectory
    for (int i = 0; i < x_vec.size(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = x_vec[i];
        pt.y = y_vec[i];
        pt.z = z_vec[i];
        trajectory_marker.points.push_back(pt);
    }
    trajectory_pub.publish(trajectory_marker);
  }
  else
  {
    std::cout << "Path not found!" << std::endl;
  }
////////////////////////////////////////////////////////////////////////////////////
    // 可视化轨迹后，检测碰撞并自动重规划
    GridMap::Ptr grid_map = astar_->getGridMap(); // 获取地图指针
    // printGridMapInfo(grid_map); // 打印地图信息


int replan_count = 0;
const int max_replan = 5;
while (checkTrajectoryCollision(x_vec, y_vec, z_vec, grid_map) && replan_count < max_replan)
{
    ROS_ERROR("checkTrajectoryCollision: Collision detected, replanning...");
    
    std::cout << "Trajectory collision detected, replanning..." << std::endl;
    // 找到发生碰撞的段，插入中点
    // int collision_idx = -1;
    // for (size_t i = 0; i < x_vec.size(); ++i)
    // {
    //     Eigen::Vector3d pt(x_vec[i], y_vec[i], z_vec[i]);
    //     if (grid_map->getInflateOccupancy(pt))
    //     {
    //         collision_idx = i;
    //         break;
    //     }
    // }
    // if (collision_idx < 0 || collision_idx >= optimal_path.size() - 1)
    // {
    //     std::cout << "No valid collision segment found, abort replanning." << std::endl;
    //     break;
    // }
    // // 在路径中插入中点
    // Eigen::Vector3d mid = (optimal_path[collision_idx] + optimal_path[collision_idx + 1]) / 2.0;
    // optimal_path.insert(optimal_path.begin() + collision_idx + 1, mid);

    // 找到发生碰撞的采样点
    int collision_idx = -1;
    for (size_t i = 0; i < x_vec.size(); ++i)
    {
        Eigen::Vector3d pt(x_vec[i], y_vec[i], z_vec[i]);
        if (grid_map->getInflateOccupancy(pt))
        {
            collision_idx = i;
            break;
        }
    }
    if (collision_idx < 0)
    {
        std::cout << "No valid collision point found, abort replanning." << std::endl;
        break;
    }

    // 找到最近的路径段并插入中点
    int nearest_seg = -1;
    double min_dist = 1e9;
    Eigen::Vector3d collision_pt(x_vec[collision_idx], y_vec[collision_idx], z_vec[collision_idx]);
    for (size_t i = 0; i < optimal_path.size() - 1; ++i)
    {
        Eigen::Vector3d seg_mid = (optimal_path[i] + optimal_path[i+1]) / 2.0;
        double dist = (collision_pt - seg_mid).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_seg = i;
        }
    }
    if (nearest_seg >= 0 && nearest_seg < optimal_path.size() - 1)
    {
        Eigen::Vector3d mid = (optimal_path[nearest_seg] + optimal_path[nearest_seg + 1]) / 2.0;
        optimal_path.insert(optimal_path.begin() + nearest_seg + 1, mid);
    }
    else
    {
        std::cout << "No valid segment for collision point, abort replanning." << std::endl;
        break;
    }


    // 重新分配时间
    Eigen::VectorXd time_vec;
    time_vec.resize(optimal_path.size() - 1);
    for (int i = 0; i < optimal_path.size() - 1; i++)
        time_vec(i) = 1.0;

    // 重新生成轨迹点
    Eigen::VectorXd pos_x(optimal_path.size()), pos_y(optimal_path.size()), pos_z(optimal_path.size());
    for (int i = 0; i < optimal_path.size(); i++)
    {
        pos_x(i) = optimal_path[i][0];
        pos_y(i) = optimal_path[i][1];
        pos_z(i) = optimal_path[i][2];
    }

    // 重新优化
    Eigen::Vector2d bound_vel_x(0, 0), bound_acc_x(0, 0);
    optimizer_->solve(pos_x, bound_vel_x, bound_acc_x, time_vec);
    // optimizer_->solve(pos_x, Eigen::Vector2d(0,0), Eigen::Vector2d(0,0), time_vec);
    std::vector<double> new_x_vec;
    coef_1d = optimizer_->getCoef1d();
    for (int i = 0; i < optimal_path.size() - 1; i++)
    {
        for (double t = 0; t < time_vec(i); t += 0.1)
        {
            Eigen::Matrix<double, 1, 6> coef_matrix;
            coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2), coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
            Eigen::Matrix<double, 6, 1> t_vector;
            for (int j = 0; j < 6; j++)
                t_vector(j) = pow(t, j);
            new_x_vec.push_back((coef_matrix * t_vector)(0, 0));
        }
    }
    new_x_vec.push_back(optimal_path.back()[0]);
    x_vec = new_x_vec;

    // y, z 同理，略（可参考你已有的轨迹采样代码）
    // 重新优化 y
    Eigen::Vector2d bound_vel_y(0, 0), bound_acc_y(0, 0);
    optimizer_->solve(pos_y, bound_vel_y, bound_acc_y, time_vec);
    // optimizer_->solve(pos_y, Eigen::Vector2d(0,0), Eigen::Vector2d(0,0), time_vec);
    std::vector<double> new_y_vec;
    coef_1d = optimizer_->getCoef1d();
    for (int i = 0; i < optimal_path.size() - 1; i++)
    {
        for (double t = 0; t < time_vec(i); t += 0.1)
        {
            Eigen::Matrix<double, 1, 6> coef_matrix;
            coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2), coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
            Eigen::Matrix<double, 6, 1> t_vector;
            for (int j = 0; j < 6; j++)
                t_vector(j) = pow(t, j);
            new_y_vec.push_back((coef_matrix * t_vector)(0, 0));
        }
    }
    new_y_vec.push_back(optimal_path.back()[1]);
    y_vec = new_y_vec;

    // 重新优化 z
    Eigen::Vector2d bound_vel_z(0, 0), bound_acc_z(0, 0);
    optimizer_->solve(pos_z, bound_vel_z, bound_acc_z, time_vec);
    // optimizer_->solve(pos_z, Eigen::Vector2d(0,0), Eigen::Vector2d(0,0), time_vec);
    std::vector<double> new_z_vec;
    coef_1d = optimizer_->getCoef1d();
    for (int i = 0; i < optimal_path.size() - 1; i++)
    {
        for (double t = 0; t < time_vec(i); t += 0.1)
        {
            Eigen::Matrix<double, 1, 6> coef_matrix;
            coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2), coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
            Eigen::Matrix<double, 6, 1> t_vector;
            for (int j = 0; j < 6; j++)
                t_vector(j) = pow(t, j);
            new_z_vec.push_back((coef_matrix * t_vector)(0, 0));
        }
    }
    new_z_vec.push_back(optimal_path.back()[2]);
    z_vec = new_z_vec;


        replan_count++;
      ROS_ERROR("Replan count: %d", replan_count);




}
if (replan_count >= max_replan)
{
    ROS_ERROR("Replan count: %d", replan_count);
    // 如果重规划次数超过最大限制，输出错误信息
    std::cout << "Replan failed after " << max_replan << " attempts!" << std::endl;
}
    // visualize th trajectory
  
    for (int i = 0; i < x_vec.size(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = x_vec[i];
        pt.y = y_vec[i];
        pt.z = z_vec[i];
        trajectory_marker_optimized.points.push_back(pt);
    }
    trajectory_pub_optimized.publish(trajectory_marker_optimized);

    if(trajectory_marker_optimized==trajectory_marker)
    {
        ROS_WARN("The init Trajectory optimized successfully!");
    }
    else
    {
        ROS_WARN("Trajectory optimization is not perfect!");
    }
   


////////////////////////////////////////////////////////////////////////////////////

  path.clear();
  optimal_path.clear();
  trajectory_marker.points.clear();
  trajectory_marker_optimized.points.clear();
  x_vec.clear();
  y_vec.clear();
  z_vec.clear();
//   rrt_star_->reset();
  astar_->reset();
  optimizer_->reset();
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_minimum_jerk_astar");
  ros::NodeHandle nh("~");

  goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, &GoalCallback);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, &OdomCallback);
  // 3. 在 main 函数中初始化发布器
  poly_traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_traj_server/traj", 1);
  trajectory_pub = nh.advertise<visualization_msgs::Marker>("/trajectory", 10);

  trajectory_marker.header.frame_id = "world";
  trajectory_marker.header.stamp = ros::Time::now();
  trajectory_marker.ns = "trajectory";
  trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_marker.action = visualization_msgs::Marker::ADD;
  trajectory_marker.pose.orientation.w = 1.0;
  trajectory_marker.scale.x = 0.1;
  trajectory_marker.scale.y = 0.1;
  trajectory_marker.scale.z = 0.1;
  trajectory_marker.color.a = 1.0;
  trajectory_marker.color.r = 1.0;
  trajectory_marker.color.g = 0.0;
  trajectory_marker.color.b = 1.0;



  trajectory_pub_optimized = nh.advertise<visualization_msgs::Marker>("/trajectory_optimized", 10);

  trajectory_marker_optimized.header.frame_id = "world";
  trajectory_marker_optimized.header.stamp = ros::Time::now();
  trajectory_marker_optimized.ns = "trajectory_optimized";
  trajectory_marker_optimized.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_marker_optimized.action = visualization_msgs::Marker::ADD;
  trajectory_marker_optimized.pose.orientation.w = 1.0;
  trajectory_marker_optimized.scale.x = 0.1;
  trajectory_marker_optimized.scale.y = 0.1;
  trajectory_marker_optimized.scale.z = 0.1;
  trajectory_marker_optimized.color.a = 1.0;
  trajectory_marker_optimized.color.r = 0.0;
  trajectory_marker_optimized.color.g = 1.0;
  trajectory_marker_optimized.color.b = 0.0;

  optimal_path_points_pub = nh.advertise<visualization_msgs::Marker>("/optimal_path_points", 1);
  optimal_path_points.header.frame_id = "world";
  optimal_path_points.header.stamp = ros::Time::now();
  optimal_path_points.ns = "optimal_path_points";
  optimal_path_points.type = visualization_msgs::Marker::POINTS;
  optimal_path_points.action = visualization_msgs::Marker::ADD;
  optimal_path_points.pose.orientation.w = 1.0;
  optimal_path_points.scale.x = 0.2; // 点的大小
  optimal_path_points.scale.y = 0.2;
  optimal_path_points.color.a = 1.0;
  optimal_path_points.color.r = 1.0;
  optimal_path_points.color.g = 0.5;
  optimal_path_points.color.b = 0.0;
  




  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);
  
//   rrt_star_ = std::make_shared<path_searching::RRTStar>();
  astar_ = std::make_shared<path_searching::Astar>();
  optimizer_ = std::make_shared<traj_optimization::MinimumControl>();

//   rrt_star_->setParam(nh);
//   rrt_star_->setGridMap(grid_map);
//   rrt_star_->init();
  astar_->setParam(nh);
  astar_->setGridMap(grid_map);
  astar_->init();

  ros::spin();
  return 0;
}