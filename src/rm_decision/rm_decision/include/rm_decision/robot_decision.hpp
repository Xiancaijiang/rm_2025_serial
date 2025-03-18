#ifndef RM_DECISION_ROBOT_DECISION_HPP
#define RM_DECISION_ROBOT_DECISION_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace rm_decision
{

// 机器人状态
enum class RobotState
{
  INIT,           // 初始化状态
  MOVING,         // 正在移动
  WAITING,        // 等待状态
  COMPLETED       // 任务完成
};

class RobotDecision : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // 构造函数
  explicit RobotDecision(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 析构函数
  ~RobotDecision();

private:
  // 初始化函数，设置机器人初始位置和参数
  void initialize();

  // 启动导航任务
  void startNavigation();

  // 检查是否到达目标
  bool checkGoalReached();

  // 发布可视化标记
  void publishVisualization();

  // 导航目标回调函数
  void navigateGoalResponseCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void navigateFeedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void navigateResultCallback(const GoalHandleNavigateToPose::WrappedResult & result);

  // 里程计回调函数，用于获取机器人当前位置
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  // 当前机器人状态
  RobotState current_state_;
  
  // 初始位置坐标 (场地中心)
  double init_x_;
  double init_y_;
  double init_z_;
  double init_yaw_;

  // 目标位置
  double target_x_;
  double target_y_;
  double target_z_;
  double target_yaw_;

  // 机器人当前位置
  double current_x_;
  double current_y_;
  double current_z_;
  double current_yaw_;

  // 导航目标是否完成
  bool navigation_completed_;

  // 到达目标的容差
  double position_tolerance_;
  double orientation_tolerance_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;

  // ROS2 动作客户端
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;

  // TF2 监听器
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 发布者和订阅者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

} // namespace rm_decision

#endif // RM_DECISION_ROBOT_DECISION_HPP 