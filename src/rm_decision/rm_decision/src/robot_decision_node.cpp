#include "rm_decision/robot_decision.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_decision
{

RobotDecision::RobotDecision(const rclcpp::NodeOptions & options)
: Node("robot_decision", options),
  current_state_(RobotState::INIT),
  navigation_completed_(false)
{
  // 从参数服务器获取参数
  this->declare_parameter<double>("init_x", 0.0);
  this->declare_parameter<double>("init_y", 0.0);
  this->declare_parameter<double>("init_z", 0.0);
  this->declare_parameter<double>("init_yaw", 0.0);
  this->declare_parameter<double>("target_distance", 5.0);  // 目标距离为5米
  this->declare_parameter<double>("position_tolerance", 0.1);  // 到达目标的位置容差
  this->declare_parameter<double>("orientation_tolerance", 0.1);  // 到达目标的方向容差

  // 获取参数
  init_x_ = this->get_parameter("init_x").as_double();
  init_y_ = this->get_parameter("init_y").as_double();
  init_z_ = this->get_parameter("init_z").as_double();
  init_yaw_ = this->get_parameter("init_yaw").as_double();
  double target_distance = this->get_parameter("target_distance").as_double();
  position_tolerance_ = this->get_parameter("position_tolerance").as_double();
  orientation_tolerance_ = this->get_parameter("orientation_tolerance").as_double();

  // 计算目标位置 (沿X方向前进target_distance米)
  target_x_ = init_x_ + target_distance * cos(init_yaw_);
  target_y_ = init_y_ + target_distance * sin(init_yaw_);
  target_z_ = init_z_;
  target_yaw_ = init_yaw_;

  // 初始化当前位置
  current_x_ = init_x_;
  current_y_ = init_y_;
  current_z_ = init_z_;
  current_yaw_ = init_yaw_;

  // 创建导航动作客户端
  this->navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    "navigate_to_pose");

  // 创建TF监听器
  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建可视化发布者
  this->visualization_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "robot_decision/visualization", 10);

  // 订阅里程计消息
  this->odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&RobotDecision::odomCallback, this, std::placeholders::_1));

  // 创建定时器，用于执行决策逻辑
  this->timer_ = this->create_wall_timer(
    500ms, std::bind(&RobotDecision::initialize, this));

  // 创建可视化定时器
  this->visualization_timer_ = this->create_wall_timer(
    1s, std::bind(&RobotDecision::publishVisualization, this));

  RCLCPP_INFO(this->get_logger(), "Robot Decision节点已初始化");
  RCLCPP_INFO(this->get_logger(), "初始位置: (%f, %f, %f), 朝向: %f", init_x_, init_y_, init_z_, init_yaw_);
  RCLCPP_INFO(this->get_logger(), "目标位置: (%f, %f, %f), 朝向: %f", target_x_, target_y_, target_z_, target_yaw_);
}

RobotDecision::~RobotDecision()
{
  RCLCPP_INFO(this->get_logger(), "Robot Decision节点已销毁");
}

void RobotDecision::initialize()
{
  // 检查导航客户端是否已连接
  if (!this->navigation_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(this->get_logger(), "导航动作服务器未连接");
    return;
  }

  // 修改定时器回调为状态机逻辑
  this->timer_ = this->create_wall_timer(
    500ms, [this]() {
      // 状态机逻辑
      switch (this->current_state_) {
        case RobotState::INIT:
          RCLCPP_INFO(this->get_logger(), "开始导航任务");
          this->startNavigation();
          this->current_state_ = RobotState::MOVING;
          break;
        case RobotState::MOVING:
          if (this->navigation_completed_) {
            RCLCPP_INFO(this->get_logger(), "到达目标点，开始等待");
            this->current_state_ = RobotState::WAITING;
          }
          break;
        case RobotState::WAITING:
          // 在目标点等待，后续可以添加裁判系统的集成
          break;
        case RobotState::COMPLETED:
          // 任务完成
          break;
        default:
          break;
      }
    });
}

void RobotDecision::startNavigation()
{
  auto goal_msg = NavigateToPose::Goal();
  
  // 设置目标点
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose.position.x = target_x_;
  goal_msg.pose.pose.position.y = target_y_;
  goal_msg.pose.pose.position.z = target_z_;
  
  // 使用四元数表示目标姿态
  tf2::Quaternion q;
  q.setRPY(0, 0, target_yaw_);
  goal_msg.pose.pose.orientation.x = q.x();
  goal_msg.pose.pose.orientation.y = q.y();
  goal_msg.pose.pose.orientation.z = q.z();
  goal_msg.pose.pose.orientation.w = q.w();

  // 发送导航目标
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&RobotDecision::navigateGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&RobotDecision::navigateFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&RobotDecision::navigateResultCallback, this, std::placeholders::_1);

  this->navigation_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "导航目标已发送: (%f, %f)", target_x_, target_y_);
}

bool RobotDecision::checkGoalReached()
{
  // 计算当前位置与目标位置的距离
  double dx = current_x_ - target_x_;
  double dy = current_y_ - target_y_;
  double distance = std::sqrt(dx * dx + dy * dy);

  // 计算朝向差异
  double angle_diff = std::abs(current_yaw_ - target_yaw_);
  while (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  }
  angle_diff = std::abs(angle_diff);

  // 判断是否到达目标
  return (distance <= position_tolerance_) && (angle_diff <= orientation_tolerance_);
}

void RobotDecision::publishVisualization()
{
  // 创建标记数组
  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

  // 添加目标点标记
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = this->now();
  goal_marker.ns = "robot_decision";
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position.x = target_x_;
  goal_marker.pose.position.y = target_y_;
  goal_marker.pose.position.z = target_z_;
  tf2::Quaternion q;
  q.setRPY(0, 0, target_yaw_);
  goal_marker.pose.orientation.x = q.x();
  goal_marker.pose.orientation.y = q.y();
  goal_marker.pose.orientation.z = q.z();
  goal_marker.pose.orientation.w = q.w();
  goal_marker.scale.x = 0.2;
  goal_marker.scale.y = 0.2;
  goal_marker.scale.z = 0.2;
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 0.0;
  goal_marker.color.a = 1.0;
  goal_marker.lifetime = rclcpp::Duration(0s);
  
  // 添加初始点标记
  visualization_msgs::msg::Marker init_marker;
  init_marker.header.frame_id = "map";
  init_marker.header.stamp = this->now();
  init_marker.ns = "robot_decision";
  init_marker.id = 1;
  init_marker.type = visualization_msgs::msg::Marker::SPHERE;
  init_marker.action = visualization_msgs::msg::Marker::ADD;
  init_marker.pose.position.x = init_x_;
  init_marker.pose.position.y = init_y_;
  init_marker.pose.position.z = init_z_;
  tf2::Quaternion q_init;
  q_init.setRPY(0, 0, init_yaw_);
  init_marker.pose.orientation.x = q_init.x();
  init_marker.pose.orientation.y = q_init.y();
  init_marker.pose.orientation.z = q_init.z();
  init_marker.pose.orientation.w = q_init.w();
  init_marker.scale.x = 0.2;
  init_marker.scale.y = 0.2;
  init_marker.scale.z = 0.2;
  init_marker.color.r = 0.0;
  init_marker.color.g = 1.0;
  init_marker.color.b = 0.0;
  init_marker.color.a = 1.0;
  init_marker.lifetime = rclcpp::Duration(0s);

  // 添加轨迹线标记
  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = "map";
  path_marker.header.stamp = this->now();
  path_marker.ns = "robot_decision";
  path_marker.id = 2;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  
  // 添加轨迹点
  geometry_msgs::msg::Point p1;
  p1.x = init_x_;
  p1.y = init_y_;
  p1.z = init_z_;
  path_marker.points.push_back(p1);
  
  geometry_msgs::msg::Point p2;
  p2.x = target_x_;
  p2.y = target_y_;
  p2.z = target_z_;
  path_marker.points.push_back(p2);
  
  path_marker.scale.x = 0.05;  // 线的宽度
  path_marker.color.r = 0.0;
  path_marker.color.g = 0.0;
  path_marker.color.b = 1.0;
  path_marker.color.a = 1.0;
  path_marker.lifetime = rclcpp::Duration(0s);

  // 将标记添加到数组
  marker_array->markers.push_back(goal_marker);
  marker_array->markers.push_back(init_marker);
  marker_array->markers.push_back(path_marker);
  
  // 发布标记数组
  this->visualization_publisher_->publish(std::move(marker_array));
}

void RobotDecision::navigateGoalResponseCallback(
  std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "导航目标被拒绝");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "导航目标被接受");
}

void RobotDecision::navigateFeedbackCallback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  // 处理导航反馈
  auto current_pose = feedback->current_pose;
  RCLCPP_DEBUG(this->get_logger(), "当前位置: (%f, %f)",
    current_pose.pose.position.x, current_pose.pose.position.y);
}

void RobotDecision::navigateResultCallback(
  const GoalHandleNavigateToPose::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "导航任务成功");
      this->navigation_completed_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "导航任务被中止");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "导航任务被取消");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "未知的导航结果码");
      break;
  }
}

void RobotDecision::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 更新当前位置
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_z_ = msg->pose.pose.position.z;
  
  // 从四元数中提取yaw角
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
}

} // namespace rm_decision

// 主函数
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // 创建RobotDecision节点
  auto node = std::make_shared<rm_decision::RobotDecision>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
} 