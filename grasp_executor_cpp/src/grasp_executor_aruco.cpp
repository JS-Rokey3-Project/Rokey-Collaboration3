#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <aruco_msgs/msg/marker_array.hpp>
#include <cmath>

class ArucoGraspExecutor : public rclcpp::Node {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  ArucoGraspExecutor()
  : Node("aruco_grasp_executor"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, "/gripper_controller/gripper_cmd");

    marker_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
      "/detected_markers", 10,
      std::bind(&ArucoGraspExecutor::marker_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ArUco 기반 grasp executor 시작됨.");

    // 그리퍼 서버 대기
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "그리퍼 액션 서버 연결 실패.");
    }
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr marker_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
    if (msg->markers.empty()) return;

    // 가장 가까운 마커 선택 (z값이 가장 작음)
    auto closest_marker = *std::min_element(
      msg->markers.begin(), msg->markers.end(),
      [](const auto &a, const auto &b) {
        return a.pose.pose.position.z < b.pose.pose.position.z;
      });

    // 마커 좌표를 geometry_msgs::PoseStamped로 변환
    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header.frame_id = "camera_link";
    camera_pose.header.stamp = this->get_clock()->now();
    camera_pose.pose = closest_marker.pose.pose;

    geometry_msgs::msg::PoseStamped base_pose;
    try {
      base_pose = tf_buffer_.transform(camera_pose, "base_link", tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF 변환 실패: %s", ex.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "마커 위치(base_link): x=%.3f, y=%.3f, z=%.3f",
                base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z);

    // 그리퍼 열기
    if (!open_gripper_sync()) return;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 팔 이동
    move_group_->setStartStateToCurrentState();
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.2);
    base_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 1, 0, 0));  // pitch 90도

    move_group_->setPoseTarget(base_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
      rclcpp::sleep_for(std::chrono::seconds(1));
      close_gripper_sync();
    } else {
      RCLCPP_WARN(get_logger(), "팔 경로 계획 실패.");
    }
  }

  bool open_gripper_sync() {
    GripperCommand::Goal goal;
    goal.command.position = 0.018;
    goal.command.max_effort = 10.0;

    auto goal_future = gripper_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_future) != rclcpp::FutureReturnCode::SUCCESS)
      return false;

    auto goal_handle = goal_future.get();
    if (!goal_handle) return false;

    auto result_future = gripper_client_->async_get_result(goal_handle);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  bool close_gripper_sync() {
    GripperCommand::Goal goal;
    goal.command.position = 0.0;
    goal.command.max_effort = 20.0;

    auto goal_future = gripper_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_future) != rclcpp::FutureReturnCode::SUCCESS)
      return false;

    auto goal_handle = goal_future.get();
    if (!goal_handle) return false;

    auto result_future = gripper_client_->async_get_result(goal_handle);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoGraspExecutor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}