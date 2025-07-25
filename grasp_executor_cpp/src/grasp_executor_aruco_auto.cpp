#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <aruco_msgs/msg/marker_array.hpp>
#include <cmath>
#include <thread>
#include <cstdlib>  // for exit()

class ArucoGraspExecutor : public rclcpp::Node {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  ArucoGraspExecutor()
  : Node("grasp_executor_cpp_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_) {
    RCLCPP_INFO(this->get_logger(), "노드 생성됨 (초기화 전)");

    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "arm");

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(node_ptr, "/gripper_controller/gripper_cmd");

    marker_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
      "/marker_array", 10,
      std::bind(&ArucoGraspExecutor::marker_callback, this, std::placeholders::_1));

    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "그리퍼 액션 서버 연결 실패.");
    }

    RCLCPP_INFO(get_logger(), "ArUco 기반 grasp executor 시작됨.");
  }

  bool is_finished() const {
    return finished_;
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool executed_ = false;
  bool finished_ = false;

  void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
    if (executed_ || msg->markers.empty()) return;

    const auto& marker = msg->markers[0];
    if (marker.pose.pose.position.z >= 0.05) return;

    executed_ = true;

    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header.frame_id = "camera_link";
    camera_pose.header.stamp = this->get_clock()->now();
    camera_pose.pose = marker.pose.pose;

    geometry_msgs::msg::PoseStamped base_pose;
    try {
      base_pose = tf_buffer_.transform(camera_pose, "base_link", tf2::durationFromSec(2.0));
      base_pose.pose.position.z -= 0.35;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF 변환 실패: %s", ex.what());
      executed_ = false;
      return;
    }

    std::thread([this, base_pose]() {
      open_gripper();
      std::this_thread::sleep_for(std::chrono::seconds(1));

      if (!move_arm_to_pose(base_pose)) {
        executed_ = false;
        return;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));

      close_gripper();
      std::this_thread::sleep_for(std::chrono::seconds(1));

      geometry_msgs::msg::PoseStamped release_pose = base_pose;
      release_pose.pose.position.y += 0.15;

      if (!move_arm_to_pose(release_pose)) {
        executed_ = false;
        return;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));

      open_gripper();
      std::this_thread::sleep_for(std::chrono::seconds(1));

      move_group_->setNamedTarget("home");
      moveit::planning_interface::MoveGroupInterface::Plan home_plan;
      if (move_group_->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_->execute(home_plan);
        RCLCPP_INFO(get_logger(), "홈 위치로 복귀 완료.");
      } else {
        RCLCPP_WARN(get_logger(), "홈 위치 이동 실패.");
      }

      std::this_thread::sleep_for(std::chrono::seconds(2));
      RCLCPP_INFO(get_logger(), "Grasp flow 완료. 노드를 종료합니다.");

      rclcpp::shutdown();
      exit(0);
    }).detach();
  }

  bool move_arm_to_pose(const geometry_msgs::msg::PoseStamped &pose) {
    move_group_->setStartStateToCurrentState();
    move_group_->setGoalPositionTolerance(0.1);
    move_group_->setGoalOrientationTolerance(0.5);
    move_group_->setPlanningTime(10.0);

    geometry_msgs::msg::PoseStamped target = pose;
    tf2::Quaternion q;
    q.setRPY(0, M_PI / 2.0, 0);
    target.pose.orientation = tf2::toMsg(q);

    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "팔 이동 완료");
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "팔 이동 실패");
      return false;
    }
  }

  void open_gripper() {
    GripperCommand::Goal goal;
    goal.command.position = 0.018;
    goal.command.max_effort = 10.0;

    auto result_future = gripper_client_->async_send_goal(goal);
    auto goal_handle = result_future.get();
    if (goal_handle) {
      auto result = gripper_client_->async_get_result(goal_handle).get();
      RCLCPP_INFO(get_logger(), "그리퍼 열기 %s", result.code == rclcpp_action::ResultCode::SUCCEEDED ? "성공" : "실패");
    }
  }

  void close_gripper() {
    GripperCommand::Goal goal;
    goal.command.position = 0.0;
    goal.command.max_effort = 20.0;

    auto result_future = gripper_client_->async_send_goal(goal);
    auto goal_handle = result_future.get();
    if (goal_handle) {
      auto result = gripper_client_->async_get_result(goal_handle).get();
      RCLCPP_INFO(get_logger(), "그리퍼 닫기 %s", result.code == rclcpp_action::ResultCode::SUCCEEDED ? "성공" : "실패");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoGraspExecutor>();
  rclcpp::spin(node);
  return 0;
}