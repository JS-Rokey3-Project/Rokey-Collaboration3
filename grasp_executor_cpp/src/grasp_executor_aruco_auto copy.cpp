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

    RCLCPP_INFO(get_logger(), "ArUco 기반 grasp executor 시작됨.");

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
  bool executed_ = false;

  void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
    if (executed_ || msg->markers.empty()) return;
    executed_ = true;

    auto closest_marker = *std::min_element(
      msg->markers.begin(), msg->markers.end(),
      [](const auto &a, const auto &b) {
        return a.pose.pose.position.z < b.pose.pose.position.z;
      });

    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header.frame_id = "camera_link";
    camera_pose.header.stamp = this->get_clock()->now();
    camera_pose.pose = closest_marker.pose.pose;

    geometry_msgs::msg::PoseStamped base_pose;
    try {
      base_pose = tf_buffer_.transform(camera_pose, "base_link", tf2::durationFromSec(2.0));
      base_pose.pose.position.z -= 0.35;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF 변환 실패: %s", ex.what());
      return;
    }

    RCLCPP_INFO(get_logger(), "마커 위치(base_link): x=%.3f, y=%.3f, z=%.3f",
                base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z);

    if (base_pose.pose.position.z > 0.8 || base_pose.pose.position.z < 0.05) {
      RCLCPP_WARN(get_logger(), "Z 위치 %.3f 가 허용 범위를 벗어났습니다. grasp 생략.", base_pose.pose.position.z);
      return;
    }

    open_gripper_async([this, base_pose]() {
      std::this_thread::sleep_for(std::chrono::seconds(1));

      move_arm_to_pose(base_pose, [this, base_pose]() {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        close_gripper_async([this, base_pose]() {
          std::this_thread::sleep_for(std::chrono::seconds(1));

          geometry_msgs::msg::PoseStamped release_pose = base_pose;
          release_pose.pose.position.y += 0.15;

          move_arm_to_pose(release_pose, [this]() {
            std::this_thread::sleep_for(std::chrono::seconds(1));

            open_gripper_async([this]() {
              std::this_thread::sleep_for(std::chrono::seconds(1));

              move_group_->setNamedTarget("home");
              moveit::planning_interface::MoveGroupInterface::Plan home_plan;
              if (move_group_->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                move_group_->execute(home_plan);
                RCLCPP_INFO(get_logger(), "홈 위치로 복귀 완료.");
              } else {
                RCLCPP_WARN(get_logger(), "홈 위치 이동 실패.");
              }

              std::this_thread::sleep_for(std::chrono::seconds(3));

              RCLCPP_INFO(get_logger(), "Grasp flow 완료. grasp_executor_cpp 종료 준비됨.");
              // 전체 플로우 완료 후 종료
              rclcpp::shutdown();
            });
          });
        });
      });
    });
  }

  void open_gripper_async(std::function<void()> next_step) {
    GripperCommand::Goal goal;
    goal.command.position = 0.018;
    goal.command.max_effort = 10.0;

    auto future_goal_handle = gripper_client_->async_send_goal(goal);
    std::thread([this, future_goal_handle = std::move(future_goal_handle), next_step]() mutable {
      auto goal_handle = future_goal_handle.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "그리퍼 열기 goal_handle 없음");
        return;
      }

      auto result_future = gripper_client_->async_get_result(goal_handle);
      auto result = result_future.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "그리퍼 열기 완료");
        next_step();
      } else {
        RCLCPP_WARN(this->get_logger(), "그리퍼 열기 실패");
      }
    }).detach();
  }

  void close_gripper_async(std::function<void()> next_step) {
    GripperCommand::Goal goal;
    goal.command.position = 0.0;
    goal.command.max_effort = 20.0;

    auto future_goal_handle = gripper_client_->async_send_goal(goal);
    std::thread([this, future_goal_handle = std::move(future_goal_handle), next_step]() mutable {
      auto goal_handle = future_goal_handle.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "그리퍼 닫기 goal_handle 없음");
        return;
      }

      auto result_future = gripper_client_->async_get_result(goal_handle);
      auto result = result_future.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "그리퍼 닫기 완료");
        next_step();
      } else {
        RCLCPP_WARN(this->get_logger(), "그리퍼 닫기 실패");
      }
    }).detach();
  }

  void move_arm_to_pose(const geometry_msgs::msg::PoseStamped &pose, std::function<void()> next_step) {
    move_group_->setStartStateToCurrentState();
    move_group_->setGoalPositionTolerance(0.1);
    move_group_->setGoalOrientationTolerance(0.5);
    move_group_->setPlanningTime(10.0);

    geometry_msgs::msg::PoseStamped target = pose;
    tf2::Quaternion q;
    q.setRPY(0, M_PI / 2.0, 0);  // pitch 90도
    target.pose.orientation = tf2::toMsg(q);

    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "팔 이동 완료");
      next_step();
    } else {
      RCLCPP_ERROR(this->get_logger(), "팔 이동 실패 (계획 실패 또는 제한 시간 초과)");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoGraspExecutor>();
  rclcpp::spin(node);  // spin은 내부에서 shutdown() 호출 시 자동 종료됨
  return 0;
}