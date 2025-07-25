// (1) grasp_x/y/z 입력
//       ↓
// (2) arm 조인트 경로 계획 (MoveIt)
//       ↓
// (3) 팔이 목표 위치로 이동
//       ↓
// (4) gripper 조인트 닫기 → 물체 잡기

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>

class GraspExecutor {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit GraspExecutor(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      node_, "/gripper_controller/gripper_cmd");

    RCLCPP_INFO(node_->get_logger(), "[grasp_executor_cpp] 노드 시작됨.");

    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "그리퍼 액션 서버 연결 실패.");
    }

    execute_grasp_from_params();
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  void execute_grasp_from_params() {
    node_->declare_parameter<double>("grasp_x", 0.0);
    node_->declare_parameter<double>("grasp_y", 0.0);
    node_->declare_parameter<double>("grasp_z", 0.0);
    node_->declare_parameter<std::string>("grasp_frame", "link5");

    double x, y, z;
    std::string frame_id;

    node_->get_parameter("grasp_x", x);
    node_->get_parameter("grasp_y", y);
    node_->get_parameter("grasp_z", z);
    node_->get_parameter("grasp_frame", frame_id);

    RCLCPP_INFO(node_->get_logger(), "grasp 위치: x=%.3f, y=%.3f, z=%.3f, frame=%s",
                x, y, z, frame_id.c_str());

    // 1. 그리퍼 열기
    if (!open_gripper_sync()) {
      RCLCPP_ERROR(node_->get_logger(), "그리퍼 열기 실패. 중단.");
      return;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));  // 약간 대기

    // 2. 목표 pose 설정
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = frame_id;
    target_pose.header.stamp = node_->now();
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(0, M_PI / 2.0, 0);  // pitch 90도: 아래를 향하도록
    target_pose.pose.orientation = tf2::toMsg(q);

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target_pose);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.2);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_WARN(node_->get_logger(), "경로 계획 실패.");
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "팔 이동 경로 계획 성공. 실행 중...");
    auto exec_status = move_group_->execute(plan);
    RCLCPP_INFO(node_->get_logger(), "execute() 결과 코드: %d", exec_status.val);

    if (exec_status == moveit::core::MoveItErrorCode::SUCCESS ||
        exec_status == moveit::core::MoveItErrorCode::TIMED_OUT ||
        exec_status == moveit::core::MoveItErrorCode::PREEMPTED) {
      RCLCPP_INFO(node_->get_logger(), "grasp 위치 도달. 그리퍼 닫기 실행...");
      rclcpp::sleep_for(std::chrono::seconds(1));  // 안정화 대기

      if (!close_gripper_sync()) {
        RCLCPP_ERROR(node_->get_logger(), "그리퍼 닫기 실패.");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "경로 실행 실패 (status: %d).", exec_status.val);
    }
  }

  bool open_gripper_sync() {
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = 0.018;  // 열림 위치
    goal_msg.command.max_effort = 10.0;

    auto goal_future = gripper_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "그리퍼 goal 전송 실패.");
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "그리퍼 goal 거부됨.");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "그리퍼 열기 goal 수락됨.");

    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "그리퍼 결과 수신 실패.");
      return false;
    }

    auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  bool close_gripper_sync() {
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = 0.0;   // 완전히 닫기
    goal_msg.command.max_effort = 20.0;

    auto goal_future = gripper_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "그리퍼 goal 전송 실패.");
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "그리퍼 goal 거부됨.");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "그리퍼 닫기 goal 수락됨.");

    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "그리퍼 결과 수신 실패.");
      return false;
    }

    auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("grasp_executor_cpp_node");
  auto executor = std::make_shared<GraspExecutor>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}