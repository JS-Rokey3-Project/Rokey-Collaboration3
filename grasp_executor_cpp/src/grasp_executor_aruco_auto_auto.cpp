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
#include <Eigen/Geometry>
#include <cmath>

class ArucoGraspExecutor {
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit ArucoGraspExecutor(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      tf_buffer_(node_->get_clock()),
      tf_listener_(tf_buffer_) {

    RCLCPP_INFO(node_->get_logger(), "[ArucoGraspExecutor] 노드 초기화 시작");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
    move_group_->setEndEffectorLink("end_effector_link");
    move_group_->setPoseReferenceFrame("base_link");

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(node_, "/gripper_controller/gripper_cmd");

    marker_sub_ = node_->create_subscription<aruco_msgs::msg::MarkerArray>(
      "/marker_array", 10,
      std::bind(&ArucoGraspExecutor::marker_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "ArUco grasp executor 시작됨.");

    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "그리퍼 액션 서버 연결 실패.");
    }
  }

  void spin() {
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node_);
      rate.sleep();
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
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

    RCLCPP_INFO(node_->get_logger(), "[마커 수신] ID: %d", closest_marker.id);
    RCLCPP_INFO(node_->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
                closest_marker.pose.pose.position.x,
                closest_marker.pose.pose.position.y,
                closest_marker.pose.pose.position.z);

    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header.frame_id = "camera_link";
    camera_pose.header.stamp = node_->now();
    camera_pose.pose = closest_marker.pose.pose;

    geometry_msgs::msg::PoseStamped base_pose;
    try {
      base_pose = tf_buffer_.transform(camera_pose, "base_link", tf2::durationFromSec(2.0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "TF 변환 실패: %s", ex.what());
      return;
    }

    if (base_pose.pose.position.z > 0.8 || base_pose.pose.position.z < 0.05) {
      RCLCPP_WARN(node_->get_logger(), "Z 위치 %.3f가 허용 범위 벗어남. grasp 취소", base_pose.pose.position.z);
      return;
    }

    // orientation 설정
    double roll = 0.0;
    double pitch = M_PI / 2.0;
    double yaw = 0.0;

    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    base_pose.pose.orientation.x = q.x();
    base_pose.pose.orientation.y = q.y();
    base_pose.pose.orientation.z = q.z();
    base_pose.pose.orientation.w = q.w();

    move_arm_to_pose(base_pose);
  }

  void move_arm_to_pose(const geometry_msgs::msg::PoseStamped &pose) {
    move_group_->setStartStateToCurrentState();
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.1);
    move_group_->setPlanningTime(10.0);
    move_group_->setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
      RCLCPP_INFO(node_->get_logger(), "✅ 팔 이동 완료");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "❌ 경로 계획 실패");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("grasp_executor_cpp_node");

  auto executor = std::make_shared<ArucoGraspExecutor>(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
