import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from lane_detect import slide_window
from lane_detect import camera_processing
from std_msgs.msg import Float64, UInt8
from geometry_msgs.msg import Twist
from aruco_msgs.msg import Marker, MarkerArray
import subprocess

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image,
            # 'video_frames',
            # '/camera/image',
            '/image_raw',
            self.listener_callback,
            10
        )
        self.image_publisher = self.create_publisher(Image, 'processed_frames', 10)
        # self.marker_publisher = self.create_publisher(Marker, 'lane_info_marker', 10)
        # 중앙선 위치 (Float64)
        self.pub_cmd_vel = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        self.sub_marker = self.create_subscription(
            MarkerArray,
            '/marker_array',
            self.marker_callback,
            10
        )

        # 내부 상태 변수
        self.last_error = 0.0
        self.lane_state = 0
        self.MAX_VEL = 0.22/2  # 직선 구간 최대 속도
        self.MIN_VEL = 0.08/2  # 회전 구간 속도

        self.bridge = CvBridge()

        self.camera_processor = camera_processing.CameraProcessing()
        self.slide_window_processor = slide_window.SlideWindow()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_shape = frame.shape # (240, 320) -> 480, 640
        self.get_logger().info(f'frame_shape {frame_shape}')
        detected, left, right, processed = self.lane_detect(frame)
        

        center = float((left + right) / 2)
        if detected == 'left':
            center += 0.25 * 320 # frin 640 / 2
        elif detected == 'right':
            center -= 0.25 * 320

        error = center - 320.  # 기준 중심값 320 (카메라 기준 중심)

        # PID 제어 중 PD 제어
        Kp = 0.005
        Kd = 0.007
        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        twist = Twist()

        # 속도 결정: 직선 구간은 빠르게, 곡선 구간은 정확하게
        if detected == 'both':
            twist.linear.x = self.MAX_VEL  # 직선
        else:
            twist.linear.x = self.MIN_VEL  # 곡선

        # 조향 각도 제한 (안정성)
        twist.angular.z = -max(min(angular_z, 2.0), -2.0)

        self.pub_cmd_vel.publish(twist)

        processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
        self.image_publisher.publish(processed_msg)

        info_text = f'Left position: {left}, Right position: {right}'

        # marker = Marker()
        # marker.header.frame_id = "map"
        # marker.header.stamp = self.get_clock().now().to_msg()
        # marker.type = Marker.TEXT_VIEW_FACING
        # marker.action = Marker.ADD
        # marker.pose.position.z = 2.0
        # marker.scale.z = 0.5
        # marker.color.a = 1.0
        # marker.color.r = 1.0
        # marker.color.g = 1.0
        # marker.color.b = 1.0
        # marker.text = info_text
        # self.marker_publisher.publish(marker)

    def lane_detect(self, frame):
        frame, filtered = self.camera_processor.process_image(frame)

        if frame is not None:
            slide_frame = frame[frame.shape[0] - 200 :frame.shape[0] - 150, :] # frame.shape # (240, 320) -> Real bot (480, 640)
            detected, left, right, tmp_frame = self.slide_window_processor.slide(slide_frame)
            processed_frame = self.slide_window_processor.lane_visualization(frame,left,right)
            self.processed_frame = processed_frame
            return detected, left, right, self.processed_frame
        return False, None, None, frame

    def marker_callback(self, marker_array_msg):
        if not marker_array_msg.markers:
            return

        z0 = marker_array_msg.markers[0].pose.pose.position.z
        self.get_logger().info(f"Detected marker z0 = {z0}")

        if z0 < 0.5:
            self.get_logger().info('Manipulation condition met. Launching grasp_executor...')
            import subprocess

            try:
                subprocess.run(
                    ['ros2', 'run', 'grasp_executor_cpp', 'grasp_executor'],
                    check=True
                )
                self.get_logger().info("grasp_executor launch 성공")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"grasp_executor launch 실패: {e}")
            
    # 노드 종료 시 cmd_vel = 0
    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)
              
def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)
    cv2.destroyAllWindows()
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

