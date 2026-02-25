#!/usr/bin/env python3
"""
Vive ROS2 브릿지 노드
========================
목적: vive_ros2 패키지의 VRControllerData 메시지를
      표준 ROS2 메시지(PoseStamped, Joy)로 변환하여 재발행한다.

구독 토픽: /controller_data (vive_ros2/msg/VRControllerData)
발행 토픽:
  - /vive/controller/pose (geometry_msgs/PoseStamped) : 컨트롤러 포즈
  - /vive/controller/buttons (sensor_msgs/Joy) : 버튼/축 상태

Joy 메시지 매핑:
  axes:    [trigger, trackpad_x, trackpad_y]
  buttons: [trigger_button, grip_button, menu_button, trackpad_touch, trackpad_button]

controller_role: 0=오른손, 1=왼손 (선택한 컨트롤러만 전달)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from vive_ros2.msg import VRControllerData


class ViveRos2Bridge(Node):
    """VRControllerData를 PoseStamped + Joy로 변환하는 ROS2 브릿지 노드"""

    def __init__(self):
        super().__init__('vive_ros2_bridge_node')

        # 파라미터: 토픽명 및 컨트롤러 선택
        self.declare_parameter('controller_data_topic', 'controller_data')
        self.declare_parameter('pose_out_topic', '/vive/controller/pose')
        self.declare_parameter('buttons_out_topic', '/vive/controller/buttons')
        self.declare_parameter('controller_role', 0)

        controller_data_topic = self.get_parameter('controller_data_topic').value
        pose_out_topic = self.get_parameter('pose_out_topic').value
        buttons_out_topic = self.get_parameter('buttons_out_topic').value
        role = self.get_parameter('controller_role').value
        self.controller_role = int(role) if isinstance(role, str) else role

        # 구독: VRControllerData
        self.controller_data_sub = self.create_subscription(
            VRControllerData, controller_data_topic, self.controller_data_callback, 10
        )

        # 발행: PoseStamped + Joy
        self.pose_pub = self.create_publisher(PoseStamped, pose_out_topic, 10)
        self.buttons_pub = self.create_publisher(Joy, buttons_out_topic, 10)

        role_name = '오른손' if self.controller_role == 0 else '왼손'
        self.get_logger().info(f'[브릿지] 노드 시작 (컨트롤러: {role_name})')

    def controller_data_callback(self, msg: VRControllerData):
        """VRControllerData → PoseStamped + Joy 변환 및 발행"""
        if msg.role != self.controller_role:
            return

        # abs_pose(TransformStamped) → PoseStamped 변환
        t = msg.abs_pose
        pose = PoseStamped()
        pose.header = t.header
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation
        self.pose_pub.publish(pose)

        # 버튼/축 → Joy 변환
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = 'vive_controller'
        joy.axes = [float(msg.trigger), msg.trackpad_x, msg.trackpad_y]
        joy.buttons = [
            1 if msg.trigger_button else 0,
            1 if msg.grip_button else 0,
            1 if msg.menu_button else 0,
            1 if msg.trackpad_touch else 0,
            1 if msg.trackpad_button else 0,
        ]
        self.buttons_pub.publish(joy)


def main(args=None):
    rclpy.init(args=args)
    node = ViveRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('[브릿지] 노드 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
