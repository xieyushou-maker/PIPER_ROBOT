#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import math
import time
import threading
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS 接口
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from interfaces.msg import ObjectsInfo
from kinematics_msgs.srv import SetRobotPose
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class FinalMissionNode(Node):
    def __init__(self):
        super().__init__('final_mission_node')
        self.group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # 状态变量
        self.latest_depth_img = None
        self.intrinsics = None
        self.is_arm_busy = False
        self.pick_success = False
        self.mission_state = 'IDLE'  # IDLE, NAV_TO_PICK, SCANNING, PICKING, NAV_TO_PLACE, PLACING
        self.target_class = ""

        # 导航客户端 (使用 Nav2 的导航动作)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.group)

        # 机械臂与底盘
        self.arm_pub = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
        self.ik_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target', callback_group=self.group)

        # 视觉订阅
        self.create_subscription(CameraInfo, '/ascamera/camera_publisher/rgb0/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/ascamera/camera_publisher/depth0/image_raw', self.depth_callback, 10)
        self.create_subscription(ObjectsInfo, '/yolo_node/object_detect', self.yolo_callback, 1,
                                 callback_group=self.group)

        # 启动主线程
        threading.Thread(target=self.run_mission_thread, daemon=True).start()
        self.get_logger().info('>>> 已融合 Nav2 语义导航系统！')

    # ================= 导航调度接口 =================
    def nav_to_pose(self, x, y, theta):
        """调用 Nav2 自动规划路径到达坐标点"""
        self.get_logger().info(f"导航至地图点: x={x}, y={y}")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        return True

    # ================= 任务剧本 =================
    def run_mission_thread(self):
        time.sleep(5.0)  # 等待 Nav2 完全启动
        self.move_to_home_raw()

        # 1. 导航到抓取区 (这些坐标需在 RViz 中测量填入)
        self.mission_state = 'NAV_TO_PICK'
        self.nav_to_pose(1.2, -1.0, 0.0)

        # 2. 扫描
        self.mission_state = 'SCANNING'
        self.target_class = input("请输入目标类别: ").strip()
        while not self.pick_success and rclpy.ok(): time.sleep(0.5)

        # 3. 导航到放置区
        self.mission_state = 'NAV_TO_PLACE'
        self.nav_to_pose(0.0, 0.0, math.pi)

        # 4. 放置
        self.mission_state = 'PLACING'
        self.execute_place_logic(0.15, 0.05, -0.01, -90.0)
        self.move_to_home_raw()
        self.get_logger().info("任务结束。")

    # ================= 抓取逻辑 =================
    def yolo_callback(self, msg):
        if self.mission_state != 'SCANNING' or self.is_arm_busy: return
        for obj in msg.objects:
            if obj.class_name == self.target_class:
                self.is_arm_busy = True
                cx, cy = (obj.box[0] + obj.box[2]) / 2, (obj.box[1] + obj.box[3]) / 2
                if self.execute_pick_logic(cx, cy) == "SUCCESS":
                    self.pick_success = True
                self.is_arm_busy = False
                break

    def execute_pick_logic(self, cx, cy):
        if self.latest_depth_img is None or self.intrinsics is None: return "FAILED"
        z_c = np.mean(self.latest_depth_img[int(cy) - 10:int(cy) + 10, int(cx) - 10:int(cx) + 10]) / 1000.0
        x_c = (cx - self.intrinsics[2]) * z_c / self.intrinsics[0]
        y_c = (cy - self.intrinsics[5]) * z_c / self.intrinsics[4]

        px = (z_c * 0.707) - (y_c * 0.707) + 0.085
        py = -x_c
        pz = max(0.01, min(0.22 - ((y_c * 0.707) + (z_c * 0.707)), 0.25))

        self.set_gripper('open')
        if self.move_to_target(px, py, pz + 0.05, -45.0):
            if self.move_to_target(px, py, pz, -45.0):
                self.set_gripper('close')
                self.move_to_target(px, py, pz + 0.08, -45.0)
                return "SUCCESS"
        return "FAILED"

    def execute_place_logic(self, x, y, z, p):
        self.move_to_target(x, y, z + 0.08, p)
        self.move_to_target(x, y, z, p)
        time.sleep(0.5)
        self.set_gripper('open')
        self.move_to_target(x, y, z + 0.08, p)

    # ================= 辅助方法 =================
    def info_callback(self, msg):
        self.intrinsics = msg.k

    def depth_callback(self, msg):
        self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def set_gripper(self, state):
        msg = ServosPosition(duration=0.5)
        msg.position.append(ServoPosition(id=10, position=200 if state == 'open' else 475))
        self.arm_pub.publish(msg)
        time.sleep(0.8)

    def move_to_home_raw(self):
        msg = ServosPosition(duration=2.0)
        msg.position = [ServoPosition(id=i + 1, position=p) for i, p in enumerate([500, 730, 130, 100, 500])]
        self.arm_pub.publish(msg)
        time.sleep(2.0)

    def move_to_target(self, x, y, z, pitch):
        req = SetRobotPose.Request()
        req.position, req.pitch = [float(x), float(y), float(z)], float(pitch)
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.success:
            msg = ServosPosition(duration=1.0)
            msg.position = [ServoPosition(id=i + 1, position=int(p)) for i, p in enumerate(res.pulse)]
            self.arm_pub.publish(msg)
            time.sleep(1.2)
            return True
        return False


def main():
    rclpy.init()
    node = FinalMissionNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__": main()