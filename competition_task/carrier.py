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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from interfaces.msg import ObjectsInfo
from kinematics_msgs.srv import SetRobotPose
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class MobilePickPlaceMission(Node):
    def __init__(self):
        super().__init__('mobile_pick_place_node')
        self.group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # --- 共享状态变量 ---
        self.latest_depth_img = None
        self.intrinsics = None
        self.is_arm_busy = False
        self.mission_state = 'INIT'
        self.pick_success = False
        self.detected_classes = set()
        self.target_class = ""

        # 里程计位姿变量
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.get_logger().info('⏳ [系统] 正在初始化节点与通信...')

        # 初始化控制与订阅
        self.chassis_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.group)
        self.arm_pub = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
        self.ik_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target', callback_group=self.group)

        self.create_subscription(CameraInfo, '/ascamera/camera_publisher/rgb0/camera_info', self.info_callback, 10,
                                 callback_group=self.group)
        self.create_subscription(Image, '/ascamera/camera_publisher/depth0/image_raw', self.depth_callback, 10,
                                 callback_group=self.group)
        self.create_subscription(ObjectsInfo, '/yolo_node/object_detect', self.yolo_callback, 1,
                                 callback_group=self.group)

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ [系统] 等待机械臂逆解服务启动...')

        threading.Thread(target=self.mission_workflow_thread, daemon=True).start()
        self.get_logger().info('✅ [系统] 复合机器人搬运系统已全面启动！')

    def mission_workflow_thread(self):
        time.sleep(2.0)
        self.get_logger().info('🦾 [初始化] 机械臂回归安全初始姿态')
        self.move_to_home_raw(duration=2.0)

        # 1. 前往抓取区 (PID 闭环)
        self.mission_state = 'STATE_1_MOVE_TO_PICK'
        self.get_logger().info('\n' + '=' * 40 + '\n🚀 [阶段1] 出发前往抓取区\n' + '=' * 40)
        self.move_chassis_by_odom(0.15, 0.0, 0.0, target_distance=0.7)
        self.move_chassis_by_odom(0.0, 0.0, -0.5, target_angle=math.pi / 2)
        self.move_chassis_by_odom(0.15, 0.0, 0.0, target_distance=0.5)

        # 2. 扫描与输入
        self.mission_state = 'STATE_2A_SCANNING'
        self.get_logger().info('\n' + '=' * 40 + '\n👁️ [阶段2A] 到达抓取区，开启环境扫描\n' + '=' * 40)
        time.sleep(2.5)
        self.target_class = input("\n---> 请输入你要抓取的物块类别: ").strip()
        self.get_logger().info(f'🎯 [指令] 已锁定目标类别: {self.target_class}')

        # 3. 视觉抓取
        self.mission_state = 'STATE_2B_PICKING'
        self.get_logger().info('🔄 [阶段2B] 视觉伺服启动，等待目标出现在视野中...')
        while not self.pick_success and rclpy.ok(): time.sleep(0.5)

        self.get_logger().info('🦾 [抓取完成] 机械臂回归安全姿态，准备转移')
        self.move_to_home_raw(duration=2.0)

        # 4. 前往放置区
        self.mission_state = 'STATE_3_MOVE_TO_PLACE'
        self.get_logger().info('\n' + '=' * 40 + '\n🚚 [阶段3] 抓取成功，前往放置区\n' + '=' * 40)
        self.move_chassis_by_odom(0.0, 0.0, -0.5, target_angle=math.pi / 2)
        self.move_chassis_by_odom(0.15, 0.0, 0.0, target_distance=0.8)
        self.move_chassis_by_odom(0.0, 0.0, 0.5, target_angle=math.pi / 2)
        self.move_chassis_by_odom(0.1, 0.0, 0.0, target_distance=0.1)

        # 5. 放置与返航
        self.mission_state = 'STATE_4_PLACING'
        self.get_logger().info('\n' + '=' * 40 + '\n📦 [阶段4] 抵达放置区，开始精准触地放置\n' + '=' * 40)
        self.execute_place_logic(0.15, 0.05, -0.01, -90.0)
        self.move_to_home_raw(duration=2.0)

        self.mission_state = 'STATE_5_RETURN'
        self.get_logger().info('\n' + '=' * 40 + '\n🏠 [阶段5] 放置完毕，底盘返航\n' + '=' * 40)
        self.move_chassis_by_odom(-0.15, 0.0, 0.0, target_distance=0.5)
        self.get_logger().info('🎉🎉🎉 [任务结束] 全部任务完美结束！')

    def yolo_callback(self, msg):
        if self.mission_state != 'STATE_2B_PICKING' or self.is_arm_busy or self.pick_success: return
        for obj in msg.objects:
            if obj.class_name == self.target_class:
                self.is_arm_busy = True
                cx = (obj.box[0] + obj.box[2]) / 2
                cy = (obj.box[1] + obj.box[3]) / 2
                self.get_logger().info(f'👁️ [视觉] 捕获目标 {self.target_class}，像素中心: ({cx:.1f}, {cy:.1f})')

                status = self.execute_pick_logic(cx, cy)
                if status == "SUCCESS":
                    self.get_logger().info('✅ [抓取] 目标抓取判定成功！')
                    self.pick_success = True

                self.is_arm_busy = False
                break

    def execute_pick_logic(self, cx, cy):
        coords = self.get_3d_coordinates(cx, cy)
        if coords is None:
            self.get_logger().warn('⚠️ [视觉] 深度信息解算失败，等待下一帧...')
            return "FAILED"

        x_c, y_c, z_c = coords
        px = (z_c * 0.707) - (y_c * 0.707) + 0.085
        py = -x_c
        pz = max(0.01, min(0.22 - ((y_c * 0.707) + (z_c * 0.707)), 0.25))
        self.get_logger().info(f'📐 [深度解算] 目标真实物理坐标: X={px:.3f}m, Y={py:.3f}m, Z={pz:.3f}m')

        # 判断是否需要底盘微调补偿
        if not (0.10 <= px <= 0.20 and -0.08 <= py <= 0.08):
            self.compensate_chassis(px, py)
            return "NEED_COMPENSATION"

        self.get_logger().info('🎯 [抓取] 目标已在绝佳抓取范围内，机械臂出击！')
        self.set_gripper('open')
        if self.move_to_target(px, py, pz + 0.05, -45.0):
            if self.move_to_target(px, py, pz, -45.0):
                time.sleep(0.3)
                self.set_gripper('close')
                self.move_to_target(px, py, pz + 0.08, -45.0)
                return "SUCCESS"
        return "FAILED"

    def compensate_chassis(self, px, py):
        IDEAL_X, IDEAL_Y = 0.15, 0.0
        dx, dy = px - IDEAL_X, py - IDEAL_Y
        self.get_logger().warn(f'⚠️ [视觉微调] 目标偏离安全区！偏差: 前后 {dx:.3f}m, 左右 {dy:.3f}m')

        COMP_SPEED = 0.04
        if abs(dx) > 0.03:
            self.get_logger().info(f'🚙 [视觉微调] 执行前后移动补偿...')
            self.move_chassis_time((COMP_SPEED if dx > 0 else -COMP_SPEED), 0.0, 0.0, abs(dx) / COMP_SPEED)
        if abs(dy) > 0.03:
            self.get_logger().info(f'🚙 [视觉微调] 执行左右横移补偿...')
            self.move_chassis_time(0.0, (COMP_SPEED if dy > 0 else -COMP_SPEED), 0.0, abs(dy) / COMP_SPEED)

        self.get_logger().info('⏳ [视觉微调] 补偿动作结束，强制冷却等待图像稳定 (2.0s)...')
        time.sleep(2.0)

        # ================= 里程计闭环与回调 =================

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        # 节流日志：每隔 5 秒打印一次 Odom 坐标，证明底层一直在实时更新
        self.get_logger().info(
            f'🌐 [Odom状态] 实时坐标: X={self.current_x:.2f}m, Y={self.current_y:.2f}m, Yaw={math.degrees(self.current_yaw):.1f}°',
            throttle_duration_sec=5.0)

    def move_chassis_by_odom(self, vx, vy, vz, target_distance=None, target_angle=None):
        start_x, start_y, start_yaw = self.current_x, self.current_y, self.current_yaw
        msg = Twist()

        # 平移闭环
        if target_distance and target_distance > 0:
            self.get_logger().info(f'▶️ [Odom闭环] 开始平移，目标距离: {target_distance:.3f}m')
            dist = 0.0
            while rclpy.ok():
                dist = math.hypot(self.current_x - start_x, self.current_y - start_y)
                if dist >= target_distance - 0.01: break

                msg.linear.x, msg.linear.y = vx * (1 - dist / target_distance), vy * (1 - dist / target_distance)
                msg.angular.z = 1.5 * (start_yaw - self.current_yaw)  # 航向锁
                self.chassis_pub.publish(msg)
                time.sleep(0.02)
            self.get_logger().info(f'⏹️ [Odom闭环] 平移完成，实际移动距离: {dist:.3f}m')

        # 旋转闭环
        elif target_angle and target_angle > 0:
            self.get_logger().info(f'▶️ [Odom闭环] 开始旋转，目标角度: {math.degrees(target_angle):.1f}°')
            dyaw = 0.0
            while rclpy.ok():
                dyaw = math.atan2(math.sin(self.current_yaw - start_yaw), math.cos(self.current_yaw - start_yaw))
                if abs(dyaw) >= target_angle - 0.02: break

                msg.angular.z = (0.2 if vz > 0 else -0.2)
                self.chassis_pub.publish(msg)
                time.sleep(0.02)
            self.get_logger().info(f'⏹️ [Odom闭环] 旋转完成，实际旋转角度: {math.degrees(abs(dyaw)):.1f}°')

        self.chassis_pub.publish(Twist())  # 强制停车

    def move_chassis_time(self, vx, vy, vz, duration):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, vz
        self.chassis_pub.publish(msg)
        time.sleep(duration)
        self.chassis_pub.publish(Twist())

    # ================= 辅助函数 =================
    def info_callback(self, msg):
        self.intrinsics = msg.k

    def depth_callback(self, msg):
        self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def set_gripper(self, state):
        self.get_logger().info(f'🤏 [夹爪] 执行动作: {"张开" if state == "open" else "闭合"}')
        msg = ServosPosition(duration=0.5)
        msg.position.append(ServoPosition(id=10, position=200 if state == 'open' else 475))
        self.arm_pub.publish(msg)
        time.sleep(0.8)

    def move_to_home_raw(self, duration=2.0):
        msg = ServosPosition(duration=duration)
        msg.position = [ServoPosition(id=i + 1, position=p) for i, p in enumerate([500, 730, 130, 100, 500])]
        self.arm_pub.publish(msg)
        time.sleep(duration)

    def move_to_target(self, x, y, z, pitch):
        self.get_logger().info(f'🦾 [机械臂] 逆解请求: X={x:.3f}, Y={y:.3f}, Z={z:.3f}, Pitch={pitch}')
        req = SetRobotPose.Request()
        req.position, req.pitch = [float(x), float(y), float(z)], float(pitch)
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            msg = ServosPosition(duration=1.0)
            msg.position = [ServoPosition(id=i + 1, position=int(p)) for i, p in enumerate(future.result().pulse)]
            self.arm_pub.publish(msg)
            time.sleep(1.2)
            return True
        else:
            self.get_logger().error('❌ [机械臂] 逆解失败！此坐标超出臂展范围。')
            return False

    def execute_place_logic(self, x, y, z, p):
        self.get_logger().info('📦 [放置] 机械臂开始执行下降放置动作...')
        self.move_to_target(x, y, z + 0.08, p)
        self.move_to_target(x, y, z, p)
        time.sleep(0.5)
        self.set_gripper('open')
        self.move_to_target(x, y, z + 0.08, p)

    def get_3d_coordinates(self, u, v):
        if self.latest_depth_img is None or self.intrinsics is None: return None
        u_int, v_int = int(u), int(v)
        roi = self.latest_depth_img[v_int - 10:v_int + 10, u_int - 10:u_int + 10]
        valid_depths = roi[roi > 0]
        if len(valid_depths) == 0: return None
        z_c = np.mean(valid_depths) / 1000.0
        return (u - self.intrinsics[2]) * z_c / self.intrinsics[0], (v - self.intrinsics[5]) * z_c / self.intrinsics[
            4], z_c


def main():
    rclpy.init()
    node = MobilePickPlaceMission()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('⚠️ [系统] 收到中断信号，程序退出。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__": main()