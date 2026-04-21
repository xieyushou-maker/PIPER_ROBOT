# 实现四个电机的联合转动
# 导入所需的库和工具
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ros_robot_controller_msgs.msg import MotorState, MotorsState

# 底层驱动类：麦克纳姆轮底盘运动学
class MecanumChassis:
    # 初始化定义麦克纳姆轮底盘的参数
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    # 线速度转换为电机转速（单位：rad/s)
    def speed_covert(self, speed):
        return speed / (math.pi * self.wheel_diameter)

    # 麦轮底盘运动学分析所学的四个轮子的线速度计算
    def set_velocity(self, linear_x, linear_y, angular_z):
        # 轮子对应位置，motor1:左上；motor2:左下；motor3：右上；motor4：右下
        motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        # 这里的正负号翻转，适配物理电机安装方向与数学坐标系的差异
        v_s = [self.speed_covert(v) for v in [-motor1, -motor2, motor3, motor4]]
        # 封装四个电机状态包含ID和转速
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)

        msg = MotorsState()
        msg.data = data
        return msg

class MultiMotor(Node):
    def __init__(self):
        super().__init__('multi_motor_node')
        self.pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
        # 实例化运动学对象
        self.chassis = MecanumChassis()
        # 定义速度参数
        self.vx = 0.15
        self.vy = 0.0
        self.vz = 0.0
        self.timer = self.create_timer(0.5, self.run_motors)
        print(f"正在以 vx={self.vx}, vy={self.vy}, vz={self.vz} 运动，按 Ctrl+C 停止。")

    def run_motors(self):
        msg = self.chassis.set_velocity(self.vx, self.vy, self.vz)
        self.pub.publish(msg)

    def stop_motors(self):
        print("\n底盘紧急停止...")
        msg = self.chassis.set_velocity(0.0, 0.0, 0.0)
        self.pub.publish(msg)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MultiMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_motors()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("程序已安全退出。")

if __name__ == '__main__':
    main()