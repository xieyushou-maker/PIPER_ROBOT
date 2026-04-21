# 规定速度实现底盘沿正方形
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

# 应用控制节点：走正方形任务
class KinematicSquare(Node):
    def __init__(self):
        super().__init__('kinematic_square_node')
        self.pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
        # 实例化运动学对象
        self.chassis = MecanumChassis()
        # 定义运动参数
        self.TARGET_DISTANCE = 0.5  # 目标距离：0.5米
        self.LINEAR_SPEED = 0.15  # 目标速度：0.15米/秒
        base_time = self.TARGET_DISTANCE / self.LINEAR_SPEED
        print(f"以速度 {self.LINEAR_SPEED}m/s，执行走边长 {self.TARGET_DISTANCE}m的正方形任务，")

        v = self.LINEAR_SPEED
        # 定义任务执行步骤（动作名称， vx ,vy ,vz ,持续时间）
        self.script = [
            ("第一步：沿 X 轴向前", v, 0.0, 0.0, base_time),
            ("第二步：沿 Y 轴向左", 0.0, v, 0.0, base_time),
            ("第三步：沿 X 轴向后", -v, 0.0, 0.0, base_time),
            ("第四步：沿 Y 轴向右", 0.0, -v, 0.0, base_time),
            ("终点站：刹车停止", 0.0, 0.0, 0.0, 0.5)
        ]
        self.current_step = 0
        self.step_start_time = time.time()
        # 每0.05秒响应一次run_script
        self.timer = self.create_timer(0.05, self.run_script)

    def run_script(self):
        # 定义执行步骤全部完成则退出系统
        if self.current_step >= len(self.script):
            self.timer.cancel()
            raise SystemExit

        step_name, vx, vy, vz, duration = self.script[self.current_step]
        elapsed_time = time.time() - self.step_start_time

        if elapsed_time <= duration:
            # 获取每一步骤电机转速消息并广播发布消息
            msg = self.chassis.set_velocity(vx, vy, vz)
            self.pub.publish(msg)
            if elapsed_time < 0.1:
                print(f"执行 {step_name} (预计 {duration:.2f} 秒)...")
        else:
            # 前一步骤按时完成，进行下一步骤
            self.current_step += 1
            self.step_start_time = time.time()

    def emergency_stop(self):
        print("正在紧急停止...")
        # 紧急制动将电机转速为0的消息广播发布
        msg = self.chassis.set_velocity(0.0, 0.0, 0.0)
        self.pub.publish(msg)
        time.sleep(0.2)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KinematicSquare()
    try:
        rclpy.spin(node)
    except SystemExit:
        print("完成走正方形的任务！")
    except KeyboardInterrupt:
        node.emergency_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()