# 实现单个电机的精准线速度转动
# 导入所需的库和工具
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ros_robot_controller_msgs.msg import MotorState, MotorsState

# 运动学分析
class SingleMotorChassis:
    # 初始化定义轮子的参数（仅需轮子直径）
    def __init__(self, wheel_diameter=0.065):
        self.wheel_diameter = wheel_diameter

    # 线速度转换为电机转速（单位：rps rad/秒)
    def speed_covert(self, speed):
        return speed / (math.pi * self.wheel_diameter)

    # 单电机的线速度计算与消息封装（对应最终版的 set_velocity）
    def set_velocity(self, motor_id, linear_speed):
        # 将期望的线速度转换为物理圈数
        target_rps = self.speed_covert(linear_speed)
        # 封装单个电机状态包含ID和转速
        motor = MotorState()
        motor.id = motor_id
        motor.rps = float(target_rps)
        # 将电机状态封装为消息类
        msg = MotorsState()
        msg.data = [motor]
        return msg

# 应用控制节点
class SingleMotorTask(Node):
    def __init__(self):
        super().__init__('single_motor_task_node')
        self.pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
        # 实例化运动学对象
        self.chassis = SingleMotorChassis()
        self.MOTOR_ID = 4  # 电机ID：1左上 2左下 3右上 4右下
        self.LINEAR_SPEED = 0.15  # 目标速度：0.15米/秒
        print(f"{self.MOTOR_ID}号电机将以 {self.LINEAR_SPEED}m/s 的线速度持续转动。")
        print("按下 Ctrl+C 停止转动。")
        # 定时器：每0.5秒发一次指令，保持转动
        self.timer = self.create_timer(0.5, self.run_motor)

    def run_motor(self):
        # 获取电机ID和线速度计算得到的转速消息并发布
        msg = self.chassis.set_velocity(self.MOTOR_ID, self.LINEAR_SPEED)
        self.pub.publish(msg)

    def emergency_stop(self):
        print("\n电机停止转动")
        # 紧将线速度设为 0.0
        msg = self.chassis.set_velocity(self.MOTOR_ID, 0.0)
        self.pub.publish(msg)
        time.sleep(0.2)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SingleMotorTask()
    try:
        # 程序持续运行，直到按下 Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.emergency_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("程序安全退出。")


if __name__ == '__main__':
    main()