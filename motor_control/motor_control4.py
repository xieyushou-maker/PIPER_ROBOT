# 利用速度控制指令快速完成任务
# 必须先在终端启动：ros2 launch controller controller.launch.py
import rclpy
import time
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
# 导入ROS 2标准速度消息包
from geometry_msgs.msg import Twist


class TwistSquare(Node):
    def __init__(self):
        super().__init__('twist_square_node')
        # 创建/conroller/cmd_vel话题的发布者，发送Twist消息
        self.pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        # 定义运动参数
        self.TARGET_DISTANCE = 0.5  # 目标距离：0.5米
        self.LINEAR_SPEED = 0.15  # 目标速度：0.15米/秒
        base_time = self.TARGET_DISTANCE / self.LINEAR_SPEED
        print(f"以 {self.LINEAR_SPEED}m/s 速度执行走正方形任务")
        v = self.LINEAR_SPEED
        self.script = [
            ("第一步：沿 X 轴向前", v, 0.0, 0.0, base_time),
            ("第二步：沿 Y 轴向左", 0.0, v, 0.0, base_time),
            ("第三步：沿 X 轴向后", -v, 0.0, 0.0, base_time),
            ("第四步：沿 Y 轴向右", 0.0, -v, 0.0, base_time),
            ("终点站：刹车停止", 0.0, 0.0, 0.0, 0.5)
        ]

        self.current_step = 0
        self.step_start_time = time.time()
        self.timer = self.create_timer(0.05, self.run_script)

    def run_script(self):
        if self.current_step >= len(self.script):
            self.timer.cancel()
            raise SystemExit

        step_name, vx, vy, vz, duration = self.script[self.current_step]
        elapsed_time = time.time() - self.step_start_time

        if elapsed_time <= duration:
            # 组装标准的Twist消息
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(vz)

            # 发布Twist消息
            self.pub.publish(msg)

            if elapsed_time < 0.1:
                print(f"执行 {step_name} (预计 {duration:.2f} 秒)...")
        else:
            self.current_step += 1
            self.step_start_time = time.time()

    def emergency_stop(self):
        print("\n紧急制动")
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        time.sleep(0.2)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TwistSquare()
    try:
        rclpy.spin(node)
    except SystemExit:
        print("任务结束！")
    except KeyboardInterrupt:
        node.emergency_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()