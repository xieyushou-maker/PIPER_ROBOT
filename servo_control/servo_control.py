# 机械臂单个舵机控制
# 导入所需的库和工具
import rclpy
import time
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition

def main(args=None):
    rclpy.init(args=args)
    node = Node('servo_control_node')
    # 创建发布者，明确发布消息类型为ServsPosition
    pub = node.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
    time.sleep(1.0)
    # 总线舵机消息封装
    msg = ServosPosition()
    # 规定动作执行时间
    msg.duration = 1.0
    # 单个舵机消息封装
    servo = ServoPosition()
    servo.id = 10 # 夹爪舵机ID：10
    servo.position = 500 # 目标位置脉冲值
    # 将单个舵机消息装进总线舵机消息列表
    msg.position = [servo]

    print("发送舵机控制指令！")
    pub.publish(msg)
    time.sleep(1.5)

    node.destroy_node()
    rclpy.shutdown()
    print("控制结束")

if __name__ == '__main__':
    main()