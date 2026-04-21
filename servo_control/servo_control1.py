# 机械臂两个舵机控制
# 导入所需的库和工具
import rclpy
import time
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition


def main(args=None):
    rclpy.init(args=args)
    node = Node('servo_control_node1')
    pub = node.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
    time.sleep(1.0)

    msg = ServosPosition()
    msg.duration = 1.0
    servo1 = ServoPosition()
    servo1.id = 2
    servo1.position = 365
    servo2 = ServoPosition()
    servo2.id = 4
    servo2.position = 330
    msg.position = [servo1,servo2]

    print("发送舵机控制指令！")
    pub.publish(msg)
    time.sleep(1.5)

    node.destroy_node()
    rclpy.shutdown()
    print("控制结束")

if __name__ == '__main__':
    main()