# 机械臂六个舵机联合控制
# 导入所需的库和工具
import rclpy
import time
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition

def main(args=None):
    rclpy.init(args=args)
    node = Node('servo_control_node2')
    pub = node.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
    time.sleep(1.0)

    msg = ServosPosition()
    action_duration = 2.0
    msg.duration = action_duration

    # 将目标位置写成一个字典
    targets = {
        1: 500, # 键（ID）：值（位置）
        2: 365,
        3: 180,
        4: 330,
        5: 500,
        10: 350
    }

    servo_list = [] #定义空列表用
    # 利用for循环遍历六个舵机的参数，并添加到列表
    for servo_id, pos in targets.items():
        servo = ServoPosition()
        servo.id = servo_id
        servo.position = pos
        servo_list.append(servo)
    # 将列表消息传给总线舵机消息
    msg.position = servo_list

    print(f"动作开始执行")
    pub.publish(msg)

    time.sleep(action_duration + 0.5)

    node.destroy_node()
    rclpy.shutdown()
    print("动作完成，程序自动退出")


if __name__ == '__main__':
    main()