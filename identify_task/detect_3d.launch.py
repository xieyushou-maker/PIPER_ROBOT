# 一键启动三维定位功能
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取yolo节点的功能包路径
    yolo_path = get_package_share_directory('yolov11_detect')
    # 第一步：启动深度相机与 YOLO 识别节点
    step1_log = LogInfo(msg="""
        1/3：正在启动YOLO 识别节点...
        """)
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_path, 'launch/yolov11_detect.launch.py')
        ),
        # 传递YOLO识别检测的参数
        launch_arguments={
            'model': 'tongji',
            'classes': "['grass', 'yellow', 'gray', 'blue']",
            'conf': '0.7'
        }.items()
    )

    # 第二步：启动三维空间定位节点
    # 等待YOLO话题稳定发布后，再启动定位节点
    locator_launch = TimerAction(
        period=8.0,  # 延时8秒启动
        actions=[
            LogInfo(msg="2/3： 正在启动目标三维空间定位节点 (Object 3D Locator)..."),
            Node(
                package='object_3d_locator_task',
                executable='object_3d_locator_node',
                name='object_3d_locator_node',
                output='screen'
            ),
            LogInfo(msg="定位节点已启动，正在等待YOLO识别目标 ...")
        ]
    )

    # 第三步：启动图像查看器 RQT
    image_view_launch = TimerAction(
        period=12.0,  # 延时12秒启动
        actions=[
            LogInfo(msg="3/3： 正在启动图像查看器 (rqt_image_view)..."),
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='rqt_image_view',
                output='screen'
            ),
            LogInfo(msg="""
        启动完毕！
        请在 rqt_image_view 中选择 /yolo_node/image_raw 查看识别画面。
        """)
        ]
    )

    # 返回 LaunchDescription，交由ROS 2 底层引擎执行节点
    return LaunchDescription([
        step1_log,
        yolo_launch,
        locator_launch,
        image_view_launch
    ])