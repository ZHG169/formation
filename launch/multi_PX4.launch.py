# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, TimerAction


# def px4_instance(px4_dir, instance, drone_name, pose, standalone):
#     env = {
#         'PX4_SYS_AUTOSTART': '4001',
#         'PX4_GZ_MODEL': 'x500',
#         'PX4_GZ_MODEL_POSE': pose,
#         'PX4_UXRCE_DDS_NS': drone_name,
#     }

#     if standalone:
#         env['PX4_GZ_STANDALONE'] = '1'

#     return ExecuteProcess(
#         cmd=[
#             f'{px4_dir}/build/px4_sitl_default/bin/px4',
#             '-i', str(instance)
#         ],
#         cwd=px4_dir,
#         additional_env=env,
#         output='screen',

#         name = drone_name,
#         emulate_tty = True

#     )


# def generate_launch_description():
#     px4_dir = '/home/ncrl/ncrl_mqtt/PX4-Autopilot'

#     return LaunchDescription([
#         px4_instance(px4_dir, 0, 'MAV1', '0,0,0,0,0,0', False),

#         TimerAction(
#             period=8.0,
#             actions=[
#                 px4_instance(px4_dir, 1, 'MAV2', '2,0,0,0,0,0', True)
#             ],
#         ),

#         TimerAction(
#             period=16.0,
#             actions=[
#                 px4_instance(px4_dir, 2, 'MAV3', '0,2,0,0,0,0', True)
#             ],
#         )
#     ])

import os

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


PX4_DIR = '/home/ncrl/ncrl_mqtt/PX4-Autopilot'
PX4_BINARY = os.path.join(
    PX4_DIR,
    'build',
    'px4_sitl_default',
    'bin',
    'px4',
)


def create_px4_instance(
    instance: int,
    drone_name: str,
    pose: str,
    standalone: bool,
):
    env = {
        'ROS_DOMAIN_ID': '0',

        # x500 airframe
        'PX4_SYS_AUTOSTART': '4001',

        # PX4 v1.17 建議使用這個變數
        'PX4_SIM_MODEL': 'gz_x500',

        # Gazebo world
        'PX4_GZ_WORLD': 'default',

        # Gazebo World ENU 初始位置
        'PX4_GZ_MODEL_POSE': pose,

        # ROS 2 namespace
        'PX4_UXRCE_DDS_NS': drone_name,

        # 三台共同連線至同一個 Agent
        'PX4_UXRCE_DDS_PORT': '8888',

        # 明確設定，避免繼承終端機殘留的環境變數
        'PX4_GZ_STANDALONE': (
            '1' if standalone else ''
        ),

        # 確保不是連接已存在的指定模型
        'PX4_GZ_MODEL_NAME': '',
    }

    return ExecuteProcess(
        cmd=[
            PX4_BINARY,
            '-i',
            str(instance),
        ],
        cwd=PX4_DIR,
        additional_env=env,
        output='screen',
        emulate_tty=True,
        name=drone_name,
    )

def shutdown_if_process_exits(process, drone_name):
    """
    任一 PX4 非預期結束時，關閉整個 launch。

    這可以避免只剩下一兩台 PX4 運作，
    下一次重啟時又和新的 instance 混在一起。
    """
    return RegisterEventHandler(
        OnProcessExit(
            target_action=process,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason=f'{drone_name} PX4 process exited'
                    )
                )
            ],
        )
    )


def generate_launch_description():
    if not os.path.isfile(PX4_BINARY):
        raise RuntimeError(
            f'找不到 PX4 執行檔：{PX4_BINARY}\n'
            '請先完成 px4_sitl_default 編譯。'
        )

    # instance 0
    # MAV_SYS_ID = 1
    # UXRCE_DDS_KEY = 1
    # namespace = /MAV1
    mav1 = create_px4_instance(
        instance=0,
        drone_name='MAV1',
        pose='0,0,0,0,0,0',
        standalone=False,
    )

    # instance 1
    # MAV_SYS_ID = 2
    # UXRCE_DDS_KEY = 2
    # namespace = /MAV2
    mav2 = create_px4_instance(
        instance=1,
        drone_name='MAV2',
        pose='2,0,0,0,0,0',
        standalone=True,
    )

    # instance 2
    # MAV_SYS_ID = 3
    # UXRCE_DDS_KEY = 3
    # namespace = /MAV3
    mav3 = create_px4_instance(
        instance=2,
        drone_name='MAV3',
        pose='0,2,0,0,0,0',
        standalone=True,
    )

    return LaunchDescription([
        # MAV1 先負責啟動 Gazebo world
        mav1,

        # 等待 Gazebo world 初始化
        TimerAction(
            period=10.0,
            actions=[mav2],
        ),

        # 再啟動第三台，降低同時生成模型造成的競爭
        TimerAction(
            period=30.0,
            actions=[mav3],
        ),

        # 任一 PX4 掛掉，整組關閉
        shutdown_if_process_exits(mav1, 'MAV1'),
        shutdown_if_process_exits(mav2, 'MAV2'),
        shutdown_if_process_exits(mav3, 'MAV3'),
    ])