#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import *
from coordinate_convert import *
from std_srvs.srv import Trigger

class MultiTakeoffLand(Node):

    def __init__(self):
        super().__init__('multi_takeoff_land')

        #control hz 100Hz
        self.control_frequency = 10.0

        # DDS namespace 與 MAV_SYS_ID
        self.vehicle_config = [
            ('MAV1', 1),
            ('MAV2', 2),
            ('MAV3', 3),
        ]

        # PX4 使用 NED 座標：
        # z 負值代表向上，因此 -3.0 表示起飛到 3 公尺
        self.takeoff_target_enu = VectorENU(
            east=0.0,
            north=0.0,
            up=3.0,
        )

        # ROS／Gazebo +X方向
        self.takeoff_yaw_enu = 0.0

        # idle -> warmup -> flying -> landing -> landed
        self.state = 'idle'

        # PX4 進入 Offboard 前，需要先收到一段時間的 heartbeat
        self.warmup_duration = 2.0

        self.warmup_start_time = None

        # 紀錄三台飛機的落地狀態
        self.landed_status = {
            namespace: False
            for namespace, _ in self.vehicle_config
        }


        #qos 設定 提高話題傳送品質
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicles = []

        for namespace, system_id in self.vehicle_config:

            input_prefix = f'/{namespace}/fmu/in'
            output_prefix = f'/{namespace}/fmu/out'

            vehicle = {
                'namespace': namespace,
                'system_id': system_id,

                'offboard_pub': self.create_publisher(
                    OffboardControlMode,
                    f'{input_prefix}/offboard_control_mode',
                    qos,
                ),

                'setpoint_pub': self.create_publisher(
                    TrajectorySetpoint,
                    f'{input_prefix}/trajectory_setpoint',
                    qos,
                ),

                'command_pub': self.create_publisher(
                    VehicleCommand,
                    f'{input_prefix}/vehicle_command',
                    qos,
                ),
            }
                        # 訂閱每台飛機的落地偵測
            vehicle['land_detected_sub'] = (
                self.create_subscription(
                    VehicleLandDetected,
                    f'{output_prefix}/vehicle_land_detected',
                    lambda message, name=namespace:
                        self.land_detected_callback(
                            name,
                            message,
                        ),
                    qos,
                )
            )

            self.vehicles.append(vehicle)

         # 外部呼叫的起飛、降落服務(ROS2 建立的server)
        self.create_service(
            Trigger,
            '/formation/takeoff',
            self.takeoff_callback,
        )

        self.create_service(
            Trigger,
            '/formation/land',
            self.land_callback,
        )

        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop,
        )

        self.get_logger().info(
            'Multi-UAV takeoff/land controller ready'
        )
    def timestamp_us(self):
        """PX4 message timestamp，單位為微秒。"""

        return self.get_clock().now().nanoseconds // 1000

    def takeoff_callback(self, request, response):
        # 只有初始狀態或已完成降落時，才允許再次起飛
        if self.state not in ['idle', 'landed']:

            response.success = False
            response.message = (
                f'Cannot take off while state is {self.state}'
            )

            return response

        # 進入Offboard warm-up階段
        self.state = 'warmup'

        # 記錄warm-up開始時間
        self.warmup_start_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            'Starting 2-second Offboard warm-up '
            'for all UAVs at 100 Hz'
        )

        response.success = True
        response.message = (
            'Takeoff sequence started; '
            'warming up Offboard control'
        )

        return response

    def land_callback(self, request, response):
        # 正常降落只允許在飛行狀態執行
        if self.state != 'flying':

            response.success = False
            response.message = (
                f'Cannot land while state is {self.state}'
            )

            return response

        # 對三台飛機送出PX4內建降落命令
        self.send_command_to_all(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

        # 只是開始降落，還不能直接認定已落地
        self.state = 'landing'

        self.get_logger().info(
            'LAND command sent to all UAVs; '
            'waiting for landing confirmation'
        )

        response.success = True
        response.message = (
            'Land command sent to all UAVs'
        )

        return response

    # --------------------------------------------------
    # 100 Hz 控制流程
    # --------------------------------------------------
    def land_detected_callback(
        self,
        namespace,
        message,
    ):
        self.landed_status[namespace] = (
            bool(message.landed)
        )

        if self.state != 'landing':
            return

        if all(self.landed_status.values()):

            self.state = 'landed'

            self.get_logger().info(
                'All UAVs have landed'
            )

    def control_loop(self):
        # 只有warm-up和飛行期間需要發布Offboard訊息
        if self.state not in ['warmup', 'flying']:
            return

        now = self.get_clock().now()

        timestamp = self.timestamp_us()

        # 以100 Hz向三台飛機持續發布
        for vehicle in self.vehicles:

            self.publish_offboard_mode(
                vehicle,
                timestamp,
            )

            self.publish_takeoff_setpoint(
                vehicle,
                timestamp,
            )

        # 檢查Offboard warm-up是否完成
        if self.state == 'warmup':

            elapsed_seconds = (
                now - self.warmup_start_time
            ).nanoseconds / 1e9

            if elapsed_seconds >= self.warmup_duration:

                self.get_logger().info(
                    'Offboard warm-up completed'
                )

                # 三台一起切換到Offboard mode
                self.send_command_to_all(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                )

                # 三台一起Arm
                self.send_command_to_all(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )

                self.state = 'flying'

                self.get_logger().info(
                    'OFFBOARD and ARM commands '
                    'sent to all UAVs'
                )

    # --------------------------------------------------
    # PX4 message publishing
    # --------------------------------------------------

    def publish_offboard_mode(self, vehicle, timestamp):

        message = OffboardControlMode()

        message.timestamp = timestamp

        # 使用位置控制
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False

        vehicle['offboard_pub'].publish(message)

    def publish_takeoff_setpoint(self, vehicle, timestamp):

            target_ned = enu_to_ned(
                self.takeoff_target_enu
            )

            yaw_ned = yaw_enu_to_ned(
                self.takeoff_yaw_enu
            )
            message = TrajectorySetpoint()

            message.timestamp = timestamp

            # 每一台飛機的 local origin 都是自己的起飛位置。
            # 因此 [0, 0, -3] 會讓三台各自在原地垂直起飛。
            message.position = target_ned.as_list()
            message.yaw = yaw_ned

            vehicle['setpoint_pub'].publish(
                message
            )

    def send_command_to_all(
        self,
        command,
        param1=0.0,
        param2=0.0,
    ):

        timestamp = self.timestamp_us()

        for vehicle in self.vehicles:

            message = VehicleCommand()

            message.timestamp = timestamp
            message.command = command

            message.param1 = float(param1)
            message.param2 = float(param2)

            # 必須對應各 PX4 instance 的 MAV_SYS_ID
            message.target_system = vehicle['system_id']
            message.target_component = 1

            # ROS 2 Offboard controller 的來源 ID
            message.source_system = 255
            message.source_component = 1
            message.from_external = True

            vehicle['command_pub'].publish(message)

def main(args=None):

    rclpy.init(args=args)

    node = MultiTakeoffLand()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()