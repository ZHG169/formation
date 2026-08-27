from dataclasses import dataclass
from typing import Optional
import math

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from formation.coordinate_convert import (
    VectorENU,
    VectorNED,
    enu_to_ned,
    ned_to_enu,
    yaw_enu_to_ned,
    yaw_ned_to_enu,
)

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

@dataclass # 無人機實際狀態
class VehicleState:
    vehicle_id: int #無人機編號
    namespace: str #無人機名字

    position_local_enu: Optional[VectorENU] = None 
    # 無人機目前的 local ENU 位置
    # 尚未收到 PX4 position 時為 None
    # Optional[X]是來自 typing 模組的型別提示工具，代表該變數或參數可以是指定的型別 X，
    # 或者也可以是 None。它其實就是 Union[X, None] 的簡寫。
    velocity_local_enu: Optional[VectorENU] = None #無人機在ROS座標系的速度
    yaw_local_enu: float = 0.0 #無人機在ROS座標系的偏航角
    # 無人機在 local ENU 座標系下的 yaw
    # 預設 0 rad，代表朝 ENU +X（East）方向

    armed: bool = False #無人機是否解鎖
    landed: bool = False #無人機是否降落
    preflight_ok: bool = False #無人機是否準備起飛
    position_valid: bool = False #無人機是否有有效的位子資訊

    status_received: bool = False #無人機狀態回覆
    position_received: bool = False #無人機位子回覆
    land_status_received: bool = False #無人機降落回覆

    last_position_update_ns: int = 0 # 最後一次收到位置資料的時間戳，單位 ns

    nav_state: int = 0 #無人機導航狀態
    offboard_enabled: bool = False # PX4 是否進入 Offboard 外部控制模式


@dataclass #無人機目標狀態
class VehicleSetpoint:
    position_local_enu: VectorENU
    yaw_local_enu: float
    velocity_local_enu: Optional[VectorENU] = None
    #有些控制命令需要指定「目標速度」，有些控制命令只需要指定「目標位置」，所以速度不是每次都必須提供。


class VehicleInterface:

    def __init__(self, node, namespace, system_id):
        # 外部傳入的 ROS2 Node，供 VehicleInterface 建立 pub/sub
        self.node = node
        self.namespace = namespace
        self.system_id = system_id
        self.state = VehicleState(
            vehicle_id=system_id,
            namespace=namespace,
        )
        self.last_yaw_debug_ns = 0
        self.last_target_debug_ns = 0

        input_prefix = f'/{namespace}/fmu/in'
        output_prefix = f'/{namespace}/fmu/out'
        qos = QoSProfile(
            # 資料遺失可以接受，不要求每筆可靠送達
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # Publisher 可保留資料供較晚加入的 subscriber 使用
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
             # 只保存最近的資料
            history=HistoryPolicy.KEEP_LAST,
            # 最近只留 1 筆
            depth=1,
        )

        # 持續提供 Offboard 控制模式/控制類型的訊息(告訴PX4接下來要如何控制)
        # OffboardControlMode 訊息的內容會決定 PX4 接下來要使用哪種控制模式。
        self.offboard_publisher = (
            self.node.create_publisher(
                OffboardControlMode,
                f'{input_prefix}/offboard_control_mode',
                qos,
            )
        )
        #建立 Publisher，用來發布位置、速度、yaw 等控制目標給 PX4(告訴PX4 控制項目)
        self.setpoint_publisher = (
            self.node.create_publisher(
                # TrajectorySetpoint = 給 PX4 的「運動目標值」訊息。
                # TrajectorySetpoint
                # │
                # ├── position      目標位置 [x, y, z]
                # ├── velocity      目標速度 [vx, vy, vz]
                # ├── acceleration  目標加速度 [ax, ay, az]
                # ├── yaw           目標偏航角
                # └── yawspeed      目標偏航角速度
                TrajectorySetpoint,
                f'{input_prefix}/trajectory_setpoint',
                qos,
            )
        )

        # 用來發布控制命令給 PX4(告訴PX4 要做什麼)
        self.command_publisher = (
            self.node.create_publisher(
                VehicleCommand,
                f'{input_prefix}/vehicle_command',
                qos,
            )
        )

        # 訂閱 PX4 的 VehicleStatus 訊息，取得無人機的狀態資訊
        self.status_subscription = (
            self.node.create_subscription(
                VehicleStatus,
                f'{output_prefix}/vehicle_status_v1',
                self.status_callback,
                qos,
            )
        )

        # 訂閱 PX4 的 VehicleLocalPosition 訊息，取得無人機的 local ENU 位置資訊
        self.position_subscription = (
            self.node.create_subscription(
                VehicleLocalPosition,
                f'{output_prefix}/vehicle_local_position_v1',
                self.position_callback,
                qos,
            )
        )

        # 訂閱 PX4 的 VehicleLandDetected 訊息，取得無人機的降落狀態資訊
        self.land_subscription = (
            self.node.create_subscription(
                VehicleLandDetected,
                f'{output_prefix}/vehicle_land_detected',
                self.land_detected_callback,
                qos,
            )
        )

    def status_callback(self, message):
        self.state.status_received = True
        self.state.preflight_ok = bool(
            message.pre_flight_checks_pass 
        )

        self.state.armed = (
            message.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

        self.state.nav_state = int(message.nav_state)

        self.state.offboard_enabled = (
            message.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    # 檢查 position 是否是最新的，預設 timeout 2 秒，最後回傳 True / False。
    def position_is_fresh(
        self,
        timeout_seconds: float = 2.0,
    ) -> bool:
        if not self.state.position_received:
            return False

        now_ns = self.node.get_clock().now().nanoseconds

        age_seconds = (
            now_ns - self.state.last_position_update_ns
        ) / 1e9

        return bool(
            # 判斷位子有效且是最新更新的
            self.state.position_valid
            and 0.0 <= age_seconds <= timeout_seconds
        )

    def position_callback(self, message):
        self.state.position_received = True
        # 判斷收到的位子是否是有限數據
        # all() 是一個內建函數，用來檢查可迭代物件（如串列 []）中的所有元素是否都為真（True）。
        #如果傳入空串列 all([])，它會回傳 True
        values_finite = all(
            math.isfinite(float(value))
            for value in (
                message.x,
                message.y,
                message.z,
                message.vx,
                message.vy,
                message.vz,
                message.heading,
            )
        )

        # 判斷收到的位子是否有效，必須同時滿足以下條件：
        # 1. xy_valid 為 True，表示 XY 平面位置有效
        # 2. z_valid 為 True，表示 Z 軸位置有效
        # 3. v_xy_valid 為 True，表示 XY 平面速度有效
        # 4. v_z_valid 為 True，表示 Z 軸速度有效
        # 5. values_finite 為 True，表示所有位置和速度數值都是有限的（不是 NaN 或無限大）
        self.state.position_valid = bool(
            message.xy_valid
            and message.z_valid
            and message.v_xy_valid
            and message.v_z_valid
            and values_finite
        )

        self.state.last_position_update_ns = (
            self.node.get_clock().now().nanoseconds
        )

        if not self.state.position_valid:
            self.state.position_local_enu = None
            self.state.velocity_local_enu = None
            return

        # 整理PX4的位子資訊
        position_ned = VectorNED(
            north=float(message.x),
            east=float(message.y),
            down=float(message.z),
        )

        # 將 NED 坐標轉換為 ENU 坐標，並更新 VehicleState 中的 position_local_enu
        self.state.position_local_enu = ned_to_enu(
            position_ned
        )

        velocity_ned = VectorNED(
            north=float(message.vx),
            east=float(message.vy),
            down=float(message.vz),
        )

        self.state.velocity_local_enu = ned_to_enu(
            velocity_ned
        )

        self.state.yaw_local_enu = yaw_ned_to_enu(
            float(message.heading)
        )

    def get_state(self):
        return self.state

    def publish_offboard_heartbeat(self, use_velocity=False):
        # use_velocity	position	velocity	控制方式
        # False	        True	    False	    Position
        # True	        False	    True	    Velocity
        message = OffboardControlMode()

        message.timestamp = self.timestamp_us()
        message.position = not bool(use_velocity)
        message.velocity = bool(use_velocity)
        message.acceleration = False # 並非給加速度 而是指現在不採用加速度控制
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False

        self.offboard_publisher.publish(message)

    def publish_setpoint(self, setpoint):
        message = TrajectorySetpoint()
        message.timestamp = self.timestamp_us()
        message.yaw = yaw_enu_to_ned(setpoint.yaw_local_enu)

        # 決定是要用速度控制還是位子控制 因為CPF採用速度做控制 所以大部分會走速度端
        # 因為一開始啟用位子做處理才會有現在的情況
        if setpoint.velocity_local_enu is not None:
            velocity_ned = enu_to_ned(setpoint.velocity_local_enu)
            message.position = [
                math.nan,
                math.nan,
                math.nan,
            ]
            message.velocity = velocity_ned.as_list()
        else:
            target_ned = enu_to_ned(setpoint.position_local_enu)
            message.position = target_ned.as_list()
            message.velocity = [
                math.nan,
                math.nan,
                math.nan,
            ]

        self.setpoint_publisher.publish(message)
        self.log_yaw_debug_if_enabled(setpoint, message.yaw)
        self.log_target_debug_if_enabled(setpoint, message)
    
    def node_parameter_value(self, name, default):
        try:
            return self.node.get_parameter(name).value
        except Exception:
            return default

    def normalize_angle(self, angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    def log_yaw_debug_if_enabled(self, setpoint, target_yaw_ned):
        enabled = bool(
            self.node_parameter_value('yaw_debug_enabled', False)
        )
        if not enabled:
            return

        interval = float(
            self.node_parameter_value('yaw_debug_interval', 1.0)
        )
        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns - self.last_yaw_debug_ns < interval * 1e9:
            return

        self.last_yaw_debug_ns = now_ns
        current_yaw = float(self.state.yaw_local_enu)
        target_yaw = float(setpoint.yaw_local_enu)
        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        self.node.get_logger().info(
            f'Yaw debug {self.namespace}: '
            f'current_enu={math.degrees(current_yaw):.1f} deg, '
            f'target_enu={math.degrees(target_yaw):.1f} deg, '
            f'target_ned={math.degrees(target_yaw_ned):.1f} deg, '
            f'error={math.degrees(yaw_error):.1f} deg'
        )


    def log_target_debug_if_enabled(self, setpoint, px4_message):
        enabled = bool(
            self.node_parameter_value('target_debug_enabled', False)
        )
        if not enabled:
            return

        interval = float(
            self.node_parameter_value('target_debug_interval', 1.0)
        )
        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns - self.last_target_debug_ns < interval * 1e9:
            return

        self.last_target_debug_ns = now_ns

        if setpoint.velocity_local_enu is not None:
            velocity = setpoint.velocity_local_enu
            self.node.get_logger().info(
                f'PX4 setpoint debug {self.namespace}: '
                'mode=velocity, '
                f'velocity_enu=({velocity.east:.2f}, '
                f'{velocity.north:.2f}, {velocity.up:.2f}), '
                f'velocity_ned=({px4_message.velocity[0]:.2f}, '
                f'{px4_message.velocity[1]:.2f}, '
                f'{px4_message.velocity[2]:.2f}), '
                'position_ned=(nan, nan, nan)'
            )
            return

        target = setpoint.position_local_enu
        self.node.get_logger().info(
            f'PX4 setpoint debug {self.namespace}: '
            'mode=position, '
            f'target_local_enu=({target.east:.2f}, '
            f'{target.north:.2f}, {target.up:.2f}), '
            f'target_ned=({px4_message.position[0]:.2f}, '
            f'{px4_message.position[1]:.2f}, '
            f'{px4_message.position[2]:.2f})'
        )

    def set_offboard_mode(self):
        # | `param2` | PX4 Custom Main Mode | 意義                  |
        # | -------: | -------------------- | ------------------- |
        # |        1 | `MANUAL`             | 手動控制                |
        # |        2 | `ALTCTL`             | 高度控制                |
        # |        3 | `POSCTL`             | 位置控制                |
        # |        4 | `AUTO`               | 自動模式大類              |
        # |        5 | `ACRO`               | Acro 特技/角速度類控制      |
        # |        6 | `OFFBOARD`           | 外部控制                |
        # |        7 | `STABILIZED`         | 穩定模式                |
        # |        8 | `RATTITUDE`          | Rattitude（舊/特定版本相關） |

        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def arm(self):
        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def disarm(self):
        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )

    # 降落偵測
    def land_detected_callback(self, message):
        self.state.land_status_received = True
        self.state.landed = bool(message.landed)

    # 執行降落
    def land(self): 
        self.send_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

    def send_command(self, command, param1=0.0, param2=0.0):
        message = VehicleCommand()
        message.timestamp = self.timestamp_us()
        message.command = command

        message.param1 = float(param1)
        message.param2 = float(param2)

        message.target_system = self.system_id
        message.target_component = 1 # 指令要給主要飛控

        message.source_system = 255 # 外部控制
        message.source_component = 1 # 主要的外部控制來源
        message.from_external = True

        self.command_publisher.publish(message)

    def timestamp_us(self):
        return self.node.get_clock().now().nanoseconds // 1000 # 取得時間 單位為微秒