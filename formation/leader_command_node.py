import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from formation.msg import FormationCommand, FormationStatus


class LeaderCommandNode(Node):
    """Relay leader high-level commands without reading PX4 position."""

    def __init__(self):
        super().__init__('leader_command_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_frequency', 20.0),
            ],
        )

        frequency = float(
            self.get_parameter('publish_frequency').value
        )
        if frequency <= 0.0:
            raise ValueError(
                'publish_frequency must be greater than zero'
            )

        self.publisher = self.create_publisher(
            FormationCommand,
            '/formation/command',
            10,
        )
        self.input_subscription = self.create_subscription(
            FormationCommand,
            '/formation/leader_input',
            self.leader_input_callback,
            10,
        )
        self.status_subscription = self.create_subscription(
            FormationStatus,
            '/formation/status',
            self.status_callback,
            10,
        )

        self.active_command = None
        self.active_leader_id = 0
        self.leader_generation = 0
        self.sequence = 0
        self.command_started_ns = 0

        self.timer = self.create_timer(
            1.0 / frequency,
            self.timer_callback,
        )

        self.get_logger().info(
            f'Leader command relay ready at {frequency:.1f} Hz'
        )

    def status_callback(self, message):
        leader_changed = (
            int(message.leader_id) != self.active_leader_id
            or int(message.leader_generation)
            != self.leader_generation
        )

        self.active_leader_id = int(message.leader_id)
        self.leader_generation = int(message.leader_generation)

        if leader_changed or self.active_command is None:
            self.active_command = self.create_hold_command()
            self.command_started_ns = (
                self.get_clock().now().nanoseconds
            )

    def leader_input_callback(self, message):
        if self.active_leader_id == 0:
            self.get_logger().warning(
                'Ignoring leader input before formation status'
            )
            return

        command = FormationCommand()
        command.command = int(message.command)
        command.velocity_east = float(message.velocity_east)
        command.velocity_north = float(message.velocity_north)
        command.velocity_up = float(message.velocity_up)
        command.yaw_enu = float(message.yaw_enu)
        command.yaw_rate = float(message.yaw_rate)
        command.duration = float(message.duration)
        command.formation_type = str(message.formation_type)
        command.spacing = float(message.spacing)

        self.active_command = self.prepare_command(command)
        self.command_started_ns = (
            self.get_clock().now().nanoseconds
        )

    def create_hold_command(self):
        command = FormationCommand()
        command.command = FormationCommand.HOLD
        command.duration = 0.0
        return self.prepare_command(command)

    def prepare_command(self, command):
        self.sequence += 1
        now = self.get_clock().now()

        command.stamp = now.to_msg()
        command.execute_at = now.to_msg()
        command.leader_id = self.active_leader_id
        command.leader_generation = self.leader_generation
        command.sequence = self.sequence

        return command

    def timer_callback(self):
        if self.active_command is None:
            return

        now = self.get_clock().now()

        if (
            self.active_command.duration > 0.0
            and self.command_started_ns > 0
        ):
            elapsed = (
                now.nanoseconds - self.command_started_ns
            ) / 1e9
            if elapsed >= self.active_command.duration:
                self.active_command = self.create_hold_command()
                self.command_started_ns = now.nanoseconds

        self.active_command.stamp = now.to_msg()
        self.publisher.publish(self.active_command)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderCommandNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
