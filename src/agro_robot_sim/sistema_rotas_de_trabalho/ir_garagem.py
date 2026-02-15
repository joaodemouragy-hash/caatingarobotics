#!/usr/bin/env python3
import math
import os

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class IrGaragem(Node):
    def __init__(self):
        super().__init__("ir_garagem")
        self._action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._last_pose = None
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._pose_callback,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            "/odometry/global",
            self._pose_callback,
            10,
        )
        self._odom_sub2 = self.create_subscription(
            Odometry,
            "/odom",
            self._pose_callback,
            10,
        )

        home = os.path.expanduser("~")
        self.base_path = os.path.join(
            home, "agro_robot_ws/src/agro_robot_sim/sistema_rotas_de_trabalho"
        )
        self.garagem_path = os.path.join(self.base_path, "garagem.csv")

    def _pose_callback(self, msg):
        self._last_pose = msg.pose.pose

    def _get_current_pose(self, timeout_sec=2.0):
        start = self.get_clock().now()
        timeout_ns = int(timeout_sec * 1e9)
        while (self.get_clock().now() - start).nanoseconds < timeout_ns:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_pose is not None:
                return self._last_pose
        return None

    def get_quaternion_from_euler(self, yaw):
        roll = 0
        pitch = 0
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w

    def carregar_garagem(self):
        if not os.path.exists(self.garagem_path):
            self.get_logger().error(
                f"Garagem não definida. Arquivo ausente: {self.garagem_path}"
            )
            return None
        try:
            with open(self.garagem_path, "r") as f:
                line = f.readline().strip()
                if not line:
                    return None
                x, y, yaw = map(float, line.split(","))
                return x, y, yaw
        except Exception as e:
            self.get_logger().error(f"Erro ao ler garagem: {e}")
            return None

    def send_goal(self):
        pose_data = self.carregar_garagem()
        if not pose_data:
            self.get_logger().error("Garagem inválida.")
            rclpy.shutdown()
            return
        x, y, yaw = pose_data

        # Se já está na garagem, não envia goal
        current_pose = self._get_current_pose(timeout_sec=2.0)
        if current_pose is not None:
            dx = current_pose.position.x - x
            dy = current_pose.position.y - y
            dist = math.hypot(dx, dy)
            if dist <= 0.35:
                self.get_logger().info("Robô já está na garagem.")
                rclpy.shutdown()
                return

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor Nav2 (follow_waypoints) não disponível.")
            rclpy.shutdown()
            return

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = self.get_quaternion_from_euler(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [pose]
        self.get_logger().info("Chamando robô para a garagem...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Garagem rejeitada pelo servidor Nav2.")
            rclpy.shutdown()
            return
        self.get_logger().info("Garagem aceita! Indo para a garagem.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("Chegou na garagem!")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = IrGaragem()
    node.send_goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
