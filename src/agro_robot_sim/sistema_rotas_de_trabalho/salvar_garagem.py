#!/usr/bin/env python3
import math
import os

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class SalvarGaragem(Node):
    def __init__(self):
        super().__init__("salvar_garagem")
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.listener_callback,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            "/odometry/global",
            self.odom_callback,
            10,
        )
        self._odom_sub2 = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )
        self._salvou = False

        home = os.path.expanduser("~")
        self.base_path = os.path.join(
            home, "agro_robot_ws/src/agro_robot_sim/sistema_rotas_de_trabalho"
        )
        self.garagem_path = os.path.join(self.base_path, "garagem.csv")

        self.get_logger().info("Aguardando pose do AMCL para salvar a garagem...")

    def get_yaw_from_quaternion(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def _salvar_pose(self, pose):
        if self._salvou:
            return
        pos = pose.position
        yaw = self.get_yaw_from_quaternion(pose.orientation)

        with open(self.garagem_path, "w", newline="") as file:
            file.write(f"{pos.x},{pos.y},{yaw}\n")

        self.get_logger().info(
            f"Garagem salva em {self.garagem_path}: X={pos.x:.2f}, Y={pos.y:.2f}, Yaw={yaw:.2f}"
        )
        self._salvou = True
        rclpy.shutdown()

    def listener_callback(self, msg):
        self._salvar_pose(msg.pose.pose)

    def odom_callback(self, msg):
        self._salvar_pose(msg.pose.pose)


def main(args=None):
    rclpy.init(args=args)
    node = SalvarGaragem()
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
