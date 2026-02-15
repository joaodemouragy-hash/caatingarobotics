#!/usr/bin/env python3
import csv
import math
import os
import sys
import threading

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class GravadorRotas(Node):
    def __init__(self):
        super().__init__('gravador_rotas')

        # Subscreve ao AMCL para pegar a posição exata no mapa
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        self.current_pose = None

        # Configuração do Arquivo
        self.nome_rota = input("Digite o nome da rota (ex: rota1): ")

        # CAMINHO ABSOLUTO
        home = os.path.expanduser("~")
        self.path_dir = os.path.join(
            home,
            'agro_robot_ws/src/agro_robot_sim/sistema_rotas_de_trabalho/rotas_de_trabalho',
        )

        if not os.path.exists(self.path_dir):
            os.makedirs(self.path_dir)

        self.file_path = os.path.join(self.path_dir, f'{self.nome_rota}.csv')
        self.get_logger().info(f"Gravando em: {self.file_path}")
        self.get_logger().info(
            "DIRIJA O ROBÔ. Pressione [ENTER] para salvar um waypoint. Digite [q] para sair."
        )

    def listener_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw_from_quaternion(self, orientation):
        """Convert quaternion to yaw (rotation on the Z axis)."""
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw

    def save_waypoint(self):
        if self.current_pose is None:
            self.get_logger().warn("Aguardando localização do AMCL...")
            return

        pos = self.current_pose.position
        ori = self.current_pose.orientation

        # Calcula Yaw usando a função manual
        yaw = self.get_yaw_from_quaternion(ori)

        data = [pos.x, pos.y, yaw]

        # Salva no CSV
        with open(self.file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data)

        self.get_logger().info(f"Ponto salvo: X={pos.x:.2f}, Y={pos.y:.2f}, Yaw={yaw:.2f}")


def input_thread(node):
    while rclpy.ok():
        try:
            cmd = input()
            if cmd.lower() == 'q':
                print("Saindo...")
                rclpy.shutdown()
                sys.exit(0)
            else:
                node.save_waypoint()
        except EOFError:
            pass


def main(args=None):
    rclpy.init(args=args)
    gravador = GravadorRotas()

    thread = threading.Thread(target=input_thread, args=(gravador,), daemon=True)
    thread.start()

    try:
        rclpy.spin(gravador)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            gravador.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
