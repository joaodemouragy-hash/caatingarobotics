#!/usr/bin/env python3
import csv
import math
import os
import sys  # <--- IMPORTANTE PARA LER O NOME VINDO DO PAINEL
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


class LeitorRotas(Node):
    def __init__(self):
        super().__init__('leitor_rotas')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._modo = "rota"
        self._last_pose = None
        self._current_waypoint = None
        self._current_waypoint_overall = None
        self._poses = []
        self._active_poses = []
        self._waypoint_offset = 0
        self._final_target = None
        self._final_in_range_since = None
        self._forced_finish = False
        self._goal_handle = None
        self._ignore_next_result = False
        self._pending_skip_index = None
        self._final_timer = None
        self._final_radius = 1.0
        self._final_hold_sec = 2.0
        self._last_reported_waypoint = None
        self._force_finish_srv = self.create_service(
            Trigger, "force_finish", self._handle_force_finish
        )
        self._skip_waypoint_srv = self.create_service(
            Trigger, "skip_waypoint", self._handle_skip_waypoint
        )

        home = os.path.expanduser("~")
        self.base_path = os.path.join(
            home, 'agro_robot_ws/src/agro_robot_sim/sistema_rotas_de_trabalho'
        )
        self.garagem_path = os.path.join(self.base_path, 'garagem.csv')

        self._pose_sub_odom_global = self.create_subscription(
            Odometry, "/odometry/global", self._pose_callback, 10
        )
        self._pose_sub_odom = self.create_subscription(
            Odometry, "/odom", self._pose_callback, 10
        )
        self._pose_sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._pose_callback, 10
        )

    def _pose_callback(self, msg):
        pose = None
        if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
            pose = msg.pose.pose
        elif hasattr(msg, "pose"):
            pose = msg.pose
        if pose is not None:
            self._last_pose = pose

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

    def send_goal(self, nome_rota):
        # Tenta achar o arquivo com ou sem extensão .csv
        base_path = os.path.join(self.base_path, 'rotas_de_trabalho')
        file_path = os.path.join(base_path, f'{nome_rota}.csv')

        if not os.path.exists(file_path):
            # Tenta sem o .csv caso o nome já tenha vindo com ele
            file_path = os.path.join(base_path, f'{nome_rota}')
            if not os.path.exists(file_path):
                self.get_logger().error(f"ARQUIVO NÃO ENCONTRADO: {file_path}")
                rclpy.shutdown()
                return

        poses = []
        try:
            with open(file_path, 'r') as file:
                reader = csv.reader(file)
                for i, row in enumerate(reader):
                    if not row:
                        continue
                    try:
                        x, y, yaw = map(float, row)
                    except ValueError:
                        continue

                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.0
                    qx, qy, qz, qw = self.get_quaternion_from_euler(yaw)
                    pose.pose.orientation.x = qx
                    pose.pose.orientation.y = qy
                    pose.pose.orientation.z = qz
                    pose.pose.orientation.w = qw
                    poses.append(pose)
        except Exception as e:
            self.get_logger().error(f"Erro ao ler arquivo: {e}")
            rclpy.shutdown()
            return

        if not poses:
            self.get_logger().warn("Rota vazia ou inválida.")
            rclpy.shutdown()
            return

        self._poses = poses
        self._active_poses = poses
        self._waypoint_offset = 0
        self._final_target = (
            poses[-1].pose.position.x,
            poses[-1].pose.position.y,
        )
        self._current_waypoint = None
        self._current_waypoint_overall = None
        self._forced_finish = False
        self._final_in_range_since = None
        self._goal_handle = None
        self._ignore_next_result = False
        self._last_reported_waypoint = None
        if self._final_timer is not None:
            self._final_timer.cancel()
            self._final_timer = None

        self.get_logger().info(f"Conectando ao Nav2... Carregando rota: {nome_rota}")

        # Em SLAM o Nav2 pode demorar mais para subir; espera com retries
        timeout_total = 30.0
        waited = 0.0
        while rclpy.ok() and waited < timeout_total:
            if self._action_client.wait_for_server(timeout_sec=2.0):
                break
            waited += 2.0
            self.get_logger().info("Aguardando Nav2 (follow_waypoints)...")

        if waited >= timeout_total:
            # Tenta namespace alternativo
            self.get_logger().warn(
                "Tentando action server alternativo /waypoint_follower/follow_waypoints"
            )
            self._action_client = ActionClient(
                self, FollowWaypoints, 'waypoint_follower/follow_waypoints'
            )
            waited = 0.0
            while rclpy.ok() and waited < timeout_total:
                if self._action_client.wait_for_server(timeout_sec=2.0):
                    break
                waited += 2.0
                self.get_logger().info("Aguardando Nav2 (follow_waypoints)...")

            if waited >= timeout_total:
                self.get_logger().error(
                    "Servidor Nav2 (follow_waypoints) não disponível! "
                    "Verifique se a simulação está rodando."
                )
                rclpy.shutdown()
                return

        self.get_logger().info(f"Enviando {len(poses)} waypoints...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = getattr(feedback_msg, "feedback", feedback_msg)
        current_wp = getattr(feedback, "current_waypoint", None)
        if current_wp is not None:
            self._current_waypoint = current_wp
            overall_index = self._waypoint_offset + current_wp
            self._current_waypoint_overall = overall_index
            if overall_index != self._last_reported_waypoint and self._poses:
                self._last_reported_waypoint = overall_index
                total = len(self._poses)
                current = min(overall_index + 1, total)
                remaining = max(total - current, 0)
                self.get_logger().info(f"[WAYPOINT] {current}/{total} remaining={remaining}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Rota rejeitada pelo servidor Nav2.')
            return
        self.get_logger().info('Rota aceita! O robô iniciará o trabalho.')
        self._goal_handle = goal_handle
        if self._final_timer is not None:
            self._final_timer.cancel()
        self._final_timer = self.create_timer(0.2, self._check_final_waypoint)
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if self._ignore_next_result:
            self._ignore_next_result = False
            return
        if self._forced_finish:
            return
        result = future.result()
        status = getattr(result, "status", None)

        if self._modo == "rota":
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._handle_route_success()
                return
            else:
                self.get_logger().warn(f'Rota terminou com status: {status}')
            rclpy.shutdown()
            return

        if self._modo == "garagem":
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Chegou na garagem!')
            else:
                self.get_logger().warn(f'Garagem terminou com status: {status}')
            rclpy.shutdown()

    def _handle_route_success(self):
        self.get_logger().info('Rota Finalizada com Sucesso!')
        if self._garagem_existe():
            self.get_logger().info('Retornando para a garagem...')
            self._modo = "garagem"
            self._send_garagem()
            return
        rclpy.shutdown()

    def _check_final_waypoint(self):
        if self._forced_finish or self._goal_handle is None:
            return
        if not self._poses or self._current_waypoint_overall is None:
            return
        if self._current_waypoint_overall < len(self._poses) - 1:
            return
        if self._last_pose is None or self._final_target is None:
            return
        dx = self._last_pose.position.x - self._final_target[0]
        dy = self._last_pose.position.y - self._final_target[1]
        dist = math.hypot(dx, dy)
        if dist <= self._final_radius:
            if self._final_in_range_since is None:
                self._final_in_range_since = time.monotonic()
            elif (time.monotonic() - self._final_in_range_since) >= self._final_hold_sec:
                self._force_finish_success()
        else:
            self._final_in_range_since = None

    def _force_finish_success(self):
        if self._forced_finish:
            return
        self._forced_finish = True
        if self._final_timer is not None:
            self._final_timer.cancel()
            self._final_timer = None
        self.get_logger().info('✅ Último ponto alcançado (forçado). Encerrando rota...')
        if self._goal_handle is None:
            self._handle_route_success()
            return
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._force_cancel_done)

    def _force_cancel_done(self, future):
        # Mesmo se o cancelamento falhar, seguimos o fluxo de sucesso.
        self._handle_route_success()

    def _handle_force_finish(self, request, response):
        if not self._poses:
            response.success = False
            response.message = "Sem rota ativa."
            return response
        if self._current_waypoint_overall is None:
            response.success = False
            response.message = "Sem feedback de waypoint."
            return response
        if self._current_waypoint_overall < len(self._poses) - 1:
            response.success = False
            response.message = "Nao esta no ultimo waypoint."
            return response
        self._force_finish_success()
        response.success = True
        response.message = "Forcado termino no ultimo waypoint."
        return response

    def _handle_skip_waypoint(self, request, response):
        if not self._poses:
            response.success = False
            response.message = "Sem rota ativa."
            return response
        if self._current_waypoint_overall is None:
            response.success = False
            response.message = "Sem feedback de waypoint."
            return response
        if self._current_waypoint_overall >= len(self._poses) - 1:
            response.success = False
            response.message = "Ja esta no ultimo waypoint."
            return response
        target_index = self._current_waypoint_overall + 1
        if self._goal_handle is not None:
            self._pending_skip_index = target_index
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._skip_cancel_done)
        else:
            self._send_remaining_goal(target_index)
        response.success = True
        response.message = "Pulando para o proximo waypoint."
        return response

    def _skip_cancel_done(self, future):
        resp = future.result()
        rc = getattr(resp, "return_code", None)
        if rc is None or rc == CancelGoal.Response.ERROR_NONE:
            self._ignore_next_result = True
            self._send_remaining_goal(self._pending_skip_index)
        else:
            self.get_logger().warn("Nao foi possivel cancelar o goal para pular waypoint.")
        self._pending_skip_index = None

    def _send_remaining_goal(self, start_index):
        remaining = self._poses[start_index:]
        if not remaining:
            self._handle_route_success()
            return
        self._active_poses = remaining
        self._waypoint_offset = start_index
        self._current_waypoint = None
        self._current_waypoint_overall = None
        self._last_reported_waypoint = None
        self._final_in_range_since = None
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = remaining
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def _garagem_existe(self):
        return os.path.exists(self.garagem_path)

    def _carregar_garagem(self):
        try:
            with open(self.garagem_path, 'r') as f:
                line = f.readline().strip()
                if not line:
                    return None
                x, y, yaw = map(float, line.split(','))
                return x, y, yaw
        except Exception as e:
            self.get_logger().error(f"Erro ao ler garagem: {e}")
            return None

    def _send_garagem(self):
        pose_data = self._carregar_garagem()
        if not pose_data:
            self.get_logger().error("Garagem inválida.")
            rclpy.shutdown()
            return
        x, y, yaw = pose_data

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor Nav2 (follow_waypoints) não disponível!")
            rclpy.shutdown()
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
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
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = LeitorRotas()

    # --- A MÁGICA ESTÁ AQUI ---
    # Se o script receber argumentos (ex: python3 leitor.py rota1), ele usa direto.
    # Se não receber nada, ele pede para digitar (modo manual).
    if len(sys.argv) > 1:
        nome_rota = sys.argv[1]
    else:
        nome_rota = input("Digite o nome da rota (ex: rota1): ")

    node.send_goal(nome_rota)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass


if __name__ == '__main__':
    main()
