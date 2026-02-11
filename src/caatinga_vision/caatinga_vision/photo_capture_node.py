#!/usr/bin/env python3
import csv
import json
import math
import os
import re
import shutil
import time
from collections import deque
from datetime import datetime, timezone

import cv2
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


def utc_iso_now():
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


class PhotoCaptureNode(Node):
    def __init__(self):
        super().__init__("photo_capture_node")

        self.declare_parameter(
            "session_id",
            "",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("fps", 8.0)
        self.declare_parameter("capture_distance_m", 0.5)
        self.declare_parameter("speed_min_m_s", 0.05)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("reserve_space_mb", 512)
        self.declare_parameter("usb_mount_path", "")
        self.declare_parameter("status_cache_path", "/tmp/caatinga_capture_status.json")
        self.declare_parameter("preview_cache_path", "/tmp/caatinga_capture_latest.jpg")

        raw_session_id = str(self.get_parameter("session_id").value).strip()
        if not raw_session_id:
            raw_session_id = time.strftime("%Y%m%d_%H%M%S")
        sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", raw_session_id).strip("._-")
        if not sanitized:
            sanitized = time.strftime("%Y%m%d_%H%M%S")
        if not sanitized.startswith("sess_"):
            sanitized = f"sess_{sanitized}"
        self.session_id = sanitized

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.fps = max(float(self.get_parameter("fps").value), 1.0)
        self.capture_distance_m = max(float(self.get_parameter("capture_distance_m").value), 0.1)
        self.speed_min_m_s = max(float(self.get_parameter("speed_min_m_s").value), 0.0)
        self.image_width = max(int(self.get_parameter("image_width").value), 1)
        self.image_height = max(int(self.get_parameter("image_height").value), 1)
        self.jpeg_quality = max(50, min(100, int(self.get_parameter("jpeg_quality").value)))
        self.reserve_space_mb = max(int(self.get_parameter("reserve_space_mb").value), 1)
        self.usb_mount_path = os.path.realpath(os.path.expanduser(str(self.get_parameter("usb_mount_path").value)))
        self.status_cache_path = os.path.realpath(
            os.path.expanduser(str(self.get_parameter("status_cache_path").value))
        )
        self.preview_cache_path = os.path.realpath(
            os.path.expanduser(str(self.get_parameter("preview_cache_path").value))
        )

        self.output_dir = os.path.join(
            self.usb_mount_path,
            "Caatinga_Dados",
            "Fotos_Treino",
            self.session_id,
        )
        self.images_dir = os.path.join(self.output_dir, "images")
        self.metadata_csv_path = os.path.join(self.output_dir, "metadata_fotos.csv")
        self.status_final_path = os.path.join(self.output_dir, "status_final.json")

        self.started_ts = time.time()
        self.started_utc = utc_iso_now()
        self.photos_total = 0
        self.distance_total_m = 0.0
        self.distance_since_last_m = 0.0
        self.last_saved_file = ""
        self.usb_pause_count = 0
        self.usb_connected = False
        self.paused_usb = False
        self.paused_space = False
        self.free_space_mb = 0.0
        self.estimated_photos_remaining = None
        self._saved_sizes = deque(maxlen=40)
        self._default_photo_size_bytes = 300_000.0
        self._final_reason = "shutdown"
        self._status_message = "Inicializando captura."

        self._odom_source = ""
        self.current_x = None
        self.current_y = None
        self.current_speed_m_s = 0.0
        self.current_lat = None
        self.current_lon = None
        self._last_pose = None
        self._max_delta_m = 5.0

        self.cap = None
        self._last_reconnect_try = 0.0
        self._open_camera(force=True)

        self.create_subscription(
            Odometry,
            "/odometry/global",
            lambda msg: self._odom_cb(msg, "global"),
            10,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            lambda msg: self._odom_cb(msg, "odom"),
            10,
        )
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_cb, 10)

        self.timer = self.create_timer(1.0 / self.fps, self._timer_cb)
        self.get_logger().info(
            "Photo capture iniciado | session=%s camera=%d dist=%.2fm usb=%s"
            % (self.session_id, self.camera_index, self.capture_distance_m, self.usb_mount_path)
        )

    def _open_camera(self, force=False):
        now = time.time()
        if not force and (now - self._last_reconnect_try) < 2.0:
            return
        self._last_reconnect_try = now

        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.image_width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.image_height))
        if not cap.isOpened():
            self.cap = None
            self._status_message = "Falha ao abrir câmera. Tentando reconectar..."
            self.get_logger().warn(self._status_message)
            self._save_status_cache()
            return

        self.cap = cap
        self._status_message = "Câmera ativa."

    def _is_usb_connected(self):
        return (
            bool(self.usb_mount_path)
            and os.path.isdir(self.usb_mount_path)
            and os.path.ismount(self.usb_mount_path)
            and os.access(self.usb_mount_path, os.W_OK)
        )

    def _ensure_output_dirs(self):
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(os.path.dirname(self.status_cache_path), exist_ok=True)
        os.makedirs(os.path.dirname(self.preview_cache_path), exist_ok=True)
        if not os.path.exists(self.metadata_csv_path):
            with open(self.metadata_csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "timestamp_utc",
                        "filename",
                        "session_id",
                        "odom_x_m",
                        "odom_y_m",
                        "lat",
                        "lon",
                        "distancia_acumulada_m",
                        "distancia_desde_ultima_m",
                        "velocidade_m_s",
                        "camera_index",
                        "image_width",
                        "image_height",
                        "jpeg_quality",
                        "usb_mount_path",
                    ]
                )

    def _disk_free_bytes(self):
        try:
            usage = shutil.disk_usage(self.usb_mount_path)
            self.free_space_mb = float(usage.free) / (1024.0 * 1024.0)
            return int(usage.free)
        except Exception:
            self.free_space_mb = 0.0
            return 0

    def _estimate_remaining(self, free_bytes):
        reserve_bytes = int(self.reserve_space_mb * 1024 * 1024)
        usable = max(int(free_bytes) - reserve_bytes, 0)
        if self._saved_sizes:
            avg = float(sum(self._saved_sizes)) / float(len(self._saved_sizes))
        else:
            avg = self._default_photo_size_bytes
        avg = max(avg, 1.0)
        return int(usable // avg)

    def _odom_cb(self, msg, source):
        if source == "global":
            self._odom_source = "global"
        elif self._odom_source == "global":
            return
        elif not self._odom_source:
            self._odom_source = "odom"
        elif self._odom_source != "odom":
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        speed = math.hypot(vx, vy)
        self.current_speed_m_s = speed
        self.current_x = x
        self.current_y = y

        if self._last_pose is None:
            self._last_pose = (x, y)
            return

        delta = math.hypot(x - self._last_pose[0], y - self._last_pose[1])
        self._last_pose = (x, y)

        if delta > self._max_delta_m:
            return
        if speed < self.speed_min_m_s:
            return

        self.distance_total_m += delta
        self.distance_since_last_m += delta

    def _gps_cb(self, msg):
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if math.isnan(lat) or math.isnan(lon):
            return
        self.current_lat = lat
        self.current_lon = lon

    def _save_preview(self, frame):
        tmp_path = self.preview_cache_path + ".tmp.jpg"
        try:
            ok = cv2.imwrite(tmp_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                os.replace(tmp_path, self.preview_cache_path)
            elif os.path.exists(tmp_path):
                os.remove(tmp_path)
        except Exception:
            if os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except Exception:
                    pass

    def _append_metadata(self, filename, distance_since_last):
        row = [
            utc_iso_now(),
            filename,
            self.session_id,
            "" if self.current_x is None else f"{self.current_x:.3f}",
            "" if self.current_y is None else f"{self.current_y:.3f}",
            "" if self.current_lat is None else f"{self.current_lat:.6f}",
            "" if self.current_lon is None else f"{self.current_lon:.6f}",
            f"{self.distance_total_m:.3f}",
            f"{distance_since_last:.3f}",
            f"{self.current_speed_m_s:.3f}",
            self.camera_index,
            self.image_width,
            self.image_height,
            self.jpeg_quality,
            self.usb_mount_path,
        ]
        with open(self.metadata_csv_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(row)

    def _capture_photo(self, frame):
        now_tag = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
        next_idx = self.photos_total + 1
        filename = f"frame_{next_idx:06d}_{now_tag}Z.jpg"
        output_path = os.path.join(self.images_dir, filename)
        distance_since_last = float(self.distance_since_last_m)

        ok = cv2.imwrite(
            output_path,
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
        )
        if not ok:
            self._status_message = "Falha ao salvar imagem no pendrive."
            self.get_logger().warn(self._status_message)
            return

        try:
            file_size = os.path.getsize(output_path)
            self._saved_sizes.append(float(file_size))
        except Exception:
            pass

        self.photos_total = next_idx
        self.last_saved_file = filename
        self.distance_since_last_m = 0.0

        try:
            self._append_metadata(filename, distance_since_last)
        except Exception as exc:
            self.get_logger().warn("Falha ao escrever metadata CSV: %s" % str(exc))

        self._save_preview(frame)
        self._status_message = "Captura ativa."

    def _save_status_cache(self):
        status = {
            "timestamp_utc": utc_iso_now(),
            "running": True,
            "session_id": self.session_id,
            "usb_mount_path": self.usb_mount_path,
            "usb_connected": bool(self.usb_connected),
            "paused_usb": bool(self.paused_usb),
            "paused_space": bool(self.paused_space),
            "message": self._status_message,
            "photos_total": int(self.photos_total),
            "distance_total_m": round(float(self.distance_total_m), 4),
            "distance_since_last_m": round(float(self.distance_since_last_m), 4),
            "speed_m_s": round(float(self.current_speed_m_s), 4),
            "free_space_mb": round(float(self.free_space_mb), 2),
            "estimated_photos_remaining": self.estimated_photos_remaining,
            "pauses_usb": int(self.usb_pause_count),
            "last_saved_file": self.last_saved_file,
            "camera_index": int(self.camera_index),
            "image_width": int(self.image_width),
            "image_height": int(self.image_height),
            "jpeg_quality": int(self.jpeg_quality),
            "capture_distance_m": round(float(self.capture_distance_m), 4),
            "speed_min_m_s": round(float(self.speed_min_m_s), 4),
            "reserve_space_mb": int(self.reserve_space_mb),
            "lat": None if self.current_lat is None else round(float(self.current_lat), 6),
            "lon": None if self.current_lon is None else round(float(self.current_lon), 6),
            "odom_source": self._odom_source or "",
            "started_utc": self.started_utc,
        }

        tmp_path = self.status_cache_path + ".tmp"
        try:
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(status, f, indent=2, ensure_ascii=False)
            os.replace(tmp_path, self.status_cache_path)
        except Exception as exc:
            self.get_logger().warn("Falha ao atualizar status cache: %s" % str(exc))
            if os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except Exception:
                    pass

    def _save_status_final(self):
        elapsed = max(time.time() - self.started_ts, 0.0)
        payload = {
            "session_id": self.session_id,
            "motivo_fim": self._final_reason,
            "fotos_total": int(self.photos_total),
            "distancia_total_m": round(float(self.distance_total_m), 4),
            "tempo_total_s": round(float(elapsed), 3),
            "pausas_usb": int(self.usb_pause_count),
            "started_utc": self.started_utc,
            "ended_utc": utc_iso_now(),
            "usb_mount_path": self.usb_mount_path,
        }
        try:
            if self._is_usb_connected():
                os.makedirs(self.output_dir, exist_ok=True)
                with open(self.status_final_path, "w", encoding="utf-8") as f:
                    json.dump(payload, f, indent=2, ensure_ascii=False)
            else:
                self.get_logger().warn("USB indisponível no encerramento. status_final.json não foi salvo.")
        except Exception as exc:
            self.get_logger().warn("Falha ao salvar status_final.json: %s" % str(exc))

    def _timer_cb(self):
        if self.cap is None or not self.cap.isOpened():
            self._open_camera()
            self._save_status_cache()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self._status_message = "Falha ao capturar frame. Tentando reconectar câmera."
            self.get_logger().warn(self._status_message)
            self._open_camera()
            self._save_status_cache()
            return

        connected_now = self._is_usb_connected()
        if connected_now != self.usb_connected:
            if self.usb_connected and not connected_now:
                self.usb_pause_count += 1
                self._status_message = "Pendrive desconectado. Captura pausada."
                self.get_logger().warn(self._status_message)
            elif (not self.usb_connected) and connected_now:
                self._status_message = "Pendrive reconectado. Captura retomada."
                self.get_logger().info(self._status_message)
            self.usb_connected = connected_now

        if not connected_now:
            self.paused_usb = True
            self.paused_space = False
            self.estimated_photos_remaining = None
            self.free_space_mb = 0.0
            self._save_status_cache()
            return

        self.paused_usb = False
        try:
            self._ensure_output_dirs()
        except Exception as exc:
            self._status_message = f"Falha ao preparar pastas de saída: {exc}"
            self.get_logger().warn(self._status_message)
            self._save_status_cache()
            return

        free_bytes = self._disk_free_bytes()
        reserve_bytes = int(self.reserve_space_mb * 1024 * 1024)
        self.estimated_photos_remaining = self._estimate_remaining(free_bytes)

        if free_bytes <= reserve_bytes:
            self.paused_space = True
            self._status_message = (
                "Espaço livre insuficiente no pendrive. Captura pausada até liberar espaço."
            )
            self._save_status_cache()
            return

        self.paused_space = False

        should_capture = (self.photos_total == 0) or (self.distance_since_last_m >= self.capture_distance_m)
        if should_capture:
            self._capture_photo(frame)

        self._save_status_cache()

    def destroy_node(self):
        try:
            self._save_status_cache()
        except Exception:
            pass
        try:
            self._save_status_final()
        except Exception:
            pass
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhotoCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._final_reason = "keyboard_interrupt"
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
