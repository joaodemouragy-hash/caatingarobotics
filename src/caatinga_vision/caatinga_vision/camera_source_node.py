#!/usr/bin/env python3
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraSourceNode(Node):
    def __init__(self):
        super().__init__("camera_source_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("fps", 12.0)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("image_topic", "/caatinga_vision/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/caatinga_vision/camera/camera_info")
        self.declare_parameter("image_width", 0)
        self.declare_parameter("image_height", 0)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.fps = max(float(self.get_parameter("fps").value), 1.0)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        self.cap = None
        self._last_reconnect_try = 0.0

        self._open_camera(force=True)

        self.timer = self.create_timer(1.0 / self.fps, self._timer_cb)
        self.get_logger().info(
            "Camera source iniciado | index=%d topic=%s fps=%.1f"
            % (self.camera_index, self.image_topic, self.fps)
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
        if self.image_width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.image_width))
        if self.image_height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.image_height))

        if not cap.isOpened():
            self.cap = None
            self.get_logger().warn(
                "Nao foi possivel abrir a camera index=%d. Tentando novamente..." % self.camera_index
            )
            return

        self.cap = cap

    def _build_camera_info(self, stamp, width, height):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = int(width)
        msg.height = int(height)
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [1.0, 0.0, width / 2.0, 0.0, 1.0, height / 2.0, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [1.0, 0.0, width / 2.0, 0.0, 0.0, 1.0, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _timer_cb(self):
        if self.cap is None or not self.cap.isOpened():
            self._open_camera()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Falha ao capturar frame da webcam.")
            self._open_camera()
            return

        stamp = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        h, w = frame.shape[:2]
        info_msg = self._build_camera_info(stamp, w, h)
        self.camera_info_pub.publish(info_msg)

    def destroy_node(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
