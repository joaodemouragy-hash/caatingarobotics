#!/usr/bin/env python3
import os
import site
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

YOLO = None
YOLO_IMPORT_ERROR = ""
try:
    from ultralytics import YOLO as _YOLO
    YOLO = _YOLO
except Exception as exc:
    YOLO_IMPORT_ERROR = str(exc)
    # Some runtime launchers set PYTHONNOUSERSITE=1, hiding ~/.local packages.
    try:
        user_site = site.getusersitepackages()
        if user_site and user_site not in sys.path:
            sys.path.insert(0, user_site)
        from ultralytics import YOLO as _YOLO
        YOLO = _YOLO
        YOLO_IMPORT_ERROR = ""
    except Exception as exc_fallback:
        YOLO_IMPORT_ERROR = f"{YOLO_IMPORT_ERROR} | fallback: {exc_fallback}"


class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__("yolo_inference_node")

        self.declare_parameter("model_path", "~/models/best.pt")
        self.declare_parameter("confidence_mode", "economico")
        self.declare_parameter("confidence_threshold_conservador", 0.50)
        self.declare_parameter("confidence_threshold_economico", 0.85)
        self.declare_parameter("inference_device", "cpu")
        self.declare_parameter("image_topic", "/caatinga_vision/camera/image_raw")
        self.declare_parameter("detections_topic", "/caatinga_vision/detections")
        self.declare_parameter("overlay_topic", "/caatinga_vision/overlay/image_raw")
        self.declare_parameter("overlay_cache_path", "/tmp/caatinga_vision_overlay.jpg")

        self.model_path = os.path.expanduser(str(self.get_parameter("model_path").value))
        self.confidence_mode = str(self.get_parameter("confidence_mode").value).strip().lower()
        self.thresh_conservador = float(self.get_parameter("confidence_threshold_conservador").value)
        self.thresh_economico = float(self.get_parameter("confidence_threshold_economico").value)
        self.inference_device = str(self.get_parameter("inference_device").value).strip() or "cpu"
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.overlay_topic = str(self.get_parameter("overlay_topic").value)
        self.overlay_cache_path = os.path.expanduser(str(self.get_parameter("overlay_cache_path").value))

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self._image_cb, 10)

        self.model = None
        self.class_names = {}
        self._model_error_reported = False

        self._load_model()

        self.get_logger().info(
            "YOLO inference iniciado | model=%s mode=%s device=%s image_topic=%s"
            % (self.model_path, self.confidence_mode, self.inference_device, self.image_topic)
        )

    def _load_model(self):
        if YOLO is None:
            self.get_logger().error(
                "Falha ao importar ultralytics: %s | python=%s | PYTHONNOUSERSITE=%s | usersite=%s"
                % (
                    YOLO_IMPORT_ERROR or "erro_desconhecido",
                    sys.executable,
                    os.environ.get("PYTHONNOUSERSITE", ""),
                    site.getusersitepackages(),
                )
            )
            return

        if not os.path.isfile(self.model_path):
            self.get_logger().error(
                "Modelo YOLO não encontrado: %s. Ajuste o parâmetro model_path." % self.model_path
            )
            return

        try:
            self.model = YOLO(self.model_path)
            names = getattr(self.model, "names", {})
            if isinstance(names, dict):
                self.class_names = {int(k): str(v) for k, v in names.items()}
            else:
                self.class_names = {int(i): str(v) for i, v in enumerate(names)}
        except Exception as exc:
            self.model = None
            self.get_logger().error("Falha ao carregar modelo YOLO: %s" % str(exc))

    def _confidence_threshold(self):
        if self.confidence_mode == "conservador":
            return max(0.0, min(1.0, self.thresh_conservador))
        return max(0.0, min(1.0, self.thresh_economico))

    def _draw_overlay(self, image_bgr, detections):
        for det in detections:
            x1, y1, x2, y2, label, score = det[:6]
            cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 220, 80), 2)
            text = f"{label} {score:.2f}"
            cv2.putText(
                image_bgr,
                text,
                (x1, max(y1 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 220, 80),
                2,
                cv2.LINE_AA,
            )
        return image_bgr

    def _build_detection_array(self, header, detections):
        msg = Detection2DArray()
        msg.header = header

        for x1, y1, x2, y2, label, score, cls_idx in detections:
            det = Detection2D()
            det.header = header

            bbox = BoundingBox2D()
            center_x = float((x1 + x2) / 2.0)
            center_y = float((y1 + y2) / 2.0)
            # vision_msgs/Pose2D can differ across versions; support both layouts.
            if hasattr(bbox.center, "position"):
                bbox.center.position.x = center_x
                bbox.center.position.y = center_y
            else:
                bbox.center.x = center_x
                bbox.center.y = center_y
            bbox.center.theta = 0.0
            bbox.size_x = float(max(x2 - x1, 1.0))
            bbox.size_y = float(max(y2 - y1, 1.0))
            det.bbox = bbox
            det.id = label

            hyp = ObjectHypothesisWithPose()
            try:
                hyp.hypothesis.class_id = str(int(cls_idx))
            except Exception:
                hyp.hypothesis.class_id = str(label)
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)

            msg.detections.append(det)

        return msg

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error("Falha no CV bridge: %s" % str(exc))
            return

        detections = []
        threshold = self._confidence_threshold()

        if self.model is not None:
            try:
                result = self.model.predict(
                    frame,
                    conf=threshold,
                    verbose=False,
                    device=self.inference_device,
                )[0]
                boxes = getattr(result, "boxes", None)
                if boxes is not None:
                    for box in boxes:
                        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                        score = float(box.conf[0].item())
                        cls_idx = int(box.cls[0].item())
                        label = self.class_names.get(cls_idx, str(cls_idx))
                        detections.append((x1, y1, x2, y2, label, score, cls_idx))
            except Exception as exc:
                err_text = str(exc)
                if "CUDA error" in err_text and self.inference_device.lower() != "cpu":
                    self.get_logger().warn(
                        "Inferencia em %s falhou. Revertendo para CPU."
                        % self.inference_device
                    )
                    self.inference_device = "cpu"
                    return
                if not self._model_error_reported:
                    self.get_logger().error("Erro na inferência YOLO: %s" % err_text)
                    self._model_error_reported = True
        elif not self._model_error_reported:
            self.get_logger().error("Inferência desativada: modelo não carregado.")
            self._model_error_reported = True

        det_msg = self._build_detection_array(msg.header, detections)
        self.det_pub.publish(det_msg)

        overlay = self._draw_overlay(frame.copy(), detections)
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)

        cache_dir = os.path.dirname(self.overlay_cache_path)
        if cache_dir:
            os.makedirs(cache_dir, exist_ok=True)
        # Write atomically to avoid the GUI reading a partially-written image.
        overlay_root, overlay_ext = os.path.splitext(self.overlay_cache_path)
        if not overlay_ext:
            overlay_ext = ".jpg"
        tmp_overlay_path = f"{overlay_root}.tmp{overlay_ext}"
        try:
            ok = cv2.imwrite(tmp_overlay_path, overlay)
            if ok:
                os.replace(tmp_overlay_path, self.overlay_cache_path)
            elif os.path.exists(tmp_overlay_path):
                os.remove(tmp_overlay_path)
        except Exception as exc:
            self.get_logger().warn("Falha ao salvar overlay cache: %s" % str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
