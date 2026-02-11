#!/usr/bin/env python3
import csv
import json
import math
import os
import re
import time
import base64
import ast
from collections import Counter, defaultdict
from datetime import datetime, timezone

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

LAT_REF = -5.1873
LON_REF = -39.2936


def utc_iso_now():
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


class InfestationAnalyticsNode(Node):
    def __init__(self):
        super().__init__("infestation_analytics_node")

        self.declare_parameter(
            "session_id",
            "",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("grid_resolution_m", 1.0)
        self.declare_parameter("pump_rate_l_min", 3.5)
        self.declare_parameter("confidence_mode", "economico")
        self.declare_parameter("detections_topic", "/caatinga_vision/detections")
        self.declare_parameter("status_topic", "/caatinga_vision/infestation/status")
        self.declare_parameter("spray_recommendation_topic", "/caatinga_vision/spray/recommendation")
        self.declare_parameter("status_cache_path", "/tmp/caatinga_vision_status.json")
        self.declare_parameter("logs_base_dir", "~/agro_robot_ws/logs_rastreabilidade")
        self.declare_parameter("detection_persistence_sec", 1.5)
        self.declare_parameter("dataset_data_yaml", "~/agro_robot_ws/datasets/agro_v1/data.yaml")
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_check_status", "unknown")
        self.declare_parameter("model_names_b64", "")
        self.declare_parameter(
            "model_names_json",
            "",
            ParameterDescriptor(dynamic_typing=True),
        )

        raw_session_id = str(self.get_parameter("session_id").value).strip()
        self.session_id = raw_session_id if raw_session_id else time.strftime("%Y%m%d_%H%M%S")
        self.grid_resolution_m = max(float(self.get_parameter("grid_resolution_m").value), 0.1)
        self.pump_rate_l_min = max(float(self.get_parameter("pump_rate_l_min").value), 0.0)
        self.confidence_mode = str(self.get_parameter("confidence_mode").value).strip().lower()
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.spray_topic = str(self.get_parameter("spray_recommendation_topic").value)
        self.status_cache_path = os.path.expanduser(str(self.get_parameter("status_cache_path").value))
        self.logs_base_dir = os.path.expanduser(str(self.get_parameter("logs_base_dir").value))
        self.detection_persistence_sec = max(float(self.get_parameter("detection_persistence_sec").value), 0.1)
        self.dataset_data_yaml = os.path.expanduser(str(self.get_parameter("dataset_data_yaml").value))
        self.model_path = os.path.expanduser(str(self.get_parameter("model_path").value))
        self.model_check_status = str(self.get_parameter("model_check_status").value or "unknown").strip()
        self.model_names = self._parse_model_names_b64(
            str(self.get_parameter("model_names_b64").value or "")
        )
        if not self.model_names:
            self.model_names = self._parse_model_names_legacy(
                self.get_parameter("model_names_json").value
            )

        self.output_dir = os.path.join(self.logs_base_dir, f"Pacote_Rastreabilidade_{self.session_id}")
        os.makedirs(self.output_dir, exist_ok=True)

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.recommendation_pub = self.create_publisher(Bool, self.spray_topic, 10)

        self.detections_sub = self.create_subscription(
            Detection2DArray, self.detections_topic, self._detections_cb, 10
        )
        self.odom_global_sub = self.create_subscription(
            Odometry, "/odometry/global", self._odom_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )

        self.current_x = None
        self.current_y = None
        self.current_linear_vel = 0.0

        self.grid_counts = defaultdict(int)
        self.grid_disease_counts = defaultdict(Counter)
        self.disease_counter = Counter()
        self.inspection_rows = []
        self.dataset_class_names, self.class_source = self._load_dataset_class_names()
        self.class_name_by_norm = {}
        for class_name in self.dataset_class_names:
            norm = self._normalize_class_name(class_name)
            if norm and norm not in self.class_name_by_norm:
                self.class_name_by_norm[norm] = class_name
        self.class_counts = {name: 0 for name in self.dataset_class_names}
        self.unmapped_label_counter = Counter()
        self.unmapped_detections_total = 0

        self.total_detections = 0
        self.last_detection_time = 0.0

        self.liters_baseline = 0.0
        self.liters_spot = 0.0

        self._last_tick = time.time()
        self._last_status = {}
        self._unmapped_warned = False

        self.timer = self.create_timer(0.25, self._tick)

        self.get_logger().info(
            "Infestation analytics iniciado | session_id=%s | grid=%.2fm"
            % (self.session_id, self.grid_resolution_m)
        )
        self.get_logger().info(
            "Fonte de classes IA: %s | classes=%d | data_yaml=%s"
            % (self.class_source, len(self.class_counts), self.dataset_data_yaml)
        )
        self.get_logger().info(
            "Modelo IA em uso: %s | check=%s | classes_modelo=%d"
            % (self.model_path or "nao_informado", self.model_check_status, len(self.model_names))
        )
        self.get_logger().info(
            "Status cache IA: %s | session_id=%s"
            % (self.status_cache_path, self.session_id)
        )

    def _parse_model_names_b64(self, raw):
        text = str(raw or "").strip()
        if not text:
            return []

        padded = text + ("=" * ((4 - len(text) % 4) % 4))
        try:
            decoded = base64.urlsafe_b64decode(padded.encode("ascii")).decode("utf-8")
            payload = json.loads(decoded)
        except Exception as exc:
            self.get_logger().warn("Falha ao decodificar model_names_b64: %s" % str(exc))
            return []

        if isinstance(payload, dict):
            items = []
            for key, value in payload.items():
                try:
                    idx = int(key)
                except Exception:
                    continue
                items.append((idx, str(value)))
            items.sort(key=lambda x: x[0])
            return [name for _, name in items]

        if isinstance(payload, list):
            return [str(item) for item in payload]

        return []

    def _parse_model_names_legacy(self, raw):
        payload = raw
        if isinstance(payload, str):
            text = payload.strip()
            if not text:
                return []
            try:
                payload = json.loads(text)
            except Exception:
                try:
                    payload = ast.literal_eval(text)
                except Exception as exc:
                    self.get_logger().warn("Falha ao decodificar model_names_json legado: %s" % str(exc))
                    return []

        if isinstance(payload, dict):
            items = []
            for key, value in payload.items():
                try:
                    idx = int(key)
                except Exception:
                    continue
                items.append((idx, str(value)))
            items.sort(key=lambda x: x[0])
            return [name for _, name in items]

        if isinstance(payload, (list, tuple)):
            return [str(item) for item in payload]

        return []

    def _normalize_class_name(self, value):
        text = str(value or "").strip().lower()
        text = (
            text.replace("ã", "a")
            .replace("á", "a")
            .replace("â", "a")
            .replace("é", "e")
            .replace("ê", "e")
            .replace("í", "i")
            .replace("ó", "o")
            .replace("ô", "o")
            .replace("ú", "u")
            .replace("_", " ")
            .replace("-", " ")
        )
        text = " ".join(text.split())
        return text

    def _strip_yaml_scalar(self, value):
        text = str(value or "").strip()
        if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
            text = text[1:-1]
        return text.strip()

    def _read_names_from_data_yaml(self):
        if not os.path.isfile(self.dataset_data_yaml):
            return [], "Arquivo data.yaml nao encontrado."

        names_map = {}
        names_list = []
        in_names = False
        found_names_block = False

        try:
            with open(self.dataset_data_yaml, "r", encoding="utf-8", errors="ignore") as f:
                for raw_line in f:
                    line = raw_line.rstrip("\n")
                    stripped = line.strip()

                    if not in_names:
                        if stripped == "names:":
                            in_names = True
                            found_names_block = True
                        continue

                    if not stripped or stripped.startswith("#"):
                        continue

                    if line == line.lstrip(" \t") and re.match(r"^[A-Za-z_][A-Za-z0-9_]*\s*:", stripped):
                        break

                    if stripped.startswith("- "):
                        names_list.append(self._strip_yaml_scalar(stripped[2:]))
                        continue

                    match = re.match(r"^(\d+)\s*:\s*(.+)$", stripped)
                    if match:
                        idx = int(match.group(1))
                        names_map[idx] = self._strip_yaml_scalar(match.group(2))
                        continue

            if not found_names_block:
                return [], "Bloco 'names' ausente no data.yaml."

            if names_map:
                ordered = []
                max_idx = max(names_map.keys())
                for idx in range(max_idx + 1):
                    if idx not in names_map:
                        return [], f"Classes no data.yaml nao contiguas. Indice ausente: {idx}."
                    ordered.append(names_map[idx])
                return ordered, ""

            if names_list:
                return names_list, ""

            return [], "Bloco 'names' vazio no data.yaml."
        except Exception as exc:
            return [], f"Falha ao ler data.yaml: {exc}"

    def _load_dataset_class_names(self):
        class_names, err = self._read_names_from_data_yaml()
        if err:
            self.get_logger().warn("Nao foi possivel carregar classes do data.yaml: %s" % err)
            return [], "runtime_fallback"

        cleaned = []
        for idx, raw_name in enumerate(class_names):
            name = str(raw_name or "").strip()
            if not name:
                name = f"class_{idx}"
            cleaned.append(name)

        return cleaned, "data_yaml"

    def _ensure_runtime_class(self, class_name):
        key = str(class_name or "").strip() or "desconhecida"
        if key not in self.class_counts:
            self.class_counts[key] = 0
        return key

    def _map_detection_label(self, raw_class_id, raw_label):
        raw_class_id = str(raw_class_id or "").strip()
        raw_label = str(raw_label or "").strip()
        fallback_label = raw_label or raw_class_id or "desconhecida"

        if not self.dataset_class_names:
            return self._ensure_runtime_class(fallback_label), True

        if raw_class_id:
            try:
                idx = int(raw_class_id)
                if 0 <= idx < len(self.dataset_class_names):
                    return self.dataset_class_names[idx], True
            except Exception:
                pass

        norm = self._normalize_class_name(raw_label)
        if norm and norm in self.class_name_by_norm:
            return self.class_name_by_norm[norm], True

        return fallback_label, False

    def _odom_cb(self, msg):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        self.current_linear_vel = float(msg.twist.twist.linear.x)

    def _current_cell(self):
        if self.current_x is None or self.current_y is None:
            return None
        return (
            int(math.floor(self.current_x / self.grid_resolution_m)),
            int(math.floor(self.current_y / self.grid_resolution_m)),
        )

    def _is_recommendation_active(self, now_ts):
        return (now_ts - self.last_detection_time) <= self.detection_persistence_sec

    def _detections_cb(self, msg):
        if not msg.detections:
            return

        now_iso = utc_iso_now()
        cell = self._current_cell()

        for det in msg.detections:
            label = "desconhecida"
            raw_class_id = ""
            score = 0.0
            if det.results:
                raw_class_id = str(det.results[0].hypothesis.class_id or "").strip()
                score = float(det.results[0].hypothesis.score)
            raw_label = str(getattr(det, "id", "") or "").strip()
            if not raw_label:
                raw_label = raw_class_id or "desconhecida"
            mapped_label, mapped = self._map_detection_label(raw_class_id, raw_label)

            self.total_detections += 1

            cell_x = ""
            cell_y = ""
            nivel_m2 = 0.0
            if cell is not None:
                cell_x, cell_y = cell
                self.grid_counts[cell] += 1
                if mapped:
                    self.grid_disease_counts[cell][mapped_label] += 1
                nivel_m2 = self.grid_counts[cell] / (self.grid_resolution_m * self.grid_resolution_m)

            if mapped:
                label = mapped_label
                self.disease_counter[label] += 1
                if label not in self.class_counts:
                    self.class_counts[label] = 0
                self.class_counts[label] += 1
            else:
                label = raw_label or "desconhecida"
                self.unmapped_detections_total += 1
                self.unmapped_label_counter[label] += 1

            self.inspection_rows.append(
                {
                    "Timestamp_UTC": now_iso,
                    "Map_X_m": "" if self.current_x is None else f"{self.current_x:.3f}",
                    "Map_Y_m": "" if self.current_y is None else f"{self.current_y:.3f}",
                    "Doenca_Detectada": label,
                    "Confianca": f"{score:.4f}",
                    "Cell_X": cell_x,
                    "Cell_Y": cell_y,
                    "Nivel_Infestacao_m2": f"{nivel_m2:.4f}",
                }
            )

        self.last_detection_time = time.time()

    def _current_dominant_disease(self):
        if not self.disease_counter:
            return "Nao_Detectada"
        return self.disease_counter.most_common(1)[0][0]

    def _current_nivel(self):
        cell = self._current_cell()
        if cell is None:
            return 0.0
        count = self.grid_counts.get(cell, 0)
        if count <= 0:
            return 0.0
        return count / (self.grid_resolution_m * self.grid_resolution_m)

    def _save_status_cache(self, status):
        cache_dir = os.path.dirname(self.status_cache_path)
        if cache_dir:
            os.makedirs(cache_dir, exist_ok=True)
        with open(self.status_cache_path, "w", encoding="utf-8") as f:
            json.dump(status, f, indent=2, ensure_ascii=False)

    def _tick(self):
        now_ts = time.time()
        dt = max(now_ts - self._last_tick, 0.0)
        self._last_tick = now_ts

        moving = abs(self.current_linear_vel) > 0.05
        spray_recommendation = self._is_recommendation_active(now_ts)

        if moving:
            litros_intervalo = (self.pump_rate_l_min / 60.0) * dt
            self.liters_baseline += litros_intervalo
            if spray_recommendation:
                self.liters_spot += litros_intervalo

        litros_economizados = max(self.liters_baseline - self.liters_spot, 0.0)

        if self.unmapped_detections_total > 0 and not self._unmapped_warned:
            labels = ", ".join(
                f"{name}:{count}" for name, count in sorted(self.unmapped_label_counter.items())
            )
            self.get_logger().warn(
                "Deteccoes nao mapeadas para data.yaml: total=%d [%s]"
                % (self.unmapped_detections_total, labels)
            )
            self._unmapped_warned = True

        status = {
            "session_id": self.session_id,
            "timestamp_utc": utc_iso_now(),
            "doenca_dominante": self._current_dominant_disease(),
            "nivel_infestacao_m2": round(self._current_nivel(), 4),
            "deteccoes_total": int(self.total_detections),
            "model_path": self.model_path,
            "model_check_status": self.model_check_status,
            "model_names": list(self.model_names),
            "class_source": self.class_source,
            "class_counts": {name: int(self.class_counts.get(name, 0)) for name in self.class_counts},
            "unmapped_detections_total": int(self.unmapped_detections_total),
            "unmapped_labels": {
                name: int(count) for name, count in sorted(self.unmapped_label_counter.items())
            },
            "confidence_mode": self.confidence_mode,
            "litros_baseline_l": round(self.liters_baseline, 4),
            "litros_spot_l": round(self.liters_spot, 4),
            "litros_economizados_l": round(litros_economizados, 4),
            "spray_recommendation": bool(spray_recommendation),
        }
        self._last_status = status

        status_msg = String()
        status_msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(status_msg)

        spray_msg = Bool()
        spray_msg.data = bool(spray_recommendation)
        self.recommendation_pub.publish(spray_msg)

        try:
            self._save_status_cache(status)
        except Exception as exc:
            self.get_logger().warn("Falha ao salvar status cache: %s" % str(exc))

    def _to_lon_lat(self, x_m, y_m):
        lat = LAT_REF + (float(y_m) * 0.000009)
        lon = LON_REF + (float(x_m) * 0.000009)
        return float(f"{lon:.6f}"), float(f"{lat:.6f}")

    def _build_heatmap_geojson(self):
        features = []
        for (cell_x, cell_y), count in sorted(self.grid_counts.items()):
            x0 = cell_x * self.grid_resolution_m
            x1 = (cell_x + 1) * self.grid_resolution_m
            y0 = cell_y * self.grid_resolution_m
            y1 = (cell_y + 1) * self.grid_resolution_m

            p1 = self._to_lon_lat(x0, y0)
            p2 = self._to_lon_lat(x1, y0)
            p3 = self._to_lon_lat(x1, y1)
            p4 = self._to_lon_lat(x0, y1)

            dominant = "Nao_Detectada"
            if self.grid_disease_counts[(cell_x, cell_y)]:
                dominant = self.grid_disease_counts[(cell_x, cell_y)].most_common(1)[0][0]

            nivel = count / (self.grid_resolution_m * self.grid_resolution_m)
            features.append(
                {
                    "type": "Feature",
                    "properties": {
                        "session_id": self.session_id,
                        "cell_x": cell_x,
                        "cell_y": cell_y,
                        "deteccoes": int(count),
                        "nivel_infestacao_m2": round(nivel, 4),
                        "doenca_dominante": dominant,
                    },
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[[p1[0], p1[1]], [p2[0], p2[1]], [p3[0], p3[1]], [p4[0], p4[1]], [p1[0], p1[1]]]],
                    },
                }
            )

        return {"type": "FeatureCollection", "features": features}

    def _save_outputs(self):
        os.makedirs(self.output_dir, exist_ok=True)

        inspect_path = os.path.join(self.output_dir, "inspecao_ia.csv")
        with open(inspect_path, "w", newline="", encoding="utf-8") as f:
            fieldnames = [
                "Timestamp_UTC",
                "Map_X_m",
                "Map_Y_m",
                "Doenca_Detectada",
                "Confianca",
                "Cell_X",
                "Cell_Y",
                "Nivel_Infestacao_m2",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.inspection_rows:
                writer.writerow(row)

        heatmap_path = os.path.join(self.output_dir, "heatmap_infestacao.geojson")
        with open(heatmap_path, "w", encoding="utf-8") as f:
            json.dump(self._build_heatmap_geojson(), f, indent=2, ensure_ascii=False)

        status_path = os.path.join(self.output_dir, "ia_status_final.json")
        with open(status_path, "w", encoding="utf-8") as f:
            json.dump(self._last_status, f, indent=2, ensure_ascii=False)

        self.get_logger().info(
            "Arquivos IA salvos em %s | deteccoes=%d" % (self.output_dir, self.total_detections)
        )


def main(args=None):
    rclpy.init(args=args)
    node = InfestationAnalyticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._save_outputs()
        except Exception as exc:
            node.get_logger().error("Falha ao salvar outputs IA: %s" % str(exc))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
