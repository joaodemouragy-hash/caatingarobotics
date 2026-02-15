#!/usr/bin/env python3
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
import re
import shutil
import sys
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# --- CONFIGURACAO DA FAZENDA ---
LAT_REF = -5.1873
LON_REF = -39.2936

# --- CONFIGURACAO DO ROBO ---
VAZAO_BOMBA_L_MIN = 3.5


def _safe_text(value, default):
    text = str(value).strip() if value is not None else ""
    return text if text else default


def _to_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def _to_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if value is None:
        return bool(default)
    text = str(value).strip().lower()
    if text in ("1", "true", "t", "sim", "yes", "y", "on"):
        return True
    if text in ("0", "false", "f", "nao", "não", "no", "n", "off"):
        return False
    return bool(default)


def _safe_tag(value, default):
    text = _safe_text(value, default)
    cleaned = re.sub(r"[^A-Za-z0-9_-]", "_", text)
    cleaned = re.sub(r"_+", "_", cleaned).strip("_")
    return cleaned if cleaned else default


def _utc_iso(dt):
    return dt.astimezone(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _round6(value):
    return float(f"{float(value):.6f}")


def _format_ptbr_decimal(value, decimals):
    try:
        return f"{float(value):.{decimals}f}".replace(".", ",")
    except Exception:
        return str(value)


def _is_missing_or_placeholder(value):
    text = str(value).strip() if value is not None else ""
    if not text:
        return True
    norm = (
        text.lower()
        .replace("ã", "a")
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
    norm = " ".join(norm.split())
    return norm in ("nao informado", "nao se aplica", "n/a", "na")


def _indent_xml(elem, level=0):
    indent = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + "  "
        for child in elem:
            _indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indent
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent


class RastreabilidadeNode(Node):
    def __init__(
        self,
        produto_nome,
        agua_litros,
        dose_ml,
        registro_mapa="Nao_Informado",
        perfil="universal",
        farm_gln="7890000012345",
        epc="NAO_INFORMADO",
        robot_uuid="AGRI-BOT-X99",
        operator_id="AUTO_PILOT",
        event_type="CHEMICAL_APPLICATION",
        fuel_consumption_lh=0.0,
        session_id="",
        ia_enabled=False,
        ia_status_topic="/caatinga_vision/infestation/status",
        ia_recommendation_topic="/caatinga_vision/spray/recommendation",
    ):
        super().__init__("rastreabilidade_ue_node")

        self.produto_nome = _safe_text(produto_nome, "Nao_Informado")
        self.registro_mapa = _safe_text(registro_mapa, "Nao_Informado")
        self.perfil = _safe_text(perfil, "universal").lower()
        if self.perfil not in ("universal", "ue", "brasil", "operacional"):
            self.perfil = "universal"

        self.farm_gln = _safe_text(farm_gln, "7890000012345")
        self.epc = _safe_text(epc, "NAO_INFORMADO")
        self.robot_uuid = _safe_text(robot_uuid, "AGRI-BOT-X99")
        self.operator_id = _safe_text(operator_id, "AUTO_PILOT")
        self.event_type = _safe_text(event_type, "CHEMICAL_APPLICATION")
        self.fuel_consumption_lh = max(_to_float(fuel_consumption_lh, 0.0), 0.0)
        self.ia_enabled = _to_bool(ia_enabled, False)
        self.ia_status_topic = _safe_text(ia_status_topic, "/caatinga_vision/infestation/status")
        self.ia_recommendation_topic = _safe_text(
            ia_recommendation_topic, "/caatinga_vision/spray/recommendation"
        )

        self.agua_litros = _to_float(agua_litros, 50.0)
        self.dose_ml = _to_float(dose_ml, 0.0)
        self.mistura_formatada = f"{self.agua_litros}L Agua + {self.dose_ml}ml Ingrediente Ativo"

        self.volume_restante = max(self.agua_litros, 0.0)
        self.volume_inicial_total = max(self.agua_litros, 0.0)
        self.tanque_vazio = False

        self.start_dt_utc = datetime.now(timezone.utc)
        self.start_time_tag = _safe_tag(session_id, self.start_dt_utc.strftime("%Y%m%d_%H%M%S"))
        self.lote_id = f"LOTE-{self.start_time_tag}"
        self.variedade_cultivar = "NAO_INFORMADO"
        self.last_time = time.time()

        self.track_points = []
        self.operation_points = []
        self.csv_data = []
        self.speed_samples_kmh = []
        self.volume_aplicado_total = 0.0
        self._last_linear_vel = 0.0
        self._last_map_x = 0.0
        self._last_map_y = 0.0

        self.ia_status_last = {}
        self.ia_spray_recommendation = False

        self.user_home = os.path.expanduser("~")
        self.log_dir = os.path.join(self.user_home, "agro_robot_ws/logs_rastreabilidade")
        os.makedirs(self.log_dir, exist_ok=True)

        self._setup_pose_subscription()
        if self.ia_enabled:
            self._setup_ia_subscriptions()
        self._validar_campos_normativos()

        self.get_logger().info(
            "Rastreio iniciado | Perfil: %s | Produto: %s | MAPA: %s"
            % (self.perfil, self.produto_nome, self.registro_mapa)
        )

    def _validar_campos_normativos(self):
        if not (self.farm_gln.isdigit() and len(self.farm_gln) == 13):
            self.get_logger().warn(
                "GLN fora do padrao GS1 (13 digitos). Valor atual: %s" % self.farm_gln
            )

    def validar_entrada_defensiva(self):
        erros = []
        perfil_brasil_ou_universal = self.perfil in ("brasil", "universal")

        if not self.robot_uuid.strip():
            erros.append("Identificador do robô é obrigatório.")

        if not self.operator_id.strip():
            erros.append("Identificador do operador é obrigatório.")

        if perfil_brasil_ou_universal:
            if not (self.farm_gln.isdigit() and len(self.farm_gln) == 13):
                erros.append(
                    "GLN inválido: deve conter exatamente 13 dígitos para perfil Brasil/Universal."
                )
            if _is_missing_or_placeholder(self.epc):
                erros.append("EPC é obrigatório para perfil Brasil/Universal.")

        if perfil_brasil_ou_universal and self.event_type == "CHEMICAL_APPLICATION":
            if _is_missing_or_placeholder(self.produto_nome):
                erros.append(
                    "Produto químico é obrigatório para aplicação química "
                    "no perfil Brasil/Universal."
                )
            if _is_missing_or_placeholder(self.registro_mapa):
                erros.append(
                    "Registro MAPA é obrigatório para aplicação química "
                    "no perfil Brasil/Universal."
                )

        return erros

    def _setup_pose_subscription(self):
        topics = dict(self.get_topic_names_and_types())
        if "/odometry/global" in topics and "nav_msgs/msg/Odometry" in topics["/odometry/global"]:
            self.subscription = self.create_subscription(
                Odometry, "/odometry/global", self.listener_callback_odom, 10
            )
            self.get_logger().info("Rastreabilidade usando /odometry/global")
            return
        if "/odom" in topics and "nav_msgs/msg/Odometry" in topics["/odom"]:
            self.subscription = self.create_subscription(
                Odometry, "/odom", self.listener_callback_odom, 10
            )
            self.get_logger().info("Rastreabilidade usando /odom")
            return
        if (
            "/amcl_pose" in topics
            and "geometry_msgs/msg/PoseWithCovarianceStamped" in topics["/amcl_pose"]
        ):
            self.subscription = self.create_subscription(
                PoseWithCovarianceStamped, "/amcl_pose", self.listener_callback_amcl, 10
            )
            self.get_logger().info("Rastreabilidade usando /amcl_pose")
            return

        self.subscription = self.create_subscription(
            Odometry, "/odom", self.listener_callback_odom, 10
        )
        self.get_logger().warn("Rastreabilidade usando /odom (fallback)")

    def _setup_ia_subscriptions(self):
        self.ia_status_subscription = self.create_subscription(
            String, self.ia_status_topic, self._ia_status_callback, 10
        )
        self.ia_recommendation_subscription = self.create_subscription(
            Bool, self.ia_recommendation_topic, self._ia_recommendation_callback, 10
        )
        self.get_logger().info(
            "IA habilitada | status_topic=%s | recommendation_topic=%s"
            % (self.ia_status_topic, self.ia_recommendation_topic)
        )

    def _ia_status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.ia_status_last = payload
        except Exception:
            pass

    def _ia_recommendation_callback(self, msg):
        self.ia_spray_recommendation = bool(msg.data)

    def _ia_csv_fields(self):
        status = self.ia_status_last if isinstance(self.ia_status_last, dict) else {}

        doenca = _safe_text(status.get("doenca_dominante"), "Nao_Detectada")
        nivel = _to_float(status.get("nivel_infestacao_m2"), 0.0)
        confidence_mode = _safe_text(status.get("confidence_mode"), "economico")
        litros_baseline = _to_float(status.get("litros_baseline_l"), 0.0)
        litros_spot = _to_float(status.get("litros_spot_l"), 0.0)
        litros_economizados = max(_to_float(status.get("litros_economizados_l"), 0.0), 0.0)
        spray_recommendation = bool(
            status.get("spray_recommendation", self.ia_spray_recommendation)
        )
        class_counts = status.get("class_counts", {})
        if isinstance(class_counts, dict):
            class_counts = {str(k): int(_to_float(v, 0.0)) for k, v in class_counts.items()}
            class_counts_json = json.dumps(class_counts, ensure_ascii=False, sort_keys=True)
        else:
            class_counts_json = "{}"
        unmapped_total = int(_to_float(status.get("unmapped_detections_total"), 0.0))
        unmapped_labels = status.get("unmapped_labels", {})
        if isinstance(unmapped_labels, dict):
            unmapped_labels = {str(k): int(_to_float(v, 0.0)) for k, v in unmapped_labels.items()}
            unmapped_labels_json = json.dumps(unmapped_labels, ensure_ascii=False, sort_keys=True)
        else:
            unmapped_labels_json = "{}"

        return {
            "Doenca_Detectada": doenca,
            "Nivel_Infestacao_m2": nivel,
            "Spray_Recommendation": spray_recommendation,
            "Confidence_Mode": confidence_mode,
            "Litros_Baseline_L": litros_baseline,
            "Litros_Pontual_L": litros_spot,
            "Litros_Economizados_L": litros_economizados,
            "IA_Class_Counts_JSON": class_counts_json,
            "IA_Unmapped_Total": unmapped_total,
            "IA_Unmapped_Labels_JSON": unmapped_labels_json,
        }

    def _append_unique_point(self, collection, lon, lat):
        point = [_round6(lon), _round6(lat)]
        if not collection or collection[-1] != point:
            collection.append(point)

    def _process_xy(self, x, y):
        current_time = time.time()
        dt = max(current_time - self.last_time, 0.0)
        self.last_time = current_time
        self._last_map_x = float(x)
        self._last_map_y = float(y)

        lat = LAT_REF + (y * 0.000009)
        lon = LON_REF + (x * 0.000009)
        timestamp_utc = _utc_iso(datetime.now(timezone.utc))

        self._append_unique_point(self.track_points, lon, lat)

        vel_linear = self._last_linear_vel
        speed_kmh = abs(vel_linear) * 3.6
        self.speed_samples_kmh.append(speed_kmh)

        vazao_atual = 0.0

        if self.ia_enabled:
            vazao_atual = 0.0
        elif "Mecanico" in self.produto_nome or "Sem" in self.produto_nome:
            vazao_atual = 0.0
        elif abs(vel_linear) > 0.1 and self.volume_restante > 0:
            litros_neste_instante = (VAZAO_BOMBA_L_MIN / 60.0) * dt
            if self.volume_restante >= litros_neste_instante:
                self.volume_restante -= litros_neste_instante
                self.volume_aplicado_total += litros_neste_instante
                vazao_atual = VAZAO_BOMBA_L_MIN
            else:
                self.volume_aplicado_total += self.volume_restante
                self.volume_restante = 0.0
                self.tanque_vazio = True
                self.get_logger().warn("ALERTA: CALDA ACABOU. Retornar para reabastecer.")
                vazao_atual = 0.0

        if abs(vel_linear) > 0.05:
            self._append_unique_point(self.operation_points, lon, lat)

        volume_esperado = max(self.volume_inicial_total - self.volume_aplicado_total, 0.0)
        erro_balanco = self.volume_restante - volume_esperado
        if abs(erro_balanco) > 0.02:
            self.get_logger().warn(
                "AJUSTE DE BALANCO: aplicado=%.3fL restante=%.3fL esperado=%.3fL erro=%.3fL"
                % (self.volume_aplicado_total, self.volume_restante, volume_esperado, erro_balanco)
            )
            self.volume_restante = volume_esperado
            erro_balanco = 0.0

        ia_should_log = self.ia_enabled and (
            abs(vel_linear) > 0.05 or self.ia_spray_recommendation or bool(self.ia_status_last)
        )

        if vazao_atual > 0 or self.tanque_vazio or ia_should_log:
            ia_fields = self._ia_csv_fields()
            self.csv_data.append(
                [
                    timestamp_utc,
                    f"{_round6(lat):.6f}",
                    f"{_round6(lon):.6f}",
                    self.produto_nome,
                    self.registro_mapa,
                    self.mistura_formatada,
                    f"{vazao_atual:.3f}",
                    f"{self.volume_aplicado_total:.3f}",
                    f"{self.volume_restante:.3f}",
                    f"{volume_esperado:.3f}",
                    f"{erro_balanco:.3f}",
                    self.event_type,
                    ia_fields["Doenca_Detectada"],
                    f"{ia_fields['Nivel_Infestacao_m2']:.4f}",
                    str(bool(ia_fields["Spray_Recommendation"])),
                    ia_fields["Confidence_Mode"],
                    f"{ia_fields['Litros_Baseline_L']:.4f}",
                    f"{ia_fields['Litros_Pontual_L']:.4f}",
                    f"{ia_fields['Litros_Economizados_L']:.4f}",
                    ia_fields["IA_Class_Counts_JSON"],
                    str(int(ia_fields["IA_Unmapped_Total"])),
                    ia_fields["IA_Unmapped_Labels_JSON"],
                ]
            )
            if self.tanque_vazio:
                self.tanque_vazio = False

    def listener_callback_odom(self, msg):
        self._last_linear_vel = msg.twist.twist.linear.x
        self._process_xy(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def listener_callback_amcl(self, msg):
        self._last_linear_vel = 0.0
        self._process_xy(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _convex_hull(self, points):
        pts = sorted({(float(x), float(y)) for x, y in points})
        if len(pts) <= 1:
            return pts

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        lower = []
        for p in pts:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)

        upper = []
        for p in reversed(pts):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)

        return lower[:-1] + upper[:-1]

    def _build_polygon_coords(self):
        source = self.operation_points if len(self.operation_points) >= 3 else self.track_points

        if not source:
            lon = _round6(LON_REF)
            lat = _round6(LAT_REF)
            d = 0.00001
            return [
                [_round6(lon - d), _round6(lat - d)],
                [_round6(lon + d), _round6(lat - d)],
                [_round6(lon + d), _round6(lat + d)],
                [_round6(lon - d), _round6(lat + d)],
                [_round6(lon - d), _round6(lat - d)],
            ]

        hull = self._convex_hull(source)
        if len(hull) < 3:
            lon, lat = source[0]
            d = 0.00001
            polygon = [
                (_round6(lon - d), _round6(lat - d)),
                (_round6(lon + d), _round6(lat - d)),
                (_round6(lon + d), _round6(lat + d)),
                (_round6(lon - d), _round6(lat + d)),
            ]
        else:
            polygon = [(_round6(lon), _round6(lat)) for lon, lat in hull]

        if polygon[0] != polygon[-1]:
            polygon.append(polygon[0])

        return [[lon, lat] for lon, lat in polygon]

    def _calc_area_ha(self, polygon_coords):
        if len(polygon_coords) < 4:
            return 0.0

        pts = polygon_coords[:-1] if polygon_coords[0] == polygon_coords[-1] else polygon_coords
        if len(pts) < 3:
            return 0.0

        lat0 = math.radians(sum(lat for _, lat in pts) / len(pts))
        meters_per_deg_lat = 111132.92
        meters_per_deg_lon = 111320.0 * math.cos(lat0)

        projected = [(lon * meters_per_deg_lon, lat * meters_per_deg_lat) for lon, lat in pts]

        area = 0.0
        for i in range(len(projected)):
            x1, y1 = projected[i]
            x2, y2 = projected[(i + 1) % len(projected)]
            area += (x1 * y2) - (x2 * y1)

        area_m2 = abs(area) / 2.0
        return round(area_m2 / 10000.0, 4)

    def _avg_speed_kmh(self):
        if not self.speed_samples_kmh:
            return 0.0
        return round(sum(self.speed_samples_kmh) / len(self.speed_samples_kmh), 3)

    def _last_read_point(self):
        if self.track_points:
            lon, lat = self.track_points[-1]
            return {"lon": _round6(lon), "lat": _round6(lat)}
        return {"lon": _round6(LON_REF), "lat": _round6(LAT_REF)}

    def _build_track_geojson(self):
        coords = self.track_points if self.track_points else [[_round6(LON_REF), _round6(LAT_REF)]]
        return {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {
                        "event_type": self.event_type,
                        "produto_utilizado": self.produto_nome,
                        "numero_registro_mapa": self.registro_mapa,
                    },
                    "geometry": {"type": "LineString", "coordinates": coords},
                }
            ],
        }

    def _build_eudr_geojson(self, polygon_coords, area_ha):
        return {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {
                        "area_ha": area_ha,
                        "produto_utilizado": self.produto_nome,
                        "numero_registro_mapa": self.registro_mapa,
                        "farm_gln": self.farm_gln,
                        "timestamp_start_utc": _utc_iso(self.start_dt_utc),
                    },
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [polygon_coords],
                    },
                }
            ],
        }

    def _build_ia_summary(self):
        if not isinstance(self.ia_status_last, dict) or not self.ia_status_last:
            return {}

        status = self.ia_status_last
        class_counts_raw = status.get("class_counts", {})
        unmapped_raw = status.get("unmapped_labels", {})

        class_counts = {}
        if isinstance(class_counts_raw, dict):
            for key, value in class_counts_raw.items():
                name = str(key).strip()
                if not name:
                    continue
                try:
                    class_counts[name] = int(value)
                except Exception:
                    class_counts[name] = 0

        unmapped = {}
        if isinstance(unmapped_raw, dict):
            for key, value in unmapped_raw.items():
                name = str(key).strip()
                if not name:
                    continue
                try:
                    unmapped[name] = int(value)
                except Exception:
                    unmapped[name] = 0

        model_names_raw = status.get("model_names", [])
        model_names = []
        if isinstance(model_names_raw, list):
            model_names = [str(item).strip() for item in model_names_raw if str(item).strip()]

        return {
            "model_path": _safe_text(status.get("model_path"), ""),
            "model_check_status": _safe_text(status.get("model_check_status"), "unknown"),
            "model_names": model_names,
            "class_source": _safe_text(status.get("class_source"), "desconhecida"),
            "class_counts": class_counts,
            "unmapped_detections_total": int(
                _to_float(status.get("unmapped_detections_total"), 0.0)
            ),
            "unmapped_labels": unmapped,
            "doenca_dominante": _safe_text(status.get("doenca_dominante"), "Nao_Detectada"),
            "deteccoes_total": int(_to_float(status.get("deteccoes_total"), 0.0)),
            "nivel_infestacao_m2": round(_to_float(status.get("nivel_infestacao_m2"), 0.0), 4),
            "spray_recommendation": _to_bool(status.get("spray_recommendation"), False),
        }

    def _build_universal_payload(self, polygon_coords, area_ha, end_dt_utc):
        payload = {
            "header": {
                "version": "2.1",
                "robot_uuid": self.robot_uuid,
                "farm_gln": self.farm_gln,
                "operator_id": self.operator_id,
                "timestamp_start": _utc_iso(self.start_dt_utc),
                "timestamp_end": _utc_iso(end_dt_utc),
                "profile": self.perfil,
            },
            "spatial_data": {
                "standard": "RFC7946",
                "crs": "EPSG:4326",
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [polygon_coords],
                },
                "area_ha": area_ha,
            },
            "operation_data": {
                "type": self.event_type,
                "inputs": [
                    {
                        "type": "CHEMICAL" if "CHEMICAL" in self.event_type else "OPERATION",
                        "epc": self.epc,
                        "mapa_registration": self.registro_mapa,
                        "product_name": self.produto_nome,
                        "mixture": self.mistura_formatada,
                        "rate_applied": {
                            "value": round(self.dose_ml, 3),
                            "unit": "ml",
                        },
                    }
                ],
            },
            "telemetry_avg": {
                "speed_kmh": self._avg_speed_kmh(),
                "fuel_consumption_lh": round(self.fuel_consumption_lh, 3),
            },
        }
        ia_summary = self._build_ia_summary()
        if ia_summary:
            payload["ia_summary"] = ia_summary
        return payload

    def _build_brasil_event_json(self, end_dt_utc):
        quantidade_l = round(max(self.volume_aplicado_total, 0.0), 3)
        quantidade_l = (
            quantidade_l
            if quantidade_l > 0
            else round(max(self.volume_inicial_total, 0.0), 3)
        )
        return {
            "event_type": self.event_type,
            "epc": self.epc,
            "gln": self.farm_gln,
            "read_point": self._last_read_point(),
            "timestamp_utc": _utc_iso(end_dt_utc),
            "produto": self.produto_nome,
            "registro_mapa": self.registro_mapa,
            "norma_referencia": {
                "inc_anvisa_mapa": "02/2018",
                "pnrv": "Programa Nacional de Rastreabilidade Voluntaria",
            },
            "inc_02_2018_minimo": {
                "produto_vegetal": {
                    "nome_produto": self.produto_nome,
                    "variedade_ou_cultivar": self.variedade_cultivar,
                    "quantidade_produto": quantidade_l,
                    "unidade": "L",
                    "identificacao_lote": self.lote_id,
                    "data_recebimento_utc": _utc_iso(self.start_dt_utc),
                },
                "fornecedor": {
                    "nome_ou_razao_social": "NAO_INFORMADO",
                    "cpf_ie_cnpj_ou_cgc_mapa": "NAO_INFORMADO",
                    "endereco_ou_ccir": "NAO_INFORMADO",
                },
                "comprador": {
                    "nome_ou_razao_social": "NAO_INFORMADO",
                    "cpf_ie_cnpj_ou_cgc_mapa": "NAO_INFORMADO",
                    "endereco_ou_ccir": "NAO_INFORMADO",
                },
            },
        }

    def _build_manifesto_xml(self, end_dt_utc):
        read_point = self._last_read_point()

        root = ET.Element("ManifestoInsumo")
        header = ET.SubElement(root, "Header")
        ET.SubElement(header, "Version").text = "1.0"
        ET.SubElement(header, "TimestampUTC").text = _utc_iso(end_dt_utc)

        origem = ET.SubElement(root, "Origem")
        ET.SubElement(origem, "GLN").text = self.farm_gln
        ET.SubElement(origem, "RobotUUID").text = self.robot_uuid
        ET.SubElement(origem, "OperatorID").text = self.operator_id

        insumo = ET.SubElement(root, "Insumo")
        ET.SubElement(insumo, "EPC").text = self.epc
        ET.SubElement(insumo, "Produto").text = self.produto_nome
        ET.SubElement(insumo, "RegistroMAPA").text = self.registro_mapa
        ET.SubElement(insumo, "LoteID").text = self.lote_id
        ET.SubElement(insumo, "VariedadeCultivar").text = self.variedade_cultivar

        leitura = ET.SubElement(root, "ReadPoint")
        ET.SubElement(leitura, "Lon").text = f"{read_point['lon']:.6f}"
        ET.SubElement(leitura, "Lat").text = f"{read_point['lat']:.6f}"

        _indent_xml(root)
        return ET.tostring(root, encoding="utf-8", xml_declaration=True)

    def _build_taskdata_xml(self, end_dt_utc):
        root = ET.Element("TASKDATA", attrib={"Version": "1.0", "DataTransferOrigin": "Robot"})
        task = ET.SubElement(
            root,
            "TSK",
            attrib={"TaskID": f"TSK-{self.start_time_tag}", "Type": self.event_type},
        )
        ET.SubElement(
            task,
            "TIM",
            attrib={
                "StartUTC": _utc_iso(self.start_dt_utc),
                "EndUTC": _utc_iso(end_dt_utc),
            },
        )

        for idx, (lon, lat) in enumerate(self.track_points, start=1):
            ET.SubElement(
                task,
                "PTN",
                attrib={
                    "Seq": str(idx),
                    "Lon": f"{_round6(lon):.6f}",
                    "Lat": f"{_round6(lat):.6f}",
                },
            )

        compliance = ET.SubElement(task, "Compliance")
        ET.SubElement(compliance, "NormaBrasil").text = "INC_ANVISA_MAPA_02_2018"
        ET.SubElement(compliance, "NormaUE").text = "EUDR_2023_1115"

        _indent_xml(root)
        return ET.tostring(root, encoding="utf-8", xml_declaration=True)

    def detect_usb_storage(self):
        user = os.environ.get("USER", "ubuntu")
        media_base = os.path.join("/media", user)
        if os.path.exists(media_base):
            try:
                devices = [
                    os.path.join(media_base, d)
                    for d in os.listdir(media_base)
                    if os.path.isdir(os.path.join(media_base, d))
                ]
                if devices:
                    return devices[0]
            except Exception:
                return None
        return None

    def _save_csv_or_summary(self, output_dir):
        csv_path = os.path.join(output_dir, "operacao_detalhada.csv")
        if self.csv_data:
            headers = [
                "Timestamp_UTC",
                "Lat",
                "Lon",
                "Produto_Principal",
                "Registro_MAPA",
                "Composicao_Mistura",
                "Vazao_L_min",
                "Vol_Aplicado_L",
                "Vol_Restante_L",
                "Vol_Esperado_L",
                "Erro_Balanco_L",
                "Event_Type",
                "Doenca_Detectada",
                "Nivel_Infestacao_m2",
                "Spray_Recommendation",
                "Confidence_Mode",
                "Litros_Baseline_L",
                "Litros_Pontual_L",
                "Litros_Economizados_L",
                "IA_Class_Counts_JSON",
                "IA_Unmapped_Total",
                "IA_Unmapped_Labels_JSON",
            ]
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                writer.writerows(self.csv_data)

            # Arquivo auxiliar para Excel PT-BR (separador ';' e decimal com vírgula).
            csv_ptbr_path = os.path.join(output_dir, "operacao_detalhada_ptbr.csv")
            with open(csv_ptbr_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f, delimiter=";")
                writer.writerow(headers)
                for row in self.csv_data:
                    row_ptbr = list(row)
                    precisao = {
                        1: 6, 2: 6, 6: 3, 7: 3, 8: 3, 9: 3, 10: 3, 13: 4, 16: 4,
                        17: 4, 18: 4,
                    }
                    for idx, decimals in precisao.items():
                        row_ptbr[idx] = _format_ptbr_decimal(row_ptbr[idx], decimals)
                    writer.writerow(row_ptbr)

            return [csv_path, csv_ptbr_path]

        txt_path = os.path.join(output_dir, "operacao_resumo.txt")
        with open(txt_path, "w", encoding="utf-8") as f:
            f.write(
                f"Perfil: {self.perfil}\n"
                f"Operacao: {self.event_type}\n"
                f"Produto: {self.produto_nome}\n"
                f"Registro_MAPA: {self.registro_mapa}\n"
                f"Mistura: {self.mistura_formatada}\n"
                "Nenhuma aplicacao registrada.\n"
            )
        return [txt_path]

    def save_files(self):
        end_dt_utc = datetime.now(timezone.utc)

        output_dir = os.path.join(self.log_dir, f"Pacote_Rastreabilidade_{self.start_time_tag}")
        os.makedirs(output_dir, exist_ok=True)

        polygon_coords = self._build_polygon_coords()
        area_ha = self._calc_area_ha(polygon_coords)

        arquivos_gerados = []

        # Base operacional (sempre)
        universal_payload = self._build_universal_payload(polygon_coords, area_ha, end_dt_utc)
        universal_path = os.path.join(output_dir, "universal_payload.json")
        with open(universal_path, "w", encoding="utf-8") as f:
            json.dump(universal_payload, f, indent=2, ensure_ascii=False)
        arquivos_gerados.append(universal_path)

        track_geojson = self._build_track_geojson()
        track_path = os.path.join(output_dir, "trilha_operacao.geojson")
        with open(track_path, "w", encoding="utf-8") as f:
            json.dump(track_geojson, f, indent=2, ensure_ascii=False)
        arquivos_gerados.append(track_path)

        arquivos_gerados.extend(self._save_csv_or_summary(output_dir))

        for extra_name in (
            "inspecao_ia.csv",
            "heatmap_infestacao.geojson",
            "ia_status_final.json",
        ):
            extra_path = os.path.join(output_dir, extra_name)
            if os.path.exists(extra_path):
                arquivos_gerados.append(extra_path)

        # Perfis normativos
        if self.perfil in ("universal", "ue"):
            eudr_geojson = self._build_eudr_geojson(polygon_coords, area_ha)
            eudr_path = os.path.join(output_dir, "eudr_area.geojson")
            with open(eudr_path, "w", encoding="utf-8") as f:
                json.dump(eudr_geojson, f, indent=2, ensure_ascii=False)
            arquivos_gerados.append(eudr_path)

        if self.perfil in ("universal", "brasil"):
            br_event = self._build_brasil_event_json(end_dt_utc)
            br_event_path = os.path.join(output_dir, "brasil_evento_operacao.json")
            with open(br_event_path, "w", encoding="utf-8") as f:
                json.dump(br_event, f, indent=2, ensure_ascii=False)
            arquivos_gerados.append(br_event_path)

            manifesto_xml = self._build_manifesto_xml(end_dt_utc)
            manifesto_path = os.path.join(output_dir, "manifesto_insumo.xml")
            with open(manifesto_path, "wb") as f:
                f.write(manifesto_xml)
            arquivos_gerados.append(manifesto_path)

            taskdata_xml = self._build_taskdata_xml(end_dt_utc)
            taskdata_path = os.path.join(output_dir, "TASKDATA.XML")
            with open(taskdata_path, "wb") as f:
                f.write(taskdata_xml)
            arquivos_gerados.append(taskdata_path)

        # Compatibilidade com formato legado ja utilizado na operacao
        legacy_base = f"Relatorio_Certificacao_{self.start_time_tag}"
        legacy_geojson_path = os.path.join(self.log_dir, f"{legacy_base}.geojson")
        legacy_txt_path = os.path.join(self.log_dir, f"{legacy_base}.txt")
        legacy_geojson = (
            self._build_eudr_geojson(polygon_coords, area_ha)
            if self.perfil in ("universal", "ue")
            else track_geojson
        )
        with open(legacy_geojson_path, "w", encoding="utf-8") as f:
            json.dump(legacy_geojson, f, indent=2, ensure_ascii=False)
        with open(legacy_txt_path, "w", encoding="utf-8") as f:
            f.write(
                f"Perfil: {self.perfil}\n"
                f"Operacao: {self.event_type}\n"
                f"Produto: {self.produto_nome}\n"
                f"Registro_MAPA: {self.registro_mapa}\n"
                f"GLN: {self.farm_gln}\n"
                f"Lote: {self.lote_id}\n"
                f"Mistura: {self.mistura_formatada}\n"
                f"Area_ha: {area_ha}\n"
            )
        arquivos_gerados.extend([legacy_geojson_path, legacy_txt_path])

        self.get_logger().info(
            "Pacote salvo em %s | Arquivos: %d | Area_ha: %.4f"
            % (output_dir, len(arquivos_gerados), area_ha)
        )

        usb_path = self.detect_usb_storage()
        if usb_path:
            try:
                usb_dest = os.path.join(usb_path, "Caatinga_Dados", os.path.basename(output_dir))
                os.makedirs(usb_dest, exist_ok=True)
                for arquivo in arquivos_gerados:
                    if os.path.exists(arquivo):
                        shutil.copy2(arquivo, usb_dest)
                self.get_logger().info("Arquivos copiados para USB: %s" % usb_dest)
            except Exception as e:
                self.get_logger().error("Erro ao copiar para USB: %s" % str(e))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Gerador de pacote de rastreabilidade (UE + Brasil)"
    )
    parser.add_argument("--produto", default="Nao_Informado")
    parser.add_argument("--agua", default="50.0")
    parser.add_argument("--dose", default="0")
    parser.add_argument("--registro_mapa", default="Nao_Informado")
    parser.add_argument(
        "--perfil",
        default="universal",
        choices=["universal", "ue", "brasil", "operacional"],
    )
    parser.add_argument("--farm_gln", default="7890000012345")
    parser.add_argument("--epc", default="NAO_INFORMADO")
    parser.add_argument("--robot_uuid", default="AGRI-BOT-X99")
    parser.add_argument("--operator_id", default="AUTO_PILOT")
    parser.add_argument("--event_type", default="CHEMICAL_APPLICATION")
    parser.add_argument("--fuel_lh", default="0.0")
    parser.add_argument("--session_id", default="")
    parser.add_argument("--ia_enabled", default="false")
    parser.add_argument("--ia_status_topic", default="/caatinga_vision/infestation/status")
    parser.add_argument(
        "--ia_recommendation_topic",
        default="/caatinga_vision/spray/recommendation",
    )
    return parser.parse_args(argv)


def main(args=None):
    parsed = parse_args(args if args is not None else sys.argv[1:])
    exit_code = 0
    permitir_salvar = False
    rclpy.init()
    node = RastreabilidadeNode(
        produto_nome=parsed.produto,
        agua_litros=parsed.agua,
        dose_ml=parsed.dose,
        registro_mapa=parsed.registro_mapa,
        perfil=parsed.perfil,
        farm_gln=parsed.farm_gln,
        epc=parsed.epc,
        robot_uuid=parsed.robot_uuid,
        operator_id=parsed.operator_id,
        event_type=parsed.event_type,
        fuel_consumption_lh=parsed.fuel_lh,
        session_id=parsed.session_id,
        ia_enabled=parsed.ia_enabled,
        ia_status_topic=parsed.ia_status_topic,
        ia_recommendation_topic=parsed.ia_recommendation_topic,
    )

    try:
        erros = node.validar_entrada_defensiva()
        if erros:
            for err in erros:
                node.get_logger().error(f"VALIDACAO: {err}")
            exit_code = 2
            return exit_code

        permitir_salvar = True
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if permitir_salvar:
            node.save_files()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
