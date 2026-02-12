#!/usr/bin/env python3
import base64
import csv
from glob import glob
import json
import math
import os
import random
import re
import shlex
import shutil
import signal
import site
import subprocess
import sys
import tempfile
import threading
import time
import zipfile

from PyQt5.QtCore import QEventLoop, QProcess, Qt, QThread, QTimer, QUrl, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPixmap, QTextCursor
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDialog,
                             QDialogButtonBox, QDoubleSpinBox, QFileDialog, QFrame,
                             QGridLayout, QHBoxLayout, QInputDialog, QLabel,
                             QLineEdit, QListWidget, QListWidgetItem, QMainWindow,
                             QMessageBox, QProgressBar, QPushButton, QSpinBox,
                             QTabWidget, QTextEdit, QTreeWidget, QTreeWidgetItem,
                             QVBoxLayout, QWidget)

WEBENGINE_ERROR = ""
try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    WEBENGINE_DISPONIVEL = True
except Exception as e:
    QWebEngineView = None
    WEBENGINE_DISPONIVEL = False
    WEBENGINE_ERROR = str(e)

# Configuração gráfica para Linux
os.environ["QT_QPA_PLATFORM"] = "xcb"


class SensorTopViewPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background-color: #1e1e1e; color: #e0e0e0;")

        self._sensor_states = {
            "esquerda": False,
            "frente": False,
            "direita": False,
        }
        self._sensor_widgets = {}
        self._status_labels = {}
        self._build_ui()
        self.set_sensor_states(esquerda=False, frente=False, direita=False)

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(12)

        lbl_title = QLabel("Vista Superior do Robô")
        lbl_title.setStyleSheet("font-size: 15px; font-weight: bold; color: #eeeeee;")
        lbl_title.setAlignment(Qt.AlignCenter)
        root.addWidget(lbl_title)

        lbl_hint = QLabel("Frente para cima")
        lbl_hint.setStyleSheet("color: #b0bec5; font-size: 11px;")
        lbl_hint.setAlignment(Qt.AlignCenter)
        root.addWidget(lbl_hint)

        diagram_frame = QFrame()
        diagram_frame.setStyleSheet(
            "QFrame { background-color: #121212; border: 1px solid #2f2f2f; border-radius: 12px; }"
        )
        diagram_layout = QGridLayout(diagram_frame)
        diagram_layout.setContentsMargins(20, 20, 20, 20)
        diagram_layout.setHorizontalSpacing(10)
        diagram_layout.setVerticalSpacing(10)

        lbl_orient = QLabel("FRENTE ↑")
        lbl_orient.setStyleSheet((
            "color: #cfd8dc; font-size: 11px; font-weight: bold; border:"
            "0px;"
        ))
        lbl_orient.setAlignment(Qt.AlignCenter)
        diagram_layout.addWidget(lbl_orient, 0, 1, 1, 1)

        frente = QFrame()
        frente.setFixedSize(220, 24)
        diagram_layout.addWidget(frente, 1, 1, 1, 1, Qt.AlignHCenter)

        esquerda = QFrame()
        esquerda.setFixedSize(24, 180)
        diagram_layout.addWidget(esquerda, 2, 0, 1, 1, Qt.AlignRight | Qt.AlignVCenter)

        corpo = QFrame()
        corpo.setFixedSize(220, 180)
        corpo.setStyleSheet(
            "QFrame { background-color: #2d3640; border: 1px solid #46535f; border-radius: 8px; }"
        )
        corpo_layout = QVBoxLayout(corpo)
        corpo_layout.setContentsMargins(12, 12, 12, 12)
        corpo_layout.setSpacing(6)
        lbl_corpo = QLabel("ROBÔ")
        lbl_corpo.setAlignment(Qt.AlignCenter)
        lbl_corpo.setStyleSheet("font-size: 18px; font-weight: bold; color: #e8f5e9; border: 0px;")
        lbl_dim = QLabel("Chassi / Lataria")
        lbl_dim.setAlignment(Qt.AlignCenter)
        lbl_dim.setStyleSheet("font-size: 11px; color: #b0bec5; border: 0px;")
        corpo_layout.addStretch(1)
        corpo_layout.addWidget(lbl_corpo)
        corpo_layout.addWidget(lbl_dim)
        corpo_layout.addStretch(1)
        diagram_layout.addWidget(corpo, 2, 1, 1, 1, Qt.AlignCenter)

        direita = QFrame()
        direita.setFixedSize(24, 180)
        diagram_layout.addWidget(direita, 2, 2, 1, 1, Qt.AlignLeft | Qt.AlignVCenter)

        self._sensor_widgets["esquerda"] = esquerda
        self._sensor_widgets["frente"] = frente
        self._sensor_widgets["direita"] = direita
        root.addWidget(diagram_frame)

        status_frame = QFrame()
        status_frame.setStyleSheet(
            "QFrame { background-color: #171717; border: 1px solid #2f2f2f; border-radius: 10px; }"
        )
        status_layout = QGridLayout(status_frame)
        status_layout.setContentsMargins(12, 10, 12, 10)
        status_layout.setHorizontalSpacing(10)
        status_layout.setVerticalSpacing(8)

        labels = {
            "esquerda": "Sensor Esquerda",
            "frente": "Sensor Frente",
            "direita": "Sensor Direita",
        }
        for row, sensor in enumerate(("esquerda", "frente", "direita")):
            lbl_sensor = QLabel(labels[sensor])
            lbl_sensor.setStyleSheet("color: #cfd8dc; font-weight: bold; border: 0px;")
            lbl_state = QLabel("OFF")
            lbl_state.setAlignment(Qt.AlignCenter)
            lbl_state.setFixedWidth(58)
            status_layout.addWidget(lbl_sensor, row, 0)
            status_layout.addWidget(lbl_state, row, 1, 1, 1, Qt.AlignRight)
            self._status_labels[sensor] = lbl_state

        root.addWidget(status_frame)
        root.addStretch(1)

    def _sensor_style(self, ativo):
        cor = "#d32f2f" if ativo else "#5f6368"
        return (
            "QFrame { "
            f"background-color: {cor}; "
            "border: 1px solid #a8a8a8; "
            "border-radius: 5px; "
            "}"
        )

    def _status_style(self, ativo):
        cor_texto = "#ffebee" if ativo else "#eceff1"
        cor_fundo = "#b71c1c" if ativo else "#37474f"
        return (
            "QLabel { "
            f"color: {cor_texto}; "
            f"background-color: {cor_fundo}; "
            "padding: 2px 6px; "
            "font-weight: bold; "
            "border-radius: 6px; "
            "}"
        )

    def _refresh_sensor(self, sensor):
        ativo = self._sensor_states[sensor]
        self._sensor_widgets[sensor].setStyleSheet(self._sensor_style(ativo))
        self._status_labels[sensor].setText("ON" if ativo else "OFF")
        self._status_labels[sensor].setStyleSheet(self._status_style(ativo))

    def set_sensor_state(self, nome_sensor, ativo):
        sensor = str(nome_sensor).strip().lower()
        if sensor not in self._sensor_states:
            raise ValueError(f"Sensor desconhecido: {nome_sensor}")
        self._sensor_states[sensor] = bool(ativo)
        self._refresh_sensor(sensor)

    def set_sensor_states(self, esquerda=None, frente=None, direita=None):
        updates = {
            "esquerda": esquerda,
            "frente": frente,
            "direita": direita,
        }
        for sensor, valor in updates.items():
            if valor is None:
                continue
            self._sensor_states[sensor] = bool(valor)
            self._refresh_sensor(sensor)

    def get_sensor_states(self):
        return dict(self._sensor_states)


class RosNodesCollectorThread(QThread):
    result_ready = pyqtSignal(bool, object, str)

    def __init__(self, workspace, parent=None):
        super().__init__(parent)
        self.workspace = workspace

    def _run_ros2_cli(self, command, timeout=4):
        setup_ws = shlex.quote(os.path.join(self.workspace, "install/setup.bash"))
        cmd = (
            "source /opt/ros/humble/setup.bash && "
            f"source {setup_ws} && "
            f"{command}"
        )
        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except Exception as e:
            return False, "", str(e)

        if result.returncode != 0:
            erro = (result.stderr or result.stdout or "Falha ao executar comando ROS2.").strip()
            return False, result.stdout, erro
        return True, result.stdout, ""

    def _parse_ros2_node_publishers(self, node_info_text):
        publishers = []
        in_publishers = False
        for line in node_info_text.splitlines():
            stripped = line.strip()
            if stripped == "Publishers:":
                in_publishers = True
                continue
            if not in_publishers:
                continue
            if stripped.endswith(":") and not stripped.startswith("/"):
                break
            if not stripped or stripped.lower() == "none":
                continue
            if stripped.startswith("/") and ":" in stripped:
                topic, msg_type = stripped.split(":", 1)
                publishers.append((topic.strip(), msg_type.strip()))
        return publishers

    def run(self):
        ok, out, erro = self._run_ros2_cli("ros2 node list", timeout=4)
        if not ok:
            self.result_ready.emit(False, [], erro)
            return

        nodes = [line.strip() for line in out.splitlines() if line.strip()]
        nodes_publicadores = []
        for node_name in nodes:
            if self.isInterruptionRequested():
                return
            ok_info, out_info, _erro_info = self._run_ros2_cli(
                f"ros2 node info {shlex.quote(node_name)}", timeout=2
            )
            if not ok_info:
                continue
            pubs = self._parse_ros2_node_publishers(out_info)
            if pubs:
                nodes_publicadores.append((node_name, pubs))

        self.result_ready.emit(True, nodes_publicadores, "")


class AgroRobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Caatinga Robotics - Central de Comando")
        self.setGeometry(0, 0, 950, 850)
        self.setStyleSheet("background-color: #212121; color: white;")

        # --- Caminhos ---
        self.home = os.path.expanduser("~")
        self.workspace = os.path.join(self.home, 'agro_robot_ws')
        self.base_path = os.path.join(
            self.workspace,
            'src/agro_robot_sim/sistema_rotas_de_trabalho',
        )

        self.path_rotas = os.path.join(self.base_path, 'rotas_de_trabalho')
        self.script_gravador = os.path.join(self.base_path, 'gravador_rotas.py')
        self.script_leitor = os.path.join(self.base_path, 'leitor_rotas.py')
        self.script_rastreabilidade = os.path.join(self.base_path, 'rastreabilidade_ue.py')
        self.script_salvar_garagem = os.path.join(self.base_path, 'salvar_garagem.py')
        self.script_ir_garagem = os.path.join(self.base_path, 'ir_garagem.py')
        self.script_photo_capture = os.path.join(
            self.workspace, "src", "caatinga_vision", "caatinga_vision", "photo_capture_node.py"
        )
        self.exec_photo_capture = os.path.join(
            self.workspace,
            "install",
            "caatinga_vision",
            "lib",
            "caatinga_vision",
            "photo_capture_node",
        )
        self.garagem_path = os.path.join(self.base_path, 'garagem.csv')
        self.dataset_root = os.path.join(self.workspace, "datasets", "agro_v1")
        self.dataset_data_yaml = os.path.join(self.dataset_root, "data.yaml")

        self.image_path = os.path.join(self.base_path, 'logoca.png')

        self.processos = {}
        self.suporte_ativo = False

        self.robot_lat = -5.842658269190423
        self.robot_lon = -40.70602809847952
        self.default_br_lat = -14.2350
        self.default_br_lon = -51.9253
        self.default_br_zoom = 5
        self.map_path = os.path.join("/tmp", "mapa_robo.html")
        self.sensor_panel = None
        self.sensor_collision_state = {
            "esquerda": False,
            "frente": False,
            "direita": False,
        }
        self.robot_em_movimento = False
        self._cpu_prev_total = None
        self._cpu_prev_idle = None
        self.cpu_temp_max_c = self._detectar_temperatura_maxima_cpu()
        self.cpu_temp_alerta_80_c = self.cpu_temp_max_c * 0.80
        self._temp_alerta_80_emitido = False
        self._temp_alerta_100_emitido = False
        self._ros_refresh_em_andamento = False
        self._ros_nodes_worker = None
        self._ros_nodes_cache = []
        self._closing = False
        self.ia_overlay_cache_path = os.path.join("/tmp", "caatinga_vision_overlay.jpg")
        self.ia_status_cache_path_default = os.path.join("/tmp", "caatinga_vision_status.json")
        self.ia_status_cache_path_active = self.ia_status_cache_path_default
        self.ia_status_cache_path = self.ia_status_cache_path_active
        self.ia_preflight_cache_path = os.path.join("/tmp", "caatinga_ia_preflight_cache.json")
        self.ia_preflight_cache_limit = 40
        self.ia_session_id = ""
        self.ia_rota_ativa = False
        self.ia_status_snapshot = {}
        self._ia_status_cache_mtime = 0.0
        self._ia_sanity_async_state = None
        self._ia_sanity_timer = QTimer(self)
        self._ia_sanity_timer.setInterval(140)
        self._ia_sanity_timer.timeout.connect(self._poll_sanidade_modelo_assincrona)
        self.capture_preview_cache_path = os.path.join("/tmp", "caatinga_capture_latest.jpg")
        self.capture_status_cache_path = os.path.join("/tmp", "caatinga_capture_status.json")
        self.capture_session_id = ""
        self.capture_rota_ativa = False
        self.capture_status_snapshot = {}
        self._capture_status_cache_mtime = 0.0
        self._yolo_train_running = False

        self.setup_ui()
        self.cpu_timer = QTimer(self)
        self.cpu_timer.setInterval(1200)
        self.cpu_timer.timeout.connect(self.atualizar_cpu_monitor)
        self.cpu_timer.start()
        self.atualizar_cpu_monitor()
        self.ros_nodes_timer = QTimer(self)
        self.ros_nodes_timer.setInterval(8000)
        self.ros_nodes_timer.timeout.connect(self.atualizar_nos_ros_publicadores)
        self.ros_nodes_timer.start()
        self.ia_monitor_timer = QTimer(self)
        self.ia_monitor_timer.setInterval(500)
        self.ia_monitor_timer.timeout.connect(self.atualizar_monitor_ia)
        self.ia_monitor_timer.start()
        self.capture_monitor_timer = QTimer(self)
        self.capture_monitor_timer.setInterval(700)
        self.capture_monitor_timer.timeout.connect(self.atualizar_monitor_captura_fotos)
        self.capture_monitor_timer.start()

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # === ESQUERDA (CONTROLES) ===
        left_frame = QFrame()
        left_frame.setStyleSheet("background-color: #333; border-radius: 10px;")
        left_frame.setFixedWidth(350)
        left_layout = QVBoxLayout(left_frame)

        # --- TOPO ESQUERDO: SUPORTE + MANUTENÇÃO ---
        top_left_controls = QVBoxLayout()
        top_left_support = QHBoxLayout()

        self.btn_suporte = QPushButton("🛟 Suporte Remoto: OFF")
        self.btn_suporte.setStyleSheet(
            "QPushButton { background-color: #1e3a5f; color: white; font-weight: bold; "
            "padding: 12px; border-radius: 8px; border: 2px solid #64b5f6; font-size: 13px; }"
            "QPushButton:hover { background-color: #24456f; border: 2px solid #90caf9; }"
        )
        self.btn_suporte.clicked.connect(self.toggle_suporte)
        top_left_support.addWidget(self.btn_suporte)
        top_left_controls.addLayout(top_left_support)

        top_left_maintenance = QHBoxLayout()

        self.btn_rebuild = QPushButton("🧹 Limpar e Recompilar")
        self.estilizar_botao(self.btn_rebuild, "#555")
        self.btn_rebuild.clicked.connect(self.limpar_e_reconstruir)
        top_left_maintenance.addWidget(self.btn_rebuild)

        self.btn_update_sw = QPushButton("⬆️ Atualizar Software")
        self.estilizar_botao(self.btn_update_sw, "#1565c0")
        self.btn_update_sw.clicked.connect(self.atualizar_software)
        top_left_maintenance.addWidget(self.btn_update_sw)
        top_left_controls.addLayout(top_left_maintenance)

        left_layout.addLayout(top_left_controls)

        # --- LOGO ---
        lbl_logo = QLabel()
        lbl_logo.setAlignment(Qt.AlignCenter)
        lbl_logo.setStyleSheet("margin-bottom: 10px; margin-top: 10px;")
        pixmap = QPixmap(self.image_path)
        if not pixmap.isNull():
            scaled_pixmap = pixmap.scaledToWidth(140, Qt.SmoothTransformation)
            lbl_logo.setPixmap(scaled_pixmap)
        else:
            lbl_logo.setText("CAATINGA\nROBOTICS")
            lbl_logo.setStyleSheet("color: #4caf50; font-weight: bold; font-size: 26px;")
        left_layout.addWidget(lbl_logo)

        # --- AÇÕES PRINCIPAIS (JUNTAS) ---
        self.btn_sistema = QPushButton("🚀 Ligar Sistema (Launch)")
        self.btn_sistema.setStyleSheet(
            "QPushButton { background-color: #e65100; color: white; font-weight: bold; "
            "padding: 14px; border-radius: 8px; font-size: 14px; border: 2px solid #ffcc80; }"
            "QPushButton:hover { background-color: #ef6c00; border: 2px solid #ffe0b2; }"
        )
        self.btn_sistema.clicked.connect(self.toggle_sistema)
        left_layout.addWidget(self.btn_sistema)

        self.btn_gravar = QPushButton("🔴 Gravar Rota")
        self.btn_gravar.setStyleSheet(
            "QPushButton { background-color: #c62828; color: white; font-weight: bold; "
            "padding: 14px; border-radius: 8px; font-size: 14px; border: 2px solid #ef9a9a; }"
            "QPushButton:hover { background-color: #d32f2f; border: 2px solid #ffcdd2; }"
        )
        self.btn_gravar.clicked.connect(self.iniciar_gravador)
        left_layout.addWidget(self.btn_gravar)

        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setStyleSheet("background-color: #555;")
        left_layout.addWidget(line1)

        left_layout.addSpacing(10)

        # --- GARAGEM (MOVIDO PARA O TOPO DIREITO) ---
        self.btn_def_garagem = QPushButton("🏠 Definir Garagem (Posição Atual)")
        self.estilizar_botao(self.btn_def_garagem, "#455a64")
        self.btn_def_garagem.clicked.connect(self.definir_garagem)

        self.btn_ir_garagem = QPushButton("📍 Chamar para Garagem")
        self.estilizar_botao(self.btn_ir_garagem, "#1b5e20")
        self.btn_ir_garagem.clicked.connect(self.chamar_garagem)

        # --- SELEÇÃO DE IMPLEMENTO (MOVIDO PARA BAIXO DE GRAVAR ROTA) ---
        lbl_impl = QLabel("🛠️ Implemento Acoplado:")
        lbl_impl.setStyleSheet("font-weight: bold; margin-top: 10px;")
        left_layout.addWidget(lbl_impl)

        self.combo_implemento = QComboBox()
        self.combo_implemento.setStyleSheet(
            "QComboBox { background-color: #444; color: white; padding: 8px; "
            "border: 1px solid #555; font-weight: bold; }"
            "QComboBox QAbstractItemView { background-color: #333; color: white; }"
        )
        # Define as opções
        self.combo_implemento.addItems([
            "✂️ Roçadeira (Corte de Mato)",
            "💦 Pulverizador (Aplicação)",
            "🧠 Pulverizador Inteligente (IA)",
            "📸 Coletor de Imagens (Treino)",
            "⚪ Nenhum Implemento"
        ])
        # Conecta a mudança de seleção para abrir a aba de configuração
        self.combo_implemento.currentIndexChanged.connect(self.on_implemento_changed)
        left_layout.addWidget(self.combo_implemento)
        # ----------------------------------------

        self.tabs_left = QTabWidget()
        self.tabs_left.setStyleSheet(
            "QTabWidget::pane { border: 1px solid #444; border-radius: 6px; "
            "background: #2a2a2a; }"
            "QTabBar::tab { background: #323232; color: #cfd8dc; padding: 8px 12px; "
            "font-weight: bold; }"
            "QTabBar::tab:selected { background: #455a64; color: #ffffff; }"
        )
        left_layout.addWidget(self.tabs_left, 1)

        tab_rotas = QWidget()
        tab_rotas_layout = QVBoxLayout(tab_rotas)
        tab_rotas_layout.setContentsMargins(8, 8, 8, 8)

        lbl_r = QLabel("Rotas Disponíveis:")
        lbl_r.setFont(QFont("Arial", 11, QFont.Bold))
        lbl_r.setStyleSheet("margin-top: 5px;")
        tab_rotas_layout.addWidget(lbl_r)

        self.lista_rotas = QListWidget()
        self.lista_rotas.setStyleSheet(
            "background-color: #444; color: white; border: 1px solid #555;"
        )
        self.atualizar_lista_rotas()
        tab_rotas_layout.addWidget(self.lista_rotas)

        hbox = QHBoxLayout()
        btn_ref = QPushButton("🔄")
        btn_ref.setFixedWidth(40)
        btn_ref.setStyleSheet("background-color: #555; color: white;")
        btn_ref.clicked.connect(self.atualizar_lista_rotas)
        hbox.addWidget(btn_ref)

        self.btn_exec = QPushButton("▶️ Executar")
        self.estilizar_botao(self.btn_exec, "#2e7d32")
        self.btn_exec.clicked.connect(self.executar_rota)
        hbox.addWidget(self.btn_exec)
        tab_rotas_layout.addLayout(hbox)
        self.tabs_left.addTab(tab_rotas, "📋 Rotas")

        # ============================================================
        # === ABA DE CONFIGURAÇÃO DO IMPLEMENTO ===
        # ============================================================
        self.tab_config_implemento = QWidget()
        tab_impl_layout = QVBoxLayout(self.tab_config_implemento)
        tab_impl_layout.setContentsMargins(8, 8, 8, 8)

        lbl_cfg_impl = QLabel("Configurações personalizadas do implemento:")
        lbl_cfg_impl.setStyleSheet("font-weight: bold; color: #cfd8dc; border: 0px;")
        tab_impl_layout.addWidget(lbl_cfg_impl)

        self.lbl_impl_sem_config = QLabel("Este implemento não exige configuração adicional.")
        self.lbl_impl_sem_config.setStyleSheet("color: #b0bec5; border: 0px; padding: 6px;")
        self.lbl_impl_sem_config.setWordWrap(True)
        tab_impl_layout.addWidget(self.lbl_impl_sem_config)

        self.frame_tanque = QFrame()
        self.frame_tanque.setStyleSheet((
            "background-color: #2a2a2a; border: 1px solid #444;"
            "border-radius: 5px; margin-top: 6px;"
        ))
        layout_tanque = QVBoxLayout(self.frame_tanque)

        lbl_tank = QLabel("🛢️ Configuração da Calda (UE):")
        lbl_tank.setStyleSheet("font-weight: bold; color: #4caf50; border: 0px;")
        layout_tanque.addWidget(lbl_tank)

        self.lbl_prod_quimico = QLabel("Produto químico utilizado:")
        self.lbl_prod_quimico.setStyleSheet("font-size: 11px; color: #aaa; border: 0px;")
        layout_tanque.addWidget(self.lbl_prod_quimico)

        self.input_produto_quimico = QLineEdit()
        self.input_produto_quimico.setPlaceholderText("Ex.: Abamectina 18 EC")
        self.input_produto_quimico.setStyleSheet(
            "background-color: #444; padding: 5px; color: #ffffff; border: 0px;"
        )
        layout_tanque.addWidget(self.input_produto_quimico)

        self.lbl_reg_mapa = QLabel("Nº Registro no MAPA:")
        self.lbl_reg_mapa.setStyleSheet("font-size: 11px; color: #aaa; border: 0px;")
        layout_tanque.addWidget(self.lbl_reg_mapa)

        self.input_registro_mapa = QLineEdit()
        self.input_registro_mapa.setPlaceholderText("Ex.: 12345")
        self.input_registro_mapa.setStyleSheet(
            "background-color: #444; padding: 5px; color: #ffffff; border: 0px;"
        )
        layout_tanque.addWidget(self.input_registro_mapa)

        # 2. Layout Horizontal para Água e Produto
        h_mix = QHBoxLayout()

        # Campo Água
        v_agua = QVBoxLayout()
        lbl_agua = QLabel("Água (L):")
        lbl_agua.setStyleSheet("font-size: 11px; color: #aaa; border: 0px;")
        self.spin_agua = QDoubleSpinBox()
        self.spin_agua.setRange(0.0, 100.0)
        self.spin_agua.setValue(50.0)
        self.spin_agua.setSingleStep(1.0)
        self.spin_agua.setStyleSheet((
            "background-color: #444; padding: 3px; color: #00ffff;"
            "font-weight: bold; border: 0px;"
        ))
        v_agua.addWidget(lbl_agua)
        v_agua.addWidget(self.spin_agua)

        # Campo Dose
        v_dose = QVBoxLayout()
        lbl_dose = QLabel("Produto (ml):")
        lbl_dose.setStyleSheet("font-size: 11px; color: #aaa; border: 0px;")
        self.spin_dose = QSpinBox()
        self.spin_dose.setRange(0, 5000)
        self.spin_dose.setValue(0)
        self.spin_dose.setSingleStep(10)
        self.spin_dose.setStyleSheet((
            "background-color: #444; padding: 3px; color: #ffeb3b;"
            "font-weight: bold; border: 0px;"
        ))
        v_dose.addWidget(lbl_dose)
        v_dose.addWidget(self.spin_dose)

        h_mix.addLayout(v_agua)
        h_mix.addLayout(v_dose)
        layout_tanque.addLayout(h_mix)

        tab_impl_layout.addWidget(self.frame_tanque)
        tab_impl_layout.addStretch(1)
        self.tabs_left.addTab(self.tab_config_implemento, "🛠️ Config. Implemento")
        main_layout.addWidget(left_frame)

        # === DIREITA (LOGS) ===
        right_layout = QVBoxLayout()

        # --- BARRA SUPERIOR (MENU DE CELULAR) ---
        bateria_frame = QFrame()
        bateria_frame.setFixedHeight(32)
        bateria_frame.setStyleSheet(
            "background-color: #1b1b1b; border: 1px solid #2b2b2b; border-radius: 6px;"
        )
        bateria_layout = QHBoxLayout(bateria_frame)
        bateria_layout.setContentsMargins(10, 4, 10, 4)
        bateria_layout.setSpacing(8)

        self.btn_wifi = QPushButton("📶 Wi-Fi: Desconectado")
        self.btn_wifi.setCursor(Qt.PointingHandCursor)
        self.btn_wifi.setStyleSheet(
            "QPushButton { background-color: #2b2b2b; color: #e0e0e0; padding: 2px 8px; "
            "border-radius: 8px; border: 1px solid #3c3c3c; font-weight: bold; font-size: 11px; }"
            "QPushButton:hover { background-color: #353535; border: 1px solid #4a4a4a; }"
        )
        self.btn_wifi.clicked.connect(self.abrir_wifi_manager)
        bateria_layout.addWidget(self.btn_wifi)

        self.btn_rastreabilidade = QPushButton("🧾 Rastreabilidade")
        self.btn_rastreabilidade.setCursor(Qt.PointingHandCursor)
        self.btn_rastreabilidade.setStyleSheet(
            "QPushButton { background-color: #2e3f50; color: #e0e0e0; padding: 2px 8px; "
            "border-radius: 8px; border: 1px solid #415a70; font-weight: bold; font-size: 11px; }"
            "QPushButton:hover { background-color: #3a5368; border: 1px solid #4f708a; }"
        )
        self.btn_rastreabilidade.clicked.connect(self.abrir_aba_rastreabilidade)
        bateria_layout.addWidget(self.btn_rastreabilidade)

        bateria_layout.addStretch(1)

        lbl_battery_icon = QLabel("🔋")
        lbl_battery_icon.setStyleSheet("color: #cfd8dc;")
        bateria_layout.addWidget(lbl_battery_icon)

        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setTextVisible(False)
        self.battery_bar.setFixedHeight(8)
        self.battery_bar.setFixedWidth(80)
        self.battery_bar.setStyleSheet(
            "QProgressBar { background: #0f0f0f; border: 1px solid #333; border-radius: 4px; }"
            "QProgressBar::chunk { background: #4caf50; border-radius: 4px; }"
        )
        bateria_layout.addWidget(self.battery_bar)

        self.lbl_battery_percent = QLabel("80%")
        self.lbl_battery_percent.setStyleSheet(
            "color: #4caf50; font-weight: bold; min-width: 32px;"
        )
        self.lbl_battery_percent.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        bateria_layout.addWidget(self.lbl_battery_percent)

        self.lbl_charging = QLabel("⚡ Não carregando")
        self.lbl_charging.setAlignment(Qt.AlignCenter)
        self.lbl_charging.setMinimumWidth(120)
        self.lbl_charging.setStyleSheet(
            "background-color: #455a64; color: #ffffff; padding: 3px 10px; border-radius: 8px; "
            "border: 1px solid #546e7a; font-weight: bold; font-size: 12px;"
        )
        bateria_layout.addWidget(self.lbl_charging)

        # --- TOPO DIREITO: GARAGEM ---
        top_right_controls = QHBoxLayout()
        top_right_controls.addWidget(self.btn_def_garagem)
        top_right_controls.addWidget(self.btn_ir_garagem)
        right_layout.addLayout(top_right_controls)

        h_waypoints = QHBoxLayout()
        self.frame_status_luz = QFrame()
        self.frame_status_luz.setStyleSheet(
            "QFrame { background-color: #121820; border: 2px solid #2f3d4f; border-radius: 12px; }"
        )
        status_luz_layout = QHBoxLayout(self.frame_status_luz)
        status_luz_layout.setContentsMargins(8, 4, 8, 4)
        status_luz_layout.setSpacing(6)

        self.luz_status = QLabel()
        self.luz_status.setFixedSize(22, 22)
        self.luz_status.setAlignment(Qt.AlignCenter)
        status_luz_layout.addWidget(self.luz_status)

        self.lbl_luz_status = QLabel("PARADO")
        self.lbl_luz_status.setStyleSheet("color: #eeeeee; font-weight: bold; font-size: 11px;")
        status_luz_layout.addWidget(self.lbl_luz_status)
        h_waypoints.addWidget(self.frame_status_luz)

        self.frame_cpu = QFrame()
        self.frame_cpu.setStyleSheet(
            "QFrame { background-color: #121820; border: 1px solid #2f3d4f; border-radius: 12px; }"
        )
        cpu_layout = QVBoxLayout(self.frame_cpu)
        cpu_layout.setContentsMargins(8, 4, 8, 4)
        cpu_layout.setSpacing(3)

        cpu_row = QHBoxLayout()
        cpu_row.setSpacing(6)

        self.lbl_cpu = QLabel("CPU: --%")
        self.lbl_cpu.setStyleSheet("color: #cfd8dc; font-weight: bold; font-size: 11px;")
        cpu_row.addWidget(self.lbl_cpu)

        self.cpu_bar = QProgressBar()
        self.cpu_bar.setRange(0, 100)
        self.cpu_bar.setTextVisible(False)
        self.cpu_bar.setFixedHeight(10)
        self.cpu_bar.setFixedWidth(86)
        self.cpu_bar.setStyleSheet(
            "QProgressBar { background: #0f141a; border: 1px solid #32404f; border-radius: 5px; }"
            "QProgressBar::chunk { background: #43a047; border-radius: 5px; }"
        )
        cpu_row.addWidget(self.cpu_bar)
        cpu_layout.addLayout(cpu_row)

        self.lbl_cpu_temp = QLabel(f"Temp CPU: --.-\u00b0C / max {self.cpu_temp_max_c:.1f}\u00b0C")
        self.lbl_cpu_temp.setStyleSheet("color: #90a4ae; font-size: 10px; font-weight: bold;")
        cpu_layout.addWidget(self.lbl_cpu_temp)
        h_waypoints.addWidget(self.frame_cpu)

        self.lbl_waypoints = QLabel("Waypoints: --")
        self.lbl_waypoints.setStyleSheet("color: #ffeb3b; font-weight: bold;")
        h_waypoints.addSpacing(10)
        h_waypoints.addWidget(self.lbl_waypoints)
        h_waypoints.addStretch(1)

        self.btn_proximo_wp = QPushButton("Próximo")
        self.estilizar_botao_destaque(self.btn_proximo_wp, "#546e7a")
        self.btn_proximo_wp.setFixedHeight(32)
        self.btn_proximo_wp.setMinimumWidth(120)
        self.btn_proximo_wp.clicked.connect(self.pular_waypoint)
        self.btn_proximo_wp.hide()
        h_waypoints.addWidget(self.btn_proximo_wp)

        self.btn_concluir_rota = QPushButton("Concluir")
        self.estilizar_botao_destaque(self.btn_concluir_rota, "#00695c")
        self.btn_concluir_rota.setFixedHeight(32)
        self.btn_concluir_rota.setMinimumWidth(120)
        self.btn_concluir_rota.clicked.connect(self.concluir_rota)
        self.btn_concluir_rota.hide()
        h_waypoints.addWidget(self.btn_concluir_rota)
        right_layout.addLayout(h_waypoints)

        lbl_log = QLabel("Monitor do Sistema")
        lbl_log.setFont(QFont("Arial", 12, QFont.Bold))
        right_layout.addWidget(lbl_log)

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #444; }
            QTabBar::tab { background: #2b2b2b; color: white; padding: 8px; }
            QTabBar::tab:selected { background: #444; }
        """)

        logs_tab = QWidget()
        logs_layout = QVBoxLayout(logs_tab)

        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setStyleSheet("""
            QTextEdit {
                background-color: #111; color: #00ff00;
                font-family: Monospace; font-size: 12px;
                border: 1px solid #444; border-radius: 5px;
            }
        """)
        logs_layout.addWidget(self.log_console)

        self.progress_bar = QProgressBar()
        self.progress_bar.setStyleSheet((
            "QProgressBar { border: 0px; background: #444; height: 10px; }"
            "QProgressBar::chunk { background: #4caf50; }"
        ))
        self.progress_bar.hide()
        logs_layout.addWidget(self.progress_bar)

        map_tab = QWidget()
        map_layout = QVBoxLayout(map_tab)
        self.lbl_map_status = QLabel("Mapa do robô (Folium)")
        self.lbl_map_status.setStyleSheet("color: #9ccc65; font-weight: bold; font-size: 11px;")
        self.lbl_map_status.setFixedHeight(18)
        self.lbl_map_status.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        map_layout.addWidget(self.lbl_map_status)

        if WEBENGINE_DISPONIVEL:
            self.map_view = QWebEngineView()
            self.map_view.setStyleSheet("background: #111; border: 1px solid #444;")
            map_layout.addWidget(self.map_view)
        else:
            self.map_view = None
            lbl_missing = QLabel(
                "PyQtWebEngine não carregou. Veja detalhes abaixo."
            )
            lbl_missing.setStyleSheet("color: #ffb300;")
            lbl_missing.setWordWrap(True)
            map_layout.addWidget(lbl_missing)

            lbl_diag = QLabel(
                f"Python: {sys.executable}\nErro: {WEBENGINE_ERROR}"
            )
            lbl_diag.setStyleSheet("color: #9e9e9e;")
            lbl_diag.setWordWrap(True)
            map_layout.addWidget(lbl_diag)

        self.tab_sensores = QWidget()
        sensores_layout = QVBoxLayout(self.tab_sensores)
        sensores_layout.setContentsMargins(12, 12, 12, 12)
        self.sensor_panel = SensorTopViewPanel()
        sensores_layout.addWidget(self.sensor_panel)
        self.sensor_panel.set_sensor_states(
            esquerda=self.sensor_collision_state["esquerda"],
            frente=self.sensor_collision_state["frente"],
            direita=self.sensor_collision_state["direita"],
        )

        self.tab_ros_nodes = QWidget()
        ros_nodes_layout = QVBoxLayout(self.tab_ros_nodes)
        ros_nodes_layout.setContentsMargins(12, 12, 12, 12)
        ros_nodes_layout.setSpacing(8)

        ros_nodes_top = QHBoxLayout()
        self.lbl_ros_nodes_status = QLabel("Monitorando nós ROS2 publicadores...")
        self.lbl_ros_nodes_status.setStyleSheet("color: #b0bec5; font-size: 11px;")
        ros_nodes_top.addWidget(self.lbl_ros_nodes_status)
        ros_nodes_top.addStretch(1)

        self.btn_ros_nodes_refresh = QPushButton("Atualizar")
        self.btn_ros_nodes_refresh.setFixedHeight(30)
        self.btn_ros_nodes_refresh.setMinimumWidth(110)
        self.btn_ros_nodes_refresh.setCursor(Qt.PointingHandCursor)
        self.btn_ros_nodes_refresh.setStyleSheet(
            "QPushButton { "
            "background-color: #455a64; color: #ffffff; "
            "padding: 4px 10px; border-radius: 6px; "
            "border: 1px solid #607d8b; font-weight: bold; font-size: 11px; "
            "}"
            "QPushButton:hover { background-color: #546e7a; border: 1px solid #90a4ae; }"
            "QPushButton:pressed { background-color: #37474f; }"
            "QPushButton:disabled { background-color: #2e3a44; color: "
            "#9e9e9e; border: 1px solid #455a64; }"
        )
        self.btn_ros_nodes_refresh.clicked.connect(
            lambda *_: self.atualizar_nos_ros_publicadores(forcar=True)
        )
        ros_nodes_top.addWidget(self.btn_ros_nodes_refresh)

        self.btn_ros_nodes_export = QPushButton("Exportar")
        self.btn_ros_nodes_export.setFixedHeight(30)
        self.btn_ros_nodes_export.setMinimumWidth(110)
        self.btn_ros_nodes_export.setCursor(Qt.PointingHandCursor)
        self.btn_ros_nodes_export.setEnabled(False)
        self.btn_ros_nodes_export.setStyleSheet(
            "QPushButton { "
            "background-color: #2e7d32; color: #ffffff; "
            "padding: 4px 10px; border-radius: 6px; "
            "border: 1px solid #4caf50; font-weight: bold; font-size: 11px; "
            "}"
            "QPushButton:hover { background-color: #388e3c; border: 1px solid #66bb6a; }"
            "QPushButton:pressed { background-color: #1b5e20; }"
            "QPushButton:disabled { background-color: #2d3a2f; color: "
            "#9e9e9e; border: 1px solid #455a64; }"
        )
        self.btn_ros_nodes_export.clicked.connect(self.exportar_nos_ros_publicadores)
        ros_nodes_top.addWidget(self.btn_ros_nodes_export)
        ros_nodes_layout.addLayout(ros_nodes_top)

        self.tree_ros_nodes = QTreeWidget()
        self.tree_ros_nodes.setColumnCount(3)
        self.tree_ros_nodes.setHeaderLabels(["Nó ROS2", "Publishers", "Tópicos Publicados"])
        self.tree_ros_nodes.setAlternatingRowColors(True)
        self.tree_ros_nodes.setRootIsDecorated(False)
        self.tree_ros_nodes.setUniformRowHeights(True)
        self.tree_ros_nodes.setWordWrap(False)
        self.tree_ros_nodes.setStyleSheet(
            "QTreeWidget { background-color: #151515; border: 1px solid #333; color: #eceff1; }"
            "QHeaderView::section { background-color: #263238; color: "
            "#ffffff; padding: 4px; border: 0px; font-weight: bold; }"
            "QTreeWidget::item { height: 26px; }"
            "QTreeWidget::item:selected { background-color: #37474f; }"
        )
        self.tree_ros_nodes.setColumnWidth(0, 260)
        self.tree_ros_nodes.setColumnWidth(1, 95)
        self.tree_ros_nodes.setColumnWidth(2, 520)
        ros_nodes_layout.addWidget(self.tree_ros_nodes)

        self.tab_saude_ia = QWidget()
        saude_ia_layout = QVBoxLayout(self.tab_saude_ia)
        saude_ia_layout.setContentsMargins(12, 12, 12, 12)
        saude_ia_layout.setSpacing(8)

        lbl_ia_title = QLabel("Saúde IA - Monitoramento de Pragas")
        lbl_ia_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #a5d6a7;")
        saude_ia_layout.addWidget(lbl_ia_title)

        cfg_ia_layout = QHBoxLayout()
        cfg_ia_layout.setSpacing(8)

        default_ia_model_path, _default_model_source = self._resolver_modelo_ia_padrao()

        self.input_ia_model_path = QLineEdit(default_ia_model_path)
        self.input_ia_model_path.setPlaceholderText("Caminho do modelo YOLO (.pt)")
        self.input_ia_model_path.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        cfg_ia_layout.addWidget(self.input_ia_model_path, 2)

        self.spin_ia_camera_index = QSpinBox()
        self.spin_ia_camera_index.setRange(0, 10)
        self.spin_ia_camera_index.setValue(0)
        self.spin_ia_camera_index.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        cfg_ia_layout.addWidget(self.spin_ia_camera_index, 0)

        self.combo_ia_conf_mode = QComboBox()
        self.combo_ia_conf_mode.addItems(["Econômico (85%)", "Conservador (50%)"])
        self.combo_ia_conf_mode.setStyleSheet(
                "QComboBox { background-color: #444; color: white; padding:"
                "6px; border: 1px solid #555; font-weight: bold; }"
                "QComboBox QAbstractItemView { background-color: #333; color: white; }"
        )
        cfg_ia_layout.addWidget(self.combo_ia_conf_mode, 1)
        saude_ia_layout.addLayout(cfg_ia_layout)

        cfg_ia_thr_layout = QHBoxLayout()
        cfg_ia_thr_layout.setSpacing(8)

        lbl_thr_cons = QLabel("Limiar Conservador")
        lbl_thr_cons.setStyleSheet("color: #b0bec5; font-size: 11px;")
        cfg_ia_thr_layout.addWidget(lbl_thr_cons)
        self.spin_ia_thr_cons = QDoubleSpinBox()
        self.spin_ia_thr_cons.setRange(0.0, 1.0)
        self.spin_ia_thr_cons.setSingleStep(0.01)
        self.spin_ia_thr_cons.setDecimals(3)
        self.spin_ia_thr_cons.setValue(0.50)
        self.spin_ia_thr_cons.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        cfg_ia_thr_layout.addWidget(self.spin_ia_thr_cons)

        lbl_thr_econ = QLabel("Limiar Econômico")
        lbl_thr_econ.setStyleSheet("color: #b0bec5; font-size: 11px;")
        cfg_ia_thr_layout.addWidget(lbl_thr_econ)
        self.spin_ia_thr_econ = QDoubleSpinBox()
        self.spin_ia_thr_econ.setRange(0.0, 1.0)
        self.spin_ia_thr_econ.setSingleStep(0.01)
        self.spin_ia_thr_econ.setDecimals(3)
        self.spin_ia_thr_econ.setValue(0.85)
        self.spin_ia_thr_econ.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        cfg_ia_thr_layout.addWidget(self.spin_ia_thr_econ)
        self.btn_ia_preset_teste = QPushButton("Preset Teste Sensível")
        self.btn_ia_preset_teste.setStyleSheet(
                "QPushButton { background-color: #455a64; color: #eceff1;"
                "padding: 6px 10px; border-radius: 6px; border: 1px solid"
                "#607d8b; }"
                "QPushButton:hover { background-color: #546e7a; }"
        )
        self.btn_ia_preset_teste.clicked.connect(self._aplicar_preset_teste_sensivel)
        cfg_ia_thr_layout.addWidget(self.btn_ia_preset_teste)
        cfg_ia_thr_layout.addStretch(1)
        saude_ia_layout.addLayout(cfg_ia_thr_layout)

        self.lbl_ia_feed = QLabel("Feed da IA indisponível.\nInicie uma rota com implemento IA.")
        self.lbl_ia_feed.setAlignment(Qt.AlignCenter)
        self.lbl_ia_feed.setMinimumHeight(280)
        self.lbl_ia_feed.setStyleSheet(
            "background-color: #111; border: 1px solid #333; border-radius: 6px; color: #90a4ae;"
        )
        saude_ia_layout.addWidget(self.lbl_ia_feed, 1)

        stats_frame = QFrame()
        stats_frame.setStyleSheet(
            "QFrame { background-color: #171717; border: 1px solid #2f2f2f; border-radius: 8px; }"
        )
        stats_layout = QGridLayout(stats_frame)
        stats_layout.setContentsMargins(10, 8, 10, 8)
        stats_layout.setHorizontalSpacing(10)
        stats_layout.setVerticalSpacing(6)

        stats_layout.addWidget(QLabel("Pragas Detectadas"), 0, 0)
        self.lbl_ia_pragas = QLabel("0")
        self.lbl_ia_pragas.setStyleSheet("color: #ffeb3b; font-weight: bold;")
        stats_layout.addWidget(self.lbl_ia_pragas, 0, 1)

        stats_layout.addWidget(QLabel("Modo de Confiança"), 1, 0)
        self.lbl_ia_modo = QLabel("Econômico")
        self.lbl_ia_modo.setStyleSheet("color: #80cbc4; font-weight: bold;")
        stats_layout.addWidget(self.lbl_ia_modo, 1, 1)

        stats_layout.addWidget(QLabel("Doença Dominante"), 2, 0)
        self.lbl_ia_doenca = QLabel("Não Detectada")
        self.lbl_ia_doenca.setStyleSheet("color: #ef9a9a; font-weight: bold;")
        stats_layout.addWidget(self.lbl_ia_doenca, 2, 1)

        stats_layout.addWidget(QLabel("Nível de Infestação (m²)"), 3, 0)
        self.lbl_ia_nivel = QLabel("0.0000")
        self.lbl_ia_nivel.setStyleSheet("color: #b3e5fc; font-weight: bold;")
        stats_layout.addWidget(self.lbl_ia_nivel, 3, 1)

        stats_layout.addWidget(QLabel("Litros Economizados"), 4, 0)
        self.lbl_ia_litros = QLabel("0.000 L")
        self.lbl_ia_litros.setStyleSheet("color: #c5e1a5; font-weight: bold;")
        stats_layout.addWidget(self.lbl_ia_litros, 4, 1)

        stats_layout.addWidget(QLabel("Recomendação de Pulverização"), 5, 0)
        self.lbl_ia_recomendacao = QLabel("Não")
        self.lbl_ia_recomendacao.setStyleSheet("color: #ffcc80; font-weight: bold;")
        stats_layout.addWidget(self.lbl_ia_recomendacao, 5, 1)

        stats_layout.addWidget(QLabel("Contagem por Classe"), 6, 0)
        self.lbl_ia_class_counts = QLabel("--")
        self.lbl_ia_class_counts.setStyleSheet("color: #dce775; font-weight: bold;")
        self.lbl_ia_class_counts.setWordWrap(True)
        stats_layout.addWidget(self.lbl_ia_class_counts, 6, 1)

        stats_layout.addWidget(QLabel("Avisos de Mapeamento"), 7, 0)
        self.lbl_ia_unmapped = QLabel("Mapeamento OK")
        self.lbl_ia_unmapped.setStyleSheet("color: #9ccc65; font-weight: bold;")
        self.lbl_ia_unmapped.setWordWrap(True)
        stats_layout.addWidget(self.lbl_ia_unmapped, 7, 1)

        saude_ia_layout.addWidget(stats_frame)

        self.tab_treino_ia = QWidget()
        treino_ia_layout = QVBoxLayout(self.tab_treino_ia)
        treino_ia_layout.setContentsMargins(12, 12, 12, 12)
        treino_ia_layout.setSpacing(8)

        lbl_treino_title = QLabel("Treino IA - Conjunto de Dados YOLO")
        lbl_treino_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #81d4fa;")
        treino_ia_layout.addWidget(lbl_treino_title)

        self.input_treino_session_id = QLineEdit(self._gerar_session_id())
        self.input_treino_session_id.setPlaceholderText("session_id (ex.: sess_20260208_120000)")
        self.input_treino_session_id.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        treino_ia_layout.addWidget(self.input_treino_session_id)

        self.input_treino_dataset_root = QLineEdit(self.dataset_root)
        self.input_treino_dataset_root.setReadOnly(True)
        self.input_treino_dataset_root.setStyleSheet(
            "background-color: #303030; padding: 6px; color: #b0bec5; border: 1px solid #555;"
        )
        treino_ia_layout.addWidget(self.input_treino_dataset_root)

        self.input_treino_data_yaml = QLineEdit(self.dataset_data_yaml)
        self.input_treino_data_yaml.setReadOnly(True)
        self.input_treino_data_yaml.setStyleSheet(
            "background-color: #303030; padding: 6px; color: #b0bec5; border: 1px solid #555;"
        )
        treino_ia_layout.addWidget(self.input_treino_data_yaml)

        self.input_treino_model_path = QLineEdit("yolo11n.pt")
        self.input_treino_model_path.setPlaceholderText((
            "Modelo YOLO para treino (ex.: yolo11n.pt ou"
            "/caminho/modelo.pt)"
        ))
        self.input_treino_model_path.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        treino_ia_layout.addWidget(self.input_treino_model_path)

        treino_cfg_grid = QGridLayout()
        treino_cfg_grid.setHorizontalSpacing(8)
        treino_cfg_grid.setVerticalSpacing(6)
        treino_cfg_grid.addWidget(QLabel("Épocas"), 0, 0)
        self.spin_treino_epochs = QSpinBox()
        self.spin_treino_epochs.setRange(1, 10000)
        self.spin_treino_epochs.setValue(100)
        self.spin_treino_epochs.setStyleSheet(
            "background-color: #444; padding: 4px; color: #ffffff; border: 1px solid #555;"
        )
        treino_cfg_grid.addWidget(self.spin_treino_epochs, 0, 1)

        treino_cfg_grid.addWidget(QLabel("Lote"), 0, 2)
        self.spin_treino_batch = QSpinBox()
        self.spin_treino_batch.setRange(1, 1024)
        self.spin_treino_batch.setValue(8)
        self.spin_treino_batch.setStyleSheet(
            "background-color: #444; padding: 4px; color: #ffffff; border: 1px solid #555;"
        )
        treino_cfg_grid.addWidget(self.spin_treino_batch, 0, 3)

        treino_cfg_grid.addWidget(QLabel("Tam. Imagem"), 1, 0)
        self.spin_treino_imgsz = QSpinBox()
        self.spin_treino_imgsz.setRange(64, 2048)
        self.spin_treino_imgsz.setSingleStep(32)
        self.spin_treino_imgsz.setValue(640)
        self.spin_treino_imgsz.setStyleSheet(
            "background-color: #444; padding: 4px; color: #ffffff; border: 1px solid #555;"
        )
        treino_cfg_grid.addWidget(self.spin_treino_imgsz, 1, 1)

        treino_cfg_grid.addWidget(QLabel("Dispositivo"), 1, 2)
        self.combo_treino_device = QComboBox()
        self.combo_treino_device.addItems(["cpu", "0"])
        self.combo_treino_device.setStyleSheet(
                "QComboBox { background-color: #444; color: white; padding:"
                "4px; border: 1px solid #555; font-weight: bold; }"
                "QComboBox QAbstractItemView { background-color: #333; color: white; }"
        )
        treino_cfg_grid.addWidget(self.combo_treino_device, 1, 3)
        treino_ia_layout.addLayout(treino_cfg_grid)

        treino_import_row = QHBoxLayout()
        self.btn_treino_import_zip = QPushButton("Importar ZIP (Roboflow COCO)")
        self.estilizar_botao(self.btn_treino_import_zip, "#2e7d32")
        self.btn_treino_import_zip.clicked.connect(self.importar_zip_roboflow_coco_treino_ia)
        treino_import_row.addWidget(self.btn_treino_import_zip)

        self.btn_treino_import_pasta = QPushButton("Importar Pasta (imagens + labels YOLO)")
        self.estilizar_botao(self.btn_treino_import_pasta, "#00897b")
        self.btn_treino_import_pasta.clicked.connect(self.importar_pasta_yolo_treino_ia)
        treino_import_row.addWidget(self.btn_treino_import_pasta)
        treino_ia_layout.addLayout(treino_import_row)

        lbl_manual_hint = QLabel("Fluxo manual/avançado (opcional):")
        lbl_manual_hint.setStyleSheet("color: #b0bec5; font-size: 11px;")
        treino_ia_layout.addWidget(lbl_manual_hint)

        treino_upload_row = QHBoxLayout()
        self.btn_treino_upload_imgs = QPushButton("Subir Imagens (Manual)")
        self.estilizar_botao(self.btn_treino_upload_imgs, "#2e7d32")
        self.btn_treino_upload_imgs.clicked.connect(self.subir_imagens_treino_ia)
        treino_upload_row.addWidget(self.btn_treino_upload_imgs)

        self.btn_treino_upload_labels = QPushButton("Subir Labels (Manual)")
        self.estilizar_botao(self.btn_treino_upload_labels, "#1565c0")
        self.btn_treino_upload_labels.clicked.connect(self.subir_labels_treino_ia)
        treino_upload_row.addWidget(self.btn_treino_upload_labels)
        treino_ia_layout.addLayout(treino_upload_row)

        treino_ops_row = QHBoxLayout()
        self.btn_treino_split = QPushButton("Organizar Split 80/10/10")
        self.estilizar_botao(self.btn_treino_split, "#6a1b9a")
        self.btn_treino_split.clicked.connect(self.organizar_split_treino_ia)
        treino_ops_row.addWidget(self.btn_treino_split)

        self.btn_treino_contagem = QPushButton("Teste de Contagem por Split")
        self.estilizar_botao(self.btn_treino_contagem, "#455a64")
        self.btn_treino_contagem.clicked.connect(self.testar_contagem_split_treino_ia)
        self._btn_treino_contagem_default_style = self.btn_treino_contagem.styleSheet()
        treino_ops_row.addWidget(self.btn_treino_contagem)
        treino_ia_layout.addLayout(treino_ops_row)

        treino_exec_row = QHBoxLayout()
        self.btn_treino_smoke = QPushButton("Teste Rápido (1 época)")
        self.estilizar_botao(self.btn_treino_smoke, "#f57c00")
        self.btn_treino_smoke.clicked.connect(self.executar_smoke_test_treino_ia)
        treino_exec_row.addWidget(self.btn_treino_smoke)

        self.btn_treino_exec = QPushButton("Treino Completo")
        self.estilizar_botao(self.btn_treino_exec, "#00897b")
        self.btn_treino_exec.clicked.connect(self.executar_treino_completo_ia)
        treino_exec_row.addWidget(self.btn_treino_exec)
        treino_ia_layout.addLayout(treino_exec_row)

        self.lbl_treino_status = QLabel("Pronto para importar ZIP/Pasta ou usar fluxo manual.")
        self.lbl_treino_status.setStyleSheet("color: #b0bec5; font-size: 11px;")
        self.lbl_treino_status.setWordWrap(True)
        treino_ia_layout.addWidget(self.lbl_treino_status)
        treino_ia_layout.addStretch(1)

        self.tab_coleta_fotos = QWidget()
        coleta_layout = QVBoxLayout(self.tab_coleta_fotos)
        coleta_layout.setContentsMargins(12, 12, 12, 12)
        coleta_layout.setSpacing(8)

        lbl_coleta_title = QLabel("Coleta de Fotos para Treino")
        lbl_coleta_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #ffcc80;")
        coleta_layout.addWidget(lbl_coleta_title)

        self.lbl_coleta_session = QLabel("Sessão: --")
        self.lbl_coleta_session.setStyleSheet("color: #b0bec5; font-size: 11px;")
        coleta_layout.addWidget(self.lbl_coleta_session)

        self.lbl_coleta_usb = QLabel("Pendrive: não selecionado")
        self.lbl_coleta_usb.setStyleSheet("color: #b0bec5; font-size: 11px;")
        self.lbl_coleta_usb.setWordWrap(True)
        coleta_layout.addWidget(self.lbl_coleta_usb)

        cfg_capture = QGridLayout()
        cfg_capture.setHorizontalSpacing(8)
        cfg_capture.setVerticalSpacing(6)

        cfg_capture.addWidget(QLabel("camera_index"), 0, 0)
        self.spin_capture_camera_index = QSpinBox()
        self.spin_capture_camera_index.setRange(0, 10)
        self.spin_capture_camera_index.setValue(0)
        self.spin_capture_camera_index.setStyleSheet(
            "background-color: #444; padding: 4px; color: #ffffff; border: 1px solid #555;"
        )
        cfg_capture.addWidget(self.spin_capture_camera_index, 0, 1)

        cfg_capture.addWidget(QLabel("distância entre fotos (m)"), 0, 2)
        self.spin_capture_distance = QDoubleSpinBox()
        self.spin_capture_distance.setRange(0.1, 50.0)
        self.spin_capture_distance.setSingleStep(0.1)
        self.spin_capture_distance.setValue(0.5)
        self.spin_capture_distance.setDecimals(2)
        self.spin_capture_distance.setStyleSheet(
            "background-color: #444; padding: 4px; color: #ffffff; border: 1px solid #555;"
        )
        cfg_capture.addWidget(self.spin_capture_distance, 0, 3)

        cfg_capture.addWidget(QLabel("resolução"), 1, 0)
        self.combo_capture_resolution = QComboBox()
        self.combo_capture_resolution.addItems(["640x480", "1280x720", "1920x1080"])
        self.combo_capture_resolution.setCurrentText("1280x720")
        self.combo_capture_resolution.setStyleSheet(
                "QComboBox { background-color: #444; color: white; padding:"
                "4px; border: 1px solid #555; font-weight: bold; }"
                "QComboBox QAbstractItemView { background-color: #333; color: white; }"
        )
        cfg_capture.addWidget(self.combo_capture_resolution, 1, 1)

        cfg_capture.addWidget(QLabel("jpeg_quality"), 1, 2)
        self.spin_capture_jpeg_quality = QSpinBox()
        self.spin_capture_jpeg_quality.setRange(50, 100)
        self.spin_capture_jpeg_quality.setValue(90)
        self.spin_capture_jpeg_quality.setStyleSheet(
            "background-color: #444; padding: 4px; color: #ffffff; border: 1px solid #555;"
        )
        cfg_capture.addWidget(self.spin_capture_jpeg_quality, 1, 3)

        cfg_capture.addWidget(QLabel("velocidade mínima (m/s)"), 2, 0)
        self.spin_capture_speed_min = QDoubleSpinBox()
        self.spin_capture_speed_min.setRange(0.0, 2.0)
        self.spin_capture_speed_min.setSingleStep(0.01)
        self.spin_capture_speed_min.setDecimals(2)
        self.spin_capture_speed_min.setValue(0.05)
        self.spin_capture_speed_min.setStyleSheet(
            "background-color: #444; padding: 4px; color: #ffffff; border: 1px solid #555;"
        )
        cfg_capture.addWidget(self.spin_capture_speed_min, 2, 1)

        cfg_capture.addWidget(QLabel("reserva espaço (MB)"), 2, 2)
        self.spin_capture_reserve_mb = QSpinBox()
        self.spin_capture_reserve_mb.setRange(128, 65536)
        self.spin_capture_reserve_mb.setValue(512)
        self.spin_capture_reserve_mb.setStyleSheet(
            "background-color: #444; padding: 4px; color: #ffffff; border: 1px solid #555;"
        )
        cfg_capture.addWidget(self.spin_capture_reserve_mb, 2, 3)
        coleta_layout.addLayout(cfg_capture)

        monitor_frame = QFrame()
        monitor_frame.setStyleSheet(
            "QFrame { background-color: #171717; border: 1px solid #2f2f2f; border-radius: 8px; }"
        )
        monitor_grid = QGridLayout(monitor_frame)
        monitor_grid.setContentsMargins(10, 8, 10, 8)
        monitor_grid.setHorizontalSpacing(10)
        monitor_grid.setVerticalSpacing(6)

        monitor_grid.addWidget(QLabel("Fotos salvas"), 0, 0)
        self.lbl_capture_count = QLabel("0")
        self.lbl_capture_count.setStyleSheet("color: #ffeb3b; font-weight: bold;")
        monitor_grid.addWidget(self.lbl_capture_count, 0, 1)

        monitor_grid.addWidget(QLabel("Distância acumulada"), 1, 0)
        self.lbl_capture_distance_total = QLabel("0.00 m")
        self.lbl_capture_distance_total.setStyleSheet("color: #80cbc4; font-weight: bold;")
        monitor_grid.addWidget(self.lbl_capture_distance_total, 1, 1)

        monitor_grid.addWidget(QLabel("Espaço livre"), 2, 0)
        self.lbl_capture_free_space = QLabel("--")
        self.lbl_capture_free_space.setStyleSheet("color: #90caf9; font-weight: bold;")
        monitor_grid.addWidget(self.lbl_capture_free_space, 2, 1)

        monitor_grid.addWidget(QLabel("Fotos restantes (estimado)"), 3, 0)
        self.lbl_capture_est_remaining = QLabel("--")
        self.lbl_capture_est_remaining.setStyleSheet("color: #c5e1a5; font-weight: bold;")
        monitor_grid.addWidget(self.lbl_capture_est_remaining, 3, 1)
        coleta_layout.addWidget(monitor_frame)

        self.lbl_capture_preview = QLabel("Preview indisponível.")
        self.lbl_capture_preview.setAlignment(Qt.AlignCenter)
        self.lbl_capture_preview.setMinimumHeight(220)
        self.lbl_capture_preview.setStyleSheet(
            "background-color: #111; border: 1px solid #333; border-radius: 6px; color: #90a4ae;"
        )
        coleta_layout.addWidget(self.lbl_capture_preview, 1)

        self.lbl_capture_status = QLabel("Aguardando início da rota com implemento de coleta.")
        self.lbl_capture_status.setStyleSheet("color: #b0bec5; font-size: 11px;")
        self.lbl_capture_status.setWordWrap(True)
        coleta_layout.addWidget(self.lbl_capture_status)

        self.tab_rastreabilidade = QWidget()
        rast_layout = QVBoxLayout(self.tab_rastreabilidade)
        rast_layout.setContentsMargins(12, 12, 12, 12)

        lbl_rast_title = QLabel("Rastreabilidade e Conformidade Global")
        lbl_rast_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #aed581;")
        rast_layout.addWidget(lbl_rast_title)

        lbl_rast_desc = QLabel(
            "Geração padronizada de dados para UE (EUDR) e Brasil (PNRV/Brasil-ID)."
        )
        lbl_rast_desc.setStyleSheet("color: #cfd8dc; font-size: 11px;")
        lbl_rast_desc.setWordWrap(True)
        rast_layout.addWidget(lbl_rast_desc)

        lbl_links = QLabel("Como obter identificadores GS1 (produtor):")
        lbl_links.setStyleSheet("color: #b0bec5; font-size: 11px; margin-top: 6px;")
        rast_layout.addWidget(lbl_links)

        lbl_link_gln = QLabel((
            '<a href="https://www.gs1br.org/gln">GLN (13 dígitos) - página'
            'oficial GS1 Brasil</a>'
        ))
        lbl_link_gln.setOpenExternalLinks(True)
        lbl_link_gln.setTextInteractionFlags(Qt.TextBrowserInteraction)
        lbl_link_gln.setStyleSheet("color: #90caf9; font-size: 11px;")
        rast_layout.addWidget(lbl_link_gln)

        lbl_link_epc = QLabel((
            '<a href="https://www.gs1br.org/epc-rfid">EPC/RFID - como'
            'funciona e como implementar</a>'
        ))
        lbl_link_epc.setOpenExternalLinks(True)
        lbl_link_epc.setTextInteractionFlags(Qt.TextBrowserInteraction)
        lbl_link_epc.setStyleSheet("color: #90caf9; font-size: 11px;")
        rast_layout.addWidget(lbl_link_epc)

        lbl_link_assoc = QLabel((
            '<a'
            'href="https://www.gs1br.org/solicitar-codigo-de-barras">Filiação'
            'GS1 (necessária para emitir padrões oficiais)</a>'
        ))
        lbl_link_assoc.setOpenExternalLinks(True)
        lbl_link_assoc.setTextInteractionFlags(Qt.TextBrowserInteraction)
        lbl_link_assoc.setStyleSheet("color: #90caf9; font-size: 11px;")
        rast_layout.addWidget(lbl_link_assoc)

        self.chk_rastreabilidade = QCheckBox(
            "Gerar Pacote Universal de Rastreabilidade (UE + Brasil)"
        )
        self.chk_rastreabilidade.setStyleSheet(
            "color: #81c784; font-weight: bold; margin-top: 8px;"
        )
        self.chk_rastreabilidade.setChecked(True)
        rast_layout.addWidget(self.chk_rastreabilidade)

        self.lbl_perfil = QLabel("Perfil de saída:")
        self.lbl_perfil.setStyleSheet("color: #b0bec5; font-size: 11px; margin-top: 8px;")
        rast_layout.addWidget(self.lbl_perfil)

        self.combo_rastreabilidade_perfil = QComboBox()
        self.combo_rastreabilidade_perfil.setStyleSheet(
            "QComboBox { background-color: #444; color: white; padding: "
            "6px; border: 1px solid #555; font-weight: bold; }"
            "QComboBox QAbstractItemView { background-color: #333; color: white; }"
        )
        self.combo_rastreabilidade_perfil.addItems([
            "Universal (UE + Brasil)",
            "Somente UE (EUDR)",
            "Somente Brasil (PNRV/Brasil-ID)",
            "Operacional interno"
        ])
        self.combo_rastreabilidade_perfil.currentTextChanged.connect(
            self.atualizar_campos_rastreabilidade_dinamicos
        )
        rast_layout.addWidget(self.combo_rastreabilidade_perfil)

        self.lbl_gln = QLabel("GLN da fazenda (13 dígitos):")
        self.lbl_gln.setStyleSheet("color: #b0bec5; font-size: 11px; margin-top: 8px;")
        rast_layout.addWidget(self.lbl_gln)

        self.input_farm_gln = QLineEdit("7890000012345")
        self.input_farm_gln.setPlaceholderText("Ex.: 7890000012345")
        self.input_farm_gln.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        rast_layout.addWidget(self.input_farm_gln)

        self.lbl_epc = QLabel("EPC do insumo (Brasil-ID):")
        self.lbl_epc.setStyleSheet("color: #b0bec5; font-size: 11px; margin-top: 8px;")
        rast_layout.addWidget(self.lbl_epc)

        self.input_epc = QLineEdit("NAO_INFORMADO")
        self.input_epc.setPlaceholderText("Ex.: urn:epc:id:sgtin:...")
        self.input_epc.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        rast_layout.addWidget(self.input_epc)

        self.lbl_robot = QLabel("Identificador do robô:")
        self.lbl_robot.setStyleSheet("color: #b0bec5; font-size: 11px; margin-top: 8px;")
        rast_layout.addWidget(self.lbl_robot)

        self.input_robot_uuid = QLineEdit("AGRI-BOT-X99")
        self.input_robot_uuid.setPlaceholderText("Ex.: AGRI-BOT-X99")
        self.input_robot_uuid.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        rast_layout.addWidget(self.input_robot_uuid)

        self.lbl_operador = QLabel("Identificador do operador:")
        self.lbl_operador.setStyleSheet("color: #b0bec5; font-size: 11px; margin-top: 8px;")
        rast_layout.addWidget(self.lbl_operador)

        self.input_operator_id = QLineEdit("AUTO_PILOT")
        self.input_operator_id.setPlaceholderText("Ex.: AUTO_PILOT")
        self.input_operator_id.setStyleSheet(
            "background-color: #444; padding: 6px; color: #ffffff; border: 1px solid #555;"
        )
        rast_layout.addWidget(self.input_operator_id)

        line_rast_files = QFrame()
        line_rast_files.setFrameShape(QFrame.HLine)
        line_rast_files.setStyleSheet((
            "background-color: #3f3f3f; margin-top: 10px; margin-bottom:"
            "6px;"
        ))
        rast_layout.addWidget(line_rast_files)

        lbl_rast_files = QLabel("Arquivos de rastreabilidade")
        lbl_rast_files.setStyleSheet("color: #cfd8dc; font-size: 11px; font-weight: bold;")
        rast_layout.addWidget(lbl_rast_files)

        rast_files_actions = QHBoxLayout()
        rast_files_actions.setSpacing(8)

        self.btn_exportar_pen_drive = QPushButton("Exportar para Pen Drive")
        self.btn_exportar_pen_drive.setCursor(Qt.PointingHandCursor)
        self.btn_exportar_pen_drive.setStyleSheet(
            "QPushButton { "
            "background-color: #2e7d32; color: #ffffff; "
            "padding: 8px 10px; border-radius: 6px; "
            "border: 1px solid #4caf50; font-weight: bold; font-size: 11px; "
            "}"
            "QPushButton:hover { background-color: #388e3c; border: 1px solid #66bb6a; }"
            "QPushButton:pressed { background-color: #1b5e20; }"
        )
        self.btn_exportar_pen_drive.clicked.connect(
            self.exportar_arquivos_rastreabilidade_pen_drive
        )
        rast_files_actions.addWidget(self.btn_exportar_pen_drive)

        self.btn_excluir_rastreabilidade = QPushButton("Excluir arquivos do sistema")
        self.btn_excluir_rastreabilidade.setCursor(Qt.PointingHandCursor)
        self.btn_excluir_rastreabilidade.setStyleSheet(
            "QPushButton { "
            "background-color: #b71c1c; color: #ffffff; "
            "padding: 8px 10px; border-radius: 6px; "
            "border: 1px solid #ef5350; font-weight: bold; font-size: 11px; "
            "}"
            "QPushButton:hover { background-color: #c62828; border: 1px solid #ef9a9a; }"
            "QPushButton:pressed { background-color: #7f0000; }"
        )
        self.btn_excluir_rastreabilidade.clicked.connect(
            self.excluir_arquivos_rastreabilidade_local
        )
        rast_files_actions.addWidget(self.btn_excluir_rastreabilidade)

        rast_layout.addLayout(rast_files_actions)
        rast_layout.addStretch(1)

        self.tabs.addTab(logs_tab, "Logs")
        self.tabs.addTab(map_tab, "Mapa")
        self.tabs.addTab(self.tab_sensores, "Sensores")
        self.tabs.addTab(self.tab_ros_nodes, "Nós ROS2")
        self.tabs.addTab(self.tab_saude_ia, "Saúde IA")
        self.tabs.addTab(self.tab_treino_ia, "Treino IA")
        self.tabs.addTab(self.tab_coleta_fotos, "Coleta Fotos")
        self.tabs.addTab(self.tab_rastreabilidade, "Rastreabilidade")
        self.tabs.currentChanged.connect(self._on_tabs_right_changed)
        self.tabs.setCurrentIndex(1)
        right_layout.addWidget(self.tabs)
        right_layout.addWidget(bateria_frame)

        main_layout.addLayout(right_layout)

        # Chama uma vez para garantir o estado inicial correto
        self.atualizar_interface_tanque(abrir_aba=False)
        self.atualizar_campos_rastreabilidade_dinamicos()
        self.atualizar_wifi(False, None)
        self.atualizar_bateria(80, False)
        self.atualizar_mapa(self.robot_lat, self.robot_lon)
        self._atualizar_luz_estado()
        self.atualizar_monitor_ia()
        self.atualizar_monitor_captura_fotos()
        self._ensure_dataset_dirs()

    def estilizar_botao(self, btn, cor):
        btn.setStyleSheet(
            f"QPushButton {{ background-color: {cor}; color: white; font-weight: bold; "
            "padding: 12px; border-radius: 6px; font-size: 13px; }"
            f"QPushButton:hover {{ background-color: {cor}; opacity: 0.9; "
            "border: 1px solid white; }"
        )

    def estilizar_botao_destaque(self, btn, cor):
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {cor};
                color: white;
                font-weight: bold;
                padding: 8px 16px;
                border-radius: 8px;
                font-size: 14px;
                border: 2px solid #90a4ae;
            }}
            QPushButton:hover {{
                background-color: {cor};
                border: 2px solid #cfd8dc;
            }}
            QPushButton:pressed {{
                background-color: #455a64;
            }}
        """)

    def log(self, text, color="#00ff00"):
        self.log_console.setTextColor(QColor(color))
        self.log_console.append(text)
        c = self.log_console.textCursor()
        c.movePosition(QTextCursor.End)
        self.log_console.setTextCursor(c)

    def obter_modo_confianca_ia_codigo(self):
        if not hasattr(self, "combo_ia_conf_mode"):
            return "economico"
        texto = self.combo_ia_conf_mode.currentText().strip().lower()
        return "conservador" if "conservador" in texto else "economico"

    def _modo_confianca_ptbr(self, valor):
        raw = str(valor or "").strip().lower()
        if raw in ("economico", "econômico"):
            return "Econômico"
        if raw == "conservador":
            return "Conservador"
        return str(valor or "Econômico")

    def _doenca_ptbr(self, valor):
        raw = str(valor or "").strip()
        norm = raw.lower().replace("ã", "a")
        if norm in ("", "nao_detectada", "não_detectada", "none", "null"):
            return "Não Detectada"
        return raw.replace("_", " ")

    def _bool_ptbr(self, valor):
        if isinstance(valor, str):
            raw = valor.strip().lower()
            if raw in ("1", "true", "sim", "yes", "y"):
                return "Sim"
            if raw in ("0", "false", "nao", "não", "no", "n"):
                return "Não"
        return "Sim" if bool(valor) else "Não"

    def _format_ia_class_counts(self, payload):
        if not isinstance(payload, dict) or not payload:
            return "--"
        parts = []
        for class_name, count in payload.items():
            try:
                parts.append(f"{class_name}: {int(count)}")
            except Exception:
                parts.append(f"{class_name}: {count}")
        return " | ".join(parts)

    def _format_ia_unmapped_labels(self, payload):
        if not isinstance(payload, dict) or not payload:
            return ""
        parts = []
        for label, count in payload.items():
            try:
                parts.append(f"{label}: {int(count)}")
            except Exception:
                parts.append(f"{label}: {count}")
        return ", ".join(parts)

    def _aplicar_preset_teste_sensivel(self):
        if hasattr(self, "spin_ia_thr_cons"):
            self.spin_ia_thr_cons.setValue(0.03)
        if hasattr(self, "spin_ia_thr_econ"):
            self.spin_ia_thr_econ.setValue(0.10)
        self.log(
            "[IA] Preset 'Teste Sensível' aplicado (conservador=0.03, econômico=0.10).",
            "#90caf9",
        )

    def _resolver_modelo_ia_padrao(self):
        runs_root = os.path.join(self.workspace, "runs", "detect")

        def latest_by_mtime(pattern):
            candidates = [p for p in glob(pattern) if os.path.isfile(p)]
            candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
            return candidates

        latest_best = latest_by_mtime(os.path.join(runs_root, "*", "weights", "best.pt"))
        if latest_best:
            return latest_best[0], "ultimo_best"

        latest_last = latest_by_mtime(os.path.join(runs_root, "*", "weights", "last.pt"))
        if latest_last:
            return latest_last[0], "ultimo_last"

        home_best = os.path.join(self.home, "models", "best.pt")
        if os.path.isfile(home_best):
            return home_best, "home_models_best"

        return "", "nao_encontrado"

    def _sugerir_modelo_ia_padrao_se_necessario(self, force=False):
        if not hasattr(self, "input_ia_model_path"):
            return

        current = self.input_ia_model_path.text().strip()
        current_expanded = os.path.expanduser(current) if current else ""
        base = os.path.basename(current_expanded).lower() if current_expanded else ""
        needs_suggest = bool(force or not current_expanded or base in ("yolo26n.pt", "yolov8n.pt"))
        if not needs_suggest:
            return

        suggested, source = self._resolver_modelo_ia_padrao()
        if not suggested:
            return

        self.input_ia_model_path.setText(suggested)
        if force or source in ("ultimo_best", "ultimo_last"):
            self.log(f"[IA] Modelo sugerido automaticamente ({source}): {suggested}", "#90caf9")

    def _resolve_model_path_abs(self, model_path):
        model_path = str(model_path or "").strip()
        if not model_path:
            return ""
        model_path_expanded = os.path.expanduser(model_path)
        if os.path.isabs(model_path_expanded):
            return model_path_expanded
        home_candidate = os.path.join(self.home, model_path_expanded)
        if os.path.isfile(home_candidate):
            return home_candidate
        return os.path.abspath(model_path_expanded)

    def _run_python_json(self, code, args, timeout_sec=45):
        try:
            result = subprocess.run(
                ["python3", "-c", code, *args],
                capture_output=True,
                text=True,
                timeout=timeout_sec,
            )
        except subprocess.TimeoutExpired:
            return None, f"Timeout na validação Python após {float(timeout_sec):.1f}s."
        except Exception as e:
            return None, f"Falha ao executar validação Python: {e}"

        if result.returncode != 0:
            err = (result.stderr or result.stdout or "").strip()
            return None, err or f"Processo retornou código {result.returncode}."

        lines = [ln.strip() for ln in (result.stdout or "").splitlines() if ln.strip()]
        if not lines:
            return None, "Saída vazia da validação Python."

        raw = lines[-1]
        try:
            return json.loads(raw), ""
        except Exception as e:
            return None, f"Falha ao decodificar JSON da validação: {e} | Saída: {raw[:200]}"

    def _load_ia_preflight_cache(self):
        path = self.ia_preflight_cache_path
        if not os.path.isfile(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            if isinstance(payload, dict) and isinstance(payload.get("entries"), dict):
                return payload.get("entries", {})
            if isinstance(payload, dict):
                return payload
        except Exception:
            return {}
        return {}

    def _save_ia_preflight_cache(self, entries):
        if not isinstance(entries, dict):
            return
        # Limita crescimento do cache mantendo os mais recentes.
        items = []
        for key, value in entries.items():
            if not isinstance(value, dict):
                continue
            updated_at = float(value.get("updated_at", 0.0) or 0.0)
            items.append((key, updated_at, value))
        items.sort(key=lambda item: item[1], reverse=True)
        limited = {key: value for key, _ts, value in items[: self.ia_preflight_cache_limit]}
        payload = {"entries": limited}
        path = self.ia_preflight_cache_path
        tmp_path = f"{path}.tmp"
        try:
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            os.replace(tmp_path, path)
        except Exception:
            try:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except Exception:
                pass

    def _preflight_cache_key(self, model_path_abs):
        try:
            model_stat = os.stat(model_path_abs)
            data_stat = os.stat(self.dataset_data_yaml)
        except Exception as e:
            return "", {}, f"Falha ao obter metadata para cache: {e}"

        ident = {
            "model_path_abs": model_path_abs,
            "model_mtime_ns": int(
                getattr(model_stat, "st_mtime_ns", int(model_stat.st_mtime * 1e9))
            ),
            "data_yaml_path": self.dataset_data_yaml,
            "data_yaml_mtime_ns": int(
                getattr(data_stat, "st_mtime_ns", int(data_stat.st_mtime * 1e9))
            ),
        }
        return json.dumps(ident, ensure_ascii=False, sort_keys=True), ident, ""

    def _update_ia_preflight_cache_entry(self, cache_key, entry_updates):
        if not cache_key or not isinstance(entry_updates, dict):
            return
        cache = self._load_ia_preflight_cache()
        current = cache.get(cache_key, {})
        if not isinstance(current, dict):
            current = {}
        current.update(entry_updates)
        current["updated_at"] = time.time()
        cache[cache_key] = current
        self._save_ia_preflight_cache(cache)

    def _start_sanidade_modelo_assincrona(self, session_id, model_path_abs, cache_key):
        # Evita múltiplas avaliações simultâneas para o mesmo modelo/sessão.
        current = self._ia_sanity_async_state
        if isinstance(current, dict) and not current.get("done", True):
            if current.get("cache_key") == cache_key and current.get("session_id") == session_id:
                return

        state = {
            "session_id": session_id,
            "model_path_abs": model_path_abs,
            "cache_key": cache_key,
            "done": False,
            "result": None,
            "elapsed_ms": 0.0,
            "started_at": time.time(),
        }
        self._ia_sanity_async_state = state

        def _worker():
            t0 = time.perf_counter()
            try:
                result = self._avaliar_sanidade_modelo_ia(model_path_abs, conf=0.01, max_images=3)
            except Exception as e:
                result = {
                    "ok": False,
                    "status": "erro_sanidade",
                    "message": f"Falha inesperada na sanidade assíncrona: {e}",
                    "stats": {},
                    "suggested_model_path": "",
                }
            state["result"] = result
            state["elapsed_ms"] = (time.perf_counter() - t0) * 1000.0
            state["done"] = True

        threading.Thread(target=_worker, daemon=True).start()
        if not self._ia_sanity_timer.isActive():
            self._ia_sanity_timer.start()

    def _poll_sanidade_modelo_assincrona(self):
        state = self._ia_sanity_async_state
        if not isinstance(state, dict):
            self._ia_sanity_timer.stop()
            return
        if not state.get("done", False):
            return

        result = state.get("result") or {}
        elapsed_ms = float(state.get("elapsed_ms", 0.0) or 0.0)
        cache_key = str(state.get("cache_key", "") or "")
        session_id = str(state.get("session_id", "") or "")
        status = str(result.get("status", "erro_sanidade"))
        stats = result.get("stats", {}) if isinstance(result.get("stats", {}), dict) else {}
        message = str(result.get("message", "") or "")

        self._update_ia_preflight_cache_entry(
            cache_key,
            {
                "sanity_status": status,
                "sanity_stats": stats,
                "sanity_message": message,
                "sanity_elapsed_ms": round(elapsed_ms, 2),
            },
        )

        if status == "weak_recommend_last":
            suggested = str(result.get("suggested_model_path", "") or "").strip()
            extra = f" | sugestão={suggested}" if suggested else ""
            self.log(
                f"[IA] Sanidade assíncrona: modelo fraco | session_id={session_id} | "
                f"t_sanity_ms={elapsed_ms:.1f}{extra}",
                "#ffb74d",
            )
        elif status == "ok":
            self.log(
                (
                    f"[IA] Sanidade assíncrona concluída | session_id={session_id} |"
                    f"t_sanity_ms={elapsed_ms:.1f}"
                ),
                "#80cbc4",
            )
        else:
            self.log(
                f"[IA] Sanidade assíncrona com aviso ({status}) | session_id={session_id} | "
                f"t_sanity_ms={elapsed_ms:.1f} | {message}",
                "#ffb74d",
            )

        self._ia_sanity_async_state = None
        self._ia_sanity_timer.stop()

    def _obter_classes_modelo_ia(self, model_path_abs, timeout_sec=50):
        script = (
            "import json, sys\n"
            "import torch\n"
            "path = sys.argv[1]\n"
            "names = None\n"
            "errors = []\n"
            "try:\n"
            "    try:\n"
            "        ckpt = torch.load(path, map_location='cpu', weights_only=False)\n"
            "    except TypeError:\n"
            "        ckpt = torch.load(path, map_location='cpu')\n"
            "    if isinstance(ckpt, dict):\n"
            "        model = ckpt.get('model')\n"
            "        if model is not None and hasattr(model, 'names'):\n"
            "            names = getattr(model, 'names')\n"
            "except Exception as e:\n"
            "    errors.append('torch:' + str(e))\n"
            "if not names:\n"
            "    try:\n"
            "        from ultralytics import YOLO\n"
            "        model = YOLO(path)\n"
            "        names = getattr(model, 'names', {})\n"
            "    except Exception as e:\n"
            "        errors.append('yolo:' + str(e))\n"
            "if not names:\n"
            "raise RuntimeError(' ; '.join(errors) or 'nao foi possivel "
            "extrair classes do modelo')\\n"
            "ordered = []\n"
            "if isinstance(names, dict):\n"
            "    items = []\n"
            "    for k, v in names.items():\n"
            "        try:\n"
            "            idx = int(k)\n"
            "        except Exception:\n"
            "            continue\n"
            "        items.append((idx, str(v)))\n"
            "    items.sort(key=lambda x: x[0])\n"
            "    ordered = [v for _, v in items]\n"
            "else:\n"
            "    ordered = [str(v) for v in names]\n"
            "print(json.dumps({'names': ordered}, ensure_ascii=False))\n"
        )
        payload, err = self._run_python_json(script, [model_path_abs], timeout_sec=timeout_sec)
        if err:
            return None, f"Falha ao ler classes do modelo: {err}"
        names = payload.get("names") if isinstance(payload, dict) else None
        if not isinstance(names, list) or not names:
            return None, "Modelo sem classes válidas."
        return [str(n).strip() for n in names], ""

    def _validar_modelo_vs_data_yaml(self, model_path_abs, timeout_sec=50):
        names_yaml, err_yaml = self._ler_names_data_yaml()
        if err_yaml:
            return {
                "ok": False,
                "status": "erro_data_yaml",
                "message": err_yaml,
                "yaml_names": [],
                "model_names": [],
            }

        model_names, err_model = self._obter_classes_modelo_ia(
            model_path_abs,
            timeout_sec=timeout_sec,
        )
        if err_model:
            return {
                "ok": False,
                "status": "erro_modelo",
                "message": err_model,
                "yaml_names": names_yaml,
                "model_names": [],
            }

        norm_yaml = [self._normalizar_nome_classe(n) for n in names_yaml]
        norm_model = [self._normalizar_nome_classe(n) for n in model_names]
        yaml_set = set(norm_yaml)
        model_set = set(norm_model)

        issues = []
        if len(norm_yaml) != len(norm_model):
            issues.append(
                    f"Quantidade de classes diferente (data.yaml={len(norm_yaml)} |"
                    f"modelo={len(norm_model)})."
            )

        missing_in_model = [
            names_yaml[idx]
            for idx, norm_name in enumerate(norm_yaml)
            if norm_name not in model_set
        ]
        extra_in_model = [
            model_names[idx]
            for idx, norm_name in enumerate(norm_model)
            if norm_name not in yaml_set
        ]
        if missing_in_model:
            issues.append("Classes ausentes no modelo: " + ", ".join(missing_in_model))
        if extra_in_model:
            issues.append("Classes extras no modelo: " + ", ".join(extra_in_model))

        mismatch_idx = []
        for idx in range(min(len(norm_yaml), len(norm_model))):
            if norm_yaml[idx] != norm_model[idx]:
                mismatch_idx.append(
                    f"{idx}:yaml='{names_yaml[idx]}' vs model='{model_names[idx]}'"
                )
        if mismatch_idx:
            issues.append("Ordem/índice divergente: " + " | ".join(mismatch_idx))

        if issues:
            return {
                "ok": False,
                "status": "incompativel",
                "message": "\n".join(issues),
                "yaml_names": names_yaml,
                "model_names": model_names,
            }

        return {
            "ok": True,
            "status": "ok",
            "message": "",
            "yaml_names": names_yaml,
            "model_names": model_names,
        }

    def _coletar_imagens_dataset_para_sanidade(self, limit=3):
        images = []
        for split in ("train", "val", "test"):
            split_dir = os.path.join(self.dataset_root, "images", split)
            images.extend(self._list_image_files(split_dir))
        return images[:max(int(limit), 0)]

    def _avaliar_sanidade_modelo_ia(self, model_path_abs, conf=0.01, max_images=3):
        sample_images = self._coletar_imagens_dataset_para_sanidade(limit=max_images)
        if not sample_images:
            return {
                "ok": True,
                "status": "skip_no_images",
                "message": "Sem imagens locais para teste rápido de sanidade.",
                "stats": {},
                "suggested_model_path": "",
            }

        script = (
            "import json, sys\n"
            "from ultralytics import YOLO\n"
            "model_path = sys.argv[1]\n"
            "conf = float(sys.argv[2])\n"
            "images = json.loads(sys.argv[3])\n"
            "model = YOLO(model_path)\n"
            "stats = {'images_tested': 0, 'detections_total': 0, "
            "'best_confidence': 0.0, 'per_image': []}\\n"
            "for img_path in images:\n"
            "    r = model.predict(img_path, conf=conf, verbose=False, device='cpu')[0]\n"
            "    n = 0 if r.boxes is None else len(r.boxes)\n"
            "    best = 0.0\n"
            "    if n:\n"
            "        best = float(r.boxes.conf.max().item())\n"
            "    stats['images_tested'] += 1\n"
            "    stats['detections_total'] += int(n)\n"
            "    stats['best_confidence'] = max(float(stats['best_confidence']), best)\n"
            "stats['per_image'].append({'image': img_path, 'detections': "
            "int(n), 'best_confidence': best})\\n"
            "print(json.dumps(stats, ensure_ascii=False))\n"
        )

        payload, err = self._run_python_json(
            script,
            [model_path_abs, f"{float(conf):.6f}", json.dumps(sample_images, ensure_ascii=False)],
            timeout_sec=80,
        )
        if err:
            return {
                "ok": False,
                "status": "erro_sanidade",
                "message": f"Falha no teste rápido de sanidade: {err}",
                "stats": {},
                "suggested_model_path": "",
            }

        detections_total = (
            int(payload.get("detections_total", 0))
            if isinstance(payload, dict)
            else 0
        )
        best_conf = (
            float(payload.get("best_confidence", 0.0))
            if isinstance(payload, dict)
            else 0.0
        )
        weak = detections_total <= 0 or best_conf < 0.01

        suggested_last = ""
        if weak and model_path_abs.endswith("/weights/best.pt"):
            candidate_last = model_path_abs[:-len("best.pt")] + "last.pt"
            if os.path.isfile(candidate_last):
                suggested_last = candidate_last

        if weak:
            msg = (
                "Modelo com baixa sanidade no teste rápido "
                f"(imagens={payload.get('images_tested', 0)} | "
                f"detecções={detections_total} | best_conf={best_conf:.4f})."
            )
            if suggested_last:
                msg += f" Sugestão imediata: usar {suggested_last}"
            return {
                "ok": True,
                "status": "weak_recommend_last",
                "message": msg,
                "stats": payload if isinstance(payload, dict) else {},
                "suggested_model_path": suggested_last,
            }

        return {
            "ok": True,
            "status": "ok",
            "message": (
                "Sanidade do modelo OK "
                f"(imagens={payload.get('images_tested', 0)} | "
                f"detecções={detections_total} | best_conf={best_conf:.4f})."
            ),
            "stats": payload if isinstance(payload, dict) else {},
            "suggested_model_path": "",
        }

    def _gerar_session_id(self):
        # Prefixo textual evita conversao para inteiro ao passar por launch params YAML.
        return "sess_" + time.strftime("%Y%m%d_%H%M%S")

    def _encerrar_processo(self, nome, timeout_ms=3000):
        proc = self.processos.get(nome)
        if proc is None or proc.state() == QProcess.NotRunning:
            return
        try:
            os.kill(proc.processId(), signal.SIGINT)
            proc.waitForFinished(timeout_ms)
        except Exception:
            pass
        if proc.state() != QProcess.NotRunning:
            try:
                proc.kill()
                proc.waitForFinished(800)
            except Exception:
                pass

    def _buscar_pids_por_padrao(self, pattern):
        try:
            result = subprocess.run(
                ["pgrep", "-f", pattern],
                capture_output=True,
                text=True,
                timeout=2,
            )
        except Exception:
            return []

        if result.returncode not in (0, 1):
            return []

        pids = []
        for line in (result.stdout or "").splitlines():
            try:
                pid = int(line.strip())
            except Exception:
                continue
            if pid <= 0 or pid == os.getpid():
                continue
            pids.append(pid)
        return pids

    def _encerrar_processos_externos_por_padrao(self, patterns, contexto):
        pids = set()
        for pattern in patterns:
            for pid in self._buscar_pids_por_padrao(pattern):
                pids.add(int(pid))

        if not pids:
            return 0

        for pid in sorted(pids):
            try:
                os.kill(pid, signal.SIGINT)
            except Exception:
                pass

        time.sleep(0.25)

        restantes = []
        for pid in sorted(pids):
            try:
                os.kill(pid, 0)
                restantes.append(pid)
            except Exception:
                pass

        for pid in restantes:
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass

        self.log(
            (
                f"[CAMERA] Encerrado(s) {len(pids)} processo(s) antigo(s) em"
                f"{contexto}: {sorted(pids)}"
            ),
            "#ffb300",
        )
        return len(pids)

    def _parar_pipeline_ia(self):
        self._encerrar_processo("IA_PIPELINE")
        # Garante limpeza de nós órfãos da IA quando o launch pai cai/crasha.
        self._encerrar_processos_externos_por_padrao(
            [
                "caatinga_vision/camera_source_node",
                "caatinga_vision.camera_source_node",
                "camera_source_node",
                "caatinga_vision/yolo_inference_node",
                "caatinga_vision.yolo_inference_node",
                "yolo_inference_node",
                "caatinga_vision/infestation_analytics_node",
                "caatinga_vision.infestation_analytics_node",
                "infestation_analytics_node",
            ],
            "parada do pipeline IA",
        )
        self.ia_session_id = ""
        self.ia_rota_ativa = False

    def _encerrar_rastreabilidade(self):
        self._encerrar_processo("RASTREABILIDADE")

    def _qprocess_error_name(self, err):
        try:
            e = int(err)
        except Exception:
            return str(err)
        mapping = {
            int(QProcess.FailedToStart): "FailedToStart",
            int(QProcess.Crashed): "Crashed",
            int(QProcess.Timedout): "Timedout",
            int(QProcess.WriteError): "WriteError",
            int(QProcess.ReadError): "ReadError",
            int(QProcess.UnknownError): "UnknownError",
        }
        return mapping.get(e, str(e))

    def _on_process_error(self, name, err):
        err_name = self._qprocess_error_name(err)
        self.log(f"[{name}] Erro QProcess: {int(err)} ({err_name})", "#ff5555")

        if name == "IA_PIPELINE":
            # Se o launch da IA crashar, fecha qualquer nó remanescente da câmera/inferência.
            self._parar_pipeline_ia()
            return

        if name == "SISTEMA_FAZENDA":
            # Em falha do sistema principal, evita ficar com câmera/nós IA presos.
            self._parar_pipeline_ia()
            self._parar_captura_fotos()
            self._encerrar_rastreabilidade()
            self._set_movimento_ativo(False)
            if hasattr(self, "btn_sistema"):
                self.btn_sistema.setText("🚀 Ligar Sistema (Launch)")
                self.btn_sistema.setStyleSheet(
                    self.btn_sistema.styleSheet().replace("#444", "#e65100")
                )

    def _aguardar_analytics_ia_ativo(
        self,
        session_id,
        status_cache_path,
        mtime_min,
        timeout_sec=8.0,
    ):
        timeout_sec = max(float(timeout_sec), 0.2)
        # Abertura do ros2 launch pode levar alguns segundos antes dos nós IA ficarem ativos.
        startup_grace_sec = max(timeout_sec + 4.0, 12.0)
        start_ts = time.time()
        analytics_boot_ts = None
        last_error = ""
        result = {"ok": False, "err": ""}
        loop = QEventLoop(self)
        timer = QTimer(self)
        timer.setInterval(120)

        def _finish(ok, err=""):
            result["ok"] = bool(ok)
            result["err"] = str(err or "")
            if timer.isActive():
                timer.stop()
            if loop.isRunning():
                loop.quit()

        def _check():
            nonlocal last_error, analytics_boot_ts
            proc = self.processos.get("IA_PIPELINE")
            if proc is None or proc.state() == QProcess.NotRunning:
                _finish(False, "Pipeline de IA encerrou antes de iniciar o analytics.")
                return

            now_ts = time.time()
            analytics_nodes_online = False
            try:
                analytics_nodes_online = bool(
                    self._buscar_pids_por_padrao("camera_source_node")
                    or self._buscar_pids_por_padrao("yolo_inference_node")
                    or self._buscar_pids_por_padrao("infestation_analytics_node")
                )
            except Exception:
                analytics_nodes_online = False
            if analytics_nodes_online and analytics_boot_ts is None:
                analytics_boot_ts = now_ts

            try:
                if os.path.exists(status_cache_path):
                    stat = os.stat(status_cache_path)
                    if stat.st_mtime > float(mtime_min):
                        with open(status_cache_path, "r", encoding="utf-8") as f:
                            status = json.load(f)
                        if isinstance(status, dict):
                            sid = str(status.get("session_id", "")).strip()
                            if sid == session_id:
                                self.ia_status_snapshot = status
                                self._ia_status_cache_mtime = stat.st_mtime
                                _finish(True, "")
                                return
                            if sid:
                                last_error = (
                                    f"Status IA pertence à sessão '{sid}', "
                                    f"esperado '{session_id}'."
                                )
                        else:
                            last_error = "Arquivo de status IA não contém JSON objeto válido."
            except Exception as e:
                last_error = f"Falha ao ler status IA: {e}"

            # Fase 1: espera nós da IA subirem (bootstrap).
            if analytics_boot_ts is None:
                if (now_ts - start_ts) >= startup_grace_sec:
                    if not last_error:
                        last_error = (
                            "Pipeline de IA não concluiu bootstrap em tempo hábil "
                            f"({startup_grace_sec:.1f}s): nós camera/yolo/analytics "
                            "não ficaram ativos."
                        )
                    _finish(False, last_error)
                return

            # Fase 2: nós já subiram, então exige status da sessão no prazo.
            if (now_ts - analytics_boot_ts) >= timeout_sec:
                if not last_error:
                    last_error = (
                        f"Arquivo de status não atualizou para a sessão em até {timeout_sec:.1f}s "
                        f"após bootstrap ({status_cache_path})."
                    )
                _finish(False, last_error)

        timer.timeout.connect(_check)
        timer.start()
        QTimer.singleShot(0, _check)
        loop.exec_()
        timer.deleteLater()
        return result["ok"], result["err"]

    def iniciar_pipeline_ia(self, session_id):
        self._parar_pipeline_ia()
        self._parar_captura_fotos()
        self._encerrar_processos_externos_por_padrao(
            [
                "caatinga_vision/photo_capture_node",
                "caatinga_vision.photo_capture_node",
            ],
            "início da Saúde IA",
        )
        self.ia_status_snapshot = {}
        self._ia_status_cache_mtime = 0.0
        self.ia_session_id = session_id
        status_cache_path_session = os.path.join(
            "/tmp",
            f"caatinga_vision_status_{session_id}.json",
        )
        self.ia_status_cache_path_active = status_cache_path_session
        self.ia_status_cache_path = status_cache_path_session
        try:
            if os.path.exists(status_cache_path_session):
                os.remove(status_cache_path_session)
        except Exception:
            pass

        confidence_mode = self.obter_modo_confianca_ia_codigo()
        model_path = (
            self.input_ia_model_path.text().strip()
            if hasattr(self, "input_ia_model_path")
            else ""
        )
        if not model_path:
            suggested_model, _ = self._resolver_modelo_ia_padrao()
            if suggested_model:
                model_path = suggested_model
                if hasattr(self, "input_ia_model_path"):
                    self.input_ia_model_path.setText(model_path)
        model_path_abs = self._resolve_model_path_abs(model_path)
        if not os.path.isfile(model_path_abs):
            msg = f"Modelo YOLO não encontrado: {model_path_abs}"
            self.log(f"[IA] {msg}", "#ff5555")
            QMessageBox.warning(self, "Saúde IA", msg)
            return False

        names_yaml, err_yaml = self._ler_names_data_yaml()
        if err_yaml:
            msg = f"Falha ao ler data.yaml para validação de IA: {err_yaml}"
            self.log(f"[IA] {msg}", "#ff5555")
            QMessageBox.warning(self, "Saúde IA", msg)
            return False

        preflight_key, preflight_ident, preflight_key_err = self._preflight_cache_key(
            model_path_abs
        )
        cache_hit = False
        cached_entry = None
        if preflight_key:
            cache = self._load_ia_preflight_cache()
            entry = cache.get(preflight_key)
            if isinstance(entry, dict):
                cache_hit = True
                cached_entry = entry

        t_compat_start = time.perf_counter()
        compat_retry_timeout = False
        force_refresh_from_cache = False
        if cached_entry is not None:
            cached_compat_ok = bool(cached_entry.get("compat_ok", False))
            cached_compat_status = str(cached_entry.get("compat_status", "") or "").strip() or (
                "ok" if cached_compat_ok else "incompativel"
            )
            cached_compat_msg = str(cached_entry.get("compat_message", "") or "")
            if (not cached_compat_ok) and (
                cached_compat_status in ("erro_modelo", "erro_data_yaml")
                or "timeout" in cached_compat_msg.lower()
                or "falha ao ler classes do modelo" in cached_compat_msg.lower()
            ):
                force_refresh_from_cache = True
                cache_hit = False

            compat = {
                "ok": cached_compat_ok,
                "status": cached_compat_status,
                "message": cached_compat_msg,
                "yaml_names": names_yaml,
                "model_names": list(cached_entry.get("model_names", []) or []),
            }
        if cached_entry is None or force_refresh_from_cache:
            compat = self._validar_modelo_vs_data_yaml(model_path_abs, timeout_sec=10)
            if (
                compat.get("status") == "erro_modelo"
                and "timeout" in str(compat.get("message", "")).lower()
            ):
                # Primeiro acesso ao .pt pode demorar mais (I/O e load inicial do
                # torch/ultralytics).
                compat_retry_timeout = True
                self.log(
                    (
                        "[IA] Validação inicial do modelo excedeu 10s; tentando"
                        "novamente com timeout ampliado."
                    ),
                    "#ffb74d",
                )
                compat = self._validar_modelo_vs_data_yaml(model_path_abs, timeout_sec=35)
            if preflight_key:
                cache_payload = {}
                if isinstance(preflight_ident, dict):
                    cache_payload.update(preflight_ident)
                cache_payload.update(
                    {
                        "compat_ok": bool(compat.get("ok", False)),
                        "compat_status": str(compat.get("status", "") or ""),
                        "compat_message": str(compat.get("message", "")),
                        "model_names": list(compat.get("model_names", []) or []),
                        "sanity_status": "pending_async",
                        "sanity_stats": {},
                        "sanity_message": "",
                        "sanity_elapsed_ms": 0.0,
                    }
                )
                self._update_ia_preflight_cache_entry(preflight_key, cache_payload)
        t_model_compat_ms = (time.perf_counter() - t_compat_start) * 1000.0

        if not compat.get("ok", False):
            compat_status = str(compat.get("status", "") or "")
            compat_msg = str(
                compat.get("message", "Compatibilidade inválida.")
                or "Compatibilidade inválida."
            )
            if compat_status == "erro_modelo":
                msg = (
                    f"Falha ao ler classes do modelo.\n\nModelo: {model_path_abs}\n"
                    f"{compat_msg}\n"
                    "Dica: aguarde alguns segundos e tente iniciar novamente."
                )
            elif compat_status == "erro_data_yaml":
                msg = (
                    f"Falha ao validar data.yaml.\\n\\nArquivo:"
                    f"{self.dataset_data_yaml}\\n{compat_msg}"
                )
            else:
                msg = (
                    f"Modelo incompatível com data.yaml.\n\nModelo: {model_path_abs}\n"
                    f"{compat_msg}"
                )
            self.log(f"[IA] {msg}", "#ff5555")
            QMessageBox.warning(self, "Saúde IA", msg)
            return False

        model_names_json = json.dumps(compat.get("model_names", []), ensure_ascii=False)
        model_names_b64 = base64.urlsafe_b64encode(
            model_names_json.encode("utf-8")
        ).decode("ascii")
        sanity_cached_status_raw = str((cached_entry or {}).get("sanity_status", "") or "").strip()
        sanity_cached_status = (
            sanity_cached_status_raw
            if sanity_cached_status_raw not in ("", "pending_async")
            else ""
        )
        sanity_cached_ms = float((cached_entry or {}).get("sanity_elapsed_ms", 0.0) or 0.0)
        model_check_status = sanity_cached_status or "pending_async"

        camera_index = (
            int(self.spin_ia_camera_index.value())
            if hasattr(self, "spin_ia_camera_index")
            else 0
        )
        thr_cons = (
            float(self.spin_ia_thr_cons.value())
            if hasattr(self, "spin_ia_thr_cons")
            else 0.50
        )
        thr_econ = (
            float(self.spin_ia_thr_econ.value())
            if hasattr(self, "spin_ia_thr_econ")
            else 0.85
        )
        status_mtime_before = 0.0
        try:
            if os.path.exists(status_cache_path_session):
                status_mtime_before = os.stat(status_cache_path_session).st_mtime
        except Exception:
            status_mtime_before = 0.0

        cmd = (
            "ros2 launch caatinga_vision ia_pipeline.launch.py "
            f"session_id:={shlex.quote(session_id)} "
            f"camera_index:={camera_index} "
            f"model_path:={shlex.quote(model_path_abs)} "
            f"confidence_mode:={shlex.quote(confidence_mode)} "
            f"confidence_threshold_conservador:={thr_cons:.3f} "
            f"confidence_threshold_economico:={thr_econ:.3f} "
            f"overlay_cache_path:={shlex.quote(self.ia_overlay_cache_path)} "
            f"status_cache_path:={shlex.quote(status_cache_path_session)} "
            f"logs_base_dir:={shlex.quote(os.path.join(self.workspace, 'logs_rastreabilidade'))} "
            f"dataset_data_yaml:={shlex.quote(self.dataset_data_yaml)} "
            f"model_check_status:={shlex.quote(str(model_check_status))} "
            f"model_names_b64:={shlex.quote(model_names_b64)}"
        )
        proc = self.run_process("IA_PIPELINE", cmd)
        if proc:
            t_ready_start = time.perf_counter()
            analytics_ok, analytics_err = self._aguardar_analytics_ia_ativo(
                session_id=session_id,
                status_cache_path=status_cache_path_session,
                mtime_min=status_mtime_before,
                timeout_sec=8.0,
            )
            t_pipeline_ready_ms = (time.perf_counter() - t_ready_start) * 1000.0
            if not analytics_ok:
                self._parar_pipeline_ia()
                msg = (
                    "Analytics IA não iniciou corretamente.\n\n"
                    f"Sessão: {session_id}\n"
                    f"Detalhe: {analytics_err}"
                )
                sanity_metric = f"{sanity_cached_ms:.1f}" if sanity_cached_status else "async"
                self.log(
                    f"[IA] analytics_health=fail | cache_hit={str(cache_hit).lower()} | "
                    f"t_model_compat_ms={t_model_compat_ms:.1f} | t_sanity_ms={sanity_metric} | "
                    f"t_pipeline_ready_ms={t_pipeline_ready_ms:.1f} |"
                    f"compat_retry_timeout={str(compat_retry_timeout).lower()} |"
                    f"{msg}",
                    "#ff5555",
                )
                QMessageBox.warning(self, "Saúde IA", msg)
                return False

            if not sanity_cached_status:
                self._start_sanidade_modelo_assincrona(
                    session_id=session_id,
                    model_path_abs=model_path_abs,
                    cache_key=preflight_key,
                )
            else:
                self.log(
                    f"[IA] Sanidade do modelo obtida do cache ({sanity_cached_status}) | "
                    f"t_sanity_ms={sanity_cached_ms:.1f}",
                    "#90caf9",
                )

            sanity_metric = f"{sanity_cached_ms:.1f}" if sanity_cached_status else "async"
            self.log(
                f"[IA] Pipeline iniciado | session_id={session_id} | modo={confidence_mode} | "
                f"model={model_path_abs} | analytics_health=ok | "
                f"cache_hit={str(cache_hit).lower()} | "
                f"t_model_compat_ms={t_model_compat_ms:.1f} | "
                f"t_sanity_ms={sanity_metric} | t_pipeline_ready_ms={t_pipeline_ready_ms:.1f} | "
                f"compat_retry_timeout={str(compat_retry_timeout).lower()}",
                "#80cbc4",
            )
            return True
        return False

    def atualizar_monitor_ia(self):
        if hasattr(self, "lbl_ia_feed"):
            pixmap = QPixmap(self.ia_overlay_cache_path)
            if not pixmap.isNull():
                scaled = pixmap.scaled(
                    self.lbl_ia_feed.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
                self.lbl_ia_feed.setPixmap(scaled)
                self.lbl_ia_feed.setText("")
            elif self.lbl_ia_feed.pixmap() is None:
                self.lbl_ia_feed.setText(
                    "Feed da IA indisponível.\nInicie uma rota com implemento IA."
                )

        try:
            status_cache_path = getattr(
                self,
                "ia_status_cache_path_active",
                "",
            ) or self.ia_status_cache_path
            if not os.path.exists(status_cache_path):
                return
            stat = os.stat(status_cache_path)
            if stat.st_mtime <= self._ia_status_cache_mtime:
                return
            self._ia_status_cache_mtime = stat.st_mtime

            with open(status_cache_path, "r", encoding="utf-8") as f:
                status = json.load(f)
            if not isinstance(status, dict):
                return
            self.ia_status_snapshot = status
        except Exception:
            return

        status = self.ia_status_snapshot
        if hasattr(self, "lbl_ia_pragas"):
            self.lbl_ia_pragas.setText(str(status.get("deteccoes_total", 0)))
        if hasattr(self, "lbl_ia_modo"):
            self.lbl_ia_modo.setText(
                self._modo_confianca_ptbr(status.get("confidence_mode", "economico"))
            )
        if hasattr(self, "lbl_ia_doenca"):
            self.lbl_ia_doenca.setText(
                self._doenca_ptbr(status.get("doenca_dominante", "Nao_Detectada"))
            )
        if hasattr(self, "lbl_ia_nivel"):
            try:
                self.lbl_ia_nivel.setText(f"{float(status.get('nivel_infestacao_m2', 0.0)):.4f}")
            except Exception:
                self.lbl_ia_nivel.setText("0.0000")
        if hasattr(self, "lbl_ia_litros"):
            try:
                litros = float(status.get("litros_economizados_l", 0.0))
                self.lbl_ia_litros.setText(f"{litros:.3f} L")
            except Exception:
                self.lbl_ia_litros.setText("0.000 L")
        if hasattr(self, "lbl_ia_recomendacao"):
            self.lbl_ia_recomendacao.setText(
                self._bool_ptbr(status.get("spray_recommendation", False))
            )
        if hasattr(self, "lbl_ia_class_counts"):
            self.lbl_ia_class_counts.setText(
                self._format_ia_class_counts(status.get("class_counts", {}))
            )
        if hasattr(self, "lbl_ia_unmapped"):
            unmapped_total = 0
            try:
                unmapped_total = int(status.get("unmapped_detections_total", 0))
            except Exception:
                unmapped_total = 0
            if unmapped_total > 0:
                details = self._format_ia_unmapped_labels(status.get("unmapped_labels", {}))
                detail_txt = f" | {details}" if details else ""
                self.lbl_ia_unmapped.setText(
                    f"{unmapped_total} detecção(ões) não mapeada(s){detail_txt}"
                )
                self.lbl_ia_unmapped.setStyleSheet("color: #ff8a80; font-weight: bold;")
            else:
                self.lbl_ia_unmapped.setText("Mapeamento OK")
                self.lbl_ia_unmapped.setStyleSheet("color: #9ccc65; font-weight: bold;")

    def _log_treino_ia(self, texto, color="#80cbc4"):
        self.log(f"[TREINO_IA] {texto}", color)

    def _set_treino_status(self, texto, color="#b0bec5"):
        if hasattr(self, "lbl_treino_status"):
            self.lbl_treino_status.setText(texto)
            self.lbl_treino_status.setStyleSheet(f"color: {color}; font-size: 11px;")

    def _set_treino_botoes_execucao_enabled(self, enabled):
        if hasattr(self, "btn_treino_smoke"):
            self.btn_treino_smoke.setEnabled(bool(enabled))
        if hasattr(self, "btn_treino_exec"):
            self.btn_treino_exec.setEnabled(bool(enabled))

    def _is_yolo_job_running(self):
        if self._yolo_train_running:
            return True
        for nome in ("YOLO_SMOKE", "YOLO_TREINO"):
            proc = self.processos.get(nome)
            if proc is not None and proc.state() != QProcess.NotRunning:
                return True
        return False

    def _current_session_id_treino(self):
        session_id = (
            self.input_treino_session_id.text().strip()
            if hasattr(self, "input_treino_session_id")
            else ""
        )
        if not session_id:
            session_id = self._gerar_session_id()
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", session_id):
            session_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", session_id)
            session_id = session_id.strip("._-") or self._gerar_session_id()
        if hasattr(self, "input_treino_session_id"):
            self.input_treino_session_id.setText(session_id)
        return session_id

    def _ensure_dataset_dirs(self):
        try:
            dirs = [
                os.path.join(self.dataset_root, "raw"),
                os.path.join(self.dataset_root, "images", "train"),
                os.path.join(self.dataset_root, "images", "val"),
                os.path.join(self.dataset_root, "images", "test"),
                os.path.join(self.dataset_root, "labels", "train"),
                os.path.join(self.dataset_root, "labels", "val"),
                os.path.join(self.dataset_root, "labels", "test"),
            ]
            for directory in dirs:
                os.makedirs(directory, exist_ok=True)
            if hasattr(self, "input_treino_dataset_root"):
                self.input_treino_dataset_root.setText(self.dataset_root)
            if hasattr(self, "input_treino_data_yaml"):
                self.input_treino_data_yaml.setText(self.dataset_data_yaml)
            return True
        except Exception as e:
            self._log_treino_ia(f"Falha ao preparar pastas do dataset: {e}", "#ff5555")
            self._set_treino_status("Erro ao preparar estrutura do dataset.", "#ff8a80")
            return False

    def _session_dirs(self, session_id):
        raw_session_dir = os.path.join(self.dataset_root, "raw", session_id)
        return (
            os.path.join(raw_session_dir, "images"),
            os.path.join(raw_session_dir, "labels"),
        )

    def _list_image_files(self, directory):
        if not os.path.isdir(directory):
            return []
        files = []
        for pattern in ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp"):
            files.extend(glob(os.path.join(directory, pattern)))
            files.extend(glob(os.path.join(directory, pattern.upper())))
        return sorted({p for p in files if os.path.isfile(p)})

    def _safe_copy_with_suffix(self, src_path, dst_dir):
        os.makedirs(dst_dir, exist_ok=True)
        name = os.path.basename(src_path)
        stem, ext = os.path.splitext(name)
        dst_path = os.path.join(dst_dir, name)
        duplicated = False
        idx = 1
        while os.path.exists(dst_path):
            duplicated = True
            dst_name = f"{stem}_dup{idx:03d}{ext}"
            dst_path = os.path.join(dst_dir, dst_name)
            idx += 1
        shutil.copy2(src_path, dst_path)
        return dst_path, duplicated

    def _validate_label_format(self, label_path):
        try:
            with open(label_path, "r", encoding="utf-8", errors="ignore") as f:
                lines = f.read().splitlines()
        except Exception as e:
            return False, f"Falha ao ler arquivo: {e}"

        for idx, raw_line in enumerate(lines, start=1):
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 5:
                return False, f"Linha {idx}: esperado 5 colunas, recebido {len(parts)}."
            try:
                class_id = int(parts[0])
                if class_id < 0:
                    return False, f"Linha {idx}: class_id negativo."
            except Exception:
                return False, f"Linha {idx}: class_id inválido."

            try:
                x_center, y_center, width, height = [float(v) for v in parts[1:]]
            except Exception:
                return False, f"Linha {idx}: coordenadas não numéricas."

            for valor, nome in (
                (x_center, "x_center"),
                (y_center, "y_center"),
                (width, "width"),
                (height, "height"),
            ):
                if valor < 0.0 or valor > 1.0:
                    return False, f"Linha {idx}: {nome} fora de [0,1]."
            if width <= 0.0 or height <= 0.0:
                return False, f"Linha {idx}: width/height devem ser > 0."

        return True, ""

    def _count_split(self, split_name):
        img_dir = os.path.join(self.dataset_root, "images", split_name)
        lbl_dir = os.path.join(self.dataset_root, "labels", split_name)
        img_files = self._list_image_files(img_dir)
        lbl_files = sorted(glob(os.path.join(lbl_dir, "*.txt"))) if os.path.isdir(lbl_dir) else []

        img_stems = {os.path.splitext(os.path.basename(p))[0] for p in img_files}
        lbl_stems = {os.path.splitext(os.path.basename(p))[0] for p in lbl_files}
        pairs_ok = sorted(img_stems & lbl_stems)
        imgs_sem_label = sorted(img_stems - lbl_stems)
        labels_sem_img = sorted(lbl_stems - img_stems)

        return {
            "split": split_name,
            "imgs": len(img_files),
            "lbls": len(lbl_files),
            "pairs_ok": len(pairs_ok),
            "imgs_sem_label": imgs_sem_label,
            "labels_sem_img": labels_sem_img,
        }

    def _normalizar_nome_classe(self, name):
        return re.sub(r"[^a-z0-9]+", "", str(name).strip().lower())

    def _validar_nomes_classes(self, class_names):
        if not class_names:
            return False, "Nenhuma classe encontrada para atualizar o data.yaml."

        genericos_bloqueados = {
            "objeto",
            "objetos",
            "item",
            "items",
            "thing",
            "things",
        }
        genericos_permitidos = {
            "object",
            "objects",
        }
        vistos = {}
        for idx, raw_name in enumerate(class_names):
            name = str(raw_name).strip()
            if not name:
                return False, f"Classe vazia detectada no índice {idx}."

            norm = self._normalizar_nome_classe(name)
            if not norm:
                return False, f"Classe inválida no índice {idx}: '{raw_name}'."
            if norm in genericos_bloqueados:
                return False, (
                    f"Classe genérica não permitida: '{name}'. "
                    "Exceção permitida apenas para 'object/objects'. "
                    "Renomeie no dataset de origem antes de importar."
                )
            if norm in genericos_permitidos:
                pass
            if norm in vistos:
                return False, (
                    f"Classes duplicadas detectadas: '{vistos[norm]}' e '{name}'. "
                    "Corrija no dataset de origem antes de importar."
                )
            vistos[norm] = name
        return True, ""

    def _quote_yaml(self, text):
        return "'" + str(text).replace("'", "''") + "'"

    def _ler_names_data_yaml(self):
        if not os.path.isfile(self.dataset_data_yaml):
            return None, f"Arquivo data.yaml não encontrado: {self.dataset_data_yaml}"

        names = {}
        in_names = False
        try:
            with open(self.dataset_data_yaml, "r", encoding="utf-8", errors="ignore") as f:
                for raw_line in f:
                    if not raw_line.strip() or raw_line.lstrip().startswith("#"):
                        continue

                    stripped = raw_line.strip()
                    if not in_names:
                        if stripped == "names:":
                            in_names = True
                        continue

                    if re.match(r"^[A-Za-z_][A-Za-z0-9_]*\s*:", stripped):
                        break
                    m = re.match(r"^(\d+)\s*:\s*(.+)$", stripped)
                    if not m:
                        continue
                    idx = int(m.group(1))
                    value = m.group(2).strip()
                    if (
                        (value.startswith("'") and value.endswith("'"))
                        or (value.startswith('"') and value.endswith('"'))
                    ):
                        value = value[1:-1]
                    value = value.replace("''", "'").strip()
                    names[idx] = value
        except Exception as e:
            return None, f"Falha ao ler data.yaml: {e}"

        if not names:
            return None, "Bloco 'names' ausente ou vazio no data.yaml."

        max_idx = max(names.keys())
        ordered = []
        for idx in range(max_idx + 1):
            if idx not in names:
                return None, f"Classes no data.yaml não contíguas. Índice ausente: {idx}."
            ordered.append(names[idx])
        return ordered, ""

    def _atualizar_data_yaml_classes(self, class_names):
        ok, msg = self._validar_nomes_classes(class_names)
        if not ok:
            return False, msg

        try:
            os.makedirs(os.path.dirname(self.dataset_data_yaml), exist_ok=True)
            lines = [
                f"path: {self.dataset_root}",
                "train: images/train",
                "val: images/val",
                "test: images/test",
                "names:",
            ]
            for idx, name in enumerate(class_names):
                lines.append(f"  {idx}: {self._quote_yaml(name)}")
            with open(self.dataset_data_yaml, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
            if hasattr(self, "input_treino_data_yaml"):
                self.input_treino_data_yaml.setText(self.dataset_data_yaml)
            self._log_treino_ia(
                f"data.yaml atualizado com {len(class_names)} classe(s): {', '.join(class_names)}",
                "#9ccc65",
            )
            return True, ""
        except Exception as e:
            return False, f"Falha ao atualizar data.yaml: {e}"

    def _parse_coco_annotations(self, json_path):
        try:
            with open(json_path, "r", encoding="utf-8", errors="ignore") as f:
                data = json.load(f)
        except Exception as e:
            return None, f"Falha ao ler JSON COCO ({json_path}): {e}"

        if not isinstance(data, dict):
            return None, f"JSON COCO inválido em {json_path}."

        images = data.get("images", [])
        annotations = data.get("annotations", [])
        categories = data.get("categories", [])
        if not isinstance(
            images,
            list,
        ) or not isinstance(annotations, list) or not isinstance(categories, list):
            return None, f"Estrutura COCO inválida em {json_path}."

        image_map = {}
        for item in images:
            try:
                image_id = int(item.get("id"))
                file_name = str(item.get("file_name", "")).strip()
                width = int(item.get("width", 0))
                height = int(item.get("height", 0))
            except Exception:
                return None, f"Registro de imagem inválido em {json_path}."
            if not file_name:
                return None, f"Imagem sem file_name em {json_path}."
            if width <= 0 or height <= 0:
                return None, f"Imagem '{file_name}' com width/height inválidos em {json_path}."
            image_map[image_id] = {
                "file_name": file_name,
                "width": width,
                "height": height,
            }

        category_map = {}
        for item in categories:
            try:
                cat_id = int(item.get("id"))
                cat_name = str(item.get("name", "")).strip()
            except Exception:
                return None, f"Categoria inválida em {json_path}."
            if not cat_name:
                return None, f"Categoria sem nome em {json_path}."
            category_map[cat_id] = cat_name

        anns_by_image = {}
        used_category_ids = set()
        for ann in annotations:
            try:
                image_id = int(ann.get("image_id"))
                category_id = int(ann.get("category_id"))
            except Exception:
                return None, f"Anotação com image_id/category_id inválido em {json_path}."

            if image_id not in image_map:
                return None, (
                    f"Anotação referencia image_id inexistente ({image_id}) em {json_path}."
                )
            if category_id not in category_map:
                return None, (
                    f"Anotação referencia category_id inexistente ({category_id}) em {json_path}."
                )

            bbox = ann.get("bbox")
            if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
                return None, f"Anotação sem bbox válida em {json_path}."
            try:
                bbox = [float(v) for v in bbox]
            except Exception:
                return None, f"bbox não numérica em {json_path}."

            anns_by_image.setdefault(image_id, []).append(
                {
                    "category_id": category_id,
                    "bbox": bbox,
                }
            )
            used_category_ids.add(category_id)

        return {
            "image_map": image_map,
            "category_map": category_map,
            "annotations_by_image": anns_by_image,
            "used_category_ids": sorted(used_category_ids),
        }, ""

    def _converter_coco_para_yolo_detect(
        self,
        coco_parsed,
        images_root_dir,
        out_images_dir,
        out_labels_dir,
        class_id_map,
    ):
        stats = {
            "images_total": 0,
            "labels_total": 0,
            "annotations_total": 0,
        }

        os.makedirs(out_images_dir, exist_ok=True)
        os.makedirs(out_labels_dir, exist_ok=True)

        for image_id in sorted(coco_parsed["image_map"].keys()):
            image_info = coco_parsed["image_map"][image_id]
            file_name = image_info["file_name"]
            img_w = int(image_info["width"])
            img_h = int(image_info["height"])

            anns = coco_parsed["annotations_by_image"].get(image_id, [])
            if not anns:
                return False, stats, (
                    f"Imagem sem anotação detectada: '{file_name}'. "
                    "Treino estrito não permite imagens sem label."
                )

            src_img = os.path.join(images_root_dir, file_name)
            if not os.path.isfile(src_img):
                src_img = os.path.join(images_root_dir, os.path.basename(file_name))
            if not os.path.isfile(src_img):
                return False, stats, f"Imagem referenciada não encontrada: '{file_name}'."

            dst_img, _dup = self._safe_copy_with_suffix(src_img, out_images_dir)
            dst_stem = os.path.splitext(os.path.basename(dst_img))[0]
            dst_lbl = os.path.join(out_labels_dir, f"{dst_stem}.txt")

            lines = []
            for ann in anns:
                cat_id = int(ann["category_id"])
                if cat_id not in class_id_map:
                    return False, stats, (
                            f"Categoria {cat_id} não mapeada durante conversão da imagem"
                            f"'{file_name}'."
                    )

                x, y, w, h = ann["bbox"]
                x1 = max(0.0, float(x))
                y1 = max(0.0, float(y))
                x2 = min(float(img_w), float(x) + float(w))
                y2 = min(float(img_h), float(y) + float(h))
                ww = x2 - x1
                hh = y2 - y1
                if ww <= 0.0 or hh <= 0.0:
                    return False, stats, (
                        f"bbox inválida para imagem '{file_name}' (w/h <= 0 após recorte)."
                    )

                xc = ((x1 + x2) / 2.0) / float(img_w)
                yc = ((y1 + y2) / 2.0) / float(img_h)
                wn = ww / float(img_w)
                hn = hh / float(img_h)
                new_class_id = int(class_id_map[cat_id])
                lines.append(f"{new_class_id} {xc:.6f} {yc:.6f} {wn:.6f} {hn:.6f}")

            if not lines:
                return False, stats, (
                    f"Imagem '{file_name}' não gerou labels válidas. Treino estrito bloqueado."
                )

            with open(dst_lbl, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")

            ok_label, detalhe = self._validate_label_format(dst_lbl)
            if not ok_label:
                return False, stats, f"Label inválida gerada para '{file_name}': {detalhe}"

            stats["images_total"] += 1
            stats["labels_total"] += 1
            stats["annotations_total"] += len(lines)

        return True, stats, ""

    def _limpar_diretorio(self, directory):
        if not os.path.isdir(directory):
            return
        for name in os.listdir(directory):
            path = os.path.join(directory, name)
            if os.path.isfile(path) or os.path.islink(path):
                os.remove(path)
            elif os.path.isdir(path):
                shutil.rmtree(path)

    def _rebuild_split_from_all_raw_sessions(self):
        if not self._ensure_dataset_dirs():
            return False, {}, "Falha ao preparar estrutura de dataset."

        raw_root = os.path.join(self.dataset_root, "raw")
        session_ids = sorted(
            [
                s
                for s in os.listdir(raw_root)
                if os.path.isdir(os.path.join(raw_root, s))
            ]
        )
        if not session_ids:
            return False, {}, "Nenhuma sessão encontrada em raw/."

        pares = []
        for session_id in session_ids:
            session_images_dir, session_labels_dir = self._session_dirs(session_id)
            img_files = self._list_image_files(session_images_dir)
            lbl_files = sorted(glob(os.path.join(session_labels_dir, "*.txt")))

            if not img_files and not lbl_files:
                continue

            image_by_stem = {}
            for img in img_files:
                stem = os.path.splitext(os.path.basename(img))[0]
                if stem in image_by_stem:
                    return False, {}, (
                        f"Sessão '{session_id}' contém imagens com stem duplicado: '{stem}'."
                    )
                image_by_stem[stem] = img

            label_by_stem = {}
            for lbl in lbl_files:
                stem = os.path.splitext(os.path.basename(lbl))[0]
                if stem in label_by_stem:
                    return False, {}, (
                        f"Sessão '{session_id}' contém labels com stem duplicado: '{stem}'."
                    )
                label_by_stem[stem] = lbl

            missing = sorted(set(image_by_stem.keys()) - set(label_by_stem.keys()))
            extra = sorted(set(label_by_stem.keys()) - set(image_by_stem.keys()))
            if missing or extra:
                detalhe = []
                if missing:
                    detalhe.append(f"imgs sem label: {len(missing)}")
                if extra:
                    detalhe.append(f"labels sem img: {len(extra)}")
                return False, {}, (
                    f"Sessão '{session_id}' inconsistente ({', '.join(detalhe)}). "
                    "Treino estrito exige pareamento total."
                )

            for stem in sorted(image_by_stem.keys()):
                lbl_path = label_by_stem[stem]
                ok, detalhe = self._validate_label_format(lbl_path)
                if not ok:
                    return False, {}, (
                            f"Sessão '{session_id}', label '{os.path.basename(lbl_path)}'"
                            f"inválida: {detalhe}"
                    )
                try:
                    with open(lbl_path, "r", encoding="utf-8", errors="ignore") as f:
                        non_empty = [ln for ln in f.read().splitlines() if ln.strip()]
                except Exception as e:
                    return False, {}, (
                            f"Falha ao ler label '{os.path.basename(lbl_path)}' da sessão"
                            f"'{session_id}': {e}"
                    )
                if not non_empty:
                    return False, {}, (
                            f"Label vazia detectada na sessão '{session_id}'"
                            f"({os.path.basename(lbl_path)})."
                    )

                pares.append((session_id, image_by_stem[stem], lbl_path))

        total = len(pares)
        if total < 2:
            return False, {}, (
                "Dataset insuficiente para split estrito: mínimo de 2 imagens com labels "
                "(ao menos 1 para train e 1 para val)."
            )

        rng = random.Random(42)
        rng.shuffle(pares)

        n_train = max(1, math.floor(0.8 * total))
        n_val = max(1, math.floor(0.1 * total))
        if n_train + n_val > total:
            n_val = 1
            n_train = total - 1

        split_map = {
            "train": pares[:n_train],
            "val": pares[n_train:n_train + n_val],
            "test": pares[n_train + n_val:],
        }

        try:
            self._limpar_split_dirs()
        except Exception as e:
            return False, {}, f"Falha ao limpar splits antes do rebuild: {e}"

        for split_name, items in split_map.items():
            split_img_dir = os.path.join(self.dataset_root, "images", split_name)
            split_lbl_dir = os.path.join(self.dataset_root, "labels", split_name)
            os.makedirs(split_img_dir, exist_ok=True)
            os.makedirs(split_lbl_dir, exist_ok=True)

            for _session_id, img_src, lbl_src in items:
                img_dst, _dup = self._safe_copy_with_suffix(img_src, split_img_dir)
                stem_dst = os.path.splitext(os.path.basename(img_dst))[0]
                lbl_dst = os.path.join(split_lbl_dir, f"{stem_dst}.txt")
                shutil.copy2(lbl_src, lbl_dst)

        summary = {
            "total_pairs": total,
            "train": len(split_map["train"]),
            "val": len(split_map["val"]),
            "test": len(split_map["test"]),
            "sessions_used": len(session_ids),
        }
        return True, summary, ""

    def _validar_dataset_para_treino_strict(self):
        names, err_names = self._ler_names_data_yaml()
        if err_names:
            return False, [err_names], {}
        class_count = len(names)
        if class_count <= 0:
            return False, ["data.yaml sem classes válidas."], {}

        errors = []

        def add_error(msg):
            if len(errors) < 40:
                errors.append(msg)

        split_stats = {}
        total_annotations = 0
        total_imgs = 0
        for split in ("train", "val", "test"):
            img_dir = os.path.join(self.dataset_root, "images", split)
            lbl_dir = os.path.join(self.dataset_root, "labels", split)
            img_files = self._list_image_files(img_dir)
            lbl_files = sorted(glob(os.path.join(lbl_dir, "*.txt")))
            total_imgs += len(img_files)

            img_stems = {os.path.splitext(os.path.basename(p))[0] for p in img_files}
            lbl_stems = {os.path.splitext(os.path.basename(p))[0] for p in lbl_files}
            missing = sorted(img_stems - lbl_stems)
            extra = sorted(lbl_stems - img_stems)
            if missing:
                add_error(f"[{split}] imagens sem label: {len(missing)}")
            if extra:
                add_error(f"[{split}] labels sem imagem: {len(extra)}")

            ann_split = 0
            for lbl_path in lbl_files:
                ok_fmt, detalhe = self._validate_label_format(lbl_path)
                if not ok_fmt:
                    add_error(f"[{split}] {os.path.basename(lbl_path)} inválida: {detalhe}")
                    continue
                try:
                    with open(lbl_path, "r", encoding="utf-8", errors="ignore") as f:
                        lines = [ln.strip() for ln in f.read().splitlines() if ln.strip()]
                except Exception as e:
                    add_error(f"[{split}] falha ao ler {os.path.basename(lbl_path)}: {e}")
                    continue

                if not lines:
                    add_error(f"[{split}] label vazia não permitida: {os.path.basename(lbl_path)}")
                    continue

                for i, line in enumerate(lines, start=1):
                    parts = line.split()
                    try:
                        class_id = int(parts[0])
                    except Exception:
                        add_error((
                            f"[{split}] class_id inválido em"
                            f"{os.path.basename(lbl_path)}:{i}"
                        ))
                        continue
                    if class_id < 0 or class_id >= class_count:
                        add_error(
                                f"[{split}] class_id fora do intervalo em"
                                f"{os.path.basename(lbl_path)}:{i} "
                                f"(id={class_id}, classes={class_count})"
                        )
                    ann_split += 1

            split_stats[split] = {
                "images": len(img_files),
                "labels": len(lbl_files),
                "annotations": ann_split,
            }
            total_annotations += ann_split

        if split_stats["train"]["images"] <= 0:
            add_error("Split train vazio.")
        if split_stats["val"]["images"] <= 0:
            add_error("Split val vazio.")
        if total_imgs <= 0:
            add_error("Dataset sem imagens.")
        if total_annotations <= 0:
            add_error("Dataset sem anotações válidas.")

        ok = len(errors) == 0
        summary = {
            "class_count": class_count,
            "split_stats": split_stats,
            "total_annotations": total_annotations,
        }
        return ok, errors, summary

    def importar_zip_roboflow_coco_treino_ia(self):
        self._importar_zip_roboflow_coco()

    def _importar_zip_roboflow_coco(self):
        if not self._ensure_dataset_dirs():
            return

        zip_path, _ = QFileDialog.getOpenFileName(
            self,
            "Selecionar ZIP Roboflow COCO",
            self.home,
            "Arquivos ZIP (*.zip)",
        )
        if not zip_path:
            return

        if not zipfile.is_zipfile(zip_path):
            QMessageBox.warning(self, "Treino IA", "Arquivo selecionado não é um ZIP válido.")
            return

        base_session = self._current_session_id_treino()
        session_id = f"{base_session}_zip_{time.strftime('%Y%m%d_%H%M%S')}"
        raw_session_dir = os.path.join(self.dataset_root, "raw", session_id)
        suffix = 1
        while os.path.exists(raw_session_dir):
            session_id = f"{base_session}_zip_{time.strftime('%Y%m%d_%H%M%S')}_{suffix:02d}"
            raw_session_dir = os.path.join(self.dataset_root, "raw", session_id)
            suffix += 1

        session_images_dir, session_labels_dir = self._session_dirs(session_id)
        os.makedirs(session_images_dir, exist_ok=True)
        os.makedirs(session_labels_dir, exist_ok=True)

        report = {
            "source_zip": os.path.abspath(zip_path),
            "session_id": session_id,
            "status": "running",
            "errors": [],
            "warnings": [],
            "classes": [],
            "stats": {},
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        report_path = os.path.join(raw_session_dir, "import_report.json")

        try:
            with tempfile.TemporaryDirectory(prefix="coco_import_") as tmp_dir:
                with zipfile.ZipFile(zip_path, "r") as zf:
                    zf.extractall(tmp_dir)

                annotation_files = []
                for root, _dirs, files in os.walk(tmp_dir):
                    for name in files:
                        if name == "_annotations.coco.json":
                            annotation_files.append(os.path.join(root, name))
                annotation_files = sorted(annotation_files)
                if not annotation_files:
                    raise RuntimeError(
                        "ZIP inválido para esta V1: não foi encontrado '_annotations.coco.json'."
                    )

                class_names = []
                class_name_to_id = {}
                parsed_jobs = []
                for ann_path in annotation_files:
                    parsed, err = self._parse_coco_annotations(ann_path)
                    if err:
                        raise RuntimeError(err)

                    # Rejeita nomes duplicados no mesmo arquivo de anotação.
                    local_norm_to_name = {}
                    for cat_id in parsed["used_category_ids"]:
                        name = str(parsed["category_map"][cat_id]).strip()
                        norm = self._normalizar_nome_classe(name)
                        if norm in local_norm_to_name:
                            raise RuntimeError(
                                f"Categorias duplicadas/conflitantes em '{ann_path}': "
                                f"'{local_norm_to_name[norm]}' e '{name}'."
                            )
                        local_norm_to_name[norm] = name

                    local_class_map = {}
                    for cat_id in parsed["used_category_ids"]:
                        class_name = str(parsed["category_map"][cat_id]).strip()
                        if class_name not in class_name_to_id:
                            class_name_to_id[class_name] = len(class_names)
                            class_names.append(class_name)
                        local_class_map[cat_id] = class_name_to_id[class_name]

                    images_root = os.path.dirname(ann_path)
                    parsed_jobs.append((parsed, images_root, local_class_map))

                ok_names, msg_names = self._validar_nomes_classes(class_names)
                if not ok_names:
                    raise RuntimeError(msg_names)

                object_like = sorted(
                    {
                        str(name).strip()
                        for name in class_names
                        if self._normalizar_nome_classe(name) in {"object", "objects"}
                    }
                )
                if object_like:
                    warn_msg = (
                        "Classe genérica permitida detectada na importação: "
                        + ", ".join(object_like)
                        + "."
                    )
                    report["warnings"].append(warn_msg)
                    self._log_treino_ia(warn_msg, "#ffb300")

                aggregate = {
                    "images_total": 0,
                    "labels_total": 0,
                    "annotations_total": 0,
                    "annotation_files": len(parsed_jobs),
                }
                for parsed, images_root, local_class_map in parsed_jobs:
                    ok_conv, stats_conv, err_conv = self._converter_coco_para_yolo_detect(
                        coco_parsed=parsed,
                        images_root_dir=images_root,
                        out_images_dir=session_images_dir,
                        out_labels_dir=session_labels_dir,
                        class_id_map=local_class_map,
                    )
                    if not ok_conv:
                        raise RuntimeError(err_conv)
                    for key in ("images_total", "labels_total", "annotations_total"):
                        aggregate[key] += int(stats_conv.get(key, 0))

                ok_yaml, msg_yaml = self._atualizar_data_yaml_classes(class_names)
                if not ok_yaml:
                    raise RuntimeError(msg_yaml)

                ok_rebuild, summary_rebuild, msg_rebuild = (
                    self._rebuild_split_from_all_raw_sessions()
                )
                if not ok_rebuild:
                    raise RuntimeError(msg_rebuild)

                ok_strict, strict_errors, strict_summary = (
                    self._validar_dataset_para_treino_strict()
                )
                if not ok_strict:
                    raise RuntimeError(
                        "Validação estrita falhou após importação: "
                        + "; ".join(strict_errors[:5])
                    )

                report["status"] = "ok"
                report["classes"] = class_names
                report["stats"] = {
                    "import": aggregate,
                    "rebuild_split": summary_rebuild,
                    "strict_validation": strict_summary,
                }
                split_counts = (
                    f"{summary_rebuild.get('train', 0)}/"
                    f"{summary_rebuild.get('val', 0)}/"
                    f"{summary_rebuild.get('test', 0)}"
                )
                self._log_treino_ia(
                    "Importação ZIP concluída | "
                    f"imagens={aggregate['images_total']} labels={aggregate['labels_total']} "
                    f"classes={len(class_names)} split(train/val/test)={split_counts}",
                    "#9ccc65",
                )
                self._set_treino_status(
                    (
                        f"ZIP importado com sucesso. Sessão={session_id} | "
                        f"classes={len(class_names)} | "
                        f"train={summary_rebuild.get('train', 0)} "
                        f"val={summary_rebuild.get('val', 0)}"
                    ),
                    "#9ccc65",
                )
                QMessageBox.information(
                    self,
                    "Treino IA",
                    "Importação concluída com sucesso.\n\n"
                    f"Sessão: {session_id}\n"
                    f"Classes: {len(class_names)}\n"
                    f"Imagens importadas: {aggregate['images_total']}\n"
                    f"Split: train={summary_rebuild.get('train', 0)} "
                    f"val={summary_rebuild.get('val', 0)} test={summary_rebuild.get('test', 0)}",
                )
        except Exception as e:
            erro = str(e)
            report["status"] = "error"
            report["errors"].append(erro)
            self._limpar_diretorio(session_images_dir)
            self._limpar_diretorio(session_labels_dir)
            self._log_treino_ia(f"Falha na importação do ZIP: {erro}", "#ff5555")
            self._set_treino_status(
                "Falha ao importar ZIP. Verifique o relatório da sessão.",
                "#ff8a80",
            )
            QMessageBox.warning(self, "Treino IA", f"Falha ao importar ZIP:\n{erro}")
        finally:
            try:
                os.makedirs(raw_session_dir, exist_ok=True)
                with open(report_path, "w", encoding="utf-8") as f:
                    json.dump(report, f, ensure_ascii=False, indent=2)
            except Exception as e:
                self._log_treino_ia(f"Falha ao salvar import_report.json: {e}", "#ff5555")

    def importar_pasta_yolo_treino_ia(self):
        if not self._ensure_dataset_dirs():
            return

        source_root = QFileDialog.getExistingDirectory(
            self,
            "Selecionar Pasta de Dataset YOLO (imagens + labels)",
            self.home,
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if not source_root:
            return

        images_src = os.path.join(source_root, "images")
        labels_src = os.path.join(source_root, "labels")
        if not os.path.isdir(images_src):
            images_src = source_root
        if not os.path.isdir(labels_src):
            labels_src = source_root

        img_files = self._list_image_files(images_src)
        lbl_files = sorted(glob(os.path.join(labels_src, "*.txt")))
        if not img_files:
            QMessageBox.warning(
                self,
                "Treino IA",
                "Nenhuma imagem encontrada na pasta selecionada.",
            )
            return
        if not lbl_files:
            QMessageBox.warning(
                self,
                "Treino IA",
                "Nenhum arquivo .txt de label encontrado na pasta selecionada.",
            )
            return

        image_by_stem = {}
        for img in img_files:
            stem = os.path.splitext(os.path.basename(img))[0]
            if stem in image_by_stem:
                QMessageBox.warning(
                    self, "Treino IA", f"Imagens com stem duplicado detectadas na pasta: {stem}"
                )
                return
            image_by_stem[stem] = img

        label_by_stem = {}
        for lbl in lbl_files:
            stem = os.path.splitext(os.path.basename(lbl))[0]
            if stem in label_by_stem:
                QMessageBox.warning(
                    self, "Treino IA", f"Labels com stem duplicado detectadas na pasta: {stem}"
                )
                return
            label_by_stem[stem] = lbl

        missing = sorted(set(image_by_stem.keys()) - set(label_by_stem.keys()))
        extra = sorted(set(label_by_stem.keys()) - set(image_by_stem.keys()))
        if missing or extra:
            msg = []
            if missing:
                msg.append(f"Imagens sem label: {len(missing)}")
            if extra:
                msg.append(f"Labels sem imagem: {len(extra)}")
            QMessageBox.warning(
                self,
                "Treino IA",
                "Pareamento inválido na pasta importada (treino estrito).\n" + "\n".join(msg),
            )
            return

        base_session = self._current_session_id_treino()
        session_id = f"{base_session}_pasta_{time.strftime('%Y%m%d_%H%M%S')}"
        raw_session_dir = os.path.join(self.dataset_root, "raw", session_id)
        suffix = 1
        while os.path.exists(raw_session_dir):
            session_id = f"{base_session}_pasta_{time.strftime('%Y%m%d_%H%M%S')}_{suffix:02d}"
            raw_session_dir = os.path.join(self.dataset_root, "raw", session_id)
            suffix += 1
        session_images_dir, session_labels_dir = self._session_dirs(session_id)
        os.makedirs(session_images_dir, exist_ok=True)
        os.makedirs(session_labels_dir, exist_ok=True)

        copied = 0
        try:
            for stem in sorted(image_by_stem.keys()):
                lbl_src = label_by_stem[stem]
                ok, detalhe = self._validate_label_format(lbl_src)
                if not ok:
                    raise RuntimeError(
                            f"Label inválida na pasta importada"
                            f"({os.path.basename(lbl_src)}): {detalhe}"
                    )
                with open(lbl_src, "r", encoding="utf-8", errors="ignore") as f:
                    non_empty = [ln for ln in f.read().splitlines() if ln.strip()]
                if not non_empty:
                    raise RuntimeError(
                        f"Label vazia não permitida ({os.path.basename(lbl_src)})."
                    )

                img_dst, _dup = self._safe_copy_with_suffix(
                    image_by_stem[stem],
                    session_images_dir,
                )
                stem_dst = os.path.splitext(os.path.basename(img_dst))[0]
                lbl_dst = os.path.join(session_labels_dir, f"{stem_dst}.txt")
                shutil.copy2(lbl_src, lbl_dst)
                copied += 1

            ok_rebuild, summary_rebuild, msg_rebuild = self._rebuild_split_from_all_raw_sessions()
            if not ok_rebuild:
                raise RuntimeError(msg_rebuild)

            ok_strict, strict_errors, _strict_summary = self._validar_dataset_para_treino_strict()
            if not ok_strict:
                raise RuntimeError("Validação estrita falhou: " + "; ".join(strict_errors[:5]))

            report = {
                "source_folder": os.path.abspath(source_root),
                "session_id": session_id,
                "status": "ok",
                "stats": {
                    "copied_pairs": copied,
                    "rebuild_split": summary_rebuild,
                },
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            with open(
                os.path.join(raw_session_dir, "import_report.json"),
                "w",
                encoding="utf-8",
            ) as f:
                json.dump(report, f, ensure_ascii=False, indent=2)

            self._log_treino_ia(
                f"Importação de pasta concluída | sessão={session_id} | pares={copied} | "
                f"train={summary_rebuild.get('train', 0)} val={summary_rebuild.get('val', 0)}",
                "#9ccc65",
            )
            self._set_treino_status(
                f"Pasta importada com sucesso. Sessão={session_id} | pares={copied}",
                "#9ccc65",
            )
            QMessageBox.information(
                self,
                "Treino IA",
                "Importação da pasta concluída.\n\n"
                f"Sessão: {session_id}\n"
                f"Pares copiados: {copied}\n"
                f"Split: train={summary_rebuild.get('train', 0)} "
                f"val={summary_rebuild.get('val', 0)} test={summary_rebuild.get('test', 0)}",
            )
        except Exception as e:
            self._limpar_diretorio(session_images_dir)
            self._limpar_diretorio(session_labels_dir)
            report = {
                "source_folder": os.path.abspath(source_root),
                "session_id": session_id,
                "status": "error",
                "errors": [str(e)],
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            try:
                with open(
                    os.path.join(raw_session_dir, "import_report.json"),
                    "w",
                    encoding="utf-8",
                ) as f:
                    json.dump(report, f, ensure_ascii=False, indent=2)
            except Exception:
                pass
            self._log_treino_ia(f"Falha na importação da pasta: {e}", "#ff5555")
            self._set_treino_status("Falha ao importar pasta. Verifique os labels.", "#ff8a80")
            QMessageBox.warning(self, "Treino IA", f"Falha ao importar pasta:\n{e}")

    def subir_imagens_treino_ia(self):
        if not self._ensure_dataset_dirs():
            return

        session_id = self._current_session_id_treino()
        session_images_dir, _session_labels_dir = self._session_dirs(session_id)
        os.makedirs(session_images_dir, exist_ok=True)

        files, _ = QFileDialog.getOpenFileNames(
            self,
            "Selecionar Imagens para Sessão",
            self.home,
            "Imagens (*.jpg *.jpeg *.png *.bmp *.webp)",
        )
        if not files:
            return

        copied = 0
        duplicated = 0
        for file_path in files:
            if not os.path.isfile(file_path):
                continue
            _, is_dup = self._safe_copy_with_suffix(file_path, session_images_dir)
            copied += 1
            if is_dup:
                duplicated += 1

        self._log_treino_ia(
            (
                f"Upload de imagens concluído: {copied} arquivo(s) | duplicatas"
                f"renomeadas: {duplicated} | sessão={session_id}"
            ),
            "#9ccc65",
        )
        self._set_treino_status(
            f"Imagens da sessão em: {session_images_dir}",
            "#9ccc65",
        )

    def subir_labels_treino_ia(self):
        if not self._ensure_dataset_dirs():
            return

        session_id = self._current_session_id_treino()
        _session_images_dir, session_labels_dir = self._session_dirs(session_id)
        os.makedirs(session_labels_dir, exist_ok=True)

        files, _ = QFileDialog.getOpenFileNames(
            self,
            "Selecionar Labels YOLO para Sessão",
            self.home,
            "Labels YOLO (*.txt)",
        )
        if not files:
            return

        copied = 0
        duplicated = 0
        invalidos = []
        for file_path in files:
            if not os.path.isfile(file_path):
                continue
            dst_path, is_dup = self._safe_copy_with_suffix(file_path, session_labels_dir)
            copied += 1
            if is_dup:
                duplicated += 1
            ok, detalhe = self._validate_label_format(dst_path)
            if not ok:
                invalidos.append((os.path.basename(dst_path), detalhe))

        self._log_treino_ia(
            (
                f"Upload de labels concluído: {copied} arquivo(s) | duplicatas"
                f"renomeadas: {duplicated}"
            ),
            "#9ccc65",
        )
        if invalidos:
            self._log_treino_ia(
                (
                    f"Labels inválidos detectados: {len(invalidos)} (exemplo:"
                    f"{invalidos[0][0]} -> {invalidos[0][1]})"
                ),
                "#ffb300",
            )
            self._set_treino_status(
                "Upload concluído com alertas de formato em labels.",
                "#ffcc80",
            )
        else:
            self._set_treino_status(f"Labels da sessão em: {session_labels_dir}", "#9ccc65")

    def _limpar_split_dirs(self):
        for split in ("train", "val", "test"):
            for prefix in ("images", "labels"):
                split_dir = os.path.join(self.dataset_root, prefix, split)
                if not os.path.isdir(split_dir):
                    continue
                for entry in os.listdir(split_dir):
                    entry_path = os.path.join(split_dir, entry)
                    if os.path.islink(entry_path) or os.path.isfile(entry_path):
                        os.remove(entry_path)
                    elif os.path.isdir(entry_path):
                        shutil.rmtree(entry_path)

    def organizar_split_treino_ia(self):
        if not self._ensure_dataset_dirs():
            return

        session_id = self._current_session_id_treino()
        session_images_dir, session_labels_dir = self._session_dirs(session_id)
        img_files = self._list_image_files(session_images_dir)
        if len(img_files) < 3:
            QMessageBox.warning(
                self,
                "Treino IA",
                "A sessão precisa de pelo menos 3 imagens para organizar train/val/test.",
            )
            self._set_treino_status("Sessão sem imagens suficientes para split.", "#ff8a80")
            return

        label_map = {}
        if os.path.isdir(session_labels_dir):
            for lbl_path in sorted(glob(os.path.join(session_labels_dir, "*.txt"))):
                stem = os.path.splitext(os.path.basename(lbl_path))[0]
                if stem in label_map:
                    QMessageBox.warning(
                        self,
                        "Treino IA",
                        f"Sessão contém labels duplicadas por stem: {stem}",
                    )
                    self._set_treino_status("Falha: labels duplicadas na sessão.", "#ff8a80")
                    return
                label_map[stem] = lbl_path

        image_stems = [os.path.splitext(os.path.basename(p))[0] for p in img_files]
        missing_labels = sorted([stem for stem in image_stems if stem not in label_map])
        image_stems_set = set(image_stems)
        labels_sem_img = sorted(
            [stem for stem in label_map.keys() if stem not in image_stems_set]
        )
        if missing_labels or labels_sem_img:
            detalhe = []
            if missing_labels:
                detalhe.append(f"Imagens sem label: {len(missing_labels)}")
            if labels_sem_img:
                detalhe.append(f"Labels sem imagem: {len(labels_sem_img)}")
            QMessageBox.warning(
                self,
                "Treino IA",
                "Split estrito bloqueado: sessão com pareamento inválido.\n" + "\n".join(detalhe),
            )
            self._set_treino_status("Split bloqueado: sessão com imagens sem label.", "#ff8a80")
            return

        for stem in sorted(set(image_stems)):
            lbl_path = label_map.get(stem)
            ok_lbl, detalhe_lbl = self._validate_label_format(lbl_path)
            if not ok_lbl:
                QMessageBox.warning(
                    self,
                    "Treino IA",
                    f"Label inválida na sessão ({os.path.basename(lbl_path)}): {detalhe_lbl}",
                )
                self._set_treino_status("Split bloqueado: label inválida.", "#ff8a80")
                return
            try:
                with open(lbl_path, "r", encoding="utf-8", errors="ignore") as f:
                    non_empty = [ln for ln in f.read().splitlines() if ln.strip()]
            except Exception as e:
                QMessageBox.warning(
                    self,
                    "Treino IA",
                    f"Falha ao ler label da sessão ({os.path.basename(lbl_path)}): {e}",
                )
                self._set_treino_status("Split bloqueado: falha ao ler labels.", "#ff8a80")
                return
            if not non_empty:
                QMessageBox.warning(
                    self,
                    "Treino IA",
                    f"Label vazia não permitida ({os.path.basename(lbl_path)}).",
                )
                self._set_treino_status("Split bloqueado: label vazia.", "#ff8a80")
                return

        confirm = QMessageBox.question(
            self,
            "Treino IA",
            (
                "Esta ação limpará os arquivos atuais de"
                "images/{train,val,test} e labels/{train,val,test}.\\n\\nDeseja"
                "continuar?"
            ),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if confirm != QMessageBox.Yes:
            return

        pares = []
        for img_path in img_files:
            stem = os.path.splitext(os.path.basename(img_path))[0]
            pares.append((img_path, label_map[stem]))

        rng = random.Random(42)
        rng.shuffle(pares)
        total = len(pares)
        n_train = math.floor(0.8 * total)
        n_val = max(1, math.floor(0.1 * total))
        n_test = total - n_train - n_val
        if n_test <= 0:
            if n_train > 1:
                n_train -= 1
                n_test += 1
            elif n_val > 1:
                n_val -= 1
                n_test += 1

        split_map = {
            "train": pares[:n_train],
            "val": pares[n_train:n_train + n_val],
            "test": pares[n_train + n_val:],
        }
        if len(split_map["test"]) == 0 and len(split_map["train"]) > 1:
            split_map["test"].append(split_map["train"].pop())

        try:
            self._limpar_split_dirs()
        except Exception as e:
            QMessageBox.warning(self, "Treino IA", f"Falha ao limpar splits atuais: {e}")
            self._set_treino_status("Falha ao limpar splits anteriores.", "#ff8a80")
            return

        for split_name, split_items in split_map.items():
            split_img_dir = os.path.join(self.dataset_root, "images", split_name)
            split_lbl_dir = os.path.join(self.dataset_root, "labels", split_name)
            os.makedirs(split_img_dir, exist_ok=True)
            os.makedirs(split_lbl_dir, exist_ok=True)
            for img_src, lbl_src in split_items:
                img_dst, _dup = self._safe_copy_with_suffix(img_src, split_img_dir)
                stem_dst = os.path.splitext(os.path.basename(img_dst))[0]
                lbl_dst = os.path.join(split_lbl_dir, f"{stem_dst}.txt")
                shutil.copy2(lbl_src, lbl_dst)

        contagens = [self._count_split(split) for split in ("train", "val", "test")]
        has_mismatch = any(
            item["imgs_sem_label"] or item["labels_sem_img"] for item in contagens
        )
        if has_mismatch and hasattr(self, "btn_treino_contagem"):
            self.btn_treino_contagem.setStyleSheet(
                    "QPushButton { background-color: #8e0000; color: white;"
                    "font-weight: bold; padding: 12px; border-radius: 6px;"
                    "font-size: 13px; }"
                    "QPushButton:hover { background-color: #b71c1c; border: 1px solid white; }"
            )
        elif hasattr(
            self,
            "btn_treino_contagem",
        ) and hasattr(self, "_btn_treino_contagem_default_style"):
            self.btn_treino_contagem.setStyleSheet(self._btn_treino_contagem_default_style)

        resumo = " | ".join(
            [(
                f"{item['split']}: imgs={item['imgs']} lbls={item['lbls']}"
                f"pares={item['pairs_ok']}"
            ) for item in contagens]
        )
        self._log_treino_ia(
            f"Split 80/10/10 concluído ({total} imagens) | {resumo}",
            "#9ccc65",
        )
        self._set_treino_status("Split organizado com sucesso.", "#9ccc65")
        QMessageBox.information(
            self,
            "Treino IA",
            "Split concluído com sucesso.\n\n"
            f"Total de imagens: {total}\n"
            f"{resumo}"
        )

    def testar_contagem_split_treino_ia(self):
        if not self._ensure_dataset_dirs():
            return

        contagens = [self._count_split(split) for split in ("train", "val", "test")]
        has_mismatch = any(
            item["imgs_sem_label"] or item["labels_sem_img"] for item in contagens
        )

        lines = []
        for item in contagens:
            lines.append(
                    f"{item['split']}: imgs={item['imgs']} lbls={item['lbls']}"
                    f"pares={item['pairs_ok']} "
                    f"imgs_sem_label={len(item['imgs_sem_label'])}"
                    f"labels_sem_img={len(item['labels_sem_img'])}"
            )

        resumo = "\n".join(lines)
        for linha in lines:
            self._log_treino_ia(linha, "#90caf9")

        if has_mismatch:
            if hasattr(self, "btn_treino_contagem"):
                self.btn_treino_contagem.setStyleSheet(
                        "QPushButton { background-color: #8e0000; color: white;"
                        "font-weight: bold; padding: 12px; border-radius: 6px;"
                        "font-size: 13px; }"
                        "QPushButton:hover { background-color: #b71c1c; border: 1px solid white; }"
                )
            self._set_treino_status("Contagem encontrou inconsistências de pareamento.", "#ff8a80")
            QMessageBox.warning(
                self,
                "Treino IA - Contagem por Split",
                "Inconsistências detectadas entre imagens e labels.\n\n" + resumo,
            )
        else:
            if (
                hasattr(self, "btn_treino_contagem")
                and hasattr(self, "_btn_treino_contagem_default_style")
            ):
                self.btn_treino_contagem.setStyleSheet(self._btn_treino_contagem_default_style)
            self._set_treino_status("Contagem por split validada sem inconsistências.", "#9ccc65")
            QMessageBox.information(
                self,
                "Treino IA - Contagem por Split",
                "Contagem validada com sucesso.\n\n" + resumo,
            )

    def _resolve_model_ref_treino(self):
        model_raw = (
            self.input_treino_model_path.text().strip()
            if hasattr(self, "input_treino_model_path")
            else ""
        )
        if not model_raw:
            model_raw = "yolo11n.pt"
            if hasattr(self, "input_treino_model_path"):
                self.input_treino_model_path.setText(model_raw)
        is_local_path = (
            model_raw.startswith("/")
            or model_raw.startswith(".")
            or model_raw.startswith("~")
            or os.path.sep in model_raw
        )
        if is_local_path:
            expanded = os.path.expanduser(model_raw)
            model_ref = expanded if os.path.isabs(expanded) else os.path.abspath(expanded)
            return model_ref, True
        return model_raw, False

    def _validar_pronto_treino(self):
        if not self._ensure_dataset_dirs():
            return None, None
        if self._is_yolo_job_running():
            QMessageBox.warning(
                self,
                "Treino IA",
                "Já existe um processo de treino/smoke em execução.",
            )
            return None, None

        yolo_bin = shutil.which("yolo")
        if not yolo_bin:
            QMessageBox.warning(self, "Treino IA", "Comando 'yolo' não encontrado no ambiente.")
            return None, None

        if not os.path.isfile(self.dataset_data_yaml):
            QMessageBox.warning(
                self,
                "Treino IA",
                f"Arquivo data.yaml não encontrado: {self.dataset_data_yaml}",
            )
            return None, None

        ok_strict, strict_errors, strict_summary = self._validar_dataset_para_treino_strict()
        if not ok_strict:
            top_errors = strict_errors[:6]
            msg = "Dataset inválido para treino (modo estrito):\n- " + "\n- ".join(top_errors)
            if len(strict_errors) > len(top_errors):
                msg += f"\n- ... +{len(strict_errors) - len(top_errors)} erro(s)"
            QMessageBox.warning(self, "Treino IA", msg)
            self._set_treino_status(
                "Treino bloqueado: dataset inconsistente (modo estrito).",
                "#ff8a80",
            )
            self._log_treino_ia(msg.replace("\n", " | "), "#ff5555")
            return None, None
        self._log_treino_ia(
            "Validação estrita OK | "
            f"classes={strict_summary.get('class_count', 0)} "
            f"ann={strict_summary.get('total_annotations', 0)}",
            "#9ccc65",
        )

        model_ref, is_local = self._resolve_model_ref_treino()
        if is_local and not os.path.isfile(model_ref):
            QMessageBox.warning(self, "Treino IA", f"Modelo local não encontrado: {model_ref}")
            return None, None

        if not is_local and model_ref.endswith(".pt"):
            self._log_treino_ia(
                (
                    f"Modelo '{model_ref}' será resolvido pelo Ultralytics. Pode"
                    f"exigir internet para download."
                ),
                "#ffcc80",
            )

        return yolo_bin, model_ref

    def _iniciar_treino_yolo(self, smoke):
        yolo_bin, model_ref = self._validar_pronto_treino()
        if not yolo_bin:
            return

        imgsz = int(self.spin_treino_imgsz.value()) if hasattr(self, "spin_treino_imgsz") else 640
        batch = int(self.spin_treino_batch.value()) if hasattr(self, "spin_treino_batch") else 8
        epochs = (
            1
            if smoke
            else int(self.spin_treino_epochs.value())
            if hasattr(self, "spin_treino_epochs")
            else 100
        )
        device = (
            self.combo_treino_device.currentText().strip()
            if hasattr(self, "combo_treino_device")
            else "cpu"
        )
        run_name = ("smoke_" if smoke else "train_") + time.strftime("%Y%m%d_%H%M%S")
        process_name = "YOLO_SMOKE" if smoke else "YOLO_TREINO"
        project_dir = os.path.join(self.workspace, "runs", "detect")
        os.makedirs(project_dir, exist_ok=True)

        cmd = (
            f"cd {shlex.quote(self.workspace)} && "
            f"{shlex.quote(yolo_bin)} detect train "
            f"model={shlex.quote(model_ref)} "
            f"data={shlex.quote(self.dataset_data_yaml)} "
            f"imgsz={imgsz} "
            f"epochs={epochs} "
            f"batch={batch} "
            f"device={shlex.quote(device)} "
            f"project={shlex.quote(project_dir)} "
            f"name={shlex.quote(run_name)} "
            "exist_ok=True"
        )

        proc = self.run_process(process_name, cmd)
        if not proc:
            return

        self._yolo_train_running = True
        self._set_treino_botoes_execucao_enabled(False)
        self._set_treino_status(f"{'Smoke test' if smoke else 'Treino'} em execução...", "#ffcc80")
        self._log_treino_ia(
            (
                f"Iniciado {process_name} | model={model_ref} | epochs={epochs}"
                f"| batch={batch} | imgsz={imgsz} | device={device}"
            ),
            "#80cbc4",
        )
        proc.finished.connect(
            lambda exit_code, _exit_status, name=process_name, run=run_name:
            self._on_treino_yolo_finished(name, int(exit_code), run)
        )

    def executar_smoke_test_treino_ia(self):
        self._iniciar_treino_yolo(smoke=True)

    def executar_treino_completo_ia(self):
        self._iniciar_treino_yolo(smoke=False)

    def _on_treino_yolo_finished(self, process_name, exit_code, run_name):
        self._yolo_train_running = False
        self._set_treino_botoes_execucao_enabled(True)
        output_dir = os.path.join(self.workspace, "runs", "detect", run_name)
        if exit_code == 0:
            self._log_treino_ia(
                f"{process_name} concluído com sucesso. Saída: {output_dir}",
                "#9ccc65",
            )
            self._set_treino_status(f"{process_name} concluído. Saída: {output_dir}", "#9ccc65")
        else:
            self._log_treino_ia(
                f"{process_name} finalizado com falha (exit_code={exit_code}). Verifique o log.",
                "#ff5555",
            )
            self._set_treino_status(f"{process_name} falhou. Verifique logs.", "#ff8a80")

    def _set_capture_status(self, text, color="#b0bec5"):
        if hasattr(self, "lbl_capture_status"):
            self.lbl_capture_status.setText(text)
            self.lbl_capture_status.setStyleSheet(f"color: {color}; font-size: 11px;")

    def _set_capture_session_label(self, session_id):
        if hasattr(self, "lbl_coleta_session"):
            self.lbl_coleta_session.setText(f"Sessão: {session_id if session_id else '--'}")

    def _set_capture_usb_label(self, mount_path):
        if hasattr(self, "lbl_coleta_usb"):
            if mount_path:
                self.lbl_coleta_usb.setText(f"Pendrive: {mount_path}")
                self.lbl_coleta_usb.setStyleSheet("color: #9ccc65; font-size: 11px;")
            else:
                self.lbl_coleta_usb.setText("Pendrive: não selecionado")
                self.lbl_coleta_usb.setStyleSheet("color: #b0bec5; font-size: 11px;")

    def _capture_resolution(self):
        raw = (
            self.combo_capture_resolution.currentText().strip()
            if hasattr(self, "combo_capture_resolution")
            else "1280x720"
        )
        m = re.match(r"^\s*(\d+)\s*x\s*(\d+)\s*$", raw)
        if not m:
            return 1280, 720
        return max(1, int(m.group(1))), max(1, int(m.group(2)))

    def _validar_camera_coleta(self, camera_index):
        try:
            import cv2
        except Exception as e:
            return False, f"OpenCV indisponível no painel: {e}"
        cap = cv2.VideoCapture(int(camera_index))
        if not cap.isOpened():
            return False, f"Não foi possível abrir a câmera index={camera_index}."
        try:
            cap.release()
        except Exception:
            pass
        return True, ""

    def _selecionar_pendrive_para_coleta(self):
        destinos = self._listar_pontos_montagem_pen_drive()
        if not destinos:
            return None, "Nenhum pendrive gravável detectado."
        if len(destinos) == 1:
            return destinos[0], ""

        destino, ok = QInputDialog.getItem(
            self,
            "Selecionar Pen Drive (Coleta)",
            "Escolha o pen drive para salvar as fotos:",
            destinos,
            0,
            False,
        )
        if not ok:
            return None, "Seleção de pendrive cancelada."
        return destino, ""

    def _resolver_launcher_coleta_fotos(self):
        if os.path.isfile(self.exec_photo_capture) and os.access(self.exec_photo_capture, os.X_OK):
            return "ros2 run caatinga_vision photo_capture_node", ""

        if os.path.isfile(self.script_photo_capture):
            return (
                f"python3 {shlex.quote(self.script_photo_capture)}",
                (
                    "Executável instalado 'photo_capture_node' não encontrado."
                    "Usando script fonte como fallback."
                ),
            )

        return (
            "",
            "Executável de captura ausente no install e script fonte não localizado. "
            "Recompile com: colcon build --packages-select caatinga_vision",
        )

    def iniciar_captura_fotos(self, session_id):
        self._parar_pipeline_ia()
        self._encerrar_processos_externos_por_padrao(
            [
                "caatinga_vision/camera_source_node",
                "caatinga_vision/yolo_inference_node",
                "caatinga_vision/infestation_analytics_node",
            ],
            "início da coleta de fotos",
        )
        usb_mount, erro_usb = self._selecionar_pendrive_para_coleta()
        if not usb_mount:
            QMessageBox.warning(self, "Coleta de Fotos", erro_usb or "Pendrive não disponível.")
            self._set_capture_status(
                "Bloqueado: pendrive obrigatório para iniciar coleta.",
                "#ff8a80",
            )
            return False

        camera_index = (
            int(self.spin_capture_camera_index.value())
            if hasattr(self, "spin_capture_camera_index")
            else 0
        )
        ok_camera, erro_camera = self._validar_camera_coleta(camera_index)
        if not ok_camera:
            QMessageBox.warning(self, "Coleta de Fotos", erro_camera)
            self._set_capture_status("Bloqueado: câmera não disponível.", "#ff8a80")
            return False

        launcher, launcher_msg = self._resolver_launcher_coleta_fotos()
        if not launcher:
            QMessageBox.warning(self, "Coleta de Fotos", launcher_msg)
            self._set_capture_status("Bloqueado: executável de captura não encontrado.", "#ff8a80")
            self.log(f"[COLETA_FOTOS] {launcher_msg}", "#ff5555")
            return False
        if launcher_msg:
            self.log(f"[COLETA_FOTOS] {launcher_msg}", "#ffb300")

        width, height = self._capture_resolution()
        distance_m = (
            float(self.spin_capture_distance.value())
            if hasattr(self, "spin_capture_distance")
            else 0.5
        )
        jpeg_quality = (
            int(self.spin_capture_jpeg_quality.value())
            if hasattr(self, "spin_capture_jpeg_quality")
            else 90
        )
        speed_min = (
            float(self.spin_capture_speed_min.value())
            if hasattr(self, "spin_capture_speed_min")
            else 0.05
        )
        reserve_mb = (
            int(self.spin_capture_reserve_mb.value())
            if hasattr(self, "spin_capture_reserve_mb")
            else 512
        )

        cmd = (
            f"{launcher} --ros-args "
            f"-p session_id:={shlex.quote(session_id)} "
            f"-p camera_index:={camera_index} "
            f"-p capture_distance_m:={distance_m:.3f} "
            f"-p image_width:={width} "
            f"-p image_height:={height} "
            f"-p jpeg_quality:={jpeg_quality} "
            f"-p speed_min_m_s:={speed_min:.3f} "
            f"-p reserve_space_mb:={reserve_mb} "
            f"-p usb_mount_path:={shlex.quote(usb_mount)} "
            f"-p status_cache_path:={shlex.quote(self.capture_status_cache_path)} "
            f"-p preview_cache_path:={shlex.quote(self.capture_preview_cache_path)}"
        )

        proc = self.run_process("PHOTO_CAPTURE", cmd)
        if not proc:
            self._set_capture_status("Falha ao iniciar processo de captura.", "#ff8a80")
            return False

        if not proc.waitForStarted(2000):
            self._encerrar_processo("PHOTO_CAPTURE")
            msg = "Processo de captura não iniciou corretamente."
            self._set_capture_status(msg, "#ff8a80")
            self.log(f"[COLETA_FOTOS] {msg}", "#ff5555")
            return False

        proc.finished.connect(
            lambda exit_code,
            _status: self._on_photo_capture_finished(int(exit_code)),
        )
        self.capture_rota_ativa = True
        self.capture_session_id = session_id
        self.capture_status_snapshot = {}
        self._capture_status_cache_mtime = 0.0
        self._set_capture_session_label(session_id)
        self._set_capture_usb_label(usb_mount)
        self._set_capture_status("Coleta de fotos iniciada com sucesso.", "#9ccc65")
        self.log(
            (
                f"[COLETA_FOTOS] Captura iniciada | sessão={session_id} |"
                f"usb={usb_mount} | distância={distance_m:.2f}m"
            ),
            "#9ccc65",
        )
        return True

    def _on_photo_capture_finished(self, exit_code):
        if not self.capture_rota_ativa:
            return

        self.capture_rota_ativa = False
        if exit_code == 0:
            self._set_capture_status("Captura de fotos finalizada.", "#b0bec5")
            self.log("[COLETA_FOTOS] Processo de captura finalizado.", "#b0bec5")
            return

        msg = (
            f"Processo de captura finalizado com falha (exit_code={exit_code}). "
            "A rota continua, mas sem captura de fotos."
        )
        self._set_capture_status(msg, "#ff8a80")
        self.log(f"[COLETA_FOTOS] {msg}", "#ff5555")
        rota_ativa = (
            "ROTA" in self.processos
            and self.processos["ROTA"].state() != QProcess.NotRunning
        )
        if rota_ativa and not self._closing:
            QMessageBox.warning(self, "Coleta de Fotos", msg)

    def _parar_captura_fotos(self):
        self.capture_rota_ativa = False
        self._encerrar_processo("PHOTO_CAPTURE")
        self.capture_session_id = ""
        self._set_capture_session_label("")
        self._set_capture_status("Captura de fotos parada.", "#b0bec5")

    def atualizar_monitor_captura_fotos(self):
        if hasattr(self, "lbl_capture_preview"):
            pixmap = QPixmap(self.capture_preview_cache_path)
            if not pixmap.isNull():
                scaled = pixmap.scaled(
                    self.lbl_capture_preview.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
                self.lbl_capture_preview.setPixmap(scaled)
                self.lbl_capture_preview.setText("")
            elif self.lbl_capture_preview.pixmap() is None:
                self.lbl_capture_preview.setText("Preview indisponível.")

        if not os.path.exists(self.capture_status_cache_path):
            return

        try:
            stat = os.stat(self.capture_status_cache_path)
            if stat.st_mtime <= self._capture_status_cache_mtime:
                return
            self._capture_status_cache_mtime = stat.st_mtime
            with open(self.capture_status_cache_path, "r", encoding="utf-8") as f:
                status = json.load(f)
            if not isinstance(status, dict):
                return
            self.capture_status_snapshot = status
        except Exception:
            return

        status = self.capture_status_snapshot
        self._set_capture_session_label(str(status.get("session_id", self.capture_session_id)))
        self._set_capture_usb_label(str(status.get("usb_mount_path", "")))

        if hasattr(self, "lbl_capture_count"):
            self.lbl_capture_count.setText(str(int(status.get("photos_total", 0))))
        if hasattr(self, "lbl_capture_distance_total"):
            try:
                self.lbl_capture_distance_total.setText(
                    f"{float(status.get('distance_total_m', 0.0)):.2f} m"
                )
            except Exception:
                self.lbl_capture_distance_total.setText("0.00 m")
        if hasattr(self, "lbl_capture_free_space"):
            try:
                self.lbl_capture_free_space.setText(
                    f"{float(status.get('free_space_mb', 0.0)):.1f} MB"
                )
            except Exception:
                self.lbl_capture_free_space.setText("--")
        if hasattr(self, "lbl_capture_est_remaining"):
            est = status.get("estimated_photos_remaining")
            self.lbl_capture_est_remaining.setText(str(est if est is not None else "--"))

        mensagem = str(status.get("message", "Coleta ativa."))
        paused_usb = bool(status.get("paused_usb", False))
        paused_space = bool(status.get("paused_space", False))
        if paused_usb or paused_space:
            self._set_capture_status(mensagem, "#ff8a80")
        elif bool(status.get("running", False)):
            self._set_capture_status(mensagem, "#9ccc65")
        else:
            self._set_capture_status(mensagem, "#b0bec5")

    def on_implemento_changed(self, _index):
        self.atualizar_interface_tanque(abrir_aba=True)
        self.atualizar_campos_rastreabilidade_dinamicos()

    def abrir_aba_rastreabilidade(self):
        if hasattr(self, "tabs") and hasattr(self, "tab_rastreabilidade"):
            self.tabs.setCurrentWidget(self.tab_rastreabilidade)

    def _on_tabs_right_changed(self, _index):
        if hasattr(self, "tabs") and hasattr(self, "tab_ros_nodes"):
            if self.tabs.currentWidget() == self.tab_ros_nodes:
                self.atualizar_nos_ros_publicadores(forcar=True)
            if hasattr(self, "tab_saude_ia") and self.tabs.currentWidget() == self.tab_saude_ia:
                self._sugerir_modelo_ia_padrao_se_necessario(force=False)
                self.atualizar_monitor_ia()
            if hasattr(self, "tab_treino_ia") and self.tabs.currentWidget() == self.tab_treino_ia:
                self._ensure_dataset_dirs()
            if (
                hasattr(self, "tab_coleta_fotos")
                and self.tabs.currentWidget() == self.tab_coleta_fotos
            ):
                self.atualizar_monitor_captura_fotos()

    def obter_perfil_rastreabilidade_codigo(self):
        perfil_texto = (
            self.combo_rastreabilidade_perfil.currentText()
            if hasattr(self, "combo_rastreabilidade_perfil")
            else ""
        )
        perfil_map = {
            "Universal (UE + Brasil)": "universal",
            "Somente UE (EUDR)": "ue",
            "Somente Brasil (PNRV/Brasil-ID)": "brasil",
            "Operacional interno": "operacional",
        }
        return perfil_map.get(perfil_texto, "universal")

    def atualizar_campos_rastreabilidade_dinamicos(self, *_args):
        perfil = self.obter_perfil_rastreabilidade_codigo()
        implemento = (
            self.combo_implemento.currentText()
            if hasattr(self, "combo_implemento")
            else ""
        )
        is_pulverizador = "Pulverizador" in implemento

        exige_gln_epc = perfil in ("universal", "brasil")
        exige_produto = is_pulverizador and perfil in ("universal", "brasil", "ue")
        exige_registro = is_pulverizador and perfil in ("universal", "brasil")

        mostra_gln_epc = perfil in ("universal", "brasil")
        mostra_produto_registro = is_pulverizador and perfil in ("universal", "brasil", "ue")

        self.lbl_gln.setVisible(mostra_gln_epc)
        self.input_farm_gln.setVisible(mostra_gln_epc)
        self.lbl_epc.setVisible(mostra_gln_epc)
        self.input_epc.setVisible(mostra_gln_epc)

        self.lbl_prod_quimico.setVisible(mostra_produto_registro)
        self.input_produto_quimico.setVisible(mostra_produto_registro)
        self.lbl_reg_mapa.setVisible(mostra_produto_registro)
        self.input_registro_mapa.setVisible(mostra_produto_registro)

        self.lbl_gln.setText(f"GLN da fazenda (13 dígitos):{' *' if exige_gln_epc else ''}")
        self.lbl_epc.setText(f"EPC do insumo (Brasil-ID):{' *' if exige_gln_epc else ''}")
        self.lbl_robot.setText("Identificador do robô: *")
        self.lbl_operador.setText("Identificador do operador: *")
        self.lbl_prod_quimico.setText(f"Produto químico utilizado:{' *' if exige_produto else ''}")
        registro_suffix = " *" if exige_registro else " (opcional)"
        self.lbl_reg_mapa.setText(f"Nº Registro no MAPA:{registro_suffix}")

    def validar_campos_rastreabilidade(self, implemento, perfil):
        erros = []
        foco_widget = None

        def is_placeholder(value):
            text = (value or "").strip()
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

        def add_erro(msg, widget):
            nonlocal foco_widget
            erros.append(msg)
            if foco_widget is None:
                foco_widget = widget

        is_pulverizador = "Pulverizador" in implemento

        robot_uuid = self.input_robot_uuid.text().strip()
        operator_id = self.input_operator_id.text().strip()
        gln = self.input_farm_gln.text().strip()
        epc = self.input_epc.text().strip()
        produto = self.input_produto_quimico.text().strip()
        registro_mapa = self.input_registro_mapa.text().strip()

        if not robot_uuid:
            add_erro("Identificador do robô é obrigatório.", self.input_robot_uuid)
        if not operator_id:
            add_erro("Identificador do operador é obrigatório.", self.input_operator_id)

        if perfil in ("universal", "brasil"):
            if not re.fullmatch(r"\d{13}", gln):
                add_erro("GLN deve conter exatamente 13 dígitos.", self.input_farm_gln)
            if is_placeholder(epc):
                add_erro("EPC é obrigatório para este perfil.", self.input_epc)

        if is_pulverizador:
            if perfil in ("universal", "brasil", "ue") and not produto:
                add_erro(
                    "Produto químico é obrigatório para pulverizador neste perfil.",
                    self.input_produto_quimico,
                )
            if perfil in ("universal", "brasil"):
                if is_placeholder(registro_mapa):
                    add_erro(
                        "Número de Registro no MAPA é obrigatório para este perfil.",
                        self.input_registro_mapa,
                    )

        return (len(erros) == 0, erros, foco_widget)

    # --- FUNÇÃO PARA ESCONDER/MOSTRAR ---
    def atualizar_interface_tanque(self, abrir_aba=False):
        implemento = self.combo_implemento.currentText()
        if "Pulverizador" in implemento:
            self.frame_tanque.show()  # Mostra
            self.lbl_impl_sem_config.hide()
        else:
            self.frame_tanque.hide()  # Esconde
            self.lbl_impl_sem_config.show()

        if abrir_aba and hasattr(self, "tabs_left"):
            self.tabs_left.setCurrentWidget(self.tab_config_implemento)

    def atualizar_bateria(self, nivel, carregando=False):
        try:
            nivel_int = int(round(float(nivel)))
        except Exception:
            nivel_int = 0
        nivel_int = max(0, min(100, nivel_int))

        if nivel_int <= 20:
            cor = "#ef5350"
        elif nivel_int <= 50:
            cor = "#ffb300"
        else:
            cor = "#4caf50"

        self.battery_bar.setValue(nivel_int)
        self.battery_bar.setStyleSheet(
            "QProgressBar { background: #111; border: 1px solid #333; border-radius: 6px; }"
            f"QProgressBar::chunk {{ background: {cor}; border-radius: 6px; }}"
        )
        self.lbl_battery_percent.setText(f"{nivel_int}%")
        self.lbl_battery_percent.setStyleSheet(
            f"color: {cor}; font-weight: bold; min-width: 36px;"
        )

        if carregando:
            self.lbl_charging.setText("⚡ Carregando")
            self.lbl_charging.setStyleSheet(
                "background-color: #2e7d32; color: #ffffff; padding: 4px 10px;"
                "border-radius: 8px; "
                "border: 1px solid #43a047; font-weight: bold; font-size: 12px;"
            )
        else:
            self.lbl_charging.setText("⚡ Não carregando")
            self.lbl_charging.setStyleSheet(
                "background-color: #455a64; color: #ffffff; padding: 4px 10px;"
                "border-radius: 8px; "
                "border: 1px solid #546e7a; font-weight: bold; font-size: 12px;"
            )

    def atualizar_wifi(self, conectado=False, ssid=None):
        if conectado:
            nome = ssid if ssid else "Conectado"
            self.btn_wifi.setText(f"📶 Wi-Fi: {nome}")
            self.btn_wifi.setStyleSheet(
                "QPushButton { background-color: #1b5e20; color: #ffffff; padding: 2px 8px; "
                "border-radius: 8px; border: 1px solid #2e7d32; font-weight:"
                "bold; font-size: 11px; }"
                "QPushButton:hover { background-color: #2e7d32; border: 1px solid #43a047; }"
            )
        else:
            self.btn_wifi.setText("📶 Wi-Fi: Desconectado")
            self.btn_wifi.setStyleSheet(
                "QPushButton { background-color: #2b2b2b; color: #e0e0e0; padding: 2px 8px; "
                "border-radius: 8px; border: 1px solid #3c3c3c; font-weight:"
                "bold; font-size: 11px; }"
                "QPushButton:hover { background-color: #353535; border: 1px solid #4a4a4a; }"
            )

    def abrir_wifi_manager(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Wi-Fi")
        dialog.setModal(True)
        dialog.setFixedSize(520, 360)
        dialog.setStyleSheet("background-color: #232323; color: #e0e0e0;")

        layout = QVBoxLayout(dialog)

        lbl_title = QLabel("📶 Gerenciador de Wi-Fi")
        lbl_title.setStyleSheet("font-size: 14px; font-weight: bold; color: #ffffff;")
        layout.addWidget(lbl_title)

        lbl_hint = QLabel("Selecione uma rede e clique em Conectar.")
        lbl_hint.setStyleSheet("color: #b0bec5; font-size: 11px;")
        layout.addWidget(lbl_hint)

        lista = QListWidget()
        lista.setStyleSheet(
            "QListWidget { background: #1a1a1a; border: 1px solid #333; padding: 6px; }"
            "QListWidget::item { padding: 8px; }"
            "QListWidget::item:selected { background: #2e3b44; }"
        )
        layout.addWidget(lista, 1)

        lbl_status = QLabel("")
        lbl_status.setStyleSheet("color: #9e9e9e; font-size: 11px;")
        layout.addWidget(lbl_status)

        btn_row = QHBoxLayout()
        btn_refresh = QPushButton("🔄 Atualizar")
        self.estilizar_botao(btn_refresh, "#455a64")
        btn_connect = QPushButton("✅ Conectar")
        self.estilizar_botao(btn_connect, "#2e7d32")
        btn_close = QPushButton("Fechar")
        self.estilizar_botao(btn_close, "#555")
        btn_row.addWidget(btn_refresh)
        btn_row.addWidget(btn_connect)
        btn_row.addStretch(1)
        btn_row.addWidget(btn_close)
        layout.addLayout(btn_row)

        def preencher_lista():
            lista.clear()
            lbl_status.setText("Procurando redes...")
            QApplication.setOverrideCursor(Qt.WaitCursor)
            redes, erro = self._scan_wifi()
            QApplication.restoreOverrideCursor()
            if erro:
                lbl_status.setText(erro)
            else:
                lbl_status.setText(f"{len(redes)} redes encontradas.")

            if not redes:
                item = QListWidgetItem("Nenhuma rede disponível.")
                item.setFlags(Qt.NoItemFlags)
                lista.addItem(item)
                return

            for r in redes:
                sec_text = r["security"]
                if sec_text in ("--", "", "NONE", "OPEN"):
                    sec_text = "Aberta"
                active = " • Conectada" if r.get("active") else ""
                texto = f"{r['ssid']}  •  Sinal {r['signal']}%  •  {sec_text}{active}"
                item = QListWidgetItem(texto)
                item.setData(Qt.UserRole, r)
                if r.get("active"):
                    item.setForeground(QColor("#80cbc4"))
                lista.addItem(item)

        def conectar_selecionada():
            item = lista.currentItem()
            if not item:
                QMessageBox.warning(dialog, "Aviso", "Selecione uma rede.")
                return
            data = item.data(Qt.UserRole)
            if not isinstance(data, dict) or not data.get("ssid"):
                QMessageBox.warning(dialog, "Aviso", "Selecione uma rede válida.")
                return

            ssid = data["ssid"]
            security = data.get("security", "")
            senha = None
            if self._wifi_requer_senha(security):
                senha = self._pedir_senha_wifi(ssid)
                if senha is None:
                    return

            lbl_status.setText("Conectando...")
            QApplication.setOverrideCursor(Qt.WaitCursor)
            ok, msg = self._conectar_wifi(ssid, senha)
            QApplication.restoreOverrideCursor()
            if ok:
                self.atualizar_wifi(True, ssid)
                lbl_status.setText("Conectado com sucesso.")
                dialog.accept()
            else:
                lbl_status.setText(msg)
                QMessageBox.warning(dialog, "Falha ao conectar", msg)

        btn_refresh.clicked.connect(preencher_lista)
        btn_connect.clicked.connect(conectar_selecionada)
        btn_close.clicked.connect(dialog.reject)
        lista.itemDoubleClicked.connect(lambda *_: conectar_selecionada())

        preencher_lista()
        dialog.exec_()

    def _scan_wifi(self):
        if shutil.which("nmcli") is None:
            return [], "nmcli não encontrado. Instale o NetworkManager."
        try:
            cmd = [
                "nmcli", "-m", "multiline",
                "-f", "IN-USE,SSID,SIGNAL,SECURITY",
                "dev", "wifi", "list", "--rescan", "yes"
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        except Exception as e:
            return [], f"Erro ao listar redes: {e}"

        if result.returncode != 0:
            msg = (result.stderr or result.stdout or "Falha ao listar redes.").strip()
            return [], msg

        redes_brutas = []
        atual = {}
        for line in result.stdout.splitlines():
            line = line.strip()
            if not line:
                if atual:
                    redes_brutas.append(atual)
                    atual = {}
                continue
            if ":" not in line:
                continue
            key, val = line.split(":", 1)
            atual[key.strip().lower()] = val.strip()
        if atual:
            redes_brutas.append(atual)

        redes = {}
        for r in redes_brutas:
            ssid = r.get("ssid", "").strip()
            if not ssid:
                continue
            try:
                signal = int(r.get("signal", "0"))
            except Exception:
                signal = 0
            security = r.get("security", "").strip().upper()
            active = r.get("in-use", "").strip() == "*"
            entry = redes.get(ssid)
            if not entry or signal > entry["signal"] or active:
                redes[ssid] = {
                    "ssid": ssid,
                    "signal": signal,
                    "security": security,
                    "active": active,
                }

        lista = list(redes.values())
        lista.sort(key=lambda x: (x.get("active") is False, -x.get("signal", 0)))
        return lista, None

    def _wifi_requer_senha(self, security):
        sec = (security or "").strip().lower()
        if sec in ("--", "", "none", "open", "aberta"):
            return False
        return True

    def _pedir_senha_wifi(self, ssid):
        dialog = QDialog(self)
        dialog.setWindowTitle("Senha da rede")
        dialog.setModal(True)
        dialog.setFixedSize(380, 160)
        dialog.setStyleSheet("background-color: #232323; color: #e0e0e0;")

        layout = QVBoxLayout(dialog)
        lbl = QLabel(f"Digite a senha da rede:\n{ssid}")
        lbl.setStyleSheet("font-weight: bold;")
        layout.addWidget(lbl)

        input_senha = QLineEdit()
        input_senha.setEchoMode(QLineEdit.Password)
        input_senha.setPlaceholderText("Senha")
        input_senha.setStyleSheet(
            "background-color: #1a1a1a; border: 1px solid #333; padding: 6px; color: #fff;"
        )
        layout.addWidget(input_senha)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Ok).setText("Conectar")
        buttons.button(QDialogButtonBox.Cancel).setText("Cancelar")
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if dialog.exec_() == QDialog.Accepted:
            return input_senha.text().strip()
        return None

    def _conectar_wifi(self, ssid, senha):
        if shutil.which("nmcli") is None:
            return False, "nmcli não encontrado. Instale o NetworkManager."
        try:
            cmd = ["nmcli", "dev", "wifi", "connect", ssid]
            if senha:
                cmd += ["password", senha]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        except Exception as e:
            return False, f"Erro ao conectar: {e}"

        if result.returncode != 0:
            msg = (result.stderr or result.stdout or "Falha ao conectar.").strip()
            return False, msg
        return True, "Conectado."

    def toggle_suporte(self):
        if not self.suporte_ativo:
            reply = QMessageBox.question(
                self,
                'Permitir Acesso?',
                "Autorizar controle remoto?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply == QMessageBox.No:
                return
            self.suporte_ativo = True
            self.btn_suporte.setText("📡 Suporte: ON")
            self.btn_suporte.setStyleSheet(
                "QPushButton { background-color: #2962ff; color: white; font-weight: "
                "bold; padding: 12px; border-radius: 6px; border: 2px solid #00b0ff; }"
            )
            self.log(">>> SUPORTE REMOTO ATIVADO <<<", "cyan")
            self.run_process("SUPORTE", "echo 'VPN ON' && sleep 2")
        else:
            self.suporte_ativo = False
            self.btn_suporte.setText("🛟 Suporte: OFF")
            self.estilizar_botao(self.btn_suporte, "#546e7a")
            self.log(">>> SUPORTE DESCONECTADO <<<", "orange")
            self.run_process("SUPORTE_OFF", "echo 'VPN OFF'")

    def run_process(self, name, command):
        if name in self.processos and self.processos[name].state() != QProcess.NotRunning:
            if "SUPORTE" not in name:
                QMessageBox.warning(self, "Aviso", f"{name} já está rodando!")
                return None
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_output(name, process))
        process.finished.connect(lambda *_: self.processos.pop(name, None))
        process.errorOccurred.connect(
            lambda err, proc_name=name: self._on_process_error(proc_name, err)
        )
        env_prefix = ""
        if name in ("IA_PIPELINE", "PHOTO_CAPTURE"):
            # Garante user-site habilitada para a IA (ultralytics) e mantém libs de sistema para
            # cv2.
            env_prefix = "unset PYTHONNOUSERSITE; "
            if name == "IA_PIPELINE":
                user_site = ""
                try:
                    user_site = str(site.getusersitepackages() or "").strip()
                except Exception:
                    user_site = ""
                if user_site:
                    env_prefix += (
                        f"export PYTHONPATH={shlex.quote(user_site)}:"
                        "/usr/lib/python3/dist-packages:${PYTHONPATH}; "
                    )
                else:
                    env_prefix += (
                        "export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH}; "
                    )
            else:
                env_prefix += "export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH}; "
        cmd_string = (
            f"source /opt/ros/humble/setup.bash && "
            f"source {self.workspace}/install/setup.bash && "
            f"{env_prefix}{command}"
        )
        self.log(f"--- {name} ---", "yellow")
        process.start("bash", ["-c", cmd_string])
        self.processos[name] = process
        return process

    def handle_output(self, name, process):
        data = process.readAllStandardOutput()
        text = bytes(data).decode("utf8", errors="ignore").strip()
        if text:
            if "[WAYPOINT]" in text:
                self._update_waypoint_label(text)
            if "GeoJSON" in text:
                self.log(f"[{name}]: {text}", "#00ffff")
            elif "Restante" in text:
                self.log(f"[{name}]: {text}", "#ff00ff")
            elif "error" in text.lower():
                self.log(f"[{name}]: {text}", "#ff5555")
            else:
                self.log(f"[{name}]: {text}")

    def _update_waypoint_label(self, text):
        match = re.search(r"\[WAYPOINT\]\s*(\d+)\s*/\s*(\d+)", text)
        if not match:
            return
        current = int(match.group(1))
        total = int(match.group(2))
        remaining = max(total - current, 0)
        self.lbl_waypoints.setText(f"Waypoints: {current} de {total} (faltam {remaining})")
        if current >= total:
            self.btn_concluir_rota.show()
        else:
            self.btn_concluir_rota.hide()
        if current < total:
            self.btn_proximo_wp.show()
        else:
            self.btn_proximo_wp.hide()

    def _reset_waypoint_ui(self):
        self.lbl_waypoints.setText("Waypoints: --")
        self.btn_concluir_rota.hide()
        self.btn_proximo_wp.hide()

    def _set_movimento_ativo(self, ativo):
        self.robot_em_movimento = bool(ativo)
        self._atualizar_luz_estado()

    def _atualizar_luz_estado(self):
        if any(self.sensor_collision_state.values()):
            cor = "#e53935"
            texto = "COLISÃO"
            cor_texto = "#ef9a9a"
        elif self.robot_em_movimento:
            cor = "#43a047"
            texto = "ANDANDO"
            cor_texto = "#a5d6a7"
        else:
            cor = "#ffffff"
            texto = "PARADO"
            cor_texto = "#eeeeee"

        if hasattr(self, "luz_status"):
            self.luz_status.setStyleSheet(
                "QLabel { "
                f"background-color: {cor}; "
                "border: 2px solid #eceff1; "
                "border-radius: 11px; "
                "}"
            )
        if hasattr(self, "frame_status_luz"):
            self.frame_status_luz.setStyleSheet(
                "QFrame { "
                "background-color: #121820; "
                f"border: 2px solid {cor}; "
                "border-radius: 12px; "
                "}"
            )
        if hasattr(self, "lbl_luz_status"):
            self.lbl_luz_status.setText(texto)
            self.lbl_luz_status.setStyleSheet(
                f"color: {cor_texto}; font-weight: bold; font-size: 11px;"
            )

    def _read_cpu_usage_percent(self):
        try:
            with open("/proc/stat", "r", encoding="utf-8") as f:
                line = f.readline().strip()
            parts = line.split()
            if len(parts) < 5 or parts[0] != "cpu":
                return None
            values = [int(v) for v in parts[1:]]
            total = sum(values)
            idle = values[3] + (values[4] if len(values) > 4 else 0)

            if self._cpu_prev_total is None:
                self._cpu_prev_total = total
                self._cpu_prev_idle = idle
                return None

            delta_total = total - self._cpu_prev_total
            delta_idle = idle - self._cpu_prev_idle
            self._cpu_prev_total = total
            self._cpu_prev_idle = idle

            if delta_total <= 0:
                return 0.0
            usage = (1.0 - (delta_idle / float(delta_total))) * 100.0
            return max(0.0, min(100.0, usage))
        except Exception:
            return None

    def _read_sysfs_temp_c(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                raw = f.read().strip()
            if not raw:
                return None
            value = float(raw)
            if value > 1000.0:
                value = value / 1000.0
            if 0.0 < value < 250.0:
                return value
            return None
        except Exception:
            return None

    def _listar_thermal_zones(self):
        base = "/sys/class/thermal"
        if not os.path.isdir(base):
            return []
        zones = []
        for name in sorted(os.listdir(base)):
            if name.startswith("thermal_zone"):
                path = os.path.join(base, name)
                if os.path.isdir(path):
                    zones.append(path)
        return zones

    def _detectar_temperatura_maxima_cpu(self):
        candidatos = []
        for zone in self._listar_thermal_zones():
            try:
                entries = os.listdir(zone)
            except Exception:
                continue
            for entry in entries:
                if entry.startswith("trip_point_") and entry.endswith("_temp"):
                    temp_c = self._read_sysfs_temp_c(os.path.join(zone, entry))
                    if temp_c is not None:
                        candidatos.append(temp_c)
        if candidatos:
            return max(candidatos)
        return 100.0

    def _read_cpu_temp_c(self):
        temps = []
        for zone in self._listar_thermal_zones():
            temp_c = self._read_sysfs_temp_c(os.path.join(zone, "temp"))
            if temp_c is not None:
                temps.append(temp_c)
        if temps:
            return max(temps)
        return None

    def _desligar_sistema_por_temperatura(self):
        nome = "SISTEMA_FAZENDA"
        if nome in self.processos and self.processos[nome].state() != QProcess.NotRunning:
            self._parar_pipeline_ia()
            self._parar_captura_fotos()
            self._encerrar_rastreabilidade()
            try:
                os.kill(self.processos[nome].processId(), signal.SIGINT)
                self.processos[nome].waitForFinished(3000)
            except Exception:
                pass
            subprocess.run(["pkill", "-f", "ros2 launch"])

        if hasattr(self, "btn_sistema"):
            self.btn_sistema.setText("🚀 Ligar Sistema (Launch)")
            self.btn_sistema.setStyleSheet(
                self.btn_sistema.styleSheet().replace("#444", "#e65100")
            )
        self._set_movimento_ativo(False)

    def _avaliar_alertas_temperatura_cpu(self, temp_c):
        if temp_c is None:
            return

        if temp_c >= self.cpu_temp_max_c:
            if not self._temp_alerta_100_emitido:
                self._temp_alerta_100_emitido = True
                self._temp_alerta_80_emitido = True
                msg = (
                    f"Temperatura crítica da CPU: {temp_c:.1f}°C "
                    f"(limite: {self.cpu_temp_max_c:.1f}°C).\n"
                    "O sistema será desligado por segurança."
                )
                self.log(f">>> ALERTA CRÍTICO CPU: {msg} <<<", "#ff5555")
                QMessageBox.critical(self, "Temperatura Crítica da CPU", msg)
                self._desligar_sistema_por_temperatura()
            return

        if temp_c >= self.cpu_temp_alerta_80_c:
            if not self._temp_alerta_80_emitido:
                self._temp_alerta_80_emitido = True
                msg = (
                    f"CPU em alta temperatura: {temp_c:.1f}°C "
                    f"(>= 80% do limite, {self.cpu_temp_alerta_80_c:.1f}°C)."
                )
                self.log(f">>> ALERTA CPU: {msg} <<<", "#ffb300")
                QMessageBox.warning(self, "Alerta de Temperatura da CPU", msg)

        # Histerese para evitar alertas repetitivos quando a temperatura oscila.
        if temp_c <= (self.cpu_temp_alerta_80_c - 3.0):
            self._temp_alerta_80_emitido = False
        if temp_c <= (self.cpu_temp_max_c - 3.0):
            self._temp_alerta_100_emitido = False

    def atualizar_cpu_monitor(self):
        uso_cpu = self._read_cpu_usage_percent()
        if uso_cpu is None:
            try:
                cpus = max(1, int(os.cpu_count() or 1))
                uso_cpu = max(0.0, min(100.0, (os.getloadavg()[0] / cpus) * 100.0))
            except Exception:
                return

        uso_cpu_int = int(round(uso_cpu))
        if hasattr(self, "cpu_bar"):
            self.cpu_bar.setValue(uso_cpu_int)
            if uso_cpu_int <= 60:
                cor = "#43a047"
            elif uso_cpu_int <= 85:
                cor = "#f9a825"
            else:
                cor = "#e53935"
            self.cpu_bar.setStyleSheet(
                    "QProgressBar { background: #0f141a; border: 1px solid #32404f;"
                    "border-radius: 5px; }"
                    f"QProgressBar::chunk {{ background: {cor}; border-radius: 5px; }}"
            )
        if hasattr(self, "lbl_cpu"):
            self.lbl_cpu.setText(f"CPU: {uso_cpu_int}%")

        temp_c = self._read_cpu_temp_c()
        if hasattr(self, "lbl_cpu_temp"):
            if temp_c is None:
                self.lbl_cpu_temp.setText(
                    f"Temp CPU: --.-\u00b0C / max {self.cpu_temp_max_c:.1f}\u00b0C"
                )
                self.lbl_cpu_temp.setStyleSheet(
                    "color: #90a4ae; font-size: 10px; font-weight: bold;"
                )
            else:
                if temp_c >= self.cpu_temp_max_c:
                    cor_temp = "#ff5252"
                elif temp_c >= self.cpu_temp_alerta_80_c:
                    cor_temp = "#ffd54f"
                else:
                    cor_temp = "#80cbc4"
                self.lbl_cpu_temp.setText((
                    f"Temp CPU: {temp_c:.1f}\\u00b0C / max"
                    f"{self.cpu_temp_max_c:.1f}\\u00b0C"
                ))
                self.lbl_cpu_temp.setStyleSheet(
                    f"color: {cor_temp}; font-size: 10px; font-weight: bold;"
                )

        self._avaliar_alertas_temperatura_cpu(temp_c)

    def atualizar_nos_ros_publicadores(self, forcar=False):
        if self._closing or self._ros_refresh_em_andamento:
            return

        # Evita custo no startup e durante operação normal quando a aba não está ativa.
        if (
            not forcar
            and hasattr(self, "tabs")
            and hasattr(self, "tab_ros_nodes")
            and self.tabs.currentWidget() != self.tab_ros_nodes
        ):
            return

        self._ros_refresh_em_andamento = True
        if hasattr(self, "lbl_ros_nodes_status"):
            self.lbl_ros_nodes_status.setText("Atualizando nós ROS2...")
        if hasattr(self, "btn_ros_nodes_refresh"):
            self.btn_ros_nodes_refresh.setEnabled(False)
        if hasattr(self, "btn_ros_nodes_export"):
            self.btn_ros_nodes_export.setEnabled(False)

        self._ros_nodes_worker = RosNodesCollectorThread(self.workspace, self)
        self._ros_nodes_worker.result_ready.connect(self._on_ros_nodes_coletados)
        self._ros_nodes_worker.finished.connect(self._on_ros_nodes_worker_finished)
        self._ros_nodes_worker.start()

    def _on_ros_nodes_coletados(self, ok, nodes_publicadores, erro):
        if self._closing:
            return

        if not ok:
            self._ros_nodes_cache = []
            if hasattr(self, "tree_ros_nodes"):
                self.tree_ros_nodes.clear()
            if hasattr(self, "lbl_ros_nodes_status"):
                self.lbl_ros_nodes_status.setText(f"Sem dados ROS2 no momento: {erro}")
            if hasattr(self, "btn_ros_nodes_export"):
                self.btn_ros_nodes_export.setEnabled(False)
            return

        self._ros_nodes_cache = list(nodes_publicadores)

        if hasattr(self, "tree_ros_nodes"):
            self.tree_ros_nodes.setUpdatesEnabled(False)
            self.tree_ros_nodes.clear()
            for node_name, pubs in sorted(nodes_publicadores, key=lambda x: x[0].lower()):
                topicos_lista = [f"{t} ({m})" if m else t for t, m in pubs]
                if len(topicos_lista) > 8:
                    topicos = "; ".join(topicos_lista[:8]) + f"; ... (+{len(topicos_lista) - 8})"
                else:
                    topicos = "; ".join(topicos_lista)
                item = QTreeWidgetItem([node_name, str(len(pubs)), topicos])
                item.setToolTip(0, node_name)
                item.setToolTip(2, "; ".join(topicos_lista))
                self.tree_ros_nodes.addTopLevelItem(item)
            self.tree_ros_nodes.setUpdatesEnabled(True)

        total_nodes = len(nodes_publicadores)
        total_pubs = sum(len(p) for _, p in nodes_publicadores)
        if hasattr(self, "lbl_ros_nodes_status"):
            if total_nodes == 0:
                self.lbl_ros_nodes_status.setText("Nenhum nó publicando dados no momento.")
            else:
                self.lbl_ros_nodes_status.setText(
                        f"{total_nodes} nós publicando ({total_pubs} tópicos)."
                        f"Atualização automática a cada 8s."
                )
        if hasattr(self, "btn_ros_nodes_export"):
            self.btn_ros_nodes_export.setEnabled(total_nodes > 0)

    def _logs_rastreabilidade_dir(self):
        return os.path.join(self.workspace, "logs_rastreabilidade")

    def _contar_arquivos_em_diretorio(self, diretorio):
        total = 0
        for _raiz, _subdirs, arquivos in os.walk(diretorio):
            total += len(arquivos)
        return total

    def _listar_pontos_montagem_pen_drive(self):
        usuario = os.getenv("USER") or os.path.basename(self.home)
        bases = [
            os.path.join("/media", usuario),
            os.path.join("/run/media", usuario),
            "/mnt",
        ]
        destinos = []
        vistos = set()

        def adicionar_se_valido(caminho):
            caminho_real = os.path.realpath(caminho)
            if caminho_real in vistos:
                return
            if not os.path.isdir(caminho_real):
                return
            if not os.path.ismount(caminho_real):
                return
            if not os.access(caminho_real, os.W_OK):
                return
            vistos.add(caminho_real)
            destinos.append(caminho_real)

        for base in bases:
            if not os.path.isdir(base):
                continue
            adicionar_se_valido(base)
            try:
                nivel_1 = sorted(os.listdir(base))
            except Exception:
                continue
            for nome_1 in nivel_1:
                caminho_1 = os.path.join(base, nome_1)
                adicionar_se_valido(caminho_1)
                if not os.path.isdir(caminho_1):
                    continue
                try:
                    nivel_2 = sorted(os.listdir(caminho_1))
                except Exception:
                    continue
                for nome_2 in nivel_2:
                    adicionar_se_valido(os.path.join(caminho_1, nome_2))

        return destinos

    def exportar_arquivos_rastreabilidade_pen_drive(self):
        origem = self._logs_rastreabilidade_dir()
        if not os.path.isdir(origem):
            QMessageBox.warning(
                self,
                "Exportar para Pen Drive",
                "A pasta de rastreabilidade não foi encontrada no sistema."
            )
            return

        total_arquivos = self._contar_arquivos_em_diretorio(origem)
        if total_arquivos == 0:
            QMessageBox.information(
                self,
                "Exportar para Pen Drive",
                "Não há arquivos de rastreabilidade para exportar."
            )
            return

        destinos = self._listar_pontos_montagem_pen_drive()
        if not destinos:
            QMessageBox.warning(
                self,
                "Exportar para Pen Drive",
                "Nenhum pen drive montado foi encontrado.\\nConecte o"
                "dispositivo e tente novamente."
            )
            return

        destino_pen_drive = destinos[0]
        if len(destinos) > 1:
            destino_pen_drive, ok = QInputDialog.getItem(
                self,
                "Selecionar Pen Drive",
                "Escolha o destino para exportação:",
                destinos,
                0,
                False,
            )
            if not ok:
                return

        tag = time.strftime("%Y%m%d_%H%M%S")
        destino_exportacao = os.path.join(destino_pen_drive, f"Exportacao_Rastreabilidade_{tag}")

        try:
            shutil.copytree(origem, destino_exportacao)
        except Exception as e:
            QMessageBox.warning(
                self,
                "Exportar para Pen Drive",
                f"Falha ao exportar arquivos: {e}",
            )
            return

        self.log(f"[Rastreabilidade] Exportado para pen drive: {destino_exportacao}", "#9ccc65")
        QMessageBox.information(
            self,
            "Exportar para Pen Drive",
            "Exportação concluída.\n\n"
            f"Arquivos exportados: {total_arquivos}\n"
            f"Destino: {destino_exportacao}"
        )

    def excluir_arquivos_rastreabilidade_local(self):
        diretorio = self._logs_rastreabilidade_dir()
        if not os.path.isdir(diretorio):
            QMessageBox.warning(
                self,
                "Excluir arquivos",
                "A pasta de rastreabilidade não foi encontrada no sistema."
            )
            return

        total_arquivos = self._contar_arquivos_em_diretorio(diretorio)
        if total_arquivos == 0:
            QMessageBox.information(
                self,
                "Excluir arquivos",
                "Não há arquivos de rastreabilidade para excluir."
            )
            return

        confirmacao = QMessageBox.warning(
            self,
            "Exclusão permanente",
            "Você está prestes a excluir PERMANENTEMENTE todos os arquivos"
            "de rastreabilidade do sistema.\\n"
            "Não haverá como recuperar após a confirmação.\n\n"
            "Deseja continuar?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if confirmacao != QMessageBox.Yes:
            return

        try:
            for nome in os.listdir(diretorio):
                caminho = os.path.join(diretorio, nome)
                if os.path.islink(caminho) or os.path.isfile(caminho):
                    os.remove(caminho)
                elif os.path.isdir(caminho):
                    shutil.rmtree(caminho)
        except Exception as e:
            QMessageBox.warning(self, "Excluir arquivos", f"Falha ao excluir arquivos: {e}")
            return

        self.log(
            (
                f"[Rastreabilidade] Exclusão permanente concluída:"
                f"{total_arquivos} arquivos removidos."
            ),
            "#ff8a80",
        )
        QMessageBox.information(
            self,
            "Excluir arquivos",
            "Exclusão concluída com sucesso.\n"
            "Os arquivos removidos não podem ser recuperados pelo sistema."
        )

    def exportar_nos_ros_publicadores(self):
        if not self._ros_nodes_cache:
            QMessageBox.warning(
                self,
                "Exportação ROS2",
                "Não há dados de nós publicadores para exportar.\\nAbra a aba e"
                "aguarde a atualização."
            )
            return

        output_dir = self._logs_rastreabilidade_dir()
        os.makedirs(output_dir, exist_ok=True)
        tag = time.strftime("%Y%m%d_%H%M%S")
        snapshot_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

        resumo_path = os.path.join(output_dir, f"ros2_publicadores_resumo_{tag}.csv")
        detalhado_path = os.path.join(output_dir, f"ros2_publicadores_detalhado_{tag}.csv")

        try:
            with open(resumo_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["Snapshot_UTC", "Node", "Publishers_Count", "Topics"])
                for node_name, pubs in sorted(self._ros_nodes_cache, key=lambda x: x[0].lower()):
                    topics = "; ".join([f"{t} ({m})" if m else t for t, m in pubs])
                    writer.writerow([snapshot_utc, node_name, len(pubs), topics])

            with open(detalhado_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "Snapshot_UTC",
                        "Node",
                        "Publisher_Index",
                        "Topic",
                        "Message_Type",
                        "Publishers_Count_Node",
                    ]
                )
                for node_name, pubs in sorted(self._ros_nodes_cache, key=lambda x: x[0].lower()):
                    for idx, (topic, msg_type) in enumerate(pubs, start=1):
                        writer.writerow([snapshot_utc, node_name, idx, topic, msg_type, len(pubs)])
        except Exception as e:
            QMessageBox.warning(self, "Exportação ROS2", f"Falha ao exportar: {e}")
            return

        self.log(f"[ROS2] Exportado: {resumo_path}", "#9ccc65")
        self.log(f"[ROS2] Exportado: {detalhado_path}", "#9ccc65")
        QMessageBox.information(
            self,
            "Exportação ROS2",
            "Exportação concluída.\n\n"
            f"Resumo: {resumo_path}\n"
            f"Detalhado: {detalhado_path}"
        )

    def _on_ros_nodes_worker_finished(self):
        self._ros_refresh_em_andamento = False
        if hasattr(self, "btn_ros_nodes_refresh"):
            self.btn_ros_nodes_refresh.setEnabled(True)
        if self._ros_nodes_worker is not None:
            self._ros_nodes_worker.deleteLater()
            self._ros_nodes_worker = None

    def _on_rota_finished(self):
        self._reset_waypoint_ui()
        self._set_movimento_ativo(False)
        if self.ia_rota_ativa:
            self._parar_pipeline_ia()
            self._encerrar_rastreabilidade()
        if self.capture_rota_ativa:
            self._parar_captura_fotos()

    def limpar_e_reconstruir(self):
        reply = QMessageBox.question(
            self,
            'Confirmação',
            "Limpar build/install e recompilar?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.No:
            return
        self.log(">>> RECOMPILANDO... <<<", "cyan")
        self.btn_rebuild.setEnabled(False)
        self.progress_bar.show()
        # Garante que sessões antigas não fiquem presas
        subprocess.run(["pkill", "-f", "gzserver"])
        subprocess.run(["pkill", "-f", "gzclient"])
        subprocess.run(["pkill", "-f", "rviz2"])
        subprocess.run(["pkill", "-f", "rqt"])
        cmd = (
            f"cd {self.workspace} && rm -rf build/ install/ && "
            "colcon build --packages-select agro_robot_sim caatinga_vision && . install/setup.bash"
        )
        proc = self.run_process("REBUILD", cmd)
        if proc:
            proc.finished.connect(self.fim_rebuild)

    def _listar_repos_git(self):
        repos = []
        if os.path.isdir(os.path.join(self.workspace, ".git")):
            repos.append(self.workspace)

        src_dir = os.path.join(self.workspace, "src")
        if os.path.isdir(src_dir):
            for nome in sorted(os.listdir(src_dir)):
                caminho = os.path.join(src_dir, nome)
                if os.path.isdir(caminho) and os.path.isdir(os.path.join(caminho, ".git")):
                    repos.append(caminho)

        return repos

    def atualizar_software(self):
        reply = QMessageBox.question(
            self,
            "Confirmação",
            "Buscar atualizações de software e recompilar o pacote?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.No:
            return

        self.log(">>> ATUALIZANDO SOFTWARE... <<<", "cyan")
        self.btn_update_sw.setEnabled(False)
        self.progress_bar.show()

        comandos = []
        repos_git = self._listar_repos_git()

        if shutil.which("git") is None:
            self.log("Git não encontrado. Seguindo apenas com compilação local.", "orange")
        elif repos_git:
            self.log(f"Repos Git detectados: {len(repos_git)}", "#9ccc65")
            for repo in repos_git:
                comandos.append(f"cd {shlex.quote(repo)} && git pull --ff-only")
        else:
            self.log(
                "Nenhum repositório Git detectado. Seguindo apenas com compilação local.",
                "orange",
            )

        comandos.append(
            f"cd {shlex.quote(self.workspace)} && "
            "colcon build --packages-select agro_robot_sim caatinga_vision && . install/setup.bash"
        )
        cmd = " && ".join(comandos)

        proc = self.run_process("UPDATE_SOFTWARE", cmd)
        if proc:
            proc.finished.connect(self.fim_atualizacao_software)
        else:
            self.btn_update_sw.setEnabled(True)
            self.progress_bar.hide()

    def fim_atualizacao_software(self, exit_code=0, _exit_status=None):
        self.btn_update_sw.setEnabled(True)
        self.progress_bar.hide()
        if exit_code == 0:
            self.log(">>> SOFTWARE ATUALIZADO COM SUCESSO <<<", "cyan")
            QMessageBox.information(self, "Sucesso", "Atualização de software concluída.")
        else:
            self.log(">>> FALHA NA ATUALIZAÇÃO DE SOFTWARE <<<", "#ff5555")
            QMessageBox.warning(self, "Erro", "Falha ao atualizar software. Verifique os logs.")

    def fim_rebuild(self):
        self.btn_rebuild.setEnabled(True)
        self.progress_bar.hide()
        self.log(">>> COMPILAÇÃO OK <<<", "cyan")
        QMessageBox.information(self, "Sucesso", "Ambiente pronto!")

    def toggle_sistema(self):
        nome = "SISTEMA_FAZENDA"
        if nome in self.processos and self.processos[nome].state() != QProcess.NotRunning:
            self._parar_pipeline_ia()
            self._parar_captura_fotos()
            self._encerrar_rastreabilidade()
            os.kill(self.processos[nome].processId(), signal.SIGINT)
            self.processos[nome].waitForFinished(2000)
            subprocess.run(["pkill", "-f", "ros2 launch"])
            self.btn_sistema.setText("🚀 Ligar Sistema (Launch)")
            self.btn_sistema.setStyleSheet(
                self.btn_sistema.styleSheet().replace("#444", "#e65100")
            )
            self._set_movimento_ativo(False)
        else:
            cmd_launch = (
                "ros2 launch agro_robot_sim fazenda_completa.launch.py"
                "use_sim_time:=True autostart:=True"
            )
            proc_sistema = self.run_process(nome, cmd_launch)
            if proc_sistema:
                proc_sistema.finished.connect(
                    lambda exit_code, _status: self._on_sistema_fazenda_finished(int(exit_code))
                )
            self.btn_sistema.setText("⏹️ Parar")
            self.btn_sistema.setStyleSheet(
                self.btn_sistema.styleSheet().replace("#e65100", "#444")
            )

    def _on_sistema_fazenda_finished(self, exit_code):
        self._set_movimento_ativo(False)
        if exit_code != 0:
            self.log(f"[SISTEMA_FAZENDA] Encerrado com falha (exit_code={exit_code}).", "#ff5555")
            self._parar_pipeline_ia()
            self._parar_captura_fotos()
            self._encerrar_rastreabilidade()
        if hasattr(self, "btn_sistema"):
            self.btn_sistema.setText("🚀 Ligar Sistema (Launch)")
            self.btn_sistema.setStyleSheet(
                self.btn_sistema.styleSheet().replace("#444", "#e65100")
            )

    def iniciar_gravador(self):
        cmd = f"gnome-terminal -- python3 {self.script_gravador}"
        QProcess.startDetached("bash", ["-c", cmd])

    def definir_garagem(self):
        if os.path.exists(self.garagem_path):
            reply = QMessageBox.question(
                self,
                "Atenção",
                "A garagem já foi definida.\n"
                "Alterar só é recomendado em casos permanentes.\n"
                "Deseja substituir a garagem atual?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply == QMessageBox.No:
                return
        self.log(">>> DEFININDO GARAGEM (POSIÇÃO ATUAL) <<<", "cyan")
        self.run_process("GARAGEM_DEF", f"python3 {self.script_salvar_garagem}")

    def chamar_garagem(self):
        if not os.path.exists(self.garagem_path):
            QMessageBox.warning(self, "Aviso", "Garagem não definida. Use 'Definir Garagem'.")
            return
        self.log(">>> CHAMANDO ROBÔ PARA GARAGEM <<<", "cyan")
        proc = self.run_process("GARAGEM_CALL", f"python3 {self.script_ir_garagem}")
        if proc:
            self._set_movimento_ativo(True)
            proc.finished.connect(lambda *_: self._set_movimento_ativo(False))

    def executar_rota(self):
        item = self.lista_rotas.currentItem()
        if not item:
            QMessageBox.warning(self, "Aviso", "Selecione uma rota!")
            return
        self._reset_waypoint_ui()

        # --- LÓGICA INTELIGENTE DE IMPLEMENTO ---
        implemento = self.combo_implemento.currentText()

        is_ia = "Inteligente" in implemento
        is_photo = "Coletor de Imagens" in implemento
        if is_ia:
            produto = self.input_produto_quimico.text().strip() or "Nao_Informado"
            registro_mapa = self.input_registro_mapa.text().strip() or "Nao_Informado"
            agua_litros = str(self.spin_agua.value())
            dose_ml = str(self.spin_dose.value())
            event_type = "INTELLIGENT_SPRAY_RECOMMENDATION"
            msg_log = (
                f">>> IA ATIVA: monitoramento inteligente | {agua_litros}L Agua + "
                f"{dose_ml}ml {produto} | MAPA: {registro_mapa} <<<"
            )
        elif is_photo:
            produto = "Coleta_Imagens_Treinamento"
            registro_mapa = "Nao_Se_Aplica"
            agua_litros = "0"
            dose_ml = "0"
            event_type = "IMAGE_DATA_COLLECTION"
            msg_log = ">>> COLETA DE FOTOS PARA TREINO ATIVA <<<"
        elif "Pulverizador" in implemento:
            # Pega os dados reais dos campos visíveis
            produto = self.input_produto_quimico.text().strip() or "Nao_Informado"
            registro_mapa = self.input_registro_mapa.text().strip() or "Nao_Informado"
            agua_litros = str(self.spin_agua.value())
            dose_ml = str(self.spin_dose.value())
            event_type = "CHEMICAL_APPLICATION"
            msg_log = (
                f">>> PREPARO: {agua_litros}L Agua + {dose_ml}ml {produto} | "
                f"MAPA: {registro_mapa} <<<"
            )
        elif "Roçadeira" in implemento:
            # Roçadeira: Envia ZEROS para o sistema de rastreabilidade
            produto = "Operacao_Mecanica_Rocadeira"
            registro_mapa = "Nao_Se_Aplica"
            agua_litros = "0"
            dose_ml = "0"
            event_type = "MECHANICAL_CUTTING"
            msg_log = ">>> OPERAÇÃO MECÂNICA (ROÇADEIRA) - SEM APLICAÇÃO <<<"
        else:
            # Sem implemento acoplado
            produto = "Operacao_Sem_Implemento"
            registro_mapa = "Nao_Se_Aplica"
            agua_litros = "0"
            dose_ml = "0"
            event_type = "TRANSIT"
            msg_log = ">>> OPERAÇÃO SEM IMPLEMENTO ACOPLADO <<<"

        perfil = self.obter_perfil_rastreabilidade_codigo()
        if self.chk_rastreabilidade.isChecked():
            ok, erros, foco_widget = self.validar_campos_rastreabilidade(implemento, perfil)
            if not ok:
                self.abrir_aba_rastreabilidade()
                msg = "Preencha os campos obrigatórios para continuar:\n\n- " + "\n- ".join(erros)
                QMessageBox.warning(self, "Rastreabilidade incompleta", msg)
                if foco_widget is not None:
                    foco_widget.setFocus()
                return

        session_id = self._gerar_session_id() if (is_ia or is_photo) else ""
        if is_ia:
            if not self.iniciar_pipeline_ia(session_id):
                QMessageBox.warning(
                    self,
                    "Saúde IA",
                    "Falha ao iniciar o pipeline de IA. Verifique logs e caminho do modelo."
                )
                return
            self.ia_rota_ativa = True
            if hasattr(self, "tabs") and hasattr(self, "tab_saude_ia"):
                self.tabs.setCurrentWidget(self.tab_saude_ia)
        else:
            self.ia_rota_ativa = False

        if is_photo:
            if not self.iniciar_captura_fotos(session_id):
                if is_ia:
                    self._parar_pipeline_ia()
                    self.ia_rota_ativa = False
                return
            self.capture_rota_ativa = True
            if hasattr(self, "tabs") and hasattr(self, "tab_coleta_fotos"):
                self.tabs.setCurrentWidget(self.tab_coleta_fotos)
        else:
            self.capture_rota_ativa = False
            self.capture_session_id = ""
            self._set_capture_session_label("")
            self._set_capture_usb_label("")

        proc_rota = self.run_process(
            "ROTA",
            f"python3 {shlex.quote(self.script_leitor)} {shlex.quote(item.text())}"
        )
        if not proc_rota:
            if is_ia:
                self._parar_pipeline_ia()
            if is_photo:
                self._parar_captura_fotos()
            return
        self._set_movimento_ativo(True)
        proc_rota.finished.connect(lambda *_: self._on_rota_finished())

        if self.chk_rastreabilidade.isChecked():
            self.log(msg_log, "cyan")
            farm_gln = (
                self.input_farm_gln.text().strip()
                if hasattr(self, "input_farm_gln")
                else ""
            )
            epc = self.input_epc.text().strip() if hasattr(self, "input_epc") else ""
            robot_uuid = (
                self.input_robot_uuid.text().strip()
                if hasattr(self, "input_robot_uuid")
                else ""
            )
            operator_id = (
                self.input_operator_id.text().strip()
                if hasattr(self, "input_operator_id")
                else ""
            )

            if hasattr(self, "combo_rastreabilidade_perfil"):
                perfil_texto = self.combo_rastreabilidade_perfil.currentText()
                self.log(
                    f"Perfil de rastreabilidade: {perfil_texto}",
                    "#9ccc65",
                )

            # Envia dados para gerar pacote padronizado (UE + Brasil)
            cmd_rastreio = (
                f"python3 {shlex.quote(self.script_rastreabilidade)} "
                f"--produto {shlex.quote(produto)} "
                f"--agua {shlex.quote(agua_litros)} "
                f"--dose {shlex.quote(dose_ml)} "
                f"--registro_mapa {shlex.quote(registro_mapa)} "
                f"--perfil {shlex.quote(perfil)} "
                f"--farm_gln {shlex.quote(farm_gln or '7890000012345')} "
                f"--epc {shlex.quote(epc or 'NAO_INFORMADO')} "
                f"--robot_uuid {shlex.quote(robot_uuid or 'AGRI-BOT-X99')} "
                f"--operator_id {shlex.quote(operator_id or 'AUTO_PILOT')} "
                f"--event_type {shlex.quote(event_type)} "
                f"--session_id {shlex.quote(session_id)} "
                f"--ia_enabled {shlex.quote('true' if is_ia else 'false')} "
                f"--ia_status_topic /caatinga_vision/infestation/status "
                f"--ia_recommendation_topic /caatinga_vision/spray/recommendation"
            )

            self.run_process("RASTREABILIDADE", cmd_rastreio)

    def concluir_rota(self):
        if "ROTA" not in self.processos:
            QMessageBox.information(self, "Aviso", "Nenhuma rota ativa.")
            return
        cmd = "ros2 service call /force_finish std_srvs/srv/Trigger {}"
        self.run_process("FORCE_FINISH", cmd)

    def pular_waypoint(self):
        if "ROTA" not in self.processos:
            QMessageBox.information(self, "Aviso", "Nenhuma rota ativa.")
            return
        cmd = "ros2 service call /skip_waypoint std_srvs/srv/Trigger {}"
        self.run_process("SKIP_WAYPOINT", cmd)

    def atualizar_lista_rotas(self):
        self.lista_rotas.clear()
        if os.path.exists(self.path_rotas):
            for f in sorted(os.listdir(self.path_rotas)):
                if f.endswith('.csv'):
                    self.lista_rotas.addItem(f.replace('.csv', ''))

    def atualizar_sensores_colisao(self, esquerda=None, frente=None, direita=None):
        updates = {
            "esquerda": esquerda,
            "frente": frente,
            "direita": direita,
        }
        for sensor, valor in updates.items():
            if valor is not None:
                self.sensor_collision_state[sensor] = bool(valor)

        if self.sensor_panel is not None:
            self.sensor_panel.set_sensor_states(
                esquerda=self.sensor_collision_state["esquerda"],
                frente=self.sensor_collision_state["frente"],
                direita=self.sensor_collision_state["direita"],
            )
        self._atualizar_luz_estado()

        if (
            any(self.sensor_collision_state.values())
            and hasattr(self, "tabs")
            and hasattr(self, "tab_sensores")
        ):
            self.tabs.setCurrentWidget(self.tab_sensores)

    def atualizar_mapa(self, lat, lon, zoom=18):
        if not WEBENGINE_DISPONIVEL:
            return
        try:
            import folium
        except Exception:
            self.lbl_map_status.setText("Folium não instalado. Rode: pip install folium")
            return

        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except Exception:
            lat_f = self.default_br_lat
            lon_f = self.default_br_lon

        # Mantem foco no Brasil para evitar centralizacao em coordenadas indevidas.
        if not (-34.0 <= lat_f <= 6.0 and -74.0 <= lon_f <= -34.0):
            lat_f = self.default_br_lat
            lon_f = self.default_br_lon
            zoom = self.default_br_zoom

        self.robot_lat = lat_f
        self.robot_lon = lon_f

        mapa = folium.Map(location=[lat_f, lon_f], zoom_start=zoom, tiles="OpenStreetMap")
        # Remove o prefixo padrão do Leaflet e substitui por identificação local.
        mapa.get_root().html.add_child(
            folium.Element(
                f"""
                <script>
                setTimeout(function() {{
                    var m = {mapa.get_name()};
                    if (m && m.attributionControl) {{
                        m.attributionControl.setPrefix('🇧🇷 Brasil');
                    }}
                }}, 0);
                </script>
                """
            )
        )
        folium.Marker([lat_f, lon_f], tooltip="Robô", icon=folium.Icon(color="green")).add_to(mapa)
        mapa.save(self.map_path)

        self.map_view.setUrl(QUrl.fromLocalFile(self.map_path))
        self.lbl_map_status.setText(f"Mapa do robô (Lat: {lat_f:.6f}, Lon: {lon_f:.6f})")

    def closeEvent(self, event):
        self._closing = True
        if hasattr(self, "cpu_timer"):
            self.cpu_timer.stop()
        if hasattr(self, "ros_nodes_timer"):
            self.ros_nodes_timer.stop()
        if hasattr(self, "ia_monitor_timer"):
            self.ia_monitor_timer.stop()
        if hasattr(self, "capture_monitor_timer"):
            self.capture_monitor_timer.stop()
        if self._ros_nodes_worker is not None:
            self._ros_nodes_worker.requestInterruption()
            self._ros_nodes_worker.wait(500)
        self._parar_pipeline_ia()
        self._parar_captura_fotos()
        self._encerrar_rastreabilidade()
        for _nome, proc in list(self.processos.items()):
            if proc.state() != QProcess.NotRunning:
                proc.kill()
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = AgroRobotGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
