import sys
import os
import signal
from datetime import datetime
import threading
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dr_interfaces.srv import GenTraj, SaveTraj, ListTrajs, DeleteTraj, LoadTraj, SolveFK
from dr_interfaces.action import Execute as ExecuteAction
from dr_interfaces.msg import RobotState, TeachingDelta
from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget,
                             QGridLayout, QLabel, QPushButton, QDoubleSpinBox, QComboBox,
                             QLineEdit, QTableWidget, QTableWidgetItem, QSizePolicy, QHeaderView,
                             QVBoxLayout, QHBoxLayout, QDialog, QMessageBox, QProgressBar)
from PyQt5.QtCore import pyqtSignal, QTimer


class GUINode(Node):

    def __init__(self):
        super().__init__("gui_node")

        self.declare_parameter('ctrl_freq',    rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('q_max',        rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('q_min',        rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('qd_max',       rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('qdd_max',      rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('ef_v_max',     rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_a_max',     rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_omega_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ef_alpha_max', rclpy.Parameter.Type.DOUBLE)

        self.ctrl_t       = 1.0 / self.get_parameter('ctrl_freq').value
        self.q_max        = list(self.get_parameter('q_max').value)
        self.q_min        = list(self.get_parameter('q_min').value)
        self.qd_max       = list(self.get_parameter('qd_max').value)
        self.qdd_max      = list(self.get_parameter('qdd_max').value)
        self.ef_v_max     = self.get_parameter('ef_v_max').value
        self.ef_a_max     = self.get_parameter('ef_a_max').value
        self.ef_omega_max = self.get_parameter('ef_omega_max').value
        self.ef_alpha_max = self.get_parameter('ef_alpha_max').value

        self.window = None
        self.state_sub = self.create_subscription(RobotState, '/dr/robot_state', self.robot_state_callback, 10)
        self.teaching_pub = self.create_publisher(TeachingDelta, '/dr/teaching_delta', 10)
        self.gen_traj_client = self.create_client(GenTraj, '/dr/gen_traj')
        self.save_traj_client = self.create_client(SaveTraj, '/dr/save_traj')
        self.list_trajs_client = self.create_client(ListTrajs, '/dr/list_trajs')
        self.delete_traj_client = self.create_client(DeleteTraj, '/dr/delete_traj')
        self.load_traj_client = self.create_client(LoadTraj, '/dr/load_traj')
        self.fk_client = self.create_client(SolveFK, '/dr/solve_fk')
        self.execute_client = ActionClient(self, ExecuteAction, '/dr/execute')

    def robot_state_callback(self, msg):
        if self.window is not None:
            self.window.state_signal.emit(list(msg.q), list(msg.pose))

    def send_delta(self, axis, sign):
        step = sign * self.window.step_spinbox.value()
        msg = TeachingDelta()
        msg.dx     = step if axis == 'X'     else 0.0
        msg.dy     = step if axis == 'Y'     else 0.0
        msg.dz     = step if axis == 'Z'     else 0.0
        msg.droll  = step if axis == 'Roll'  else 0.0
        msg.dpitch = step if axis == 'Pitch' else 0.0
        msg.dyaw   = step if axis == 'Yaw'   else 0.0
        self.teaching_pub.publish(msg)

    def call_fk(self, q):
        request = SolveFK.Request()
        request.q = [float(x) for x in q]
        future = self.fk_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())
        event.wait()
        return list(future.result().pose)

    def finalizar(self, waypoints, nombre):
        modo_map  = {'Joint': 0, 'Linear': 1}
        speed_map = {'Lento': 1, 'Medio': 2, 'Rápido': 3}

        q_traj_total = []
        points_total = 0

        for wp in waypoints:
            request = GenTraj.Request()
            request.pose          = list(wp['pose'])
            request.mode          = modo_map[wp['modo']]
            request.speed_profile = speed_map[wp['speed']]

            future = self.gen_traj_client.call_async(request)
            event  = threading.Event()
            future.add_done_callback(lambda _: event.set())
            event.wait()

            result = future.result()
            if not result.success:
                return False, result.message

            q_traj_total += list(result.q_traj)
            points_total += result.points

        save_req        = SaveTraj.Request()
        save_req.name   = nombre
        save_req.q_traj = q_traj_total
        save_req.points = points_total

        future = self.save_traj_client.call_async(save_req)
        event  = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait()

        result = future.result()
        return result.success, result.message


class MainWindow(QMainWindow):

    state_signal          = pyqtSignal(list, list)
    finalizar_signal      = pyqtSignal(bool)
    refresh_signal        = pyqtSignal()
    status_signal         = pyqtSignal(str)
    progress_signal       = pyqtSignal(int)
    executing_signal      = pyqtSignal(bool)
    analyze_enable_signal = pyqtSignal(str)

    def __init__(self, node):
        super().__init__()

        self.node = node
        self.last_executed_nombre = None
        self.execution_data = None
        self.analyze_buttons = {}
        self._feedback_buffer = []

        self.setWindowTitle("DR - Panel de Control")
        self.setMinimumSize(800, 600)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QGridLayout()
        central_widget.setLayout(self.main_layout)

        self.tabs = QTabWidget()
        self.main_layout.addWidget(self.tabs, 0, 0)

        self.tab_teaching = QWidget()
        self.tab_execute  = QWidget()

        self.tabs.addTab(self.tab_teaching, "Teaching")
        self.tabs.addTab(self.tab_execute,  "Execute")

        # ── Teaching layout ──────────────────────────────────────────────────

        self.teaching_layout = QGridLayout()
        self.tab_teaching.setLayout(self.teaching_layout)

        self.state_table = QTableWidget(2, 6)
        self.state_table.setVerticalHeaderLabels(["q (rad)", "pose"])
        self.state_table.setHorizontalHeaderLabels(
            ["q1 / X", "q2 / Y", "q3 / Z", "q4 / Roll", "q5 / Pitch", "q6 / Yaw"])
        self.state_table.setEditTriggers(QTableWidget.NoEditTriggers)

        for i in range(6):
            self.state_table.setItem(0, i, QTableWidgetItem("-"))
            self.state_table.setItem(1, i, QTableWidgetItem("-"))

        self.teaching_layout.addWidget(self.state_table, 0, 0, 2, 4)
        self.state_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.state_table.resizeColumnsToContents()
        header = self.state_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        self.state_table.setFixedHeight(
            self.state_table.horizontalHeader().height() + self.state_table.rowHeight(0) * 2 + 2)

        self.state_signal.connect(self.update_state_table)
        self.finalizar_signal.connect(self.on_finalizar_done)
        self.waypoints = []
        self.current_pose = None

        ejes = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
        self.jog_buttons = {}

        for i, eje in enumerate(ejes):
            label   = QLabel(eje)
            btn_neg = QPushButton('-')
            btn_pos = QPushButton('+')
            self.jog_buttons[eje] = (btn_neg, btn_pos)
            self.teaching_layout.addWidget(label,   i + 2, 0)
            self.teaching_layout.addWidget(btn_neg, i + 2, 1)
            self.teaching_layout.addWidget(btn_pos, i + 2, 2)

        for eje, (btn_neg, btn_pos) in self.jog_buttons.items():
            btn_neg.clicked.connect(lambda _, e=eje: self.node.send_delta(e, -1))
            btn_pos.clicked.connect(lambda _, e=eje: self.node.send_delta(e, +1))

        label_step = QLabel("Step (m / rad):")
        self.step_spinbox = QDoubleSpinBox()
        self.step_spinbox.setMinimum(0.001)
        self.step_spinbox.setMaximum(0.1)
        self.step_spinbox.setSingleStep(0.001)
        self.step_spinbox.setValue(0.01)
        self.step_spinbox.setDecimals(3)

        self.teaching_layout.addWidget(label_step,        2, 3)
        self.teaching_layout.addWidget(self.step_spinbox, 3, 3)

        label_modo = QLabel("Modo:")
        self.combo_modo = QComboBox()
        self.combo_modo.addItems(["Joint", "Linear"])

        label_speed = QLabel("Speed profile:")
        self.combo_speed = QComboBox()
        self.combo_speed.addItems(["Lento", "Medio", "Rápido"])

        self.teaching_layout.addWidget(label_modo,       4, 3)
        self.teaching_layout.addWidget(self.combo_modo,  5, 3)
        self.teaching_layout.addWidget(label_speed,      6, 3)
        self.teaching_layout.addWidget(self.combo_speed, 7, 3)

        self.btn_capturar  = QPushButton("Capturar posición")
        self.btn_finalizar = QPushButton("Finalizar")

        self.btn_capturar.clicked.connect(self.capturar_waypoint)
        self.btn_finalizar.clicked.connect(self.on_finalizar)

        self.btn_ver_waypoints = QPushButton("Ver lista de waypoints")
        self.btn_ver_waypoints.clicked.connect(self.abrir_waypoints)
        self.teaching_layout.addWidget(self.btn_ver_waypoints, 8,  0, 1, 4)
        self.teaching_layout.addWidget(self.btn_capturar,       9,  0, 1, 4)

        label_nombre = QLabel("Nombre de trayectoria:")
        self.input_nombre = QLineEdit()
        self.input_nombre.setPlaceholderText("ej: trayectoria_1")

        self.teaching_layout.addWidget(label_nombre,      10, 0, 1, 2)
        self.teaching_layout.addWidget(self.input_nombre, 10, 2, 1, 2)
        self.teaching_layout.addWidget(self.btn_finalizar, 11, 0, 1, 4)

        # ── Execute layout ───────────────────────────────────────────────────

        self.execute_layout = QGridLayout()
        self.tab_execute.setLayout(self.execute_layout)

        self.execute_state_table = QTableWidget(2, 6)
        self.execute_state_table.setVerticalHeaderLabels(["q (rad)", "pose"])
        self.execute_state_table.setHorizontalHeaderLabels(
            ["q1 / X", "q2 / Y", "q3 / Z", "q4 / Roll", "q5 / Pitch", "q6 / Yaw"])
        self.execute_state_table.setEditTriggers(QTableWidget.NoEditTriggers)

        for i in range(6):
            self.execute_state_table.setItem(0, i, QTableWidgetItem("-"))
            self.execute_state_table.setItem(1, i, QTableWidgetItem("-"))

        self.execute_layout.addWidget(self.execute_state_table, 0, 0, 2, 4)
        self.execute_state_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        header = self.execute_state_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        self.execute_state_table.setFixedHeight(
            self.execute_state_table.horizontalHeader().height() +
            self.execute_state_table.rowHeight(0) * 2 + 2)

        self.traj_table = QTableWidget(0, 5)
        self.traj_table.setHorizontalHeaderLabels(
            ["Nombre", "Fecha", "Puntos", "Acciones", "Analizar"])
        self.traj_table.setEditTriggers(QTableWidget.NoEditTriggers)
        header = self.traj_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)

        self.execute_layout.addWidget(self.traj_table, 2, 0, 1, 4)

        self.btn_refresh = QPushButton("Actualizar lista")
        self.btn_refresh.clicked.connect(self.cargar_trayectorias)
        self.execute_layout.addWidget(self.btn_refresh, 3, 0, 1, 4)

        self.refresh_signal.connect(self.cargar_trayectorias)

        self.status_label = QLabel("Esperando comando...")
        self.execute_layout.addWidget(self.status_label, 4, 0, 1, 4)

        self.progress_bar = QProgressBar()
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        self.progress_bar.setValue(0)
        self.execute_layout.addWidget(self.progress_bar, 5, 0, 1, 4)

        self.status_signal.connect(self.status_label.setText)
        self.progress_signal.connect(self.progress_bar.setValue)
        self.executing_signal.connect(
            lambda executing: self.tab_teaching.setEnabled(not executing))
        self.analyze_enable_signal.connect(self._enable_analyze_button)

    def _enable_analyze_button(self, nombre):
        for n, btn in self.analyze_buttons.items():
            btn.setEnabled(n == nombre)

    def recorrer_trayectoria(self, nombre):
        self.waypoints = []
        self._feedback_buffer = []
        self.executing_signal.emit(True)
        self.status_signal.emit(f"Cargando '{nombre}'...")
        thread = threading.Thread(
            target=self._recorrer_thread, args=(nombre,), daemon=True)
        thread.start()

    def _recorrer_thread(self, nombre):
        load_req = LoadTraj.Request()
        load_req.name = nombre
        future = self.node.load_traj_client.call_async(load_req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait()

        result = future.result()
        if not result.success:
            self.status_signal.emit("Error al cargar trayectoria.")
            self.executing_signal.emit(False)
            return

        goal = ExecuteAction.Goal()
        goal.q_traj = result.q_traj
        goal.points = result.points

        self.status_signal.emit(f"Recorriendo '{nombre}'...")
        self.progress_signal.emit(0)

        send_future = self.node.execute_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        event2 = threading.Event()
        send_future.add_done_callback(lambda _: event2.set())
        event2.wait()

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.status_signal.emit("Goal rechazado.")
            self.executing_signal.emit(False)
            return

        result_future = goal_handle.get_result_async()
        event3 = threading.Event()
        result_future.add_done_callback(lambda _: event3.set())
        event3.wait()

        res = result_future.result().result
        if res.success:
            self.status_signal.emit("Trayectoria completada.")
            self.progress_signal.emit(100)
            self._store_execution_data(nombre)
            self.analyze_enable_signal.emit(nombre)
        else:
            self.status_signal.emit(f"Error: {res.message}")
        self.executing_signal.emit(False)

    def _store_execution_data(self, nombre):
        buf = self._feedback_buffer
        if not buf:
            return
        q_traj  = np.array([f['q']   for f in buf])
        qd_traj = np.array([f['qd']  for f in buf])
        qdd_traj = np.array([f['qdd'] for f in buf])
        self.last_executed_nombre = nombre
        self.execution_data = {
            'q_traj':   q_traj,
            'qd_traj':  qd_traj,
            'qdd_traj': qdd_traj,
        }

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.progress_signal.emit(int(fb.progress * 100))
        self._feedback_buffer.append({
            'q':   list(fb.q),
            'qd':  list(fb.qd),
            'qdd': list(fb.qdd),
        })

    def abrir_analisis(self, nombre):
        if self.execution_data is None or nombre != self.last_executed_nombre:
            return
        params = {
            'q_max':        self.node.q_max,
            'q_min':        self.node.q_min,
            'qd_max':       self.node.qd_max,
            'qdd_max':      self.node.qdd_max,
            'ef_v_max':     self.node.ef_v_max,
            'ef_a_max':     self.node.ef_a_max,
            'ef_omega_max': self.node.ef_omega_max,
            'ef_alpha_max': self.node.ef_alpha_max,
        }
        dialog = AnalysisDialog(
            q_traj=self.execution_data['q_traj'],
            qd_traj=self.execution_data['qd_traj'],
            qdd_traj=self.execution_data['qdd_traj'],
            ctrl_t=self.node.ctrl_t,
            params=params,
            fk_func=self.node.call_fk,
            nombre=nombre,
            parent=self,
        )
        dialog.show()

    def eliminar_trayectoria(self, nombre):
        reply = QMessageBox.question(
            self, "Confirmar", f"¿Eliminar la trayectoria '{nombre}'?",
            QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        thread = threading.Thread(
            target=self._eliminar_thread, args=(nombre,), daemon=True)
        thread.start()

    def _eliminar_thread(self, nombre):
        request = DeleteTraj.Request()
        request.name = nombre
        future = self.node.delete_traj_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait()
        self.refresh_signal.emit()

    def cargar_trayectorias(self):
        TRAJ_PATH = os.path.expanduser('~/ros2_PFE/src/trajectories')

        request = ListTrajs.Request()
        future = self.node.list_trajs_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait()

        if not future.result().success:
            return

        nombres = future.result().names
        self.traj_table.setRowCount(len(nombres))
        self.analyze_buttons = {}

        for i, nombre in enumerate(nombres):
            filepath = os.path.join(TRAJ_PATH, f'{nombre}.json')
            fecha = datetime.fromtimestamp(
                os.path.getmtime(filepath)).strftime('%Y-%m-%d %H:%M')

            load_req = LoadTraj.Request()
            load_req.name = nombre
            future = self.node.load_traj_client.call_async(load_req)
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            event.wait()
            puntos = future.result().points

            self.traj_table.setItem(i, 0, QTableWidgetItem(nombre))
            self.traj_table.setItem(i, 1, QTableWidgetItem(fecha))
            self.traj_table.setItem(i, 2, QTableWidgetItem(str(puntos)))

            btn_recorrer = QPushButton("Recorrer")
            btn_recorrer.clicked.connect(
                lambda _, n=nombre: self.recorrer_trayectoria(n))
            btn_eliminar = QPushButton("Eliminar")
            btn_eliminar.clicked.connect(
                lambda _, n=nombre: self.eliminar_trayectoria(n))

            acciones_widget = QWidget()
            acciones_layout = QHBoxLayout(acciones_widget)
            acciones_layout.setContentsMargins(0, 0, 0, 0)
            acciones_layout.addWidget(btn_recorrer)
            acciones_layout.addWidget(btn_eliminar)
            self.traj_table.setCellWidget(i, 3, acciones_widget)

            btn_analizar = QPushButton("Analizar")
            btn_analizar.setEnabled(nombre == self.last_executed_nombre
                                    and self.execution_data is not None)
            btn_analizar.clicked.connect(
                lambda _, n=nombre: self.abrir_analisis(n))
            self.analyze_buttons[nombre] = btn_analizar

            analizar_widget = QWidget()
            analizar_layout = QHBoxLayout(analizar_widget)
            analizar_layout.setContentsMargins(0, 0, 0, 0)
            analizar_layout.addWidget(btn_analizar)
            self.traj_table.setCellWidget(i, 4, analizar_widget)

    def update_state_table(self, q, pose):
        self.current_pose = pose
        for i in range(6):
            self.state_table.setItem(0, i, QTableWidgetItem(f"{q[i]:.4f}"))
            self.state_table.setItem(1, i, QTableWidgetItem(f"{pose[i]:.4f}"))
            self.execute_state_table.setItem(0, i, QTableWidgetItem(f"{q[i]:.4f}"))
            self.execute_state_table.setItem(1, i, QTableWidgetItem(f"{pose[i]:.4f}"))

    def abrir_waypoints(self):
        dialog = WaypointDialog(self.waypoints, parent=self)
        dialog.exec_()

    def capturar_waypoint(self):
        if self.current_pose is None:
            return
        wp = {
            'pose':  list(self.current_pose),
            'modo':  self.combo_modo.currentText(),
            'speed': self.combo_speed.currentText(),
        }
        self.waypoints.append(wp)

    def on_finalizar(self):
        nombre = self.input_nombre.text().strip()
        if not nombre:
            QMessageBox.warning(self, "Campo requerido",
                                "Ingresá un nombre para la trayectoria.")
            return
        if not self.waypoints:
            QMessageBox.warning(self, "Sin waypoints",
                                "Capturá al menos una posición antes de finalizar.")
            return
        thread = threading.Thread(
            target=self._finalizar_thread, args=(nombre,), daemon=True)
        thread.start()

    def _finalizar_thread(self, nombre):
        success, _ = self.node.finalizar(self.waypoints, nombre)
        self.finalizar_signal.emit(success)

    def on_finalizar_done(self, success):
        if success:
            self.waypoints = []
            self.input_nombre.clear()


# ── AnalysisDialog ────────────────────────────────────────────────────────────

class AnalysisDialog(QDialog):

    _data_ready = pyqtSignal(object)

    def __init__(self, q_traj, qd_traj, qdd_traj, ctrl_t, params, fk_func,
                 nombre, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Análisis — {nombre}")
        self.setMinimumSize(1300, 750)
        self.setAttribute(0x00000002)  # Qt.WA_DeleteOnClose

        self.q_traj   = q_traj
        self.qd_traj  = qd_traj
        self.qdd_traj = qdd_traj
        self.ctrl_t   = ctrl_t
        self.params   = params
        self.fk_func  = fk_func

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.loading_label = QLabel("Calculando poses cartesianas, por favor espere...")
        layout.addWidget(self.loading_label)

        self.analysis_tabs = QTabWidget()
        self.analysis_tabs.hide()
        layout.addWidget(self.analysis_tabs)

        self._data_ready.connect(self._build_tabs)
        threading.Thread(target=self._compute, daemon=True).start()

    def _compute(self):
        N = len(self.q_traj)
        c_traj   = np.array([self.fk_func(self.q_traj[i]) for i in range(N)])
        cd_traj  = np.gradient(c_traj,  self.ctrl_t, axis=0)
        cdd_traj = np.gradient(cd_traj, self.ctrl_t, axis=0)
        self._data_ready.emit({
            'c_traj':   c_traj,
            'cd_traj':  cd_traj,
            'cdd_traj': cdd_traj,
            'v_lin': np.linalg.norm(cd_traj[:,  :3], axis=1),
            'v_ang': np.linalg.norm(cd_traj[:,  3:], axis=1),
            'a_lin': np.linalg.norm(cdd_traj[:, :3], axis=1),
            'a_ang': np.linalg.norm(cdd_traj[:, 3:], axis=1),
        })

    def _build_tabs(self, data):
        self.loading_label.hide()
        t = np.arange(len(self.q_traj)) * self.ctrl_t

        self.analysis_tabs.addTab(
            self._tab_coordenadas(t, data['c_traj']),
            "Coordenadas")
        self.analysis_tabs.addTab(
            self._tab_articular(t),
            "Dinámica articular")
        self.analysis_tabs.addTab(
            self._tab_ef_lineal(t, data['v_lin'], data['a_lin']),
            "EF Lineal")
        self.analysis_tabs.addTab(
            self._tab_ef_angular(t, data['v_ang'], data['a_ang']),
            "EF Angular")

        self.analysis_tabs.show()

    # ── helpers para figuras embebidas ────────────────────────────────────────

    @staticmethod
    def _make_canvas(nrows, ncols, figsize):
        fig = Figure(figsize=figsize, tight_layout=True)
        axes = fig.subplots(nrows, ncols)
        canvas = FigureCanvas(fig)
        canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        return canvas, axes

    # ── Tab 1: Coordenadas ────────────────────────────────────────────────────

    def _tab_coordenadas(self, t, c_traj):
        widget = QWidget()
        layout = QHBoxLayout(widget)

        # columna izquierda: posición articular
        canvas_q, axes_q = self._make_canvas(2, 3, (7, 7))
        jlabels = [f'q{i+1}' for i in range(6)]
        for i, ax in enumerate(np.array(axes_q).flat):
            ax.plot(t, self.q_traj[:, i], label='q')
            ax.axhline(self.params['q_max'][i], color='r',
                       linestyle='--', label='q_max')
            ax.axhline(self.params['q_min'][i], color='g',
                       linestyle='--', label='q_min')
            ax.set_title(jlabels[i])
            ax.set_xlabel('t [s]')
            ax.set_ylabel('[rad]')
            ax.legend(fontsize=7)
            ax.grid(True)

        # columna derecha: pose cartesiana
        canvas_c, axes_c = self._make_canvas(2, 3, (7, 7))
        clabels = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        cunits  = ['m', 'm', 'm', 'rad', 'rad', 'rad']
        for i, ax in enumerate(np.array(axes_c).flat):
            ax.plot(t, c_traj[:, i])
            ax.set_title(clabels[i])
            ax.set_xlabel('t [s]')
            ax.set_ylabel(f'[{cunits[i]}]')
            ax.grid(True)

        layout.addWidget(canvas_q)
        layout.addWidget(canvas_c)
        return widget

    # ── Tab 2: Dinámica articular ─────────────────────────────────────────────

    def _tab_articular(self, t):
        widget = QWidget()
        layout = QHBoxLayout(widget)

        jlabels = [f'q{i+1}' for i in range(6)]

        # columna izquierda: velocidad articular
        canvas_v, axes_v = self._make_canvas(2, 3, (7, 7))
        for i, ax in enumerate(np.array(axes_v).flat):
            ax.plot(t, self.qd_traj[:, i], label='qd')
            ax.axhline( self.params['qd_max'][i], color='r',
                        linestyle='--', label='qd_max')
            ax.axhline(-self.params['qd_max'][i], color='r', linestyle='--')
            ax.set_title(jlabels[i])
            ax.set_xlabel('t [s]')
            ax.set_ylabel('[rad/s]')
            ax.legend(fontsize=7)
            ax.grid(True)

        # columna derecha: aceleración articular
        canvas_a, axes_a = self._make_canvas(2, 3, (7, 7))
        for i, ax in enumerate(np.array(axes_a).flat):
            ax.plot(t, self.qdd_traj[:, i], label='qdd')
            ax.axhline( self.params['qdd_max'][i], color='r',
                        linestyle='--', label='qdd_max')
            ax.axhline(-self.params['qdd_max'][i], color='r', linestyle='--')
            ax.set_title(jlabels[i])
            ax.set_xlabel('t [s]')
            ax.set_ylabel('[rad/s²]')
            ax.legend(fontsize=7)
            ax.grid(True)

        layout.addWidget(canvas_v)
        layout.addWidget(canvas_a)
        return widget

    # ── Tab 3: EF Lineal ──────────────────────────────────────────────────────

    def _tab_ef_lineal(self, t, v_lin, a_lin):
        widget = QWidget()
        layout = QHBoxLayout(widget)

        fig_v = Figure(figsize=(6, 4), tight_layout=True)
        ax_v = fig_v.add_subplot(1, 1, 1)
        ax_v.plot(t, v_lin, label='v_lin')
        ax_v.axhline(self.params['ef_v_max'], color='r', linestyle='--',
                     label=f"v_max = {self.params['ef_v_max']}")
        ax_v.set_title('Velocidad lineal EF')
        ax_v.set_xlabel('t [s]')
        ax_v.set_ylabel('[m/s]')
        ax_v.legend()
        ax_v.grid(True)
        canvas_v = FigureCanvas(fig_v)
        canvas_v.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        fig_a = Figure(figsize=(6, 4), tight_layout=True)
        ax_a = fig_a.add_subplot(1, 1, 1)
        ax_a.plot(t, a_lin, label='a_lin')
        ax_a.axhline(self.params['ef_a_max'], color='r', linestyle='--',
                     label=f"a_max = {self.params['ef_a_max']}")
        ax_a.set_title('Aceleración lineal EF')
        ax_a.set_xlabel('t [s]')
        ax_a.set_ylabel('[m/s²]')
        ax_a.legend()
        ax_a.grid(True)
        canvas_a = FigureCanvas(fig_a)
        canvas_a.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout.addWidget(canvas_v)
        layout.addWidget(canvas_a)
        return widget

    # ── Tab 4: EF Angular ─────────────────────────────────────────────────────

    def _tab_ef_angular(self, t, v_ang, a_ang):
        widget = QWidget()
        layout = QHBoxLayout(widget)

        fig_v = Figure(figsize=(6, 4), tight_layout=True)
        ax_v = fig_v.add_subplot(1, 1, 1)
        ax_v.plot(t, v_ang, label='v_ang')
        ax_v.axhline(self.params['ef_omega_max'], color='r', linestyle='--',
                     label=f"omega_max = {self.params['ef_omega_max']}")
        ax_v.set_title('Velocidad angular EF')
        ax_v.set_xlabel('t [s]')
        ax_v.set_ylabel('[rad/s]')
        ax_v.legend()
        ax_v.grid(True)
        canvas_v = FigureCanvas(fig_v)
        canvas_v.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        fig_a = Figure(figsize=(6, 4), tight_layout=True)
        ax_a = fig_a.add_subplot(1, 1, 1)
        ax_a.plot(t, a_ang, label='a_ang')
        ax_a.axhline(self.params['ef_alpha_max'], color='r', linestyle='--',
                     label=f"alpha_max = {self.params['ef_alpha_max']}")
        ax_a.set_title('Aceleración angular EF')
        ax_a.set_xlabel('t [s]')
        ax_a.set_ylabel('[rad/s²]')
        ax_a.legend()
        ax_a.grid(True)
        canvas_a = FigureCanvas(fig_a)
        canvas_a.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout.addWidget(canvas_v)
        layout.addWidget(canvas_a)
        return widget


# ── WaypointDialog ────────────────────────────────────────────────────────────

class WaypointDialog(QDialog):

    def __init__(self, waypoints, parent=None):
        super().__init__(parent)
        self.waypoints = waypoints
        self.setWindowTitle("Lista de Waypoints")
        self.setMinimumSize(700, 400)

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Posición", "Modo", "Speed", ""])
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)

        layout.addWidget(self.table)
        self.refresh_table()

    def refresh_table(self):
        self.table.setRowCount(len(self.waypoints))
        for i, wp in enumerate(self.waypoints):
            pose_str = ", ".join([f"{v:.3f}" for v in wp['pose']])
            self.table.setItem(i, 0, QTableWidgetItem(pose_str))
            self.table.setItem(i, 1, QTableWidgetItem(wp['modo']))
            self.table.setItem(i, 2, QTableWidgetItem(wp['speed']))
            btn_eliminar = QPushButton("Eliminar")
            btn_eliminar.clicked.connect(lambda _, idx=i: self.eliminar_waypoint(idx))
            self.table.setCellWidget(i, 3, btn_eliminar)

    def eliminar_waypoint(self, idx):
        self.waypoints.pop(idx)
        self.refresh_table()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):

    rclpy.init(args=args)
    node = GUINode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    node.window = window
    window.show()

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(200)

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()
