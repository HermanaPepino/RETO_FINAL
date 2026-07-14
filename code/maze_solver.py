#!/usr/bin/env python3
"""
maze_solver.py — Gran Prix CapyTown · G1
Navegación autónoma en laberinto con fusión LiDAR + cámara (señal PARE).

FIXES v3:
  - Giro por perpendicular a paredes (no yaw fijo de 90°)
  - Cap de lecturas inf del LiDAR
  - Cooldown se resetea al TERMINAR el giro, no al entrar a intersección
  - Dead-end: umbral >= en lugar de > para no fallar en el límite exacto
  - Timeout de giro (20s máx) para no quedarse girando infinito
  - CSV path corregido a /ros2_ws/

Suscribe : /scan · /odom_raw · /pare_detectado · /meta_detectado
Publica  : /cmd_vel
CSV      : /ros2_ws/metricas_granprix.csv

AJUSTES v7:
  - Seguridad anti-colisión también durante GIRAR
  - Memoria simple de celdas para evitar volver por donde vino
  - Inicialización correcta de odometría para métricas
"""

import csv
import json
import math
import os
import random
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


# ═══════════════════════════════════════════════════════════════
#  UTILIDADES LIDAR
# ═══════════════════════════════════════════════════════════════

def _sector(ranges, angle_min, angle_increment,
            range_min, range_max, deg_min, deg_max, cap=3.0):
    """Devuelve rangos válidos del sector, capando inf y lecturas > cap."""
    rad_min = math.radians(deg_min)
    rad_max = math.radians(deg_max)
    vals = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
        if r < max(range_min, 0.08) or r > min(range_max, cap):
            continue
        ang = math.atan2(
            math.sin(angle_min + i * angle_increment),
            math.cos(angle_min + i * angle_increment))
        if rad_min <= ang <= rad_max:
            vals.append(r)
    return vals


def _percentil(vals, pct):
    if not vals:
        return float('inf')
    s = sorted(vals)
    return s[int((len(s) - 1) * pct / 100.0)]


def _mediana(vals):
    return _percentil(vals, 50)


def _yaw_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def _norm(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ═══════════════════════════════════════════════════════════════
#  ESTADOS FSM
# ═══════════════════════════════════════════════════════════════

class Estado(Enum):
    EN_PASILLO   = auto()
    INTERSECCION = auto()
    PARAR_PARE   = auto()
    ESPERAR_3S   = auto()
    DECIDIR_GIRO = auto()
    GIRAR        = auto()
    META         = auto()


# ═══════════════════════════════════════════════════════════════
#  NODO
# ═══════════════════════════════════════════════════════════════

class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        # ── Parámetros compatibles con calibrar_granprix.sh ──────────
        self.declare_parameter('active',                  True)
        self.declare_parameter('v_forward',               0.050)
        self.declare_parameter('turn_speed',              0.15)
        self.declare_parameter('kp_wall',                 0.18)
        self.declare_parameter('wall_target',             0.28)
        self.declare_parameter('front_stop',              0.42)
        self.declare_parameter('side_open',               1.10)
        self.declare_parameter('intersection_cooldown_s', 6.5)
        self.declare_parameter('memory_node_radius',       0.45)
        self.declare_parameter('post_turn_guard_s',        1.50)
        self.declare_parameter('post_turn_stop_s',         0.40)
        self.declare_parameter('node_exit_repeat_penalty', 2.50)

        # Evita detectar una abertura lateral apenas arranca el nodo.
        self.declare_parameter('startup_side_guard_s',      6.50)
        self.declare_parameter('startup_min_travel_m',      0.25)

        # Una abertura lateral debe mantenerse estable y luego el robot
        # avanza un poco hasta el centro de la intersección antes de girar.
        self.declare_parameter('side_confirm_ticks',        7)

        # Rearme de intersección lateral:
        # no se acepta otra abertura hasta haber vuelto a ver una pared
        # derecha estable y haberse alejado del nodo anterior.
        self.declare_parameter('side_rearm_wall_max',       0.72)
        self.declare_parameter('side_rearm_ticks',          6)
        self.declare_parameter('intersection_min_exit_m',   0.48)

        # Centrado por distancia real, no solo por tiempo.
        self.declare_parameter('intersection_center_distance_m', 0.11)
        self.declare_parameter('intersection_center_timeout_s',  3.50)
        self.declare_parameter('intersection_center_v',     0.040)
        self.declare_parameter('intersection_center_front_min', 0.58)

        self.declare_parameter('turn_slowdown_deg',        25.0)
        self.declare_parameter('turn_near_speed',          0.10)
        self.declare_parameter('turn_brake_margin_deg',     0.5)

        # ── Parámetros de META y ronda ────────────────────────────────
        self.declare_parameter('x_meta',     3.20)
        self.declare_parameter('y_meta',     2.00)
        self.declare_parameter('radio_meta', 0.25)
        self.declare_parameter('ronda',      1)
        self.declare_parameter('meta_forward_s', 4.0)
        self.declare_parameter('pare_cooldown_s', 8.0)
        self.declare_parameter('giro_front_safety', 0.22)
        self.declare_parameter('giro_reverse_s', 0.70)
        self.declare_parameter('stuck_node_visits', 3)
        # ── Aprendizaje anti-bucle tipo Q-learning tabular ─────────────
        self.declare_parameter('qlearn_enabled', True)
        self.declare_parameter('q_alpha', 0.45)
        self.declare_parameter('q_epsilon', 0.10)
        self.declare_parameter('q_new_state_reward', 1.50)
        self.declare_parameter('q_repeat_penalty', -7.0)
        self.declare_parameter('q_timeout_penalty', -10.0)
        self.declare_parameter('q_deadend_penalty', -8.0)
        self.declare_parameter('q_meta_reward', 100.0)
        self.declare_parameter('q_state_grid_m', 0.45)
        self.declare_parameter('q_path', '/ros2_ws/q_table_granprix.json')
        self.declare_parameter('q_progress_reward', 0.25)

        # ── Control físico estable ────────────────────────────────────
        self.declare_parameter('giro_yaw_min_deg', 84.0)
        self.declare_parameter('media_yaw_min_deg', 165.0)
        self.declare_parameter('giro_timeout_s', 16.0)
        self.declare_parameter('media_timeout_s', 28.0)
        self.declare_parameter('lidar_hold_s', 0.60)
        self.declare_parameter('scan_timeout_s', 0.80)

        # META no puede activarse apenas arranca el nodo.
        self.declare_parameter('meta_min_runtime_s', 25.0)
        self.declare_parameter('meta_min_distance_m', 2.50)
        self.declare_parameter('meta_front_stop', 0.16)
        self.declare_parameter('meta_confirm_s', 0.80)

        # ── Valores fijos ─────────────────────────────────────────────
        self.dist_pared_max   = 0.70   # más lejos → sin pared
        self.dist_pared_min   = 0.12   # demasiado cerca
        self.frente_libre     = 0.50   # frente despejado para alineación
        self.vel_lenta_factor = 0.6
        self.tiempo_pare      = 3.0
        self.giro_timeout     = 16.0

        # ── Estado sensores ───────────────────────────────────────────
        self.d_frente     = float('inf')
        self.d_derecha    = float('inf')
        self.d_izquierda  = float('inf')
        self.frente_bloqueado = False
        self.frente_libre_ok  = False
        self.frente_valido = False
        self.derecha_valida = False
        self.izquierda_valida = False
        self.scan_listo   = False
        self.t_ultimo_scan = 0.0
        self.scan_timeout_latched = False

        # Últimas lecturas válidas para absorber pérdidas breves del LiDAR.
        self._lidar_last = {'F': float('inf'), 'D': float('inf'), 'I': float('inf')}
        self._lidar_t = {'F': 0.0, 'D': 0.0, 'I': 0.0}

        # Pose RELATIVA al punto/orientación donde se inició esta corrida.
        self.pos_x  = 0.0
        self.pos_y  = 0.0
        self.yaw    = 0.0
        self.odom_origen_x = None
        self.odom_origen_y = None
        self.odom_origen_yaw = None
        self.prev_raw_x = None
        self.prev_raw_y = None

        self.pare_flag = False
        self.pare_atendido = False
        self.meta_flag = False
        self.meta_latched = False
        self.meta_true_since = None
        self.meta_candidate_logged = False
        self.meta_finalizado = False
        self.t_meta = None
        self.t_ultimo_pare = 0.0
        self.t_ultimo_pare_detectado = 0.0
        self.t_seguridad_retroceso = None
        self.giro_retroceso_hecho = False
        self.giro_retroceso_inicio = None

        # ── FSM ───────────────────────────────────────────────────────
        self.estado       = Estado.EN_PASILLO
        self.giro_dir     = None      # 'derecha' | 'izquierda' | 'media'
        self.yaw_objetivo = None      # objetivo de giro
        self.yaw_inicio_giro = None    # yaw al iniciar giro
        self.t_pare       = None
        self.t_estado     = self.get_clock().now()
        self.t_log        = self.get_clock().now()
        # El cooldown también se aplica al inicio para no girar por una
        # lectura lateral grande apenas aparece el primer LaserScan.
        self.t_ultima_interseccion = time.time()
        self.t_fin_ultimo_giro = 0.0
        self.apertura_der_ticks = 0
        self.frente_cerca_ticks = 0

        # La abertura derecha solo queda habilitada después de observar
        # nuevamente una pared derecha durante varios ciclos.
        self.side_open_armed = False
        self.right_wall_ticks = 0

        # Posición del último nodo detectado. Evita detectar dos veces
        # la misma abertura mientras el robot todavía está dentro de ella.
        self.last_intersection_x = None
        self.last_intersection_y = None

        self.interseccion_motivo = None
        self.t_inicio_centrado = None
        self.centrado_inicio_x = None
        self.centrado_inicio_y = None

        # ── Memoria real anti-bucle ───────────────────────────────────
        # No memoriza el mapa fijo. Construye nodos de intersección con odometría.
        # Si vuelve cerca de una intersección ya creada, reutiliza el mismo nodo.
        self.cell_size = 0.60
        self.node_radius = 0.38
        self.visitas_celdas = {}
        self.nodos_memoria = []       # [{'id', 'x', 'y', 'visitas', 'acciones'}]
        self.nodo_anterior_id = None
        self.nodo_actual_id = None
        self.celda_interseccion_anterior = None
        self.ultima_accion = None
        # DFS por aristas físicas: cada salida se recuerda por dirección absoluta
        # (N/E/S/O), no como 'derecha/izquierda' relativa al robot.
        self.ultimo_nodo_salida = None
        self.ultima_direccion_salida = None

        # Memoria Trémaux de la corrida actual.
        # Registra aristas físicas, entradas a nodos y ramas que terminan
        # en callejón. No depende de que la Q-table reconozca exactamente
        # la misma celda después de la deriva de odometría.
        self.ultima_salida_fisica = None
        self.deadend_retorno_pendiente = False
        self.deadend_direccion_prohibida = None
        self.deadend_nodo_origen_id = None

        # ── Q-learning tabular anti-loop ───────────────────────────────
        # Estado aprendido: celda aproximada + heading + aperturas LiDAR.
        # Esto reduce dependencia de los nodos por odometría fina.
        self.q_table = {}
        self.ultima_decision_q = None   # {'state','action','cell','node_id','t'}
        self.historial_estados_q = []   # últimos estados/celdas visitados
        self.q_updates_desde_guardado = 0
        self.fallos_control_recientes = {}
        self._cargar_q_table()

        # Odometría: evita sumar un salto grande al primer mensaje.
        self.odom_iniciado = False
        self.csv_guardado = False

        # ── Métricas ─────────────────────────────────────────────────
        self.t_inicio        = time.time()
        self.dist_odom_cm    = 0.0
        self.colisiones      = 0
        self.pare_detectados = 0
        self.pare_respetados = 0
        self.pare_falsos     = 0
        self.dead_ends       = 0

        # ── ROS ───────────────────────────────────────────────────────
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan',           self.cb_scan, 10)
        self.create_subscription(Odometry,  '/odom_raw',       self.cb_odom, 10)
        self.create_subscription(Bool,      '/pare_detectado', self.cb_pare, 10)
        self.create_subscription(Bool,      '/meta_detectado', self.cb_meta, 10)
        self.timer = self.create_timer(0.1, self.loop_control)

        self.get_logger().info(
            'MazeSolver G1 v7 listo — Q-learning estable + odometría relativa + giros por yaw.')

    # ──────────────────────────────────────────────────────────
    #  LEER PARÁMETROS EN VIVO
    # ──────────────────────────────────────────────────────────

    def _p(self):
        return {
            'active': self.get_parameter('active').value,
            'v_fwd': self.get_parameter('v_forward').value,
            'turn': self.get_parameter('turn_speed').value,
            'kp': self.get_parameter('kp_wall').value,
            'target': self.get_parameter('wall_target').value,
            'f_stop': self.get_parameter('front_stop').value,
            's_open': self.get_parameter('side_open').value,
            'cooldown': self.get_parameter('intersection_cooldown_s').value,
            'mem_radius': self.get_parameter('memory_node_radius').value,
            'post_turn_guard_s': self.get_parameter('post_turn_guard_s').value,
            'post_turn_stop_s': self.get_parameter('post_turn_stop_s').value,
            'node_exit_repeat_penalty': self.get_parameter('node_exit_repeat_penalty').value,
            'startup_side_guard_s': self.get_parameter('startup_side_guard_s').value,
            'startup_min_travel_m': self.get_parameter('startup_min_travel_m').value,
            'side_confirm_ticks': self.get_parameter('side_confirm_ticks').value,
            'side_rearm_wall_max': self.get_parameter('side_rearm_wall_max').value,
            'side_rearm_ticks': self.get_parameter('side_rearm_ticks').value,
            'intersection_min_exit_m': self.get_parameter('intersection_min_exit_m').value,
            'intersection_center_distance_m': self.get_parameter('intersection_center_distance_m').value,
            'intersection_center_timeout_s': self.get_parameter('intersection_center_timeout_s').value,
            'intersection_center_v': self.get_parameter('intersection_center_v').value,
            'intersection_center_front_min': self.get_parameter('intersection_center_front_min').value,
            'turn_slowdown_deg': self.get_parameter('turn_slowdown_deg').value,
            'turn_near_speed': self.get_parameter('turn_near_speed').value,
            'turn_brake_margin_deg': self.get_parameter('turn_brake_margin_deg').value,
            'x_meta': self.get_parameter('x_meta').value,
            'y_meta': self.get_parameter('y_meta').value,
            'r_meta': self.get_parameter('radio_meta').value,
            'meta_forward_s': self.get_parameter('meta_forward_s').value,
            'pare_cooldown': self.get_parameter('pare_cooldown_s').value,
            'giro_front_safety': self.get_parameter('giro_front_safety').value,
            'giro_reverse_s': self.get_parameter('giro_reverse_s').value,
            'stuck_visits': self.get_parameter('stuck_node_visits').value,
            'qlearn_enabled': self.get_parameter('qlearn_enabled').value,
            'q_alpha': self.get_parameter('q_alpha').value,
            'q_epsilon': self.get_parameter('q_epsilon').value,
            'q_new_state_reward': self.get_parameter('q_new_state_reward').value,
            'q_progress_reward': self.get_parameter('q_progress_reward').value,
            'q_repeat_penalty': self.get_parameter('q_repeat_penalty').value,
            'q_timeout_penalty': self.get_parameter('q_timeout_penalty').value,
            'q_deadend_penalty': self.get_parameter('q_deadend_penalty').value,
            'q_meta_reward': self.get_parameter('q_meta_reward').value,
            'giro_yaw_min_deg': self.get_parameter('giro_yaw_min_deg').value,
            'media_yaw_min_deg': self.get_parameter('media_yaw_min_deg').value,
            'giro_timeout_s': self.get_parameter('giro_timeout_s').value,
            'media_timeout_s': self.get_parameter('media_timeout_s').value,
            'lidar_hold_s': self.get_parameter('lidar_hold_s').value,
            'scan_timeout_s': self.get_parameter('scan_timeout_s').value,
            'meta_min_runtime_s': self.get_parameter('meta_min_runtime_s').value,
            'meta_min_distance_m': self.get_parameter('meta_min_distance_m').value,
            'meta_front_stop': self.get_parameter('meta_front_stop').value,
            'meta_confirm_s': self.get_parameter('meta_confirm_s').value,
        }

    # ──────────────────────────────────────────────────────────
    #  CALLBACKS
    # ──────────────────────────────────────────────────────────

    def cb_scan(self, msg: LaserScan):
        r, ai, di = msg.ranges, msg.angle_min, msg.angle_increment
        rn, rx = msg.range_min, msg.range_max
        p = self._p()
        ahora = time.time()
        self.t_ultimo_scan = ahora

        frente_vals = _sector(r, ai, di, rn, rx, -7, 7)
        derecha_vals = _sector(r, ai, di, rn, rx, -115, -55)
        izq_vals = _sector(r, ai, di, rn, rx, 55, 115)

        def lectura_estable(clave, vals, estimador):
            # Exige varias muestras; si faltan por un instante conserva la última
            # lectura válida. Después del hold se marca como desconocida (inf).
            if len(vals) >= 3:
                valor = estimador(vals)
                self._lidar_last[clave] = valor
                self._lidar_t[clave] = ahora
                return valor, True
            if ahora - self._lidar_t[clave] <= float(p['lidar_hold_s']):
                valor = self._lidar_last[clave]
                return valor, math.isfinite(valor)
            return float('inf'), False

        self.d_frente, self.frente_valido = lectura_estable(
            'F', frente_vals, lambda v: _percentil(v, 25))
        self.d_derecha, self.derecha_valida = lectura_estable(
            'D', derecha_vals, _mediana)
        self.d_izquierda, self.izquierda_valida = lectura_estable(
            'I', izq_vals, _mediana)

        self.frente_bloqueado = (
            self.frente_valido and self.d_frente < float(p['f_stop']))
        self.frente_libre_ok = (
            self.frente_valido and self.d_frente > self.frente_libre)
        self.scan_listo = True

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        raw_yaw = _yaw_quat(q.x, q.y, q.z, q.w)

        if not self.odom_iniciado:
            self.odom_origen_x = p.x
            self.odom_origen_y = p.y
            self.odom_origen_yaw = raw_yaw
            self.prev_raw_x = p.x
            self.prev_raw_y = p.y
            self.pos_x = 0.0
            self.pos_y = 0.0
            self.yaw = 0.0
            self.odom_iniciado = True
            self.get_logger().info(
                f'Odom relativa iniciada: origen=({p.x:.2f},{p.y:.2f}) yaw0={math.degrees(raw_yaw):.1f}°')
            return

        # Distancia recorrida con coordenadas RAW.
        salto = math.hypot(p.x - self.prev_raw_x, p.y - self.prev_raw_y)
        if salto < 0.25:
            self.dist_odom_cm += salto * 100.0
        self.prev_raw_x, self.prev_raw_y = p.x, p.y

        # Pose local: inicio de cada corrida siempre queda en (0,0,0).
        dx = p.x - self.odom_origen_x
        dy = p.y - self.odom_origen_y
        c = math.cos(self.odom_origen_yaw)
        ss = math.sin(self.odom_origen_yaw)
        self.pos_x = c * dx + ss * dy
        self.pos_y = -ss * dx + c * dy
        self.yaw = _norm(raw_yaw - self.odom_origen_yaw)

    def cb_pare(self, msg: Bool):
        nuevo_flag = bool(msg.data)
        flanco_subida = nuevo_flag and not self.pare_flag
        self.pare_flag = nuevo_flag

        # Cuenta una sola detección por aparición física del cartel.
        # Antes cada mensaje True durante GIRAR se contaba otra vez.
        if flanco_subida and not self.pare_atendido:
            ahora = time.time()
            if ahora - self.t_ultimo_pare_detectado > 2.0:
                self.t_ultimo_pare_detectado = ahora
                self.pare_detectados += 1
                self.get_logger().info('── PARE detectado ──')

        if not self.pare_flag:
            cooldown = float(self.get_parameter('pare_cooldown_s').value)
            if time.time() - self.t_ultimo_pare > cooldown:
                self.pare_atendido = False

    def cb_meta(self, msg: Bool):
        """
        Segunda barrera contra falsos positivos.

        /meta_detectado debe permanecer verdadero de forma continua y el robot
        debe haber recorrido una distancia mínima. Una pulsación aislada de
        verde ya no puede terminar la ronda.
        """
        ahora = time.time()
        self.meta_flag = bool(msg.data)

        if self.meta_latched:
            return

        if not self.meta_flag:
            self.meta_true_since = None
            self.meta_candidate_logged = False
            return

        if self.meta_true_since is None:
            self.meta_true_since = ahora
            if not self.meta_candidate_logged:
                self.get_logger().info(
                    'META candidata: esperando confirmación continua')
                self.meta_candidate_logged = True
            return

        p = self._p()
        estable_s = ahora - self.meta_true_since
        if estable_s < float(p['meta_confirm_s']):
            return

        runtime = ahora - self.t_inicio
        distancia_m = self.dist_odom_cm / 100.0

        if runtime < float(p['meta_min_runtime_s']):
            return
        if distancia_m < float(p['meta_min_distance_m']):
            self.get_logger().warn(
                f'META ignorada por recorrido insuficiente: '
                f'{distancia_m:.2f}m < {float(p["meta_min_distance_m"]):.2f}m')
            return

        frente_seguro = (
            (not self.frente_valido) or
            self.d_frente > float(p['meta_front_stop']))
        if not frente_seguro:
            self.get_logger().warn(
                f'META visual ignorada por obstáculo frontal F={self.d_frente:.2f}')
            return

        self.meta_latched = True
        self.t_meta = ahora
        self._q_update_last(float(p['q_meta_reward']), 'META')
        self._guardar_q_table(force=True)
        self.get_logger().info(
            f'── META confirmada doblemente: '
            f'visual={estable_s:.2f}s recorrido={distancia_m:.2f}m ──')

    # ──────────────────────────────────────────────────────────
    #  LOOP PRINCIPAL
    # ──────────────────────────────────────────────────────────

    def loop_control(self):
        if not self.scan_listo:
            self._pub(0.0, 0.0)
            return

        p = self._p()

        # Failsafe: nunca continuar moviéndose con un LaserScan antiguo.
        # Antes el nodo conservaba indefinidamente F/D/I y seguía enviando
        # velocidad aunque el MS200 dejara de actualizar /scan.
        scan_age = (
            time.time() - self.t_ultimo_scan
            if self.t_ultimo_scan > 0.0 else float('inf')
        )
        if scan_age > float(p['scan_timeout_s']):
            self._pub(0.0, 0.0, force=True)
            if not self.scan_timeout_latched:
                self.get_logger().error(
                    f'FAILSAFE LiDAR: /scan sin actualizar por {scan_age:.2f}s; robot detenido')
                self.scan_timeout_latched = True
            return

        if self.scan_timeout_latched:
            self.get_logger().info('LiDAR recuperado: /scan volvió a actualizarse')
            self.scan_timeout_latched = False

        if not p['active']:
            self._pub(0.0, 0.0)
            return

        self._log_periodico()

        # Prioridad de cámara:
        # META enclavada gana a todo. PARE se respeta, pero NO debe interrumpir
        # un giro, porque los falsos positivos durante GIRAR hacían que el robot
        # se detuviera varias veces sin necesidad.
        if self.meta_latched and self.estado != Estado.META:
            self._cambiar(Estado.META)
        else:
            pare_cooldown_ok = (time.time() - self.t_ultimo_pare) > float(p['pare_cooldown'])
            pare_estado_ok = self.estado in (Estado.EN_PASILLO, Estado.INTERSECCION)
            pare_lidar_ok = self.frente_valido and math.isfinite(self.d_frente) and self.d_frente > 0.24

            if (self.pare_flag and not self.pare_atendido and
                    pare_cooldown_ok and pare_estado_ok and pare_lidar_ok):
                self._cambiar(Estado.PARAR_PARE)

        {
            Estado.EN_PASILLO:   lambda: self._en_pasillo(p),
            Estado.INTERSECCION: lambda: self._interseccion(),
            Estado.PARAR_PARE:   lambda: self._parar_pare(),
            Estado.ESPERAR_3S:   lambda: self._esperar_3s(),
            Estado.DECIDIR_GIRO: lambda: self._decidir_giro(p),
            Estado.GIRAR:        lambda: self._girar(p),
            Estado.META:         lambda: self._meta(),
        }[self.estado]()

    # ──────────────────────────────────────────────────────────
    #  ESTADOS
    # ──────────────────────────────────────────────────────────

    def _en_pasillo(self, p):
        if not self.frente_valido:
            self._pub(0.0, 0.0)
            return

        ahora = time.time()

        # Estabilización después de cada giro.
        if (self.t_fin_ultimo_giro > 0.0 and
                ahora - self.t_fin_ultimo_giro < float(p['post_turn_guard_s'])):
            self.apertura_der_ticks = 0
            self.frente_cerca_ticks = 0
            dt_post = ahora - self.t_fin_ultimo_giro

            if dt_post < float(p['post_turn_stop_s']):
                self._pub(0.0, 0.0)
            elif math.isfinite(self.d_frente) and self.d_frente < 0.24:
                self._pub(-0.022, 0.0)
            else:
                v_salida = min(float(p['v_fwd']) * 0.45, 0.025)
                self._pub(v_salida, 0.0)
            return

        # ── Rearme de abertura derecha ─────────────────────────────
        # Una abertura solo es válida si antes el robot volvió a tener
        # una pared derecha real. Después de girar dentro de un cruce,
        # D puede seguir abierta varios segundos: eso NO es otro cruce.
        pared_derecha_presente = (
            self.derecha_valida and
            math.isfinite(self.d_derecha) and
            self.d_derecha <= float(p['side_rearm_wall_max'])
        )

        if pared_derecha_presente:
            self.right_wall_ticks += 1
        else:
            self.right_wall_ticks = 0

        if (not self.side_open_armed and
                self.right_wall_ticks >= int(p['side_rearm_ticks'])):
            self.side_open_armed = True
            self.get_logger().info(
                f'Abertura lateral rearmada: pared derecha estable '
                f'D={self.d_derecha:.2f}')

        umbral_entrada = max(float(p['s_open']), 1.10)
        apertura_der = (
            self.side_open_armed and
            self.derecha_valida and
            math.isfinite(self.d_derecha) and
            umbral_entrada <= self.d_derecha <= 2.20
        )
        frente_cerca = (
            self.frente_valido and
            math.isfinite(self.d_frente) and
            self.d_frente <= float(p['f_stop'])
        )

        self.apertura_der_ticks = (
            self.apertura_der_ticks + 1 if apertura_der else 0
        )
        self.frente_cerca_ticks = (
            self.frente_cerca_ticks + 1 if frente_cerca else 0
        )

        runtime = ahora - self.t_inicio
        recorrido_m = self.dist_odom_cm / 100.0
        startup_guard = (
            runtime < float(p['startup_side_guard_s']) or
            recorrido_m < float(p['startup_min_travel_m'])
        )

        # Distancia física desde la última intersección detectada.
        if (self.last_intersection_x is None or
                self.last_intersection_y is None):
            distancia_salida = float('inf')
        else:
            distancia_salida = math.hypot(
                self.pos_x - self.last_intersection_x,
                self.pos_y - self.last_intersection_y
            )

        salio_del_nodo_anterior = (
            distancia_salida >= float(p['intersection_min_exit_m'])
        )

        interseccion_der = (
            not startup_guard and
            salio_del_nodo_anterior and
            self.apertura_der_ticks >= int(p['side_confirm_ticks'])
        )
        interseccion_frente = self.frente_cerca_ticks >= 3
        emergencia_frente = (
            math.isfinite(self.d_frente) and self.d_frente <= 0.28
        )
        cooldown_ok = (
            ahora - self.t_ultima_interseccion > float(p['cooldown'])
        )

        if emergencia_frente:
            self.interseccion_motivo = 'emergencia'
            self.t_inicio_centrado = None
            self.centrado_inicio_x = None
            self.centrado_inicio_y = None
            self.last_intersection_x = float(self.pos_x)
            self.last_intersection_y = float(self.pos_y)
            self.side_open_armed = False
            self.right_wall_ticks = 0
            self._pub(0.0, 0.0)
            self._cambiar(Estado.INTERSECCION)
            return

        if cooldown_ok and interseccion_frente:
            self.interseccion_motivo = 'frente'
            self.t_inicio_centrado = None
            self.centrado_inicio_x = None
            self.centrado_inicio_y = None
            self.last_intersection_x = float(self.pos_x)
            self.last_intersection_y = float(self.pos_y)
            self.side_open_armed = False
            self.right_wall_ticks = 0
            self._pub(0.0, 0.0)
            self._cambiar(Estado.INTERSECCION)
            return

        if cooldown_ok and interseccion_der:
            self.interseccion_motivo = 'lateral'
            self.t_inicio_centrado = ahora
            self.centrado_inicio_x = float(self.pos_x)
            self.centrado_inicio_y = float(self.pos_y)
            self.last_intersection_x = float(self.pos_x)
            self.last_intersection_y = float(self.pos_y)

            # Se consume el flanco de abertura. No se rearma hasta volver
            # a ver una pared derecha estable.
            self.side_open_armed = False
            self.right_wall_ticks = 0
            self.apertura_der_ticks = 0

            self.get_logger().info(
                f'Intersección lateral confirmada: D={self.d_derecha:.2f}; '
                f'centrando {float(p["intersection_center_distance_m"]):.2f}m')
            self._pub(0.0, 0.0)
            self._cambiar(Estado.INTERSECCION)
            return

        v = float(p['v_fwd']) * 0.65 if (
            math.isfinite(self.d_frente) and self.d_frente < 0.55
        ) else float(p['v_fwd'])
        self._pub(v, self._w_pared(p))

    def _interseccion(self):
        p = self._p()

        pare_cooldown_ok = (
            time.time() - self.t_ultimo_pare > float(p['pare_cooldown'])
        )
        pare_lidar_ok = (
            self.frente_valido and
            math.isfinite(self.d_frente) and
            self.d_frente > 0.24
        )

        if (self.pare_flag and not self.pare_atendido and
                pare_cooldown_ok and pare_lidar_ok):
            self._pub(0.0, 0.0)
            self.get_logger().info('PARE en intersección')
            self._cambiar(Estado.PARAR_PARE)
            return

        # Para una abertura lateral, continúa recto hasta que el centro
        # del chasis entre realmente en el cruce. Esto evita pivotar junto
        # a la punta de la pared.
        if self.interseccion_motivo == 'lateral':
            if self.t_inicio_centrado is None:
                self.t_inicio_centrado = time.time()
            if self.centrado_inicio_x is None:
                self.centrado_inicio_x = float(self.pos_x)
                self.centrado_inicio_y = float(self.pos_y)

            desplazamiento = math.hypot(
                self.pos_x - self.centrado_inicio_x,
                self.pos_y - self.centrado_inicio_y
            )
            dt = time.time() - self.t_inicio_centrado

            frente_permita_centrar = (
                self.frente_valido and
                math.isfinite(self.d_frente) and
                self.d_frente > float(p['intersection_center_front_min'])
            )

            falta_distancia = (
                desplazamiento <
                float(p['intersection_center_distance_m'])
            )
            dentro_timeout = (
                dt < float(p['intersection_center_timeout_s'])
            )

            if falta_distancia and dentro_timeout and frente_permita_centrar:
                v_centro = min(
                    float(p['intersection_center_v']),
                    float(p['v_fwd'])
                )
                self._pub(v_centro, 0.0)
                return

            self.get_logger().info(
                f'Centrado finalizado: avance={desplazamiento:.2f}m '
                f't={dt:.2f}s F={self.d_frente:.2f}')

        self._pub(0.0, 0.0)
        self.interseccion_motivo = None
        self.t_inicio_centrado = None
        self.centrado_inicio_x = None
        self.centrado_inicio_y = None
        self._cambiar(Estado.DECIDIR_GIRO)

    def _parar_pare(self):
        self._pub(0.0, 0.0)
        self.t_pare = time.time()
        self.t_ultimo_pare = self.t_pare
        self.pare_atendido = True
        self.pare_respetados += 1
        self._cambiar(Estado.ESPERAR_3S)

    def _esperar_3s(self):
        self._pub(0.0, 0.0)
        if time.time() - self.t_pare >= self.tiempo_pare:
            self.get_logger().info('Espera PARE completada — continuar recto')

            # Después del PARE no vuelve a decidir un giro.
            # Retoma el seguimiento normal del pasillo.
            self.t_ultima_interseccion = time.time()
            self.apertura_der_ticks = 0
            self.frente_cerca_ticks = 0
            self._cambiar(Estado.EN_PASILLO)

    def _decidir_giro(self, p):
        """
        Trémaux con retroceso real.

        Error corregido:
        la versión anterior solo consideraba derecha, recto e izquierda.
        Cuando todas esas ramas ya estaban recorridas, nunca podía regresar
        por la arista de entrada; por eso daba vueltas dentro del mismo ciclo.

        Regla:
        1) tomar una salida no recorrida;
        2) si ya no existe ninguna, regresar por donde llegó (media vuelta);
        3) Q-learning solo desempata entre salidas igualmente nuevas.
        """
        umbral_decision = 0.55
        ahora = time.time()

        der = (
            self.derecha_valida and math.isfinite(self.d_derecha) and
            umbral_decision <= self.d_derecha <= 2.20)
        izq = (
            self.izquierda_valida and math.isfinite(self.d_izquierda) and
            umbral_decision <= self.d_izquierda <= 2.20)
        frt = (
            self.frente_valido and self.frente_libre_ok and
            math.isfinite(self.d_frente) and self.d_frente > self.frente_libre)

        nodo = self._nodo_memoria_actual()
        nodo['visitas'] += 1
        self.nodo_actual_id = nodo['id']
        heading = self._heading_idx()

        # La dirección física para volver es la opuesta al rumbo de llegada.
        direccion_entrada = None
        if self.ultima_salida_fisica is not None:
            salida = self.ultima_salida_fisica
            distancia = math.hypot(
                self.pos_x - float(salida['x']),
                self.pos_y - float(salida['y']))
            dt_salida = ahora - float(salida['t'])

            if (not salida.get('llegada_registrada', False) and
                    (distancia >= 0.22 or dt_salida >= 3.5)):
                direccion_entrada = (heading + 2) % 4
                nodo['ultima_entrada'] = int(direccion_entrada)
                clave_entrada = str(direccion_entrada)
                nodo['conteo'][clave_entrada] = int(
                    nodo['conteo'].get(clave_entrada, 0)) + 1
                salida['llegada_registrada'] = True
        if direccion_entrada is None:
            entrada_guardada = nodo.get('ultima_entrada')
            if entrada_guardada is not None:
                direccion_entrada = int(entrada_guardada)

        disponibles = []
        if der:
            disponibles.append(('derecha', (heading - 1) % 4))
        if frt:
            disponibles.append(('recto', heading))
        if izq:
            disponibles.append(('izquierda', (heading + 1) % 4))

        state_key = self._q_state_key(der, frt, izq)
        state_cell = self._q_state_cell()
        self._q_evaluar_llegada(state_key, state_cell, nodo['id'], p)

        # Callejón real: no hay salida lateral ni frontal.
        if not disponibles:
            self.get_logger().info(f'Nodo {nodo["id"]}: callejón → media vuelta')
            self.dead_ends += 1
            self._q_update_last(float(p['q_deadend_penalty']), 'dead-end')

            if self.ultima_salida_fisica is not None:
                salida = self.ultima_salida_fisica
                origen_id = int(salida['node_id'])
                direccion_mala = int(salida['direction'])

                for n in self.nodos_memoria:
                    if int(n['id']) == origen_id:
                        n.setdefault('bloqueadas', set()).add(str(direccion_mala))
                        n['conteo'][str(direccion_mala)] = max(
                            2, int(n['conteo'].get(str(direccion_mala), 0)))
                        break

                self.deadend_retorno_pendiente = True
                self.deadend_direccion_prohibida = direccion_mala
                self.deadend_nodo_origen_id = origen_id
                self.get_logger().warn(
                    f'Trémaux: rama dir{direccion_mala} del nodo '
                    f'{origen_id} marcada como callejón')

            self.giro_dir = 'media'
            self.yaw_inicio_giro = self.yaw
            self.yaw_objetivo = _norm(self.yaw + math.pi)
            self.ultima_accion = 'media'
            self._preparar_giro()
            self._cambiar(Estado.GIRAR)
            return

        bloqueadas = {int(x) for x in nodo.get('bloqueadas', set())}
        normales = [
            item for item in disponibles if int(item[1]) not in bloqueadas
        ]
        if not normales:
            normales = list(disponibles)

        # Evita reingresar inmediatamente a la rama que acaba de terminar
        # en callejón, cuando existe otra alternativa.
        if (self.deadend_retorno_pendiente and
                self.deadend_direccion_prohibida is not None):
            alternativas = [
                item for item in normales
                if int(item[1]) != int(self.deadend_direccion_prohibida)
            ]
            if alternativas:
                normales = alternativas
                self.get_logger().info(
                    f'Trémaux: retorno de callejón; se evita dir'
                    f'{self.deadend_direccion_prohibida}')

        # Salidas completamente nuevas.
        nuevas = [
            item for item in normales
            if int(nodo['conteo'].get(str(item[1]), 0)) == 0
        ]

        retroceso_forzado = False
        if nuevas:
            candidatas = nuevas
            motivo_base = 'salida nueva'
        elif direccion_entrada is not None:
            # Punto clave: si no queda una rama nueva, se vuelve por donde llegó.
            # Esto rompe los ciclos locales del mapa.
            candidatas = [('media', int(direccion_entrada))]
            retroceso_forzado = True
            motivo_base = 'retroceso Trémaux'
        else:
            # Respaldo por si la odometría no permitió identificar la entrada.
            minimo = min(
                int(nodo['conteo'].get(str(d), 0))
                for _, d in normales)
            candidatas = [
                item for item in normales
                if int(nodo['conteo'].get(str(item[1]), 0)) == minimo
            ]
            motivo_base = f'marcas mínimas={minimo}'

        acciones_q = []
        for accion, direccion in candidatas:
            marcas = int(nodo['conteo'].get(str(direccion), 0))
            q = self._q_get(state_key, accion)
            q += {
                'derecha': 0.20,
                'recto': 0.15,
                'izquierda': 0.10,
                'media': 0.30,
            }.get(accion, 0.0)
            q -= float(p['node_exit_repeat_penalty']) * marcas
            acciones_q.append((q, accion, direccion))

        explorar = (
            not retroceso_forzado and
            bool(p['qlearn_enabled']) and
            nodo['visitas'] <= 2 and
            random.random() < float(p['q_epsilon'])
        )
        if explorar:
            q, accion, direccion = random.choice(acciones_q)
            motivo = f'{motivo_base}, exploración Q'
        else:
            q, accion, direccion = max(acciones_q, key=lambda x: x[0])
            motivo = f'{motivo_base}, Q={q:.2f}'

        marcas_antes = int(nodo['conteo'].get(str(direccion), 0))
        nodo['conteo'][str(direccion)] = marcas_antes + 1
        self.ultima_accion = accion

        self.ultima_salida_fisica = {
            'node_id': nodo['id'],
            'direction': int(direccion),
            'x': float(self.pos_x),
            'y': float(self.pos_y),
            't': ahora,
            'llegada_registrada': False,
        }

        self.yaw_inicio_giro = self.yaw
        self.ultima_decision_q = {
            'state': state_key,
            'action': accion,
            'direction': int(direccion),
            'cell': state_cell,
            'node_id': nodo['id'],
            't': ahora,
            'control_ok': True,
        }

        if self.deadend_retorno_pendiente:
            if (self.deadend_direccion_prohibida is None or
                    int(direccion) != int(self.deadend_direccion_prohibida)):
                self.deadend_retorno_pendiente = False
                self.deadend_direccion_prohibida = None
                self.deadend_nodo_origen_id = None

        self.get_logger().info(
            f'Trémaux Nodo {nodo["id"]} visita={nodo["visitas"]} '
            f'estado={state_key} → {accion}:dir{direccion}:m{marcas_antes} '
            f'({motivo})')

        if accion == 'derecha':
            self.giro_dir = 'derecha'
            self.yaw_objetivo = _norm(self.yaw - math.pi / 2.0)
            self._preparar_giro()
            self._cambiar(Estado.GIRAR)
        elif accion == 'izquierda':
            self.giro_dir = 'izquierda'
            self.yaw_objetivo = _norm(self.yaw + math.pi / 2.0)
            self._preparar_giro()
            self._cambiar(Estado.GIRAR)
        elif accion == 'media':
            self.giro_dir = 'media'
            self.yaw_objetivo = _norm(self.yaw + math.pi)
            self._preparar_giro()
            self._cambiar(Estado.GIRAR)
        else:
            self.t_ultima_interseccion = time.time()
            self.yaw_inicio_giro = None
            self._cambiar(Estado.EN_PASILLO)

    def _preparar_giro(self):
        self.giro_retroceso_hecho = False
        self.giro_retroceso_inicio = None
        self.t_seguridad_retroceso = None

    def _girar(self, p):
        """Giro por yaw real. LiDAR solo protege; nunca finaliza a 40°."""
        t_en_giro = self._tiempo_en_estado()
        es_media = self.giro_dir == 'media'
        timeout = float(p['media_timeout_s'] if es_media else p['giro_timeout_s'])

        if self.yaw_objetivo is None:
            if self.giro_dir == 'derecha':
                self.yaw_objetivo = _norm(self.yaw - math.pi / 2.0)
            elif self.giro_dir == 'izquierda':
                self.yaw_objetivo = _norm(self.yaw + math.pi / 2.0)
            else:
                self.yaw_objetivo = _norm(self.yaw + math.pi)

        if self.yaw_inicio_giro is None:
            self.yaw_inicio_giro = self.yaw

        delta = abs(_norm(self.yaw - self.yaw_inicio_giro))
        objetivo_param = float(
            p['media_yaw_min_deg'] if es_media else p['giro_yaw_min_deg'])
        margen = float(p['turn_brake_margin_deg'])
        if es_media:
            margen *= 1.5
        objetivo_min = math.radians(max(70.0, objetivo_param - margen))

        # La velocidad ya se reduce cerca del objetivo, así que el margen
        # de frenado es pequeño. Esto evita terminar en 81° cuando se pidieron 84°.
        if delta >= objetivo_min:
            self.get_logger().info(
                f'Giro completado por yaw: tipo={self.giro_dir} delta={math.degrees(delta):.1f}°')
            self._fin_giro()
            return

        if t_en_giro > timeout:
            self.get_logger().warn(
                f'Timeout físico de giro: tipo={self.giro_dir} delta={math.degrees(delta):.1f}°; Q no penalizada')
            if self.ultima_decision_q is not None:
                self.ultima_decision_q['control_ok'] = False
                self.fallos_control_recientes[
                    (self.ultima_decision_q['state'], self.ultima_decision_q['action'])
                ] = time.time()
            self._fin_giro()
            return

        turn_eff = max(0.15, min(0.19, abs(float(p['turn']))))
        if t_en_giro < 0.8:
            turn_eff = max(turn_eff, 0.18)

        # Reduce velocidad al acercarse al ángulo final.
        restante_deg = max(0.0, math.degrees(objetivo_min - delta))
        if restante_deg <= float(p['turn_slowdown_deg']):
            turn_eff = max(0.09, min(turn_eff, float(p['turn_near_speed'])))

        diff = _norm(self.yaw_objetivo - self.yaw)
        if es_media:
            # Mantiene siempre el mismo sentido durante la media vuelta.
            w_normal = -turn_eff
        else:
            w_normal = turn_eff if diff > 0 else -turn_eff

        # Retroceso frontal UNA sola vez por giro.
        front_safety = float(p['giro_front_safety'])
        reverse_s = float(p['giro_reverse_s'])
        if (not self.giro_retroceso_hecho and self.frente_valido and
                math.isfinite(self.d_frente) and self.d_frente < front_safety):
            if self.giro_retroceso_inicio is None:
                self.giro_retroceso_inicio = time.time()
                self.get_logger().warn(
                    f'Seguridad giro: F={self.d_frente:.2f}; único retroceso de {reverse_s:.2f}s')
            if time.time() - self.giro_retroceso_inicio < reverse_s:
                self._pub(-0.035, 0.0)
                return
            self.giro_retroceso_hecho = True
            self.giro_retroceso_inicio = None

        self._pub(0.0, w_normal)

    def _fin_giro(self):
        ahora = time.time()
        self.t_ultima_interseccion = ahora
        self.t_fin_ultimo_giro = ahora
        self.giro_dir = None
        self.yaw_objetivo = None
        self.yaw_inicio_giro = None
        self.apertura_der_ticks = 0
        self.frente_cerca_ticks = 0
        self.side_open_armed = False
        self.right_wall_ticks = 0
        self.interseccion_motivo = None
        self.t_inicio_centrado = None
        self.centrado_inicio_x = None
        self.centrado_inicio_y = None
        self.t_seguridad_retroceso = None
        self.giro_retroceso_hecho = False
        self.giro_retroceso_inicio = None
        self._cambiar(Estado.EN_PASILLO)

    def _meta(self):
        p = self._p()
        if self.t_meta is None:
            self.t_meta = time.time()

        # El avance final sigue respetando una distancia crítica real.
        if (self.frente_valido and math.isfinite(self.d_frente) and
                self.d_frente < float(p['meta_front_stop'])):
            self._pub(0.0, 0.0, force=True)
            self.get_logger().warn(
                f'META: obstáculo frontal F={self.d_frente:.2f}; detención segura')
            self.t_meta = time.time() - float(p['meta_forward_s'])

        elif time.time() - self.t_meta < float(p['meta_forward_s']):
            self._pub(p['v_fwd'], 0.0, force=True)
            return

        self._pub(0.0, 0.0, force=True)
        if not self.meta_finalizado:
            self.meta_finalizado = True
            self.get_logger().info('¡¡ META CRUZADA !!')
            self._guardar_csv(llego_meta=True)
        self.timer.cancel()
        if rclpy.ok():
            rclpy.shutdown()

    # ──────────────────────────────────────────────────────────
    #  AUXILIARES
    # ──────────────────────────────────────────────────────────


    # ──────────────────────────────────────────────────────────
    #  Q-LEARNING TABULAR ANTI-BUCLE
    # ──────────────────────────────────────────────────────────

    def _q_state_cell(self):
        """Celda gruesa para aprendizaje. Menos sensible que nodo por odometría."""
        try:
            grid = float(self.get_parameter('q_state_grid_m').value)
        except Exception:
            grid = 0.45
        grid = max(0.25, grid)
        return (int(round(self.pos_x / grid)), int(round(self.pos_y / grid)))

    def _q_state_key(self, der, frt, izq):
        cx, cy = self._q_state_cell()
        h = self._heading_idx()
        # Binning de distancias para distinguir callejón, pasillo e intersección.
        def b(v):
            if not math.isfinite(v):
                return 'X'
            if v < 0.35:
                return 'C'
            if v < 0.80:
                return 'M'
            return 'A'
        return (
            f'c{cx},{cy}|h{h}|'
            f'F{b(self.d_frente)}D{b(self.d_derecha)}I{b(self.d_izquierda)}|'
            f'o{int(der)}{int(frt)}{int(izq)}'
        )

    def _q_get(self, state, action):
        return float(self.q_table.get(state, {}).get(action, 0.0))

    def _q_set(self, state, action, value):
        self.q_table.setdefault(state, {})[action] = round(float(value), 4)

    def _q_update(self, state, action, reward, motivo=''):
        if not bool(self.get_parameter('qlearn_enabled').value):
            return
        alpha = float(self.get_parameter('q_alpha').value)
        old = self._q_get(state, action)
        new = old + alpha * (float(reward) - old)
        self._q_set(state, action, new)
        self.q_updates_desde_guardado += 1
        self.get_logger().info(
            f'Q update {motivo}: ({state}, {action}) {old:.2f} -> {new:.2f} r={reward:.1f}'
        )
        if self.q_updates_desde_guardado >= 5:
            self._guardar_q_table()

    def _q_update_last(self, reward, motivo=''):
        if self.ultima_decision_q is None:
            return
        if not self.ultima_decision_q.get('control_ok', True):
            self.get_logger().info(
                f'Q sin actualizar ({motivo}): la maniobra anterior falló físicamente')
            return
        self._q_update(
            self.ultima_decision_q['state'],
            self.ultima_decision_q['action'],
            reward,
            motivo)

    def _q_evaluar_llegada(self, state_key, state_cell, node_id, p):
        """Evalúa progreso usando estado/celda relativos, no el ID del nodo."""
        ahora = time.time()
        self.historial_estados_q = [
            item for item in self.historial_estados_q
            if ahora - item['t'] < 45.0]

        if self.ultima_decision_q is not None:
            last = self.ultima_decision_q
            dt = ahora - last['t']
            mismo_estado = last['state'] == state_key
            estado_repetido = any(
                item['state'] == state_key and ahora - item['t'] < 25.0
                for item in self.historial_estados_q)

            if (mismo_estado or estado_repetido) and dt < 35.0:
                self._q_update_last(float(p['q_repeat_penalty']), 'loop/repetición')
            else:
                dx = abs(int(last['cell'][0]) - int(state_cell[0]))
                dy = abs(int(last['cell'][1]) - int(state_cell[1]))
                pasos_celda = dx + dy

                # Solo un desplazamiento claro recibe premio de zona nueva.
                # Un cambio de una sola celda puede ser deriva de odometría.
                if pasos_celda >= 2 and dt >= 5.0:
                    self._q_update_last(
                        float(p['q_new_state_reward']), 'zona realmente nueva')
                else:
                    self._q_update_last(
                        float(p['q_progress_reward']), 'progreso local')

        self.historial_estados_q.append({
            'state': state_key,
            'cell': state_cell,
            'node_id': node_id,
            't': ahora,
        })

    def _q_fmt_opciones(self, acciones_q):
        partes = []
        for q, accion, direccion in sorted(acciones_q, key=lambda x: x[0], reverse=True):
            partes.append(f'{accion}:dir{direccion}:Q{q:.2f}')
        return ' | '.join(partes)

    def _q_path(self):
        try:
            return str(self.get_parameter('q_path').value)
        except Exception:
            return '/ros2_ws/q_table_granprix.json'

    def _cargar_q_table(self):
        path = self._q_path()
        try:
            if os.path.isfile(path):
                with open(path, 'r') as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    self.q_table = data
                    self.get_logger().info(f'Q-table cargada: {path} ({len(self.q_table)} estados)')
        except Exception as e:
            self.get_logger().warn(f'No se pudo cargar Q-table: {e}')

    def _guardar_q_table(self, force=False):
        if not force and self.q_updates_desde_guardado <= 0:
            return
        path = self._q_path()
        try:
            base = os.path.dirname(path)
            if base:
                os.makedirs(base, exist_ok=True)
            with open(path, 'w') as f:
                json.dump(self.q_table, f, indent=2, sort_keys=True)
            self.q_updates_desde_guardado = 0
            self.get_logger().info(f'Q-table guardada: {path} ({len(self.q_table)} estados)')
        except Exception as e:
            self.get_logger().warn(f'No se pudo guardar Q-table: {e}')

    def _nodo_memoria_actual(self):
        """
        Devuelve el nodo de intersección actual.
        Si el robot está cerca de un nodo anterior, reutiliza ese nodo.
        Si no, crea uno nuevo.
        """
        radius = float(self.get_parameter('memory_node_radius').value)
        mejor = None
        mejor_d = 999.0
        for nodo in self.nodos_memoria:
            d = math.hypot(self.pos_x - nodo['x'], self.pos_y - nodo['y'])
            if d < mejor_d:
                mejor = nodo
                mejor_d = d

        if mejor is not None and mejor_d <= radius:
            return mejor

        nuevo = {
            'id': len(self.nodos_memoria),
            'x': self.pos_x,
            'y': self.pos_y,
            'visitas': 0,
            'exploradas': set(),
            'conteo': {},
            'bloqueadas': set(),
            'ultima_entrada': None,
        }
        self.nodos_memoria.append(nuevo)
        self.get_logger().info(
            f'Nuevo nodo memoria {nuevo["id"]} en x={self.pos_x:.2f}, y={self.pos_y:.2f}'
        )
        return nuevo

    def _celda_actual(self):
        """Celda aproximada por odometría. Solo se usa como memoria anti-bucle."""
        return (
            int(round(self.pos_x / self.cell_size)),
            int(round(self.pos_y / self.cell_size))
        )

    def _heading_idx(self):
        """
        Dirección cardinal aproximada según yaw.
        0: +x, 1: +y, 2: -x, 3: -y
        """
        return int(round(self.yaw / (math.pi / 2.0))) % 4

    def _celda_destino(self, celda, accion):
        h = self._heading_idx()
        if accion == 'derecha':
            h = (h - 1) % 4
        elif accion == 'izquierda':
            h = (h + 1) % 4
        # recto mantiene h

        dirs = {
            0: (1, 0),
            1: (0, 1),
            2: (-1, 0),
            3: (0, -1),
        }
        dx, dy = dirs[h]
        return (celda[0] + dx, celda[1] + dy)

    def _w_pared(self, p):
        # Si no existe pared derecha confiable, no continuar curvando.
        if not math.isfinite(self.d_derecha) or self.d_derecha > self.dist_pared_max:
            return 0.0
        if self.d_derecha < self.dist_pared_min:
            return 0.10
        error = p['target'] - self.d_derecha
        return max(-0.11, min(0.11, p['kp'] * error))

    def _llegue_a_meta(self, p):
        return math.hypot(self.pos_x - p['x_meta'],
                          self.pos_y - p['y_meta']) < p['r_meta']

    def _tiempo_en_estado(self):
        return (self.get_clock().now() - self.t_estado).nanoseconds * 1e-9

    def _pub(self, v, w, force=False):
        # Guarda global normal. META usa force=True porque debe cruzar recto
        # aunque el cartel verde deje de verse o el LiDAR detecte el marco.
        # Protección física final: detener antes de tocar una pared.
        if (not force and v > 0.0 and math.isfinite(self.d_frente)
                and self.d_frente < 0.28):
            v = 0.0

        cmd = Twist()
        cmd.linear.x  = float(v)
        cmd.angular.z = float(w)
        self.pub_vel.publish(cmd)

    def _cambiar(self, nuevo: Estado):
        if nuevo != self.estado:
            self.get_logger().info(
                f'{self.estado.name} → {nuevo.name}  |  '
                f'F={self.d_frente:.2f} D={self.d_derecha:.2f} '
                f'I={self.d_izquierda:.2f} PARE={self.pare_flag}'
            )
            self.estado   = nuevo
            self.t_estado = self.get_clock().now()

    def _log_periodico(self):
        ahora = self.get_clock().now()
        if (ahora - self.t_log).nanoseconds * 1e-9 >= 1.0:
            self.get_logger().info(
                f'[{self.estado.name}] '
                f'F={self.d_frente:.2f} D={self.d_derecha:.2f} '
                f'I={self.d_izquierda:.2f} yaw={math.degrees(self.yaw):.1f}°'
            )
            self.t_log = ahora

    # ──────────────────────────────────────────────────────────
    #  CSV
    # ──────────────────────────────────────────────────────────

    def _guardar_csv(self, llego_meta=None):
        if self.csv_guardado:
            return
        self.csv_guardado = True

        llego = self.meta_finalizado if llego_meta is None else bool(llego_meta)
        tiempo_s = round(time.time() - self.t_inicio, 2)
        long_ruta = round(self.dist_odom_cm, 1)
        long_opt = 480.0
        efic = round(min(1.0, long_opt / long_ruta), 3) if llego and long_ruta > 0 else 0.0

        fila = {
            'ronda': self.get_parameter('ronda').value,
            'llego_meta': 'Si' if llego else 'No',
            'tiempo_s': tiempo_s,
            'long_ruta_cm': long_ruta,
            'long_optima_cm': long_opt,
            'eficiencia': efic,
            'colisiones': self.colisiones,
            'pare_reales': '?',
            'pare_detectados': self.pare_detectados,
            'pare_respetados': self.pare_respetados,
            'pare_falsos': self.pare_falsos,
            'dead_ends_visitados': self.dead_ends,
            'karpinchus_rodeados': 0,
        }

        base = '/ros2_ws' if os.path.isdir('/ros2_ws') else os.getcwd()
        os.makedirs(base, exist_ok=True)
        path = os.path.join(base, 'metricas_granprix.csv')
        existe = os.path.isfile(path)
        with open(path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(fila.keys()))
            if not existe:
                w.writeheader()
            w.writerow(fila)

        self._guardar_q_table(force=True)
        self.get_logger().info(
            f'CSV → {path} | meta={"Si" if llego else "No"} | t={tiempo_s}s | '
            f'Ruta={long_ruta}cm | Efic={efic} | '
            f'PARE={self.pare_respetados}/{self.pare_detectados} | DeadEnds={self.dead_ends}')


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    nodo = MazeSolver()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info('Interrumpido — guardando métricas...')
        nodo._guardar_csv(llego_meta=False)
    finally:
        try:
            if rclpy.ok():
                nodo._pub(0.0, 0.0)
        except Exception:
            pass
        try:
            nodo.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
