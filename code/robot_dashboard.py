#!/usr/bin/env python3
"""
robot_dashboard.py — Dashboard completo G1 CapyTown
Combina en una sola ventana:
  · Estado FSM inferido de /cmd_vel
  · Gauges F / D / I en tiempo real
  · Indicador PARE
  · Path tracker con trayectoria del robot
  · Sliders para calibrar parámetros en vivo (sin escribir ros2 param set)

Uso:
    DISPLAY=:0 ROS_DOMAIN_ID=20 python3 robot_dashboard.py
"""

import math
import os
import subprocess
import threading
import tkinter as tk
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


# ═══════════════════════════════════════════════════════════════
#  COLORES
# ═══════════════════════════════════════════════════════════════
BG      = '#1a1a2e'
PANEL   = '#0d0d1a'
VERDE   = '#44FF88'
AMARILLO= '#FFCC00'
ROJO    = '#FF4444'
AZUL    = '#4488FF'
GRIS    = '#555577'
BLANCO  = '#FFFFFF'
TEXTO   = '#AAAACC'

ESTADO_COLOR = {
    'MOVIENDO': VERDE,
    'GIRANDO':  AMARILLO,
    'PARADO':   GRIS,
    'PARE':     ROJO,
}

# ═══════════════════════════════════════════════════════════════
#  PARÁMETROS SLIDERS  [nombre_param, label, min, max, default, resolución]
# ═══════════════════════════════════════════════════════════════
PARAMS = [
    ('v_forward',               'v_forward',    0.02, 0.15, 0.07,  0.005),
    ('turn_speed',              'turn_speed',   0.08, 0.40, 0.16,  0.01),
    ('kp_wall',                 'kp_wall',      0.05, 0.50, 0.15,  0.01),
    ('wall_target',             'wall_target',  0.10, 0.50, 0.25,  0.01),
    ('front_stop',              'front_stop',   0.15, 0.60, 0.35,  0.01),
    ('side_open',               'side_open',    0.30, 1.50, 0.70,  0.05),
    ('intersection_cooldown_s', 'cooldown_s',   1.0,  8.0,  3.0,   0.5),
]


# ═══════════════════════════════════════════════════════════════
#  NODO ROS2
# ═══════════════════════════════════════════════════════════════
class DashboardNode(Node):
    def __init__(self, cb_scan, cb_odom, cb_vel, cb_pare):
        super().__init__('robot_dashboard')
        self.create_subscription(LaserScan, '/scan',           cb_scan, 10)
        self.create_subscription(Odometry,  '/odom_raw',       cb_odom, 10)
        self.create_subscription(Twist,     '/cmd_vel',        cb_vel,  10)
        self.create_subscription(Bool,      '/pare_detectado', cb_pare, 10)


# ═══════════════════════════════════════════════════════════════
#  DASHBOARD
# ═══════════════════════════════════════════════════════════════
class Dashboard:

    def __init__(self):
        self._running = True
        self.lock = threading.Lock()

        # Datos de sensores
        self.d_f = self.d_d = self.d_i = float('inf')
        self.vel_lin = self.vel_ang = 0.0
        self.pare_activo = False
        self.yaw = 0.0
        self.pos_x = self.pos_y = 0.0
        self.path = deque(maxlen=3000)   # historial de posiciones

        # Ventana principal
        self.root = tk.Tk()
        self.root.title('Robot Dashboard — G1 CapyTown')
        self.root.configure(bg=BG)
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self._cerrar)

        self._build_ui()

        # Hilo ROS2
        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()

        self._actualizar()
        self.root.mainloop()

    # ──────────────────────────────────────────────────────────
    #  CONSTRUCCIÓN UI
    # ──────────────────────────────────────────────────────────
    def _build_ui(self):
        root = self.root

        # ── Título ───────────────────────────────────────────
        tk.Label(root, text='Robot Dashboard — G1 CapyTown',
                 font=('Courier', 13, 'bold'), fg=BLANCO, bg=BG
                 ).pack(pady=(8, 4))

        # ── Fila principal: izquierda + derecha ───────────────
        fila = tk.Frame(root, bg=BG)
        fila.pack(fill='both', expand=True, padx=8)

        self._build_left(fila)
        self._build_right(fila)

        # ── Botón CERRAR ──────────────────────────────────────
        tk.Button(root, text='  ✕  CERRAR  ',
                  font=('Courier', 13, 'bold'),
                  bg='#cc2222', fg=BLANCO,
                  activebackground='#ff4444',
                  relief='flat', cursor='hand2',
                  command=self._cerrar
                  ).pack(pady=8, ipadx=10, ipady=5)

    # ── Panel izquierdo ───────────────────────────────────────
    def _build_left(self, parent):
        left = tk.Frame(parent, bg=BG, width=340)
        left.pack(side='left', fill='y', padx=(0, 6))
        left.pack_propagate(False)

        # Estado FSM
        tk.Label(left, text='ESTADO FSM',
                 font=('Courier', 9), fg=TEXTO, bg=BG).pack(anchor='w')
        self.lbl_estado = tk.Label(left, text='ESPERANDO...',
                                   font=('Courier', 20, 'bold'),
                                   fg=GRIS, bg=PANEL,
                                   relief='flat', width=18, pady=8)
        self.lbl_estado.pack(fill='x', pady=(2, 8))

        # Gauges F / D / I
        tk.Label(left, text='DISTANCIAS (m)',
                 font=('Courier', 9), fg=TEXTO, bg=BG).pack(anchor='w')
        self.gauge_canvas = tk.Canvas(left, width=320, height=90,
                                      bg=PANEL, highlightthickness=0)
        self.gauge_canvas.pack(fill='x', pady=(2, 8))

        # PARE indicator
        self.lbl_pare = tk.Label(left, text='● PARE: NO',
                                 font=('Courier', 12, 'bold'),
                                 fg=GRIS, bg=PANEL, pady=4)
        self.lbl_pare.pack(fill='x', pady=(0, 10))

        # Separador
        tk.Frame(left, bg=GRIS, height=1).pack(fill='x', pady=4)
        tk.Label(left, text='PARÁMETROS EN VIVO',
                 font=('Courier', 9), fg=TEXTO, bg=BG).pack(anchor='w', pady=(4, 2))

        # Sliders
        self.slider_vars = {}
        for pname, label, mn, mx, default, res in PARAMS:
            row = tk.Frame(left, bg=BG)
            row.pack(fill='x', pady=1)

            tk.Label(row, text=f'{label:<14}', font=('Courier', 9),
                     fg=TEXTO, bg=BG, width=14, anchor='w').pack(side='left')

            var = tk.DoubleVar(value=default)
            self.slider_vars[pname] = var

            sl = tk.Scale(row, variable=var, from_=mn, to=mx,
                          resolution=res, orient='horizontal',
                          length=160, sliderlength=14,
                          bg=BG, fg=BLANCO, troughcolor=PANEL,
                          highlightthickness=0, showvalue=False)
            sl.pack(side='left')
            sl.bind('<ButtonRelease-1>',
                    lambda e, n=pname, v=var: self._send_param(n, v.get()))

            lbl_val = tk.Label(row, textvariable=var,
                               font=('Courier', 9), fg=VERDE,
                               bg=BG, width=5, anchor='e')
            lbl_val.pack(side='left')

    # ── Panel derecho: Path Tracker ───────────────────────────
    def _build_right(self, parent):
        right = tk.Frame(parent, bg=BG)
        right.pack(side='left', fill='both', expand=True)

        tk.Label(right, text='PATH TRACKER',
                 font=('Courier', 9), fg=TEXTO, bg=BG).pack(anchor='w')

        self.path_canvas = tk.Canvas(right, width=460, height=460,
                                     bg=PANEL, highlightthickness=0)
        self.path_canvas.pack(pady=(2, 4))

        # Leyenda + controles
        ctrl = tk.Frame(right, bg=BG)
        ctrl.pack(fill='x')

        leyenda = [('MOVIENDO', VERDE), ('GIRANDO', AMARILLO),
                   ('PARADO', GRIS),    ('PARE', ROJO)]
        for txt, col in leyenda:
            tk.Label(ctrl, text=f'● {txt}', font=('Courier', 8),
                     fg=col, bg=BG).pack(side='left', padx=6)

        tk.Button(ctrl, text='Borrar ruta',
                  font=('Courier', 8), bg=PANEL, fg=TEXTO,
                  relief='flat', command=self._borrar_ruta
                  ).pack(side='right', padx=4)

        self.lbl_odom = tk.Label(right,
                                  text='x=0.00 y=0.00 yaw=0.0°',
                                  font=('Courier', 9), fg=TEXTO, bg=BG)
        self.lbl_odom.pack()

    # ──────────────────────────────────────────────────────────
    #  CALLBACKS ROS2
    # ──────────────────────────────────────────────────────────
    def cb_scan(self, msg):
        def sector_min(deg_min, deg_max):
            rad_min, rad_max = math.radians(deg_min), math.radians(deg_max)
            vals = []
            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r) or r < 0.08 or r > 3.0:
                    continue
                ang = math.atan2(
                    math.sin(msg.angle_min + i * msg.angle_increment),
                    math.cos(msg.angle_min + i * msg.angle_increment))
                if rad_min <= ang <= rad_max:
                    vals.append(r)
            if not vals: return float('inf')
            s = sorted(vals)
            return s[int((len(s)-1)*0.25)] if deg_min == -7 else s[len(s)//2]

        with self.lock:
            self.d_f = sector_min(-7,    7)
            self.d_d = sector_min(-115, -55)
            self.d_i = sector_min( 55,  115)

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y**2 + q.z**2))
        with self.lock:
            self.pos_x, self.pos_y, self.yaw = p.x, p.y, yaw
            estado = self._estado_inferido()
            col = ESTADO_COLOR.get(estado, GRIS)
            self.path.append((p.x, p.y, col))

    def cb_vel(self, msg):
        with self.lock:
            self.vel_lin = msg.linear.x
            self.vel_ang = msg.angular.z

    def cb_pare(self, msg):
        with self.lock:
            self.pare_activo = msg.data

    # ──────────────────────────────────────────────────────────
    #  ESTADO INFERIDO
    # ──────────────────────────────────────────────────────────
    def _estado_inferido(self):
        if self.pare_activo:
            return 'PARE'
        if abs(self.vel_lin) > 0.01:
            return 'MOVIENDO'
        if abs(self.vel_ang) > 0.02:
            return 'GIRANDO'
        return 'PARADO'

    # ──────────────────────────────────────────────────────────
    #  ACTUALIZACIÓN UI  (~12 Hz)
    # ──────────────────────────────────────────────────────────
    def _actualizar(self):
        if not self._running:
            return

        with self.lock:
            d_f, d_d, d_i = self.d_f, self.d_d, self.d_i
            px, py, yaw   = self.pos_x, self.pos_y, self.yaw
            pare          = self.pare_activo
            estado        = self._estado_inferido()
            path_copy     = list(self.path)

        # Estado FSM
        col = ESTADO_COLOR.get(estado, GRIS)
        self.lbl_estado.config(text=estado, fg=col)

        # Gauges
        self._dibujar_gauges(d_f, d_d, d_i)

        # PARE
        if pare:
            self.lbl_pare.config(text='● PARE: SÍ', fg=ROJO)
        else:
            self.lbl_pare.config(text='○ PARE: NO', fg=GRIS)

        # Path tracker
        self._dibujar_path(path_copy, px, py, yaw)

        # Odom label
        self.lbl_odom.config(
            text=f'x={px:.2f}  y={py:.2f}  yaw={math.degrees(yaw):.1f}°')

        self.root.after(80, self._actualizar)

    # ──────────────────────────────────────────────────────────
    #  GAUGES
    # ──────────────────────────────────────────────────────────
    def _dibujar_gauges(self, f, d, i):
        c = self.gauge_canvas
        c.delete('all')
        W, H = 320, 90
        max_d = 2.0
        bar_h = 18
        bar_w = 200
        x0 = 70

        sectores = [
            ('F  FRENTE', f, ROJO,   10),
            ('D  DERECHA', d, VERDE, 36),
            ('I  IZQUIERDA', i, AZUL, 62),
        ]
        for label, dist, color, y in sectores:
            # Fondo
            c.create_rectangle(x0, y, x0 + bar_w, y + bar_h,
                                fill='#111122', outline='')
            # Barra
            ratio = min(dist, max_d) / max_d if math.isfinite(dist) else 1.0
            c.create_rectangle(x0, y, x0 + int(bar_w * ratio), y + bar_h,
                                fill=color, outline='')
            # Label
            c.create_text(x0 - 4, y + bar_h // 2, text=label,
                          anchor='e', fill=color,
                          font=('Courier', 8, 'bold'))
            # Valor
            val_txt = f'{dist:.2f}m' if math.isfinite(dist) else ' inf '
            c.create_text(x0 + bar_w + 6, y + bar_h // 2, text=val_txt,
                          anchor='w', fill=color, font=('Courier', 9, 'bold'))

    # ──────────────────────────────────────────────────────────
    #  PATH TRACKER
    # ──────────────────────────────────────────────────────────
    def _dibujar_path(self, path, px, py, yaw):
        c = self.path_canvas
        c.delete('all')
        W, H = 460, 460
        CX, CY = W // 2, H // 2

        if not path:
            c.create_text(CX, CY, text='Sin datos de /odom_raw',
                          fill=GRIS, font=('Courier', 10))
            return

        # Calcular escala automática
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        rango = max(max(xs) - min(xs), max(ys) - min(ys), 0.5)
        scale = (W * 0.80) / rango
        ox = (min(xs) + max(xs)) / 2
        oy = (min(ys) + max(ys)) / 2

        def to_canvas(x, y):
            return (CX + (x - ox) * scale,
                    CY - (y - oy) * scale)

        # Grid de referencia cada 0.5m
        grid_m = 0.5
        x_ini = math.floor(min(xs) / grid_m) * grid_m
        y_ini = math.floor(min(ys) / grid_m) * grid_m
        gx = x_ini
        while gx <= max(xs) + grid_m:
            cx1, cy1 = to_canvas(gx, min(ys) - 0.2)
            cx2, cy2 = to_canvas(gx, max(ys) + 0.2)
            c.create_line(cx1, cy1, cx2, cy2, fill='#1a1a3a', width=1)
            gx += grid_m
        gy = y_ini
        while gy <= max(ys) + grid_m:
            cx1, cy1 = to_canvas(min(xs) - 0.2, gy)
            cx2, cy2 = to_canvas(max(xs) + 0.2, gy)
            c.create_line(cx1, cy1, cx2, cy2, fill='#1a1a3a', width=1)
            gy += grid_m

        # Trayectoria
        for k in range(1, len(path)):
            x0c, y0c = to_canvas(path[k-1][0], path[k-1][1])
            x1c, y1c = to_canvas(path[k][0],   path[k][1])
            c.create_line(x0c, y0c, x1c, y1c,
                          fill=path[k][2], width=2)

        # Robot actual (flecha)
        rx, ry = to_canvas(px, py)
        largo = 14
        dx = largo * math.cos(yaw)
        dy = largo * math.sin(yaw)
        # Cuerpo
        c.create_oval(rx - 7, ry - 7, rx + 7, ry + 7,
                      fill='#ffffff', outline='#aaaaaa', width=2)
        # Dirección
        c.create_line(rx, ry, rx + dx, ry - dy,
                      fill='#ff4444', width=3, arrow='last')

        # Inicio y fin
        sx, sy = to_canvas(path[0][0], path[0][1])
        c.create_oval(sx - 5, sy - 5, sx + 5, sy + 5,
                      fill=VERDE, outline='')
        c.create_text(sx + 8, sy, text='INICIO', fill=VERDE,
                      font=('Courier', 7), anchor='w')

    def _borrar_ruta(self):
        with self.lock:
            self.path.clear()

    # ──────────────────────────────────────────────────────────
    #  PARÁMETROS
    # ──────────────────────────────────────────────────────────
    def _send_param(self, name, value):
        """Envía ros2 param set en hilo separado para no bloquear UI."""
        def _run():
            try:
                subprocess.run(
                    ['ros2', 'param', 'set', '/maze_solver', name, str(round(value, 4))],
                    env=os.environ, timeout=3,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        threading.Thread(target=_run, daemon=True).start()

    # ──────────────────────────────────────────────────────────
    #  ROS2 SPIN
    # ──────────────────────────────────────────────────────────
    def _ros_spin(self):
        rclpy.init()
        self.node = DashboardNode(
            self.cb_scan, self.cb_odom, self.cb_vel, self.cb_pare)
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    # ──────────────────────────────────────────────────────────
    #  CERRAR
    # ──────────────────────────────────────────────────────────
    def _cerrar(self):
        self._running = False
        self.root.after(200, self.root.destroy)


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':
    Dashboard()
