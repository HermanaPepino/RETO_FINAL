#!/usr/bin/env python3
"""
lidar_viz.py — Visualizador LiDAR para calibración del Gran Prix G1
Muestra el scan polar en tiempo real con sectores F/D/I destacados.

Uso:
    ROS_DOMAIN_ID=20 python3 lidar_viz.py

Botón CERRAR en pantalla para salir sin matar terminales a lo bruto.
"""

import math
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# ═══════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════

W, H      = 700, 720       # ventana
CX, CY    = 330, 360       # centro del plot polar
SCALE     = 130            # px por metro
MAX_R     = 2.0            # metros máximos a dibujar

# Sectores (grados) — mismos que maze_solver
SECTORES = {
    'FRENTE':   (-7,    7,   '#FF4444', 'F'),
    'DERECHA':  (-115, -55,  '#44FF44', 'D'),
    'IZQUIERDA':(55,   115,  '#4488FF', 'I'),
}


# ═══════════════════════════════════════════════════════════════
#  NODO ROS2 (corre en hilo separado)
# ═══════════════════════════════════════════════════════════════

class ScanReader(Node):
    def __init__(self, callback):
        super().__init__('lidar_viz')
        self.cb = callback
        self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, msg):
        self.cb(msg)


# ═══════════════════════════════════════════════════════════════
#  GUI
# ═══════════════════════════════════════════════════════════════

class LidarViz:
    def __init__(self):
        self.scan_msg = None
        self.lock = threading.Lock()
        self._running = True

        # ── Ventana ──────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title('LiDAR Viz — G1 CapyTown')
        self.root.configure(bg='#1a1a2e')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self._cerrar)

        # ── Canvas polar ─────────────────────────────────────
        self.canvas = tk.Canvas(self.root, width=W, height=H - 160,
                                bg='#0d0d1a', highlightthickness=0)
        self.canvas.pack()

        # ── Panel de valores ─────────────────────────────────
        panel = tk.Frame(self.root, bg='#1a1a2e', height=100)
        panel.pack(fill='x', padx=10, pady=4)

        self.lbl = {}
        datos = [
            ('F',  'FRENTE',    '#FF4444'),
            ('D',  'DERECHA',   '#44FF44'),
            ('I',  'IZQUIERDA', '#4488FF'),
        ]
        for key, nombre, color in datos:
            col = tk.Frame(panel, bg='#1a1a2e')
            col.pack(side='left', expand=True)
            tk.Label(col, text=nombre, font=('Courier', 11, 'bold'),
                     fg=color, bg='#1a1a2e').pack()
            lbl = tk.Label(col, text='-.-- m', font=('Courier', 28, 'bold'),
                           fg=color, bg='#1a1a2e', width=7)
            lbl.pack()
            self.lbl[key] = lbl

        # ── Botón CERRAR ─────────────────────────────────────
        tk.Button(
            self.root, text='  ✕  CERRAR  ',
            font=('Courier', 14, 'bold'),
            bg='#cc2222', fg='white',
            activebackground='#ff4444', activeforeground='white',
            relief='flat', cursor='hand2',
            command=self._cerrar
        ).pack(pady=8, ipadx=10, ipady=6)

        # ── Iniciar ROS2 en hilo ──────────────────────────────
        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()

        # ── Loop de refresco ─────────────────────────────────
        self._actualizar()
        self.root.mainloop()

    # ──────────────────────────────────────────────────────────
    #  ROS2
    # ──────────────────────────────────────────────────────────

    def _ros_spin(self):
        rclpy.init()
        self.node = ScanReader(self._on_scan)
        try:
            rclpy.spin(self.node)
        except Exception:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

    def _on_scan(self, msg):
        with self.lock:
            self.scan_msg = msg

    # ──────────────────────────────────────────────────────────
    #  DIBUJO
    # ──────────────────────────────────────────────────────────

    def _actualizar(self):
        if not self._running:
            return

        with self.lock:
            msg = self.scan_msg

        self.canvas.delete('all')
        self._dibujar_fondo()

        if msg:
            self._dibujar_scan(msg)
            vals = self._calcular_sectores(msg)
            for key, val in vals.items():
                txt = f'{val:.2f} m' if val < 90 else '  inf '
                self.lbl[key].config(text=txt)
        else:
            for lbl in self.lbl.values():
                lbl.config(text='sin /scan')

        self.root.after(80, self._actualizar)   # ~12 Hz

    def _dibujar_fondo(self):
        c = self.canvas

        # Anillos de distancia
        for r_m in [0.25, 0.50, 1.0, 1.5, 2.0]:
            r_px = int(r_m * SCALE)
            c.create_oval(CX - r_px, CY - r_px, CX + r_px, CY + r_px,
                          outline='#2a2a4a', width=1)
            c.create_text(CX + r_px + 4, CY - 10,
                          text=f'{r_m}m', fill='#3a3a6a',
                          font=('Courier', 8), anchor='w')

        # Ejes
        for ang in range(0, 360, 30):
            rad = math.radians(ang)
            r_px = int(MAX_R * SCALE)
            c.create_line(CX, CY,
                          CX + r_px * math.cos(rad),
                          CY - r_px * math.sin(rad),
                          fill='#1e1e3a', width=1)

        # Sectores coloreados
        for nombre, (a1, a2, color, _) in SECTORES.items():
            self._dibujar_sector(a1, a2, color, nombre)

        # Robot en el centro
        c.create_oval(CX - 8, CY - 8, CX + 8, CY + 8,
                      fill='#ffffff', outline='#aaaaaa', width=2)
        # Flecha de frente (apunta a la derecha del canvas = frente del robot)
        c.create_line(CX, CY, CX + 18, CY,
                      fill='white', width=3, arrow='last')

    def _dibujar_sector(self, deg_min, deg_max, color, nombre):
        c = self.canvas
        r_px = int(MAX_R * SCALE)
        # Dibuja arco del sector como polígono semitransparente
        puntos = [CX, CY]
        pasos = max(2, abs(deg_max - deg_min))
        for deg in range(int(deg_min), int(deg_max) + 1,
                         max(1, pasos // 20)):
            rad = math.radians(deg)
            puntos.append(CX + r_px * math.cos(rad))
            puntos.append(CY - r_px * math.sin(rad))
        puntos += [CX, CY]
        if len(puntos) >= 6:
            c.create_polygon(puntos, fill=color, stipple='gray12',
                             outline=color, width=1)
        # Etiqueta del sector
        mid_rad = math.radians((deg_min + deg_max) / 2)
        lx = CX + (r_px + 14) * math.cos(mid_rad)
        ly = CY - (r_px + 14) * math.sin(mid_rad)
        c.create_text(lx, ly, text=nombre, fill=color,
                      font=('Courier', 9, 'bold'))

    def _dibujar_scan(self, msg):
        c = self.canvas
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > MAX_R:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            ang = math.atan2(math.sin(ang), math.cos(ang))
            px = CX + r * SCALE * math.cos(ang)
            py = CY - r * SCALE * math.sin(ang)
            # Color por distancia: rojo=cerca, amarillo=medio, verde=lejos
            ratio = min(r / MAX_R, 1.0)
            if ratio < 0.3:
                col = '#ff3333'
            elif ratio < 0.6:
                col = '#ffaa00'
            else:
                col = '#33ff88'
            c.create_oval(px - 2, py - 2, px + 2, py + 2,
                          fill=col, outline='')

    def _calcular_sectores(self, msg):
        def sector_min(deg_min, deg_max):
            rad_min = math.radians(deg_min)
            rad_max = math.radians(deg_max)
            vals = []
            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r):
                    continue
                if r < max(msg.range_min, 0.08) or r > min(msg.range_max, 3.0):
                    continue
                ang = math.atan2(
                    math.sin(msg.angle_min + i * msg.angle_increment),
                    math.cos(msg.angle_min + i * msg.angle_increment))
                if rad_min <= ang <= rad_max:
                    vals.append(r)
            if not vals:
                return float('inf')
            s = sorted(vals)
            # percentil 25 para frente, mediana para lados
            if deg_min == -7:
                return s[int((len(s) - 1) * 0.25)]
            return s[len(s) // 2]

        return {
            'F': sector_min(-7,    7),
            'D': sector_min(-115, -55),
            'I': sector_min(55,   115),
        }

    # ──────────────────────────────────────────────────────────
    #  CERRAR
    # ──────────────────────────────────────────────────────────

    def _cerrar(self):
        self._running = False
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        self.root.destroy()


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

if __name__ == '__main__':
    LidarViz()
