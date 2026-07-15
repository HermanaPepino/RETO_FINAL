#!/usr/bin/env python3
"""
ver_pare_debug.py — Visor opcional para /pare_debug con botón de cierre real.

/pare_debug es solo un tópico de imagen: pare_detector.py no puede "cerrar"
ninguna ventana porque no controla cómo la visualizas (rqt_image_view, este
script, etc). Este visor sí abre una ventana propia con cv2.imshow y le da
un cierre que funciona de tres formas:
  - Click en el botón [X] (la misma zona que se dibuja en el dashboard).
  - Tecla 'q' o ESC.
  - Botón nativo de cerrar de la ventana del sistema operativo.

No reemplaza ni modifica pare_detector.py ni sus tópicos; solo se suscribe
a /pare_debug para mostrarlo.

Uso:
    ros2 run capytown ver_pare_debug
    # o directamente:
    python3 ver_pare_debug.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
import numpy as np


WINDOW_NAME = 'pare_debug'


def imgmsg_to_bgr(msg: Image):
    """Misma conversión que pare_detector.py, sin cv_bridge."""
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == 'bgr8':
        return arr.reshape(msg.height, msg.width, 3)
    elif msg.encoding == 'rgb8':
        img = arr.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding == 'mono8':
        img = arr.reshape(msg.height, msg.width)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        raise ValueError(f'encoding no soportado: {msg.encoding}')


class VisorPareDebug(Node):

    def __init__(self):
        super().__init__('ver_pare_debug')
        self._cerrar = False
        self._btn = None  # (x0, y0, x1, y1) del botón [X], se calcula con el primer frame

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, self._on_mouse)

        self.create_subscription(Image, '/pare_debug', self.cb_imagen, 10)
        self.get_logger().info('Visor listo. Click en [X], "q" o ESC para cerrar.')

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self._btn is None:
            return
        x0, y0, x1, y1 = self._btn
        if x0 <= x <= x1 and y0 <= y <= y1:
            self._cerrar = True

    def cb_imagen(self, msg: Image):
        if self._cerrar:
            return
        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'error convirtiendo imagen: {e}')
            return

        # Misma posición/tamaño que draw_close_button() en pare_detector.py,
        # así el botón dibujado coincide con la zona clicable real.
        h, w = frame.shape[:2]
        size, margen = 22, 8
        x1b, y0b = w - margen, margen
        x0b, y1b = x1b - size, y0b + size
        self._btn = (x0b, y0b, x1b, y1b)

        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):  # 'q' o ESC
            self._cerrar = True

        if self._cerrar:
            cv2.destroyWindow(WINDOW_NAME)
            self.get_logger().info('Ventana cerrada por el usuario.')


def main(args=None):
    rclpy.init(args=args)
    nodo = VisorPareDebug()
    try:
        while rclpy.ok() and not nodo._cerrar:
            rclpy.spin_once(nodo, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
