#!/usr/bin/env python3
"""
pare_detector.py — Gran Prix CapyTown · G1
Detector integrado de señales por cámara:
  - PARE rojo  -> publica /pare_detectado
  - META verde -> publica /meta_detectado
  - Debug      -> publica /pare_debug con dashboard visual (tarjetas de
                  estado, bounding boxes limpias y panel de métricas)

Mantiene el nombre del nodo y del ejecutable: pare_detector.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import cv2
import numpy as np


def imgmsg_to_bgr(msg: Image):
    """Convierte sensor_msgs/Image a BGR numpy sin cv_bridge."""
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == 'bgr8':
        return arr.reshape(msg.height, msg.width, 3)
    elif msg.encoding == 'rgb8':
        img = arr.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif msg.encoding in ('yuv422', 'yuv422_yuy2', 'yuyv'):
        img = arr.reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
    elif msg.encoding == 'mono8':
        img = arr.reshape(msg.height, msg.width)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        raise ValueError(f'encoding no soportado: {msg.encoding}')


def bgr_to_imgmsg(img, header):
    """Convierte BGR numpy a sensor_msgs/Image bgr8, sin cv_bridge."""
    out = Image()
    out.header = header
    out.height = img.shape[0]
    out.width = img.shape[1]
    out.encoding = 'bgr8'
    out.is_bigendian = 0
    out.step = img.shape[1] * 3
    out.data = np.ascontiguousarray(img).tobytes()
    return out


# HSV PARE rojo: dos rangos porque rojo cruza H=0/180
RED_LO1 = np.array([0, 85, 60], dtype=np.uint8)
RED_HI1 = np.array([10, 255, 255], dtype=np.uint8)
RED_LO2 = np.array([165, 85, 60], dtype=np.uint8)
RED_HI2 = np.array([180, 255, 255], dtype=np.uint8)

# HSV META verde: ajustable por parámetros
GREEN_LO = np.array([35, 45, 35], dtype=np.uint8)
GREEN_HI = np.array([95, 255, 255], dtype=np.uint8)

# ROI central
ROI_Y_START = 0.01
ROI_Y_END   = 0.96
ROI_X_START = 0.04
ROI_X_END   = 0.96


# ---------------------------------------------------------------------------
# Utilidades visuales para el dashboard de /pare_debug
# (No participan en la detección; solo dibujan sobre la imagen de salida)
# ---------------------------------------------------------------------------

def _rect_alpha(img, pt1, pt2, color, alpha):
    """Dibuja un rectángulo relleno semitransparente sobre img (in-place)."""
    x0, y0 = pt1
    x1, y1 = pt2
    h, w = img.shape[:2]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)
    if x1 <= x0 or y1 <= y0:
        return
    sub = img[y0:y1, x0:x1]
    overlay = np.full_like(sub, color, dtype=np.uint8)
    blended = cv2.addWeighted(overlay, alpha, sub, 1 - alpha, 0)
    img[y0:y1, x0:x1] = blended


def _label_con_fondo(img, texto, org, escala=0.45, color_txt=(255, 255, 255),
                      color_fondo=(0, 0, 0), grosor=1, alpha=0.55, pad=4):
    """Escribe 'texto' con un fondo semitransparente para que se lea bien
    sobre cualquier parte de la imagen."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(texto, font, escala, grosor)
    x, y = org
    _rect_alpha(img, (x - pad, y - th - pad), (x + tw + pad, y + baseline + pad),
                color_fondo, alpha)
    cv2.putText(img, texto, (x, y), font, escala, color_txt, grosor, cv2.LINE_AA)


def _tarjeta_estado(img, x, y, w, h, titulo, activo, color_on, color_off):
    """Dibuja una tarjeta rectangular tipo 'PARE: ON/OFF'."""
    color = color_on if activo else color_off
    cv2.rectangle(img, (x, y), (x + w, y + h), color, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 255), 1)
    estado = 'ON' if activo else 'OFF'
    texto = f'{titulo}: {estado}'
    font = cv2.FONT_HERSHEY_SIMPLEX
    escala, grosor = 0.55, 2
    (tw, th), _ = cv2.getTextSize(texto, font, escala, grosor)
    tx = x + max(4, (w - tw) // 2)
    ty = y + (h + th) // 2
    cv2.putText(img, texto, (tx, ty), font, escala, (255, 255, 255), grosor, cv2.LINE_AA)


def draw_bbox_pare(dbg, box, area, info, activo):
    """Bounding box de PARE: roja cuando está confirmada, naranja mientras
    se está confirmando. Label con área/aspecto/densidad, evitando taparlo."""
    bx, by, bw, bh = box
    color = (0, 0, 255) if activo else (0, 140, 255)
    cv2.rectangle(dbg, (bx, by), (bx + bw, by + bh), color, 2)
    aspecto, densidad = info if info else (0.0, 0.0)
    texto = f'PARE  a={area:.0f}  r={aspecto:.2f}  d={densidad:.2f}'
    org = (bx, by - 8) if by - 22 > 0 else (bx, by + bh + 18)
    _label_con_fondo(dbg, texto, org, color_txt=(255, 255, 255), color_fondo=(0, 0, 0))


def draw_bbox_meta(dbg, box, area, info, activo):
    """Bounding box de META: verde cuando está confirmada, magenta mientras
    se está confirmando. Label con área/aspecto/densidad, evitando taparlo."""
    bx, by, bw, bh = box
    color = (0, 200, 0) if activo else (255, 0, 200)
    cv2.rectangle(dbg, (bx, by), (bx + bw, by + bh), color, 2)
    aspecto, densidad = info if info else (0.0, 0.0)
    texto = f'META  a={area:.0f}  r={aspecto:.2f}  d={densidad:.2f}'
    org = (bx, by - 8) if by - 22 > 0 else (bx, by + bh + 18)
    _label_con_fondo(dbg, texto, org, color_txt=(255, 255, 255), color_fondo=(0, 0, 0))


def draw_status_panel(dbg, pare_activo, meta_activo, pare_area, meta_area,
                       pare_info, meta_info, pare_visto=0, meta_visto=0):
    """
    Dibuja el dashboard completo sobre la imagen de debug:
      - Tarjetas grandes PARE / META arriba a la izquierda (rojo/verde vs gris).
      - Panel inferior con área, aspecto, densidad, frames vistos y estado
        de publicación de cada señal.

    pare_info / meta_info: tupla (aspecto, densidad) o None si no hay detección
    en el frame actual.
    pare_visto / meta_visto: contador de frames consecutivos vistos (opcional,
    valor por defecto 0 para no romper llamadas existentes).
    """
    h, w = dbg.shape[:2]

    # --- Tarjetas superiores -------------------------------------------------
    card_w = max(110, min(int(w * 0.24), 170))
    card_h = 34
    margen = 10
    _tarjeta_estado(dbg, margen, margen, card_w, card_h,
                     'PARE', pare_activo, color_on=(0, 0, 220), color_off=(90, 90, 90))
    _tarjeta_estado(dbg, margen, margen + card_h + 6, card_w, card_h,
                     'META', meta_activo, color_on=(0, 180, 0), color_off=(90, 90, 90))

    # --- Panel inferior --------------------------------------------------
    panel_h = 58
    py0 = h - panel_h
    _rect_alpha(dbg, (0, py0), (w, h), (20, 20, 20), 0.55)
    cv2.line(dbg, (0, py0), (w, py0), (255, 255, 255), 1)

    asp_p, dens_p = pare_info if pare_info else (0.0, 0.0)
    asp_m, dens_m = meta_info if meta_info else (0.0, 0.0)

    font = cv2.FONT_HERSHEY_SIMPLEX
    escala, color = 0.42, (230, 230, 230)

    linea1 = (f'PARE  area={pare_area:.0f}px2  asp={asp_p:.2f}  dens={dens_p:.2f}  '
              f'vistos={pare_visto}  pub={"ON" if pare_activo else "OFF"}')
    linea2 = (f'META  area={meta_area:.0f}px2  asp={asp_m:.2f}  dens={dens_m:.2f}  '
              f'vistos={meta_visto}  pub={"ON" if meta_activo else "OFF"}')

    cv2.putText(dbg, linea1, (10, py0 + 22), font, escala, color, 1, cv2.LINE_AA)
    cv2.putText(dbg, linea2, (10, py0 + 44), font, escala, color, 1, cv2.LINE_AA)


def draw_close_button(dbg, size=22, margen=8):
    """
    Dibuja una 'X' en la esquina superior derecha, estilo botón de cerrar.

    OJO: es puramente decorativo. /pare_debug es un tópico de ROS (una
    imagen publicada), no una ventana con la que se pueda interactuar, así
    que este dibujo por sí solo no cierra nada. Si se visualiza con el
    script auxiliar ver_pare_debug.py, ese sí detecta el click sobre esta
    misma zona y cierra la ventana de verdad.
    """
    h, w = dbg.shape[:2]
    x1, y0 = w - margen, margen
    x0, y1 = x1 - size, y0 + size
    cv2.rectangle(dbg, (x0, y0), (x1, y1), (60, 60, 60), -1)
    cv2.rectangle(dbg, (x0, y0), (x1, y1), (255, 255, 255), 1)
    pad = 6
    cv2.line(dbg, (x0 + pad, y0 + pad), (x1 - pad, y1 - pad), (255, 255, 255), 2)
    cv2.line(dbg, (x1 - pad, y0 + pad), (x0 + pad, y1 - pad), (255, 255, 255), 2)


class PareDetector(Node):

    def __init__(self):
        super().__init__('pare_detector')

        # Tópico de cámara. Se conserva /image_raw porque tu detector ya funcionó así.
        self.declare_parameter('image_topic', '/image_raw')

        # PARE rojo
        self.declare_parameter('pare_area_min', 650)
        self.declare_parameter('pare_area_max', 25000)
        self.declare_parameter('pare_aspecto_min', 0.55)
        self.declare_parameter('pare_aspecto_max', 1.75)
        self.declare_parameter('pare_densidad_min', 0.28)
        self.declare_parameter('pare_frames_confirm', 4)
        self.declare_parameter('pare_frames_release', 8)
        # Filtro vertical: evita huecos bajos de la madera/piso.
        # 0.0 arriba, 1.0 abajo. Ajustable en vivo.
        self.declare_parameter('pare_y_center_min', 0.04)
        self.declare_parameter('pare_y_center_max', 0.68)

        # META verde
        self.declare_parameter('meta_enabled', True)
        self.declare_parameter('meta_h_min', 35)
        self.declare_parameter('meta_h_max', 95)
        self.declare_parameter('meta_s_min', 45)
        self.declare_parameter('meta_v_min', 35)
        self.declare_parameter('meta_area_min', 3500)
        self.declare_parameter('meta_area_max', 90000)
        self.declare_parameter('meta_aspecto_min', 1.35)
        self.declare_parameter('meta_aspecto_max', 5.5)
        self.declare_parameter('meta_densidad_min', 0.28)
        self.declare_parameter('meta_frames_confirm', 6)
        self.declare_parameter('meta_frames_release', 3)

        # META puede aparecer recortada arriba o a un costado.
        # En esas posiciones su área visible baja bastante, por lo que usa
        # un umbral específico sin volver permisivo todo el detector.
        self.declare_parameter('meta_partial_area_min', 2800)
        self.declare_parameter('meta_y_center_max', 0.46)
        self.declare_parameter('meta_partial_top_frac', 0.18)
        self.declare_parameter('meta_partial_side_frac', 0.14)
        self.declare_parameter('meta_width_min_px', 80)
        self.declare_parameter('meta_height_min_px', 18)
        self.declare_parameter('meta_partial_border_px', 12)
        self.declare_parameter('meta_stable_center_px', 90.0)
        self.declare_parameter('meta_density_max', 0.94)

        self.declare_parameter('publicar_debug', True)

        # Estados internos
        self._pare_visto = 0
        self._pare_no_visto = 0
        self._pare_activo = False

        self._meta_visto = 0
        self._meta_no_visto = 0
        self._meta_activo = False
        self._meta_last_box = None
        self._meta_last_area = 0.0

        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, image_topic, self.cb_imagen, 10)

        self.pub_pare = self.create_publisher(Bool, '/pare_detectado', 10)
        self.pub_meta = self.create_publisher(Bool, '/meta_detectado', 10)
        self.pub_img = self.create_publisher(Image, '/pare_debug', 10)

        self.get_logger().info(
            f'PareDetector integrado listo | cam={image_topic} | '
            f'publica /pare_detectado y /meta_detectado'
        )

    def _get_params(self):
        return {
            'pare_area_min': float(self.get_parameter('pare_area_min').value),
            'pare_area_max': float(self.get_parameter('pare_area_max').value),
            'pare_aspecto_min': float(self.get_parameter('pare_aspecto_min').value),
            'pare_aspecto_max': float(self.get_parameter('pare_aspecto_max').value),
            'pare_densidad_min': float(self.get_parameter('pare_densidad_min').value),
            'pare_confirm': int(self.get_parameter('pare_frames_confirm').value),
            'pare_release': int(self.get_parameter('pare_frames_release').value),
            'pare_y_center_min': float(self.get_parameter('pare_y_center_min').value),
            'pare_y_center_max': float(self.get_parameter('pare_y_center_max').value),

            'meta_enabled': bool(self.get_parameter('meta_enabled').value),
            'meta_h_min': int(self.get_parameter('meta_h_min').value),
            'meta_h_max': int(self.get_parameter('meta_h_max').value),
            'meta_s_min': int(self.get_parameter('meta_s_min').value),
            'meta_v_min': int(self.get_parameter('meta_v_min').value),
            'meta_area_min': float(self.get_parameter('meta_area_min').value),
            'meta_area_max': float(self.get_parameter('meta_area_max').value),
            'meta_aspecto_min': float(self.get_parameter('meta_aspecto_min').value),
            'meta_aspecto_max': float(self.get_parameter('meta_aspecto_max').value),
            'meta_densidad_min': float(self.get_parameter('meta_densidad_min').value),
            'meta_confirm': int(self.get_parameter('meta_frames_confirm').value),
            'meta_release': int(self.get_parameter('meta_frames_release').value),
            'meta_partial_area_min': float(self.get_parameter('meta_partial_area_min').value),
            'meta_y_center_max': float(self.get_parameter('meta_y_center_max').value),
            'meta_partial_top_frac': float(self.get_parameter('meta_partial_top_frac').value),
            'meta_partial_side_frac': float(self.get_parameter('meta_partial_side_frac').value),
            'meta_width_min_px': int(self.get_parameter('meta_width_min_px').value),
            'meta_height_min_px': int(self.get_parameter('meta_height_min_px').value),
            'meta_partial_border_px': int(self.get_parameter('meta_partial_border_px').value),
            'meta_stable_center_px': float(self.get_parameter('meta_stable_center_px').value),
            'meta_density_max': float(self.get_parameter('meta_density_max').value),
            'publicar_debug': bool(self.get_parameter('publicar_debug').value),
        }

    def _buscar_mejor(self, mask, area_min, area_max, asp_min, asp_max, dens_min, x0, y0):
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mejor = None
        mejor_area = 0.0
        mejor_info = None

        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if not (area_min <= area <= area_max):
                continue

            bx, by, bw, bh = cv2.boundingRect(cnt)
            if bh == 0 or bw == 0:
                continue

            aspecto = bw / bh
            densidad = area / float(bw * bh)

            if not (asp_min <= aspecto <= asp_max):
                continue
            if densidad < dens_min:
                continue

            if area > mejor_area:
                mejor_area = area
                mejor = (bx + x0, by + y0, bw, bh)
                mejor_info = (aspecto, densidad)

        return mejor, mejor_area, mejor_info

    def _buscar_meta(self, mask, p, frame_w, frame_h):
        """
        Busca META en toda la parte superior de la imagen.

        La señal puede quedar parcialmente fuera del encuadre. En ese caso
        acepta un área menor solo cuando está cerca del borde superior o
        lateral y conserva filtros de color, tamaño, proporción y densidad.
        """
        contornos, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mejor = None
        mejor_area = 0.0
        mejor_info = None

        for cnt in contornos:
            area = float(cv2.contourArea(cnt))
            bx, by, bw, bh = cv2.boundingRect(cnt)

            if bw < int(p['meta_width_min_px']) or bh < int(p['meta_height_min_px']):
                continue
            if area > float(p['meta_area_max']):
                continue

            aspecto = bw / float(bh)
            densidad = area / float(max(1, bw * bh))
            if not (float(p['meta_aspecto_min']) <= aspecto <=
                    float(p['meta_aspecto_max'])):
                continue
            if densidad < float(p['meta_densidad_min']):
                continue
            if densidad > float(p['meta_density_max']):
                continue

            cy_frac = (by + 0.5 * bh) / float(frame_h)
            if cy_frac > float(p['meta_y_center_max']):
                continue

            # El umbral parcial solo se permite cuando el objeto está
            # físicamente cortado por un borde. Antes cualquier verde ubicado
            # en el 18 % superior recibía el umbral reducido.
            borde = int(p['meta_partial_border_px'])
            toca_arriba = by <= borde
            toca_izq = bx <= borde
            toca_der = (bx + bw) >= (frame_w - borde)
            parcial = toca_arriba or toca_izq or toca_der

            area_requerida = (
                float(p['meta_partial_area_min'])
                if parcial else float(p['meta_area_min'])
            )
            if area < area_requerida:
                continue

            if area > mejor_area:
                mejor_area = area
                mejor = (bx, by, bw, bh)
                mejor_info = (aspecto, densidad)

        return mejor, mejor_area, mejor_info

    def _update_meta_bool(self, detectado, box, area, p):
        """
        Confirma que sea el mismo rectángulo verde durante varios frames.
        Un reflejo o una cinta verde aislada no puede enclavar META.
        """
        if detectado and box is not None:
            bx, by, bw, bh = box
            cx = bx + 0.5 * bw
            cy = by + 0.5 * bh
            estable = False

            if self._meta_last_box is not None:
                lx, ly, lw, lh = self._meta_last_box
                lcx = lx + 0.5 * lw
                lcy = ly + 0.5 * lh
                desplazamiento = ((cx - lcx) ** 2 + (cy - lcy) ** 2) ** 0.5
                ratio_area = area / max(1.0, self._meta_last_area)
                estable = (
                    desplazamiento <= float(p['meta_stable_center_px']) and
                    0.45 <= ratio_area <= 2.20
                )

            self._meta_visto = self._meta_visto + 1 if estable else 1
            self._meta_no_visto = 0
            self._meta_last_box = box
            self._meta_last_area = float(area)
        else:
            self._meta_no_visto += 1
            self._meta_visto = 0
            self._meta_last_box = None
            self._meta_last_area = 0.0

        if (not self._meta_activo and
                self._meta_visto >= int(p['meta_confirm'])):
            self._meta_activo = True
            self.get_logger().info(
                f'META confirmada estable (area={area:.0f}px2, '
                f'frames={self._meta_visto})')
        elif (self._meta_activo and
                self._meta_no_visto >= int(p['meta_release'])):
            self._meta_activo = False
            self.get_logger().info('META perdida; publicación vuelve a OFF')

    def _update_bool(self, detectado, activo, visto, no_visto, n_confirm, n_release, nombre, area):
        if detectado:
            visto += 1
            no_visto = 0
        else:
            no_visto += 1
            visto = 0

        if not activo and visto >= n_confirm:
            activo = True
            self.get_logger().info(f'{nombre} confirmado (area={area:.0f}px2)')
        elif activo and no_visto >= n_release:
            activo = False
            self.get_logger().info(f'{nombre} perdido')

        return activo, visto, no_visto

    def cb_imagen(self, msg: Image):
        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'error convirtiendo imagen: {e}')
            return

        p = self._get_params()
        h, w = frame.shape[:2]

        y0 = int(h * ROI_Y_START)
        y1 = int(h * ROI_Y_END)
        x0 = int(w * ROI_X_START)
        x1 = int(w * ROI_X_END)
        roi = frame[y0:y1, x0:x1]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # PARE rojo
        mask_r1 = cv2.inRange(hsv, RED_LO1, RED_HI1)
        mask_r2 = cv2.inRange(hsv, RED_LO2, RED_HI2)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, iterations=1)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel, iterations=2)

        pare_box, pare_area, pare_info = self._buscar_mejor(
            mask_red,
            p['pare_area_min'], p['pare_area_max'],
            p['pare_aspecto_min'], p['pare_aspecto_max'],
            p['pare_densidad_min'], x0, y0
        )

        # Filtro vertical anti-falsos positivos:
        # los huecos/sombras en la madera suelen aparecer abajo o fuera de la
        # zona donde se coloca el cartel PARE. Esto NO afecta META.
        if pare_box is not None:
            bx, by, bw, bh = pare_box
            cy_frac = (by + bh * 0.5) / float(h)
            if not (p['pare_y_center_min'] <= cy_frac <= p['pare_y_center_max']):
                pare_box = None
                pare_area = 0.0
                pare_info = None

        pare_detectado = pare_box is not None

        self._pare_activo, self._pare_visto, self._pare_no_visto = self._update_bool(
            pare_detectado, self._pare_activo,
            self._pare_visto, self._pare_no_visto,
            p['pare_confirm'], p['pare_release'],
            'PARE', pare_area
        )

        # META verde: usa la imagen completa.
        # Antes compartía el ROI central de PARE y exigía el área completa.
        # Cuando el cartel aparecía arriba o lateral, quedaba recortado y su
        # área visible bajaba por debajo de meta_area_min.
        meta_box = None
        meta_area = 0.0
        meta_info = None
        if p['meta_enabled']:
            hsv_meta = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green_lo = np.array(
                [p['meta_h_min'], p['meta_s_min'], p['meta_v_min']],
                dtype=np.uint8)
            green_hi = np.array(
                [p['meta_h_max'], 255, 255], dtype=np.uint8)

            mask_green = cv2.inRange(hsv_meta, green_lo, green_hi)
            mask_green = cv2.morphologyEx(
                mask_green, cv2.MORPH_OPEN, kernel, iterations=1)
            mask_green = cv2.morphologyEx(
                mask_green, cv2.MORPH_CLOSE, kernel, iterations=2)

            meta_box, meta_area, meta_info = self._buscar_meta(
                mask_green, p, w, h)
            meta_detectado = meta_box is not None
        else:
            meta_detectado = False

        # META no queda enclavada en el detector. Debe mantenerse como el
        # mismo rectángulo verde; si desaparece durante varios frames vuelve
        # a OFF. El maze_solver aplica una segunda confirmación temporal.
        self._update_meta_bool(meta_detectado, meta_box, meta_area, p)

        # Publicar bools
        m_pare = Bool()
        m_pare.data = bool(self._pare_activo)
        self.pub_pare.publish(m_pare)

        m_meta = Bool()
        m_meta.data = bool(self._meta_activo)
        self.pub_meta.publish(m_meta)

        # Debug visual (dashboard)
        if p['publicar_debug']:
            dbg = frame.copy()

            # ROI de PARE con borde blanco.
            cv2.rectangle(dbg, (x0, y0), (x1, y1), (255, 255, 255), 1)

            # META se acepta en todo el ancho de la zona superior.
            meta_y_lim = int(h * float(p['meta_y_center_max']))
            cv2.line(dbg, (0, meta_y_lim), (w - 1, meta_y_lim),
                     (0, 255, 0), 1)

            if pare_box:
                draw_bbox_pare(dbg, pare_box, pare_area, pare_info, self._pare_activo)

            if meta_box:
                draw_bbox_meta(dbg, meta_box, meta_area, meta_info, self._meta_activo)

            draw_status_panel(
                dbg,
                pare_activo=self._pare_activo,
                meta_activo=self._meta_activo,
                pare_area=pare_area,
                meta_area=meta_area,
                pare_info=pare_info,
                meta_info=meta_info,
                pare_visto=self._pare_visto,
                meta_visto=self._meta_visto,
            )

            draw_close_button(dbg)

            try:
                self.pub_img.publish(bgr_to_imgmsg(dbg, msg.header))
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    nodo = PareDetector()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
