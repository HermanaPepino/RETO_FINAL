# Reto Final de Robótica — CapyTown Grand Prix

## 1. Descripción del proyecto

CapyTown Grand Prix es un sistema de navegación autónoma desarrollado para un robot Yahboom equipado con LiDAR, cámara y odometría.

El robot debe recorrer un laberinto sin control manual, evitar obstáculos, detectar intersecciones y callejones, respetar señales de **PARE** y detener la corrida al reconocer visualmente la **META**.

La solución integra:

- ROS 2 para la comunicación entre nodos.
- LiDAR para detectar paredes, obstáculos y caminos abiertos.
- Odometría para calcular posición, distancia recorrida y orientación.
- Cámara y OpenCV para detectar PARE y META.
- Una máquina de estados para controlar el comportamiento.
- El algoritmo de Trémaux para recordar caminos.
- Q-learning para valorar decisiones tomadas en las intersecciones.
- Interfaces gráficas para visualizar sensores, trayectoria y detecciones.
- Registro de métricas de cada corrida.

---

## 2. Funcionalidades principales

- Seguimiento autónomo de la pared derecha.
- Detección de obstáculos mediante LiDAR.
- Clasificación del LiDAR en frente, derecha e izquierda.
- Identificación de intersecciones y callejones.
- Centrado antes de realizar un giro.
- Giros controlados mediante el yaw de la odometría.
- Retroceso de seguridad cuando el robot está muy cerca de una pared.
- Memoria de caminos mediante Trémaux.
- Selección de acciones con apoyo de Q-learning.
- Detección visual de señales PARE.
- Detención de tres segundos ante una señal PARE válida.
- Detección visual y confirmación de la META.
- Detención de seguridad si se pierde la lectura del LiDAR.
- Registro de métricas en un archivo CSV.

---

## 3. Estructura del repositorio

```text
RETO_FINAL/
├── code/
│   ├── lidar_viz.py
│   ├── maze_solver.py
│   ├── pare_detector.py
│   ├── q_table_granprix_FINAL.json
│   ├── robot_dashboard.py
│   └── ver_pare_debug.py
├── images/
│   ├── lidar_Viz.png
│   ├── meta_detectado.png
│   ├── metricas_corridas.png
│   ├── pare_detectado.png
│   └── robot_dashboard.png
└── README.md
```

> Los comandos de instalación utilizados por el equipo asumen que los archivos que se copiarán al contenedor están disponibles en `/home/pi/NuevoProyecto/`.

---

# Funcionamiento del sistema

## 4. Flujo general

Los nodos se ejecutan al mismo tiempo y se comunican mediante tópicos ROS 2.

```text
Cámara física
    │
    ▼
camera_publisher.py
    │
    │ /camera/image_raw
    ▼
pare_detector.py
    ├────────► /pare_detectado
    ├────────► /meta_detectado
    └────────► /pare_debug
                  │
                  ▼
          ver_pare_debug.py


LiDAR ───────────────► /scan ───────────────┐
Odometría ───────────► /odom_raw ───────────┤
PARE ────────────────► /pare_detectado ─────┤
META ────────────────► /meta_detectado ─────┤
                                             ▼
                                       maze_solver.py
                                             │
                                             │ /cmd_vel
                                             ▼
                                      Motores del robot


/scan ───────────────────────────────► lidar_viz.py

/scan + /odom_raw + /cmd_vel
+ /pare_detectado ───────────────────► robot_dashboard.py
```

El nodo principal es `maze_solver.py`. Este recibe la información del LiDAR, la odometría y el detector visual. Después decide si debe avanzar, corregir su trayectoria, detenerse o girar.

---

## 5. Tópicos utilizados

| Tópico              | Tipo                        | Uso                                              |
| ------------------- | --------------------------- | ------------------------------------------------ |
| `/camera/image_raw` | `sensor_msgs/msg/Image`     | Transporta los frames de la cámara.              |
| `/pare_debug`       | `sensor_msgs/msg/Image`     | Imagen procesada para depuración visual.         |
| `/pare_detectado`   | `std_msgs/msg/Bool`         | Indica que una señal PARE fue detectada.         |
| `/meta_detectado`   | `std_msgs/msg/Bool`         | Indica que la META fue detectada.                |
| `/scan`             | `sensor_msgs/msg/LaserScan` | Contiene las mediciones del LiDAR.               |
| `/odom_raw`         | `nav_msgs/msg/Odometry`     | Contiene posición y orientación del robot.       |
| `/cmd_vel`          | `geometry_msgs/msg/Twist`   | Envía velocidades lineales y angulares al robot. |

---

## 6. Función de cada archivo

### `maze_solver.py`

Es el nodo central de navegación.

Se suscribe a:

```text
/scan
/odom_raw
/pare_detectado
/meta_detectado
```

Publica en:

```text
/cmd_vel
```

También guarda:

```text
/ros2_ws/metricas_granprix.csv
/ros2_ws/q_table_granprix.json
```

Sus funciones principales son:

- Procesar el LiDAR.
- Seguir la pared derecha.
- Detectar intersecciones.
- Detectar callejones.
- Controlar los giros mediante odometría.
- Aplicar la máquina de estados.
- Recordar caminos con Trémaux.
- Consultar y actualizar la tabla Q.
- Respetar señales PARE.
- Confirmar la META.
- Detener el robot si el LiDAR deja de publicar.
- Guardar métricas de la corrida.

### `pare_detector.py`

Es el nodo de visión artificial.

Recibe imágenes desde:

```text
/camera/image_raw
```

Publica:

```text
/pare_detectado
/meta_detectado
/pare_debug
```

Los parámetros usados al ejecutar el nodo permiten ajustar la sensibilidad de PARE y META, por ejemplo:

- Área de la región detectada.
- Densidad.
- Saturación.
- Brillo.
- Cantidad de frames consecutivos.

El uso de varios frames evita aceptar una detección aislada como verdadera.

### `lidar_viz.py`

Se suscribe a:

```text
/scan
```

Abre una interfaz gráfica con Tkinter y muestra:

- Los puntos detectados por el LiDAR.
- El sector frontal.
- El sector derecho.
- El sector izquierdo.
- Las distancias calculadas en cada sector.
- Anillos de referencia en metros.

Este archivo no mueve el robot. Se utiliza para verificar y calibrar las lecturas.

### `robot_dashboard.py`

Se suscribe a:

```text
/scan
/odom_raw
/cmd_vel
/pare_detectado
```

Muestra:

- Estado inferido del movimiento.
- Distancias frontal, derecha e izquierda.
- Indicador de PARE.
- Posición X e Y.
- Orientación o yaw.
- Trayectoria recorrida.
- Sliders para modificar parámetros del nodo `/maze_solver`.

Los sliders ejecutan internamente comandos equivalentes a:

```bash
ros2 param set /maze_solver NOMBRE_PARAMETRO VALOR
```

### `ver_pare_debug.py`

Se suscribe a:

```text
/pare_debug
```

Convierte mensajes `sensor_msgs/msg/Image` a imágenes OpenCV sin usar `cv_bridge`.

La ventana puede cerrarse mediante:

- El botón `[X]`.
- La tecla `q`.
- La tecla `ESC`.
- El botón de cierre del sistema operativo.

Este nodo solo muestra el procesamiento visual. No detecta señales por sí mismo.

### `q_table_granprix_FINAL.json`

Contiene los valores aprendidos por Q-learning.

Ejemplo:

```json
{
  "c1,-5|h2|FMDMIA|o001": {
    "izquierda": 0.075
  },
  "c1,-6|h3|FMDMIA|o001": {
    "izquierda": 30.0
  }
}
```

Cada clave resume un estado:

```text
c1,-5 | h2 | FMDMIA | o001
```

- `c1,-5`: celda aproximada del robot.
- `h2`: orientación cardinal aproximada.
- `F`, `D`, `I`: categorías de distancia de frente, derecha e izquierda.
- `o001`: salidas disponibles.
- El valor numérico indica qué tan favorable fue una acción.

### `camera_publisher.py`

Este archivo se crea dentro del contenedor durante la instalación.

Abre la cámara física, captura frames con OpenCV y los publica como mensajes ROS 2 en:

```text
/camera/image_raw
```

---

## 7. Procesamiento del LiDAR

El LiDAR entrega un arreglo de mediciones alrededor del robot.

El código clasifica las lecturas en tres sectores:

| Sector    |          Ángulos |
| --------- | ---------------: |
| Frente    |     `-7°` a `7°` |
| Derecha   | `-115°` a `-55°` |
| Izquierda |   `55°` a `115°` |

El proceso es:

1. Se recorren las mediciones de `LaserScan`.
2. Se descartan valores infinitos o fuera del rango válido.
3. Se calcula el ángulo de cada punto.
4. El punto se asigna al sector correspondiente.
5. Se calcula una distancia representativa.

Para el frente se usa el percentil 25. Para los lados se utiliza la mediana. Esto evita depender de un único punto con ruido.

---

## 8. Odometría y ubicación del robot

Cuando llega el primer mensaje de `/odom_raw`, el nodo guarda la posición y orientación iniciales.

Ese punto se convierte en el origen de la corrida:

```text
x = 0
y = 0
yaw = 0
```

La odometría se utiliza para:

- Calcular la distancia recorrida.
- Saber cuánto ha girado el robot.
- Reconocer intersecciones visitadas.
- Construir nodos de memoria.
- Dibujar la trayectoria.
- Generar estados para Q-learning.

---

## 9. Seguimiento de pared

Mientras está en un pasillo, el robot sigue la pared derecha.

La corrección se calcula de forma proporcional:

```text
error = wall_target - distancia_derecha
velocidad_angular = kp_wall × error
```

- Si está muy lejos de la pared, corrige hacia la derecha.
- Si está muy cerca, corrige hacia la izquierda.
- Si no encuentra una pared derecha confiable, continúa recto.
- Si el frente está cerca, reduce la velocidad.
- Si existe peligro de colisión, la velocidad lineal se detiene.

---

## 10. Detección de intersecciones

Una abertura lateral no se acepta con una sola lectura.

El sistema verifica:

- Que la abertura permanezca varios ciclos.
- Que haya pasado el tiempo de protección inicial.
- Que el robot haya recorrido una distancia mínima.
- Que se haya alejado de la intersección anterior.
- Que previamente haya vuelto a detectar una pared derecha.
- Que haya finalizado el cooldown de la última intersección.

Cuando confirma una abertura lateral, avanza una pequeña distancia para colocar el centro del robot dentro de la intersección antes de decidir.

---

## 11. Máquina de estados

Los estados reales de `maze_solver.py` son:

```text
EN_PASILLO
INTERSECCION
PARAR_PARE
ESPERAR_3S
DECIDIR_GIRO
GIRAR
META
```

### `EN_PASILLO`

Avanza, sigue la pared y busca obstáculos o intersecciones.

### `INTERSECCION`

Detiene el movimiento y, si la apertura fue lateral, centra el robot antes de decidir.

### `PARAR_PARE`

Publica velocidad cero, registra el PARE y cambia al estado de espera.

### `ESPERAR_3S`

Mantiene el robot detenido durante tres segundos.

### `DECIDIR_GIRO`

Analiza las salidas disponibles, la memoria de Trémaux y la tabla Q.

### `GIRAR`

Realiza un giro a la derecha, izquierda o una media vuelta usando el yaw de la odometría.

### `META`

Realiza el avance final, detiene el robot y guarda las métricas.

Flujo principal:

```text
EN_PASILLO
    ├── Obstáculo o apertura ──► INTERSECCION
    ├── PARE ──────────────────► PARAR_PARE ─► ESPERAR_3S
    └── META ──────────────────► META

INTERSECCION ─► DECIDIR_GIRO
                    ├── Recto ─► EN_PASILLO
                    └── Giro ──► GIRAR ─► EN_PASILLO
```

---

## 12. Trémaux y Q-learning

### Trémaux

El algoritmo de Trémaux funciona como memoria de caminos.

Cuando el robot encuentra una intersección:

1. Busca si ya existe un nodo cercano.
2. Si no existe, crea uno nuevo con la posición actual.
3. Registra por qué dirección entró.
4. Registra qué salidas ya fueron utilizadas.
5. Prefiere salidas no recorridas.
6. Si no quedan salidas nuevas, regresa por donde llegó.
7. Si encuentra un callejón, marca esa rama para no volver a elegirla inmediatamente.

### Q-learning

Q-learning asigna valores a las acciones:

```text
derecha
recto
izquierda
media
```

Las acciones pueden recibir:

- Recompensa por llegar a una zona nueva.
- Recompensa por llegar a la META.
- Penalización por repetir estados.
- Penalización por entrar a un callejón.
- Penalización por una mala decisión.

Con:

```text
q_epsilon = 0.0
```

el robot utiliza las decisiones conocidas y no realiza exploración aleatoria.

---

## 13. PARE, META y seguridad

### PARE

Cuando `/pare_detectado` cambia a verdadero:

1. El nodo comprueba que la detección sea válida.
2. Detiene el robot.
3. Espera tres segundos.
4. Marca la señal como atendida.
5. Continúa la navegación.

La señal PARE no interrumpe un giro que ya está en ejecución.

### META

La META debe mantenerse detectada durante un tiempo mínimo.

Además, el robot comprueba:

- Tiempo mínimo de ejecución.
- Distancia mínima recorrida.
- Distancia frontal segura.

Cuando se confirma:

1. Enclava la detección.
2. Recompensa la última acción de Q-learning.
3. Guarda la tabla Q.
4. Realiza el avance final.
5. Detiene el robot.
6. Guarda las métricas.

### Failsafe del LiDAR

Si `/scan` deja de actualizarse durante más tiempo que `scan_timeout_s`, el robot publica velocidad cero.

No vuelve a moverse hasta recuperar las lecturas.

---

## 14. Métricas generadas

Al terminar la corrida o interrumpir el nodo con `Ctrl + C`, se genera:

```text
/ros2_ws/metricas_granprix.csv
```

El archivo incluye:

- Número de ronda.
- Si llegó o no a la META.
- Tiempo de recorrido.
- Longitud de la ruta.
- Eficiencia.
- Cantidad de PARE detectados.
- Cantidad de PARE respetados.
- Callejones visitados.
- Colisiones registradas.

---

# Tutorial de instalación y ejecución

## 15. Requisitos previos

Antes de comenzar:

- El contenedor Docker del robot debe estar iniciado.
- ROS 2 debe funcionar dentro del contenedor.
- El LiDAR, la cámara y la odometría deben estar conectados.
- Los archivos deben estar disponibles en:

```text
/home/pi/NuevoProyecto/
```

Archivos esperados:

```text
maze_solver.py
pare_detector.py
ver_pare_debug.py
robot_dashboard.py
lidar_viz.py
q_table_granprix_FINAL.json
```

---

## 16. Identificar el contenedor

El ID del contenedor puede cambiar. Desde una terminal de la Raspberry Pi ejecutar:

```bash
docker ps
```

Guardar el ID encontrado:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Comprobarlo:

```bash
echo "$CONTAINER_ID"
```

> La variable debe volver a definirse al abrir una nueva terminal de la Raspberry Pi.

---

## 17. Crear el workspace y el paquete

Desde la Raspberry Pi:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
docker exec -it "$CONTAINER_ID" bash
```

Dentro del contenedor:

```bash
rm -rf /ros2_ws
mkdir -p /ros2_ws/src
cd /ros2_ws/src

ros2 pkg create capytown --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs nav_msgs std_msgs
```

Salir del contenedor:

```bash
exit
```

---

## 18. Copiar los archivos al contenedor

Estos comandos se ejecutan desde la Raspberry Pi, fuera del contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

```bash
docker cp /home/pi/NuevoProyecto/maze_solver.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/maze_solver.py

docker cp /home/pi/NuevoProyecto/pare_detector.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/pare_detector.py

docker cp /home/pi/NuevoProyecto/ver_pare_debug.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/ver_pare_debug.py

docker cp /home/pi/NuevoProyecto/robot_dashboard.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/robot_dashboard.py

docker cp /home/pi/NuevoProyecto/lidar_viz.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/lidar_viz.py

docker cp /home/pi/NuevoProyecto/q_table_granprix_FINAL.json \
"$CONTAINER_ID":/ros2_ws/q_table_granprix.json
```

---

## 19. Crear `camera_publisher.py`

Volver a ingresar al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Dentro del contenedor:

```bash
cat > /ros2_ws/src/capytown/capytown/camera_publisher.py <<'PY'
#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('topic', '/camera/image_raw')

        device = int(self.get_parameter('device').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        topic = str(self.get_parameter('topic').value)

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            raise RuntimeError(f'No se pudo abrir /dev/video{device}')

        self.publisher = self.create_publisher(Image, topic, 10)
        self.timer = self.create_timer(1.0 / max(self.fps, 1.0), self.publicar)

        self.get_logger().info(
            f'Cámara /dev/video{device} abierta → {topic} '
            f'({width}x{height} @ {self.fps:.1f} FPS)'
        )

    def publicar(self):
        ok, frame = self.cap.read()

        if not ok or frame is None:
            self.get_logger().warning('No se pudo leer un frame')
            return

        frame = np.ascontiguousarray(frame)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()

        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = CameraPublisher()

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
PY
```

---

## 20. Configurar `setup.py`

Dentro del contenedor:

```bash
cat > /ros2_ws/src/capytown/setup.py <<'PY'
from setuptools import find_packages, setup

package_name = 'capytown'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='g1',
    maintainer_email='g1@example.com',
    description='CapyTown Gran Prix G1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = capytown.camera_publisher:main',
            'maze_solver = capytown.maze_solver:main',
            'pare_detector = capytown.pare_detector:main',
            'ver_pare_debug = capytown.ver_pare_debug:main',
            'robot_dashboard = capytown.robot_dashboard:main',
            'lidar_viz = capytown.lidar_viz:main',
        ],
    },
)
PY
```

---

## 21. Agregar `main()` donde sea necesario

Dentro del contenedor:

```bash
python3 - <<'PY'
from pathlib import Path

archivos = [
    (
        Path("/ros2_ws/src/capytown/capytown/lidar_viz.py"),
        "LidarViz"
    ),
    (
        Path("/ros2_ws/src/capytown/capytown/robot_dashboard.py"),
        "Dashboard"
    ),
]

for ruta, clase in archivos:
    texto = ruta.read_text()

    if "def main(" in texto:
        print(f"{ruta.name}: ya tiene main()")
        continue

    texto += f"""


def main(args=None):
    {clase}()


if __name__ == '__main__':
    main()
"""

    ruta.write_text(texto)
    print(f"{ruta.name}: main() agregado usando {clase}")
PY
```

---

## 22. Compilar el paquete

Dentro del contenedor:

```bash
cd /ros2_ws
rm -rf build install log
colcon build --packages-select capytown --symlink-install
source install/setup.bash
```

Después:

```bash
cd /ros2_ws
source install/setup.bash
```

---

# Ejecución en seis terminales

Cada nodo debe permanecer abierto en su propia terminal.

En cada terminal nueva de la Raspberry Pi:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
docker exec -it "$CONTAINER_ID" bash
```

Luego, dentro del contenedor:

```bash
cd /ros2_ws
source install/setup.bash
```

---

## 23. Terminal 1 — Cámara

```bash
ROS_DOMAIN_ID=20 ros2 run capytown camera_publisher --ros-args -p device:=0
```

Publica las imágenes en:

```text
/camera/image_raw
```

Dejar esta terminal abierta.

---

## 24. Terminal 2 — Detector de PARE y META

Elegir **solo uno** de los tres perfiles.

### Perfil 1 — Estricto

```bash
ROS_DOMAIN_ID=20 ros2 run capytown pare_detector --ros-args \
-p image_topic:=/camera/image_raw \
-p meta_area_min:=12000 \
-p meta_frames_confirm:=8 \
-p meta_s_min:=80 \
-p meta_v_min:=50
```

### Perfil 2 — No estricto

```bash
ROS_DOMAIN_ID=20 ros2 run capytown pare_detector --ros-args \
-p image_topic:=/camera/image_raw \
-p meta_area_min:=8000 \
-p meta_frames_confirm:=5 \
-p meta_s_min:=60 \
-p meta_v_min:=40
```

### Perfil 3 — El más estricto

```bash
ROS_DOMAIN_ID=20 ros2 run capytown pare_detector --ros-args \
-p image_topic:=/camera/image_raw \
-p pare_area_max:=14000 \
-p pare_densidad_min:=0.45 \
-p pare_frames_confirm:=5 \
-p meta_area_min:=12000 \
-p meta_frames_confirm:=8 \
-p meta_s_min:=80 \
-p meta_v_min:=50
```

Dejar esta terminal abierta.

---

## 25. Terminal 3 — Visor de detección

```bash
DISPLAY=:0 ROS_DOMAIN_ID=20 ros2 run capytown ver_pare_debug
```

Muestra `/pare_debug`.

Dejar esta terminal abierta.

---

## 26. Terminal 4 — Visualizador LiDAR

```bash
DISPLAY=:0 ROS_DOMAIN_ID=20 ros2 run capytown lidar_viz
```

Muestra los puntos del LiDAR y las distancias de frente, derecha e izquierda.

Dejar esta terminal abierta.

---

## 27. Terminal 5 — Dashboard

```bash
DISPLAY=:0 ROS_DOMAIN_ID=20 ros2 run capytown robot_dashboard
```

Muestra el movimiento, las distancias, la odometría, el PARE y la trayectoria.

Dejar esta terminal abierta.

---

## 28. Terminal 6 — Navegación autónoma

El nodo principal se ejecuta al final, después de comprobar que los sensores y nodos anteriores funcionan.

```bash
ROS_DOMAIN_ID=20 ros2 run capytown maze_solver --ros-args \
-p ronda:=0 \
-p qlearn_enabled:=true \
-p q_epsilon:=0.0 \
-p q_alpha:=0.30 \
-p v_forward:=0.050 \
-p turn_speed:=0.16 \
-p front_stop:=0.42 \
-p side_open:=1.10 \
-p side_confirm_ticks:=7 \
-p side_rearm_wall_max:=0.72 \
-p side_rearm_ticks:=6 \
-p intersection_min_exit_m:=0.48 \
-p startup_side_guard_s:=6.50 \
-p startup_min_travel_m:=0.25 \
-p intersection_center_distance_m:=0.11 \
-p intersection_center_timeout_s:=3.50 \
-p intersection_center_v:=0.040 \
-p intersection_center_front_min:=0.58 \
-p memory_node_radius:=0.45 \
-p intersection_cooldown_s:=6.5 \
-p scan_timeout_s:=0.80 \
-p post_turn_stop_s:=0.40 \
-p turn_slowdown_deg:=25.0 \
-p turn_near_speed:=0.10 \
-p turn_brake_margin_deg:=0.5 \
-p giro_front_safety:=0.22 \
-p giro_yaw_min_deg:=84.0 \
-p media_yaw_min_deg:=165.0 \
-p giro_timeout_s:=16.0 \
-p media_timeout_s:=28.0 \
-p meta_confirm_s:=0.80 \
-p meta_min_distance_m:=2.50 \
-p meta_min_runtime_s:=25.0
```

Al ejecutar este comando, el robot comienza a moverse.

---

## 29. Orden resumido

| Terminal | Nodo               |
| -------- | ------------------ |
| 1        | `camera_publisher` |
| 2        | `pare_detector`    |
| 3        | `ver_pare_debug`   |
| 4        | `lidar_viz`        |
| 5        | `robot_dashboard`  |
| 6        | `maze_solver`      |

`maze_solver` debe ejecutarse al final.

---

## 30. Reiniciar la tabla Q

Este comando elimina la tabla Q utilizada por el robot:

```bash
rm -f /ros2_ws/q_table_granprix.json
```

Es opcional y solo debe ejecutarse cuando se desea eliminar el aprendizaje anterior.

Para volver a cargar la tabla inicial, ejecutar desde la Raspberry Pi:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

```bash
docker cp /home/pi/NuevoProyecto/q_table_granprix_FINAL.json \
"$CONTAINER_ID":/ros2_ws/q_table_granprix.json
```

---

## 31. Detener el sistema

Para detener un nodo:

```text
Ctrl + C
```

Orden recomendado:

1. `maze_solver`
2. `pare_detector`
3. `camera_publisher`
4. `ver_pare_debug`
5. `lidar_viz`
6. `robot_dashboard`
