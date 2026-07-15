# Reto Final de Robótica — CapyTown Grand Prix

## Descripción del proyecto

CapyTown Grand Prix es un sistema de navegación autónoma desarrollado para un robot Yahboom equipado con LiDAR, cámara y sensores de odometría.

El objetivo del proyecto es que el robot recorra un laberinto de forma autónoma, evite obstáculos, detecte intersecciones y callejones, respete señales de PARE y se detenga al reconocer visualmente la META.

La solución combina procesamiento de datos LiDAR, visión artificial, odometría y una máquina de estados. Además, utiliza el algoritmo de Trémaux y aprendizaje por refuerzo mediante Q-learning para reducir recorridos repetidos y mejorar la toma de decisiones durante la navegación.

## Funcionalidades principales

- Seguimiento autónomo de paredes.
- Detección de obstáculos mediante LiDAR.
- Identificación de intersecciones y callejones.
- Giros controlados utilizando la orientación del robot.
- Memoria de caminos mediante el algoritmo de Trémaux.
- Aprendizaje de decisiones mediante Q-learning.
- Detección visual de señales PARE.
- Detección visual de la META.
- Detención de seguridad ante pérdida de sensores.
- Registro de métricas del recorrido.

## Estructura del proyecto

```text
RETO_FINAL/
├── code/
│   ├── lidar_viz.py
│   ├── maze_solver.py
│   ├── pare_detector.py
│   ├── q_table_granprix_FINAL.json
│   ├── robot_dashboard.py
│   └── ver_pare_debug.py
├── docs/
│   └── RESULTADOS.md
├── images/
│   ├── lidar_Viz.png
│   ├── meta_detectado.png
│   ├── metricas_corridas.png
│   ├── pare_detectado.png
│   └── robot_dashboard.png
└── README.md
```

# Instrucciones de instalación y ejecución

## 1. Requisitos previos

Antes de comenzar, se debe comprobar lo siguiente:

- El contenedor Docker del robot debe estar iniciado.
- ROS 2 debe estar instalado y configurado dentro del contenedor.
- El LiDAR, la cámara y la odometría deben encontrarse conectados.
- Los archivos del proyecto deben estar disponibles en la Raspberry Pi.
- La carpeta del proyecto debe encontrarse en:

```bash
/home/pi/NuevoProyecto/
```

Los archivos esperados son:

```text
maze_solver.py
pare_detector.py
ver_pare_debug.py
robot_dashboard.py
lidar_viz.py
q_table_granprix_FINAL.json
```

---

# 2. Identificación del contenedor Docker

El identificador del contenedor Docker no es fijo.

Puede cambiar cuando:

- Se recrea el contenedor.
- Se elimina y vuelve a iniciar.
- Se cambia la imagen Docker.
- Se ejecuta el proyecto en otro robot.
- Docker asigna un nuevo identificador.

Por este motivo, no se utiliza un ID fijo en las instrucciones.

Desde una terminal normal de la Raspberry Pi, ejecutar:

```bash
docker ps
```

También se puede utilizar una vista más clara:

```bash
docker ps --format "table {{.ID}}\t{{.Names}}\t{{.Image}}\t{{.Status}}"
```

El comando mostrará información similar a:

```text
CONTAINER ID   NAMES          IMAGE                    STATUS
28404cc840d6   nice_euler     yahboomcar_ros2:latest   Up 20 minutes
```

Se debe identificar el contenedor que ejecuta ROS 2 para el robot.

Después, guardar su identificador en una variable:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Por ejemplo:

```bash
export CONTAINER_ID=28404cc840d6
```

El valor anterior es solamente un ejemplo. Se debe utilizar el identificador mostrado por `docker ps`.

Verificar que la variable contiene el ID:

```bash
echo "$CONTAINER_ID"
```

También se puede comprobar que el contenedor responde:

```bash
docker exec "$CONTAINER_ID" bash -lc 'echo "Contenedor encontrado correctamente"'
```

Debe aparecer:

```text
Contenedor encontrado correctamente
```

> La variable `CONTAINER_ID` solamente existe en la terminal donde fue definida.
> Al abrir una terminal nueva, se debe volver a ejecutar:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

---

# 3. Creación inicial del paquete ROS 2

Esta preparación solo es necesaria cuando:

- Se crea nuevamente el workspace.
- Se eliminó el paquete.
- Se modificó completamente la estructura.
- Se desea realizar una instalación limpia.

## Terminal de preparación

Primero, definir el ID del contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Ingresar al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Dentro del contenedor, eliminar el workspace anterior:

```bash
rm -rf /ros2_ws
```

Crear nuevamente la estructura del workspace:

```bash
mkdir -p /ros2_ws/src
cd /ros2_ws/src
```

Crear el paquete ROS 2 llamado `capytown`:

```bash
ros2 pkg create capytown \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs geometry_msgs nav_msgs std_msgs
```

Salir temporalmente del contenedor:

```bash
exit
```

---

# 4. Copia de los archivos al contenedor

Los siguientes comandos se ejecutan desde una terminal normal de la Raspberry Pi, fuera del contenedor.

Comprobar que la variable continúa definida:

```bash
echo "$CONTAINER_ID"
```

Si el comando no muestra ningún valor, volver a definirla:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

## Copiar el nodo principal de navegación

```bash
docker cp /home/pi/NuevoProyecto/maze_solver.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/maze_solver.py
```

## Copiar el detector de señales

```bash
docker cp /home/pi/NuevoProyecto/pare_detector.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/pare_detector.py
```

## Copiar el visualizador de depuración de cámara

```bash
docker cp /home/pi/NuevoProyecto/ver_pare_debug.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/ver_pare_debug.py
```

## Copiar el dashboard principal

```bash
docker cp /home/pi/NuevoProyecto/robot_dashboard.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/robot_dashboard.py
```

## Copiar el visualizador del LiDAR

```bash
docker cp /home/pi/NuevoProyecto/lidar_viz.py \
"$CONTAINER_ID":/ros2_ws/src/capytown/capytown/lidar_viz.py
```

## Copiar la tabla Q

```bash
docker cp /home/pi/NuevoProyecto/q_table_granprix_FINAL.json \
"$CONTAINER_ID":/ros2_ws/q_table_granprix.json
```

Dentro del contenedor, la tabla queda disponible en:

```text
/ros2_ws/q_table_granprix.json
```

Este archivo almacena los valores utilizados por el algoritmo de Q-learning para seleccionar acciones y conservar decisiones aprendidas.

---

# 5. Creación del nodo publicador de cámara

Ingresar nuevamente al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Crear el archivo `camera_publisher.py`:

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

        self.timer = self.create_timer(
            1.0 / max(self.fps, 1.0),
            self.publicar
        )

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

Este nodo abre la cámara física del robot y publica las imágenes en el tópico:

```text
/camera/image_raw
```

El nodo `pare_detector` recibe estas imágenes para identificar las señales de PARE y META.

---

# 6. Configuración del archivo setup.py

Crear o reemplazar el archivo `setup.py`:

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
    description='CapyTown Grand Prix G1',
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
            'dashboard_g1 = capytown.dashboard_g1:main',
        ],
    },
)
PY
```

El archivo `setup.py` registra los nodos como ejecutables de ROS 2.

Gracias a este archivo, los nodos pueden iniciarse mediante comandos como:

```bash
ros2 run capytown maze_solver
```

---

# 7. Verificación de las funciones main()

Los archivos `lidar_viz.py` y `robot_dashboard.py` deben contener una función `main()` para ser ejecutados mediante `ros2 run`.

Ejecutar dentro del contenedor:

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

Este script realiza lo siguiente:

- Revisa si cada archivo ya contiene una función `main()`.
- Si ya existe, no modifica el archivo.
- Si no existe, agrega automáticamente una función principal.
- Permite que ROS 2 pueda ejecutar los nodos mediante `setup.py`.

---

# 8. Compilación del proyecto

Dentro del contenedor, dirigirse al workspace:

```bash
cd /ros2_ws
```

Eliminar resultados de compilaciones anteriores:

```bash
rm -rf build install log
```

Compilar el paquete:

```bash
colcon build --packages-select capytown --symlink-install
```

Cuando la compilación termine correctamente, cargar el workspace:

```bash
source /ros2_ws/install/setup.bash
```

Verificar los ejecutables registrados:

```bash
ros2 pkg executables capytown
```

Deberían aparecer ejecutables similares a los siguientes:

```text
capytown camera_publisher
capytown lidar_viz
capytown maze_solver
capytown pare_detector
capytown robot_dashboard
capytown ver_pare_debug
```

Salir del contenedor:

```bash
exit
```

---

# 9. Preparación de las terminales de ejecución

Para ejecutar el sistema completo se utilizan seis terminales.

Cada nodo debe permanecer abierto en su propia terminal.

El orden recomendado es:

1. Publicador de cámara.
2. Detector de PARE y META.
3. Visualización de depuración de cámara.
4. Visualización del LiDAR.
5. Dashboard del robot.
6. Navegación autónoma.

En cada terminal nueva se debe volver a definir el identificador del contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

También se puede consultar nuevamente mediante:

```bash
docker ps --format "table {{.ID}}\t{{.Names}}\t{{.Image}}\t{{.Status}}"
```

---

# 10. Terminal 1 — Publicador de cámara

Abrir la primera terminal.

Definir el contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Ingresar al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Preparar el entorno de ROS 2:

```bash
cd /ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
```

Iniciar la cámara:

```bash
ros2 run capytown camera_publisher --ros-args \
-p device:=0
```

Esta terminal debe permanecer abierta.

El nodo publica las imágenes en:

```text
/camera/image_raw
```

---

# 11. Terminal 2 — Detector de PARE y META

Abrir la segunda terminal.

Definir el contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Ingresar al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Preparar el entorno:

```bash
cd /ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
```

Se debe elegir solamente uno de los siguientes perfiles.

No se deben ejecutar los tres perfiles al mismo tiempo.

## Perfil 1 — Detección estricta

```bash
ros2 run capytown pare_detector --ros-args \
-p image_topic:=/camera/image_raw \
-p meta_area_min:=12000 \
-p meta_frames_confirm:=8 \
-p meta_s_min:=80 \
-p meta_v_min:=50
```

Este perfil requiere:

- Una región visual más grande.
- Mayor saturación.
- Mayor nivel de brillo.
- Más fotogramas consecutivos de confirmación.

Reduce los falsos positivos, pero puede requerir que la señal se encuentre más cerca de la cámara.

## Perfil 2 — Detección menos estricta

```bash
ros2 run capytown pare_detector --ros-args \
-p image_topic:=/camera/image_raw \
-p meta_area_min:=8000 \
-p meta_frames_confirm:=5 \
-p meta_s_min:=60 \
-p meta_v_min:=40
```

Este perfil permite detectar la META desde una mayor distancia.

Es más sensible, pero también puede aumentar la posibilidad de falsas detecciones.

## Perfil 3 — Detección más estricta para PARE y META

```bash
ros2 run capytown pare_detector --ros-args \
-p image_topic:=/camera/image_raw \
-p pare_area_max:=14000 \
-p pare_densidad_min:=0.45 \
-p pare_frames_confirm:=5 \
-p meta_area_min:=12000 \
-p meta_frames_confirm:=8 \
-p meta_s_min:=80 \
-p meta_v_min:=50
```

Este perfil aplica restricciones adicionales para las señales PARE y META.

Es recomendable cuando existen objetos rojos o verdes alrededor del circuito que puedan causar falsas detecciones.

La terminal debe permanecer abierta después de ejecutar el perfil seleccionado.

---

# 12. Terminal 3 — Depuración visual de PARE y META

Abrir la tercera terminal.

Definir el contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Ingresar al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Preparar el entorno:

```bash
cd /ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
export DISPLAY=:0
```

Ejecutar el visualizador:

```bash
ros2 run capytown ver_pare_debug
```

Esta ventana permite observar:

- La imagen recibida desde la cámara.
- Las máscaras de color.
- Las regiones detectadas.
- Los contornos encontrados.
- La detección de la señal PARE.
- La detección de la META.
- El comportamiento de los umbrales visuales.

Esta terminal debe permanecer abierta.

---

# 13. Terminal 4 — Visualización del LiDAR

Abrir la cuarta terminal.

Definir el contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Ingresar al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Preparar el entorno:

```bash
cd /ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
export DISPLAY=:0
```

Ejecutar el visualizador LiDAR:

```bash
ros2 run capytown lidar_viz
```

La visualización permite observar:

- Los puntos detectados alrededor del robot.
- Las paredes del laberinto.
- La distancia frontal.
- Las distancias laterales.
- Los espacios abiertos.
- Los posibles obstáculos.
- La distribución de las mediciones del LiDAR.

Esta terminal debe permanecer abierta.

---

# 14. Terminal 5 — Dashboard del robot

Abrir la quinta terminal.

Definir el contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Ingresar al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Preparar el entorno:

```bash
cd /ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
export DISPLAY=:0
```

Ejecutar el dashboard principal:

```bash
ros2 run capytown robot_dashboard
```

El dashboard permite visualizar información como:

- Estado actual de la navegación.
- Estado de la máquina de estados.
- Distancias obtenidas por el LiDAR.
- Acción seleccionada.
- Orientación del robot.
- Información de odometría.
- Señales visuales detectadas.
- Intersecciones encontradas.
- Decisiones tomadas.
- Métricas acumuladas durante el recorrido.

Esta terminal debe permanecer abierta.

## Dashboard alternativo

El archivo `dashboard_g1.py` también se encuentra registrado como ejecutable.

Puede iniciarse con:

```bash
ros2 run capytown dashboard_g1
```

No es obligatorio ejecutar ambos dashboards al mismo tiempo.

Se debe utilizar el dashboard que corresponda a la versión final del proyecto.

---

# 15. Verificación de los sensores antes de mover el robot

Antes de ejecutar `maze_solver`, se recomienda comprobar que los sensores están publicando correctamente.

Se puede utilizar una terminal adicional temporal o alguna de las terminales antes de iniciar su nodo definitivo.

Ingresar al contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
docker exec -it "$CONTAINER_ID" bash
```

Preparar ROS 2:

```bash
cd /ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
```

Mostrar los tópicos disponibles:

```bash
ros2 topic list
```

Como mínimo, deberían encontrarse tópicos equivalentes a:

```text
/camera/image_raw
/scan
/odom_raw
/cmd_vel
```

## Verificar la cámara

```bash
ros2 topic hz /camera/image_raw
```

## Verificar el LiDAR

```bash
ros2 topic hz /scan
```

## Verificar la odometría

```bash
ros2 topic hz /odom_raw
```

## Verificar el tópico de movimiento

```bash
ros2 topic info /cmd_vel
```

Los comandos `ros2 topic hz` continúan ejecutándose hasta que se presiona:

```text
Ctrl + C
```

Si alguno de los sensores no publica información, no se debe iniciar la navegación.

---

# 16. Terminal 6 — Navegación autónoma

El nodo de navegación debe iniciarse después de confirmar que:

- La cámara está publicando imágenes.
- El detector de señales está activo.
- El LiDAR está publicando mediciones.
- La odometría está disponible.
- Las ventanas de visualización funcionan.
- El robot está correctamente colocado en la salida.
- No existen personas u objetos peligrosos frente al robot.

Abrir la sexta terminal.

Definir el contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Ingresar al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Preparar el entorno:

```bash
cd /ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
```

Ejecutar el nodo principal:

```bash
ros2 run capytown maze_solver --ros-args \
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

Al ejecutar este comando, el robot comienza la navegación autónoma.

Esta terminal debe permanecer abierta durante toda la corrida.

---

# 17. Explicación de los parámetros principales

## Parámetros de Q-learning

### `qlearn_enabled`

Activa o desactiva el uso del algoritmo de Q-learning.

```text
qlearn_enabled = true
```

Indica que el robot utilizará la tabla Q para apoyar la selección de caminos.

### `q_epsilon`

Controla la probabilidad de realizar una acción exploratoria.

```text
q_epsilon = 0.0
```

Con este valor, el robot evita decisiones aleatorias y utiliza principalmente las acciones conocidas en la tabla Q.

### `q_alpha`

Controla cuánto influyen las experiencias nuevas en los valores aprendidos.

```text
q_alpha = 0.30
```

Un valor de `0.30` permite actualizar el aprendizaje sin reemplazar completamente la información anterior.

---

## Parámetros de movimiento

### `v_forward`

Velocidad lineal utilizada durante el avance normal.

```text
v_forward = 0.050
```

### `turn_speed`

Velocidad angular principal utilizada durante los giros.

```text
turn_speed = 0.16
```

### `front_stop`

Distancia frontal a partir de la cual se considera que existe un obstáculo cercano.

```text
front_stop = 0.42
```

### `turn_slowdown_deg`

Cantidad de grados restantes a partir de la cual se reduce la velocidad de giro.

```text
turn_slowdown_deg = 25.0
```

### `turn_near_speed`

Velocidad angular utilizada cuando el robot se encuentra cerca de completar el giro.

```text
turn_near_speed = 0.10
```

### `turn_brake_margin_deg`

Margen utilizado para evitar que el robot sobrepase el ángulo objetivo.

```text
turn_brake_margin_deg = 0.5
```

---

## Parámetros de intersecciones

### `side_open`

Distancia lateral necesaria para considerar que existe un camino abierto.

```text
side_open = 1.10
```

### `side_confirm_ticks`

Cantidad de lecturas consecutivas necesarias para confirmar una apertura lateral.

```text
side_confirm_ticks = 7
```

Esto evita registrar como intersección una única lectura incorrecta del LiDAR.

### `intersection_min_exit_m`

Distancia mínima necesaria para considerar que el robot ya salió de una intersección.

```text
intersection_min_exit_m = 0.48
```

### `intersection_center_distance_m`

Distancia que el robot avanza para aproximarse al centro de la intersección.

```text
intersection_center_distance_m = 0.11
```

### `intersection_center_timeout_s`

Tiempo máximo permitido para realizar el centrado.

```text
intersection_center_timeout_s = 3.50
```

### `intersection_center_v`

Velocidad empleada durante el avance hacia el centro de la intersección.

```text
intersection_center_v = 0.040
```

### `intersection_cooldown_s`

Tiempo durante el cual se evita detectar inmediatamente la misma intersección.

```text
intersection_cooldown_s = 6.5
```

### `memory_node_radius`

Radio empleado para decidir si una posición pertenece a una intersección ya registrada.

```text
memory_node_radius = 0.45
```

---

## Parámetros de giro

### `giro_yaw_min_deg`

Cambio mínimo de orientación requerido para completar un giro cercano a 90 grados.

```text
giro_yaw_min_deg = 84.0
```

### `media_yaw_min_deg`

Cambio mínimo de orientación requerido para completar una media vuelta.

```text
media_yaw_min_deg = 165.0
```

### `giro_timeout_s`

Tiempo máximo permitido para completar un giro normal.

```text
giro_timeout_s = 16.0
```

### `media_timeout_s`

Tiempo máximo permitido para completar una media vuelta.

```text
media_timeout_s = 28.0
```

### `giro_front_safety`

Distancia frontal mínima de seguridad durante un giro.

```text
giro_front_safety = 0.22
```

---

## Parámetros de detección de META

### `meta_confirm_s`

Tiempo durante el cual la detección de META debe mantenerse antes de aceptarla.

```text
meta_confirm_s = 0.80
```

### `meta_min_distance_m`

Distancia mínima que debe recorrer el robot antes de permitir la finalización por META.

```text
meta_min_distance_m = 2.50
```

### `meta_min_runtime_s`

Tiempo mínimo desde el inicio de la corrida antes de aceptar la META.

```text
meta_min_runtime_s = 25.0
```

Estas restricciones ayudan a evitar que una detección falsa al inicio de la carrera detenga inmediatamente al robot.

---

# 18. Reinicio opcional de la tabla Q

Para comenzar una corrida sin utilizar el aprendizaje anterior, se puede eliminar la tabla Q.

Primero, ingresar al contenedor:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
docker exec -it "$CONTAINER_ID" bash
```

Eliminar la tabla:

```bash
rm -f /ros2_ws/q_table_granprix.json
```

Este comando es opcional.

Debe utilizarse únicamente cuando se desea reiniciar completamente el aprendizaje.

Al eliminar el archivo:

- Se pierden las decisiones aprendidas anteriormente.
- Se eliminan las recompensas y penalizaciones acumuladas.
- El robot inicia con una tabla vacía o con valores predeterminados.
- Las corridas anteriores dejan de influir en las decisiones.

Para volver a cargar la tabla Q original, salir del contenedor:

```bash
exit
```

Después, desde la Raspberry Pi:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Copiar nuevamente el archivo:

```bash
docker cp /home/pi/NuevoProyecto/q_table_granprix_FINAL.json \
"$CONTAINER_ID":/ros2_ws/q_table_granprix.json
```

---

# 19. Orden resumido de ejecución

| Terminal   | Nodo               | Función                                        |
| ---------- | ------------------ | ---------------------------------------------- |
| Terminal 1 | `camera_publisher` | Captura y publica las imágenes de la cámara.   |
| Terminal 2 | `pare_detector`    | Detecta las señales PARE y META.               |
| Terminal 3 | `ver_pare_debug`   | Muestra la depuración visual de la detección.  |
| Terminal 4 | `lidar_viz`        | Muestra gráficamente las mediciones del LiDAR. |
| Terminal 5 | `robot_dashboard`  | Presenta los estados y métricas del robot.     |
| Terminal 6 | `maze_solver`      | Ejecuta la navegación autónoma.                |

El nodo `maze_solver` debe ejecutarse al final.

Primero se debe comprobar que los sensores y los nodos auxiliares funcionan correctamente.

---

# 20. Resumen de comandos iniciales de cada terminal

Cada vez que se abra una terminal nueva, primero se debe ejecutar:

```bash
export CONTAINER_ID=ID_DEL_CONTENEDOR
```

Después, ingresar al contenedor:

```bash
docker exec -it "$CONTAINER_ID" bash
```

Dentro del contenedor, ejecutar:

```bash
cd /ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
```

Para los nodos con interfaz gráfica, agregar:

```bash
export DISPLAY=:0
```

Los nodos que requieren `DISPLAY=:0` son:

```text
ver_pare_debug
lidar_viz
robot_dashboard
```

---

# 21. Detención del sistema

Para detener un nodo, utilizar:

```text
Ctrl + C
```

El orden recomendado para detener el sistema es:

1. Detener `maze_solver`.
2. Detener `pare_detector`.
3. Detener `camera_publisher`.
4. Cerrar `ver_pare_debug`.
5. Cerrar `lidar_viz`.
6. Cerrar `robot_dashboard`.

El nodo `maze_solver` debe detenerse primero para evitar que el robot continúe enviando instrucciones de movimiento mientras los sensores o nodos auxiliares se están cerrando.

Como medida adicional de seguridad, dentro de un contenedor con ROS 2 configurado se puede ejecutar:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Este comando publica una velocidad lineal y angular igual a cero.
