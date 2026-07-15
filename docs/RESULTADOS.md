# Resultados y evidencias — CapyTown Grand Prix

## Introducción

En este documento se presentan las principales evidencias visuales obtenidas durante las pruebas del sistema de navegación autónoma CapyTown Grand Prix.

Las capturas permiten comprobar el funcionamiento de los componentes principales del proyecto:

- Dashboard de monitoreo.
- Visualización de datos LiDAR.
- Detección visual de señales PARE.
- Detección visual de la META.
- Registro de métricas de las corridas.

---

## 1. Dashboard del robot

El dashboard permite supervisar el comportamiento del robot durante la navegación.

Muestra información relacionada con el estado del sistema, las mediciones de los sensores, la orientación del robot, las decisiones tomadas y las métricas acumuladas durante el recorrido.

<p align="center">
  <img
    src="../images/robot_dashboard.png"
    alt="Dashboard del robot"
    width="650"
  >
</p>

---

## 2. Visualización del LiDAR

El visualizador representa gráficamente las mediciones recibidas desde el LiDAR.

Permite observar la distribución de los puntos alrededor del robot, las paredes del laberinto, los obstáculos y los espacios abiertos que pueden corresponder a intersecciones.

<p align="center">
  <img
    src="../images/lidar_Viz.png"
    alt="Visualización de las mediciones LiDAR"
    width="580"
  >
</p>

---

## 3. Detección de la señal PARE

El detector procesa las imágenes publicadas por la cámara en el tópico `/camera/image_raw`.

La detección considera características como el color, el área de la región, la densidad y la cantidad de fotogramas consecutivos en los que aparece la señal.

<p align="center">
  <img
    src="../images/pare_detectado.png"
    alt="Señal PARE detectada"
    width="500"
  >
</p>

---

## 4. Detección de la META

El sistema utiliza la cámara para identificar visualmente la señal que representa la META.

Antes de detener definitivamente el robot, la navegación verifica que la detección se mantenga durante un tiempo mínimo y que el robot haya recorrido una distancia y un tiempo suficientes.

Estas restricciones ayudan a evitar una finalización incorrecta causada por falsos positivos.

<p align="center">
  <img
    src="../images/meta_detectado.png"
    alt="META detectada por el sistema"
    width="500"
  >
</p>

---

## 5. Métricas resultantes de las corridas

El sistema registra métricas para analizar el desempeño de la navegación autónoma.

Estas métricas permiten evaluar el tiempo del recorrido, las intersecciones encontradas, las decisiones realizadas, los caminos repetidos y el comportamiento general del robot.

<p align="center">
  <img
    src="../images/metricas_corridas.png"
    alt="Métricas obtenidas durante las corridas"
    width="650"
  >
</p>

---

## Conclusión de las pruebas

Las evidencias muestran la integración de los componentes principales del sistema:

- El LiDAR proporciona información sobre paredes, obstáculos y espacios abiertos.
- La odometría permite conocer el desplazamiento y controlar los giros.
- La cámara permite detectar señales de PARE y META.
- La máquina de estados coordina las acciones del robot.
- El algoritmo de Trémaux registra los caminos recorridos.
- Q-learning apoya la selección de decisiones mediante la tabla Q.
- El dashboard presenta el estado y las métricas del recorrido.

Para consultar las instrucciones de instalación y ejecución, regresar al:

[README principal](../README.md)
