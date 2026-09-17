UAV Hexacopter Control — MATLAB/Simulink y ROS 2

Modelo dinámico, control y visualización de un hexacóptero tipo X desarrollado para un sistema UAV destinado a misiones de escaneo y reconstrucción 3D mediante fotogrametría.

El proyecto integra:

Modelo dinámico no lineal de seis grados de libertad.

Control MIMO de posición y actitud mediante MATLAB/Simulink.

Asignación de las acciones de control a seis rotores mediante un mixer.

Comunicación entre Simulink y ROS 2.

Visualización del UAV, su orientación y la trayectoria recorrida en RViz2.

Conversión de misiones de Mission Planner (.waypoints) a referencias locales para Simulink.

Requisitos

MATLAB/Simulink

MATLAB R2026a o compatible.

Simulink.

ROS Toolbox con soporte para ROS 2.

ROS 2 y visualización

Ubuntu 24.04 en WSL2.

ROS 2 Jazzy.

Python 3.

RViz2.

La integración se probó con MATLAB ejecutándose en Windows y ROS 2 ejecutándose en WSL2.

Estructura del repositorio

.
├── control_UAV.slx
├── control_UAV_ROS.slx
├── Inicializacion.m
├── parametrosHexacoptero.m
├── calcularGananciasControl.m
├── calcularParametrosAerodinamicos.m
├── secuencia_prueba_UAV.m
├── convertirWaypointsMissionPlanner.m
├── meshes/
│   └── hexacoptero.stl
├── scripts/
│   └── visualizar_hexacoptero.py
├── config/
│   └── hexacopter_pose.rviz
└── resultados/

Los archivos temporales generados por Simulink, como slprj/ y los archivos *.slxc, no forman parte del control de versiones.

Ejecución del modelo de control

Abrir MATLAB y establecer como carpeta de trabajo la raíz del repositorio.

Ejecutar los scripts de inicialización:

Inicializacion

Abrir control_UAV.slx.

Seleccionar las referencias de prueba o la referencia de misión mediante los switches disponibles.

Ejecutar la simulación.

El modelo utiliza las variables físicas, los parámetros aerodinámicos y las ganancias de control definidos en los scripts del repositorio. Las señales de posición se expresan en metros y los ángulos de actitud en radianes.

Conversión de una misión de Mission Planner

El archivo convertirWaypointsMissionPlanner.m convierte un archivo QGC WPL 110 en una matriz local para el bloque From Workspace:

[referencia, mision] = convertirWaypointsMissionPlanner( ...
    "Tarea2Mejorada(1).waypoints", 5, 45);

Los argumentos corresponden a:

5   rapidez traslacional [m/s]
45  rapidez de yaw [deg/s]

La matriz referencia tiene el formato:

[tiempo_s, X_Este_m, Y_Norte_m, Z_Arriba_m, yaw_rad]

Para utilizarla en Simulink, escribir referencia en el bloque From Workspace y activar Interpolate data. Las cuatro salidas de datos son [X,Y,Z,yaw]; la primera columna se utiliza internamente como tiempo.

El comando RTL se representa como un retorno horizontal al origen seguido de un descenso. Esta es una aproximación para simulación; el comportamiento real depende de parámetros de ArduPilot como RTL_ALT y RTL_ALT_FINAL.

Integración con ROS 2

La versión ROS publica, entre otros, los siguientes tópicos:

/hexacopter/position   std_msgs/Float64
/hexacopter/rpy        geometry_msgs/Vector3
/hexacopter/pose       geometry_msgs/PoseStamped

La pose utiliza el frame world. El script de visualización publica:

/hexacopter/mesh       visualization_msgs/Marker
/hexacopter/trail      visualization_msgs/Marker

También publica el TF:

world → hexacopter

Visualización en RViz2

Desde WSL2, cargar ROS 2 y ejecutar el visualizador:

abriendo una distrobución en ubuntu-24.04 con:

wsl -d Ubuntu-24.04


source /opt/ros/jazzy/setup.bash
cd "/mnt/c/Users/<usuario>/Documents/MATLAB/TT/UAV-Hexacopter-Control-Matlab-Simulink-main"
python3 scripts/visualizar_hexacoptero.py

En otra terminal iniciar RViz2:

source /opt/ros/jazzy/setup.bash
rviz2 -d config/hexacopter_pose.rviz

Configuración principal de RViz2:

Fixed Frame: world
Marker de la malla: /hexacopter/mesh
Marker de la trayectoria: /hexacopter/trail
Target Frame: hexacopter

La estela se construye con las posiciones reales recibidas en /hexacopter/pose, por lo que representa el recorrido publicado durante la simulación. Se filtran desplazamientos menores de 0.10 m para evitar una densidad excesiva de puntos.

Convención de coordenadas

X: Este
Y: Norte
Z: arriba
yaw: radianes

La malla STL se corrige en el visualizador para considerar la transformación de ejes y el centro de masa CAD. Esta corrección afecta la representación gráfica, no la pose publicada por Simulink.

Limitaciones actuales

El modelo y la visualización corresponden a una simulación; no sustituyen una validación experimental en vuelo.

La cámara de RViz2 puede presentar saltos si Simulink se ejecuta más rápido que el tiempo real. Se recomienda usar Simulation → Pacing Options.

Los archivos .slx son binarios; no se recomienda modificarlos simultáneamente en dos computadoras.

La conversión de RTL es aproximada y no reproduce todos los parámetros internos de ArduPilot.

Flujo de trabajo con Git

Antes de trabajar:

git pull

Después de guardar los cambios:

git status
git add .
git commit -m "Describe los cambios realizados"
git push

Se recomienda cerrar MATLAB/Simulink antes de confirmar los archivos del modelo.

Autores

Altamirano Mendoza Emilio Antoine
Hernández Carreño Guillermo
Roa Trejo Angel David

Ingeniería Mecatrónica — UPIITA, IPN