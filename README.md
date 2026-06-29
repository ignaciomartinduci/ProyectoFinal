# Diseño y simulación de un robot manipulador serial de 6 GDL para automatización logística

## Descripción

Desarrollo de un robot manipulador serial de seis grados de libertad orientado a aplicaciones de automatización logística. El proyecto abarca el modelado cinemático, resolución de cinemática inversa y generación de trayectorias, implementados primero en MATLAB y luego integrados en un sistema de control distribuido basado en ROS 2.

## Estructura del repositorio

```
├── matlab/          # Algoritmos de cinemática, trayectorias y visualización
│   ├── main.m       # Punto de entrada
│   ├── flags.m      # Selector de funcionalidades
│   ├── config.m     # Parámetros globales
│   ├── modules/     # Cinemática inversa, generación de trayectorias, etc.
│   ├── models/      # Modelos STL y CAD del robot
│   ├── tests/       # Tests de cinemática y workspace
│   ├── visualization/
│   └── third_party/ # Robotics Toolbox + Spatial Math Toolbox
├── ros2/            # Sistema de control distribuido
│   ├── Dockerfile
│   ├── docker-compose.yml
│   └── src/         # Paquetes ROS 2 (cinemática, trayectorias, GUI, etc.)
└── docs/
    └── informe/     # Informe del proyecto (LaTeX + PDF)
```

## Requisitos

### MATLAB
- MATLAB R2020b o superior
- Robotics Toolbox for MATLAB (Peter Corke) — incluido en `matlab/third_party/`
- Spatial Math Toolbox — incluido en `matlab/third_party/`

### ROS 2
- Docker y Docker Compose

## Uso

### MATLAB

1. Abrir MATLAB y establecer el directorio de trabajo en `matlab/`:

```matlab
cd matlab
```

2. Configurar las funcionalidades a ejecutar en `matlab/flags.m`:

| Flag | Descripción |
|---|---|
| `TEST_INV_KINEMATICS` | Prueba la cinemática inversa con un ejemplo |
| `TEST_LOOP_IK` | Test en loop sobre posiciones aleatorias (requiere la anterior) |
| `PRINT_SOLUTIONS` | Imprime soluciones geométricas y periódicas por consola |
| `TEST_WORKSPACE` | Visualiza el espacio de trabajo del robot |
| `SINGULARITIES` | Busca y verifica configuraciones singulares |
| `TEST_GEN_TRAJ` | Prueba la generación de trayectorias |
| `TAREA` | Ejecuta la tarea principal |
| `IK_COMPARISON` | Visualiza la periodicidad articular de las soluciones IK |
| `CAD_DESIGN` | Modelo 3D: `1` = CAD original, `2` = CAD alternativo |

3. Ejecutar:

```matlab
main
```

#### Ejemplo: correr la tarea principal desde cero

Para reproducir la demo principal (toma de pieza en la caja origen, traslado y depósito en la caja destino) partiendo de una instalación limpia:

1. En `matlab/flags.m`, activar únicamente la tarea y dejar el resto de los flags en `0`:

```matlab
TAREA = 1;
```

2. Elegir el modelo 3D a animar:

```matlab
CAD_DESIGN = 1; % 1 = models/CAD | 2 = models/CAD2
```

3. Con el directorio de trabajo en `matlab/`, ejecutar:

```matlab
main
```

Esto genera la trayectoria completa (`gen_traj`), la anima sobre el modelo 3D elegido (`myAnimate`) y grafica posiciones/velocidades/aceleraciones articulares y cartesianas junto con sus límites (`grafQ`, `grafQaE`).

### ROS 2 (Docker)

Desde la carpeta `ros2/`:

```bash
cd ros2

# Primera vez: construir la imagen
docker compose build

# Levantar el sistema
docker compose up
```

El contenedor monta `ros2/src/` como volumen, por lo que los cambios en el código fuente se reflejan sin necesidad de reconstruir la imagen. `entrypoint.sh` deja la sesión posicionada en `/root/ros2_PFE` (carpeta raíz del proyecto dentro del contenedor) con el entorno de ROS 2 sourceado.

#### Ejecutar la demo principal

Para ver el robot funcionando junto con su visualización hacen falta **dos terminales abiertas sobre la carpeta raíz del proyecto dentro del contenedor** (`/root/ros2_PFE`):

**Terminal 1** — levanta el sistema completo de nodos (cinemática, generación de trayectorias, ejecución, estado, almacenamiento, teaching y GUI):

```bash
cd ros2
docker compose up
```

Dentro de la sesión que se abre:

```bash
# Normal
ros2 launch dr_bringup dr.launch.py

# Debug
ros2 launch dr_bringup dr.launch.py debug:=true

# Debug verbose (alta frecuencia)
ros2 launch dr_bringup dr.launch.py debug:=true debug_verbose:=true
```

**Terminal 2** — abre una segunda sesión sobre el mismo contenedor para la visualización en RViz:

```bash
cd ros2
docker compose exec ros2_pfe bash
```

Dentro de esa segunda sesión (re-sourceando el workspace, ya que cada `exec` abre una shell nueva):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rviz2
```

> El repositorio no incluye una configuración `.rviz` guardada: en RViz hay que agregar manualmente los displays `RobotModel` y `TF`, y fijar `Fixed Frame` en `base_link`.

## Clonado

```bash
git clone https://github.com/ignaciomartinduci/ProyectoFinal.git
cd ProyectoFinal
```
