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

### ROS 2 (Docker)

Desde la carpeta `ros2/`:

```bash
cd ros2

# Primera vez: construir la imagen
docker compose build

# Levantar el sistema
docker compose up
```

El contenedor monta `ros2/src/` como volumen, por lo que los cambios en el código fuente se reflejan sin necesidad de reconstruir la imagen.

## Clonado

```bash
git clone https://github.com/ignaciomartinduci/ProyectoFinal.git
cd ProyectoFinal
```
