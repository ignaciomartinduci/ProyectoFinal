# Diseño y simulación de un robot manipulador serial de seis grados de libertad para automatización logística.


## Descripción

Este proyecto presenta el desarrollo de un robot manipulador serial de seis grados de libertad orientado a aplicaciones de automatización logística.

Se aborda el modelado cinemático, la resolución de la cinemática inversa y la generación de trayectorias, inicialmente implementadas en MATLAB y posteriormente integradas en un entorno de control basado en ROS 2.


## Objetivos

- Definir especificaciones del robot y de su tarea.
- Realizar un diseño conceptual preliminar.
- Desarrollar el modelo cinemático con resolución de la cinemática inversa y generación de trayectorias.
- Diseñar/adaptar modelo CAD.
- Integración de soluciones en entorno ROS 2.


## Requisitos del sistema

Para la correcta ejecución del proyecto se requiere:

- Sistema operativo: Windows, Linux o macOS
- MATLAB (versión R2020 o superior recomendada)
- Robotics Toolbox for MATLAB (Peter Corke)
- ...


## Instalación

Clonar el repositorio:

```bash
git clone https://github.com/ignaciomartinduci/ProyectoFinal.git
cd ProyectoFinal
```


## Ejecución

La ejecución del proyecto se realiza desde MATLAB mediante el archivo principal `main.m`.

Previo a la ejecución, se debe configurar el archivo `flags.m`, donde se selecciona la funcionalidad a ejecutar (por ejemplo: modelado, cinemática inversa, generación de trayectorias, simulación, etc.).

En el siguiente diagrama se pueden ver las funcionalidades implementadas https://lucid.app/lucidchart/d1f889ce-072c-4336-91ca-7c0dc102ab88/edit?viewport_loc=98%2C-364%2C4254%2C2226%2C0_0&invitationId=inv_9000ccad-9b8f-46a1-832a-70b453ff8370, esto mismo puede verse en el siguiente diagrama: # Estructura lógica del proyecto

main.m
└── flags.m
    └── robot_1.m
        ├── test_workspace.m
        │   └── (Cinemática directa → espacio de trabajo)
        │
        ├── test_inv_kinematics.m
        │   └── inv_kinematics.m
        │       └── (Validación IK + loop de testing + visualización soluciones geométricas y periódicas)
        │        
        │
        ├── singularities.m
        │   └── (Determinante del Jacobiano → detección de singularidades)
        │
        ├── test_gen_traj.m
        │   └── gen_traj.m
        │       └── inv_kinematics.m
        │           └── (Interpolación + suavizado de trayectorias)
        │
        └── tarea.m
            └── gen_traj.m
                └── inv_kinematics.m
                    └── (Ejecución de trayectoria definida)


## Árbol de directorios

/matlab
│
├── main.m                # Punto de entrada del programa
├── flags.m               # Configuración global / parámetros
├── README.md
│
├── /models               # Definición del robot
│   └── robot_1.m
│
├── /old                  # Cinemática directa e inversa
│   └── ex_inv_kinemartics.m
│
├── /modules              # Análisis del sistema
│   ├── gran_traj.m
│   └── inv_kinematics.m
│   └── singularities.m
│   └── sym_1.m
│   └── tarea.m
│
├── /tests                # Scripts de prueba
│   ├── test_inv_kinematics.m
│   ├── test_gen_traj.m
│   └── test_workspace.m
│
├── /visualization        # Gráficos y animaciones
│   ├── grafQ.m
│   ├── grafQE.m
│   └── myAnimate.m
│
├── /media                 # Datos y resultados
|   └── _IMAGENES_DESARROLLO_IK
│
└── /third_party           # Dependencias
    ├── /common
    ├── /rtb
    └── /smtb

