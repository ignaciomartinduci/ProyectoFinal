# Robot DR - ROS2 Jazzy

Sistema de control de robot manipulador desarrollado con ROS2 Jazzy.  
Universidad Nacional de Cuyo - Facultad de Ingeniería - Ignacio Martín Duci - 2026

## Requisitos

- [Docker](https://docs.docker.com/get-docker/) instalado

## Correr en Linux

**1. Instalar Docker**
```bash
curl -fsSL https://get.docker.com | sh
sudo groupadd docker
sudo usermod -aG docker $USER
# Cerrar sesión y volver a entrar
```

**2. Construir la imagen** (primera vez, ~5 minutos)
```bash
docker compose build
```

**3. Permitir GUI y lanzar**
```bash
xhost +local:docker
docker compose run --rm ros2_pfe ros2 launch dr_bringup dr.launch.py
```

---

## Correr en Windows (WSL2)

### Paso 1 — Instalar VcXsrv (servidor gráfico para Windows)

1. Descargar e instalar **VcXsrv**: https://sourceforge.net/projects/vcxsrv/
2. Abrir **XLaunch** (se instala junto con VcXsrv)
3. Configurarlo así:
   - Display settings: **Multiple windows**, Display number: **0**
   - Client startup: **Start no client**
   - Extra settings: tildar **Disable access control**
4. Hacer clic en Finish — VcXsrv queda corriendo en la barra de tareas

### Paso 2 — Configurar WSL

Abrir la terminal de WSL y ejecutar:

```bash
# Obtener la IP de Windows desde WSL
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0

# Verificar que funciona (debe abrir una ventana)
xeyes
```

Si `xeyes` abre una ventana con ojos, la GUI está funcionando.

### Paso 3 — Instalar Docker en WSL

```bash
curl -fsSL https://get.docker.com | sh
sudo groupadd docker
sudo usermod -aG docker $USER
# Cerrar y reabrir la terminal WSL
sudo chown root:docker /var/run/docker.sock
```

### Paso 4 — Construir y lanzar

```bash
cd ~/ros2_PFE
docker compose build
docker compose run --rm ros2_pfe ros2 launch dr_bringup dr.launch.py
```

> **Nota:** cada vez que se reinicia Windows hay que volver a abrir XLaunch y repetir el `export DISPLAY=...` en la terminal WSL.

---

## Nodos del sistema

| Nodo | Descripción |
|---|---|
| `ik_node` | Cinemática inversa |
| `fk_node` | Cinemática directa |
| `gen_traj_node` | Generación de trayectorias |
| `state_node` | Estado del robot |
| `executor_node` | Ejecución de movimientos |
| `storage_node` | Guardado/carga de trayectorias |
| `teaching_node` | Modo enseñanza |
| `gui_node` | Interfaz gráfica |
| `robot_state_publisher` | Publicación del modelo URDF |
| `joint_state_bridge_node` | Bridge de estados de joints |

## Trayectorias guardadas

Las trayectorias se persisten en un volumen de Docker y no se pierden al detener el contenedor.  
Para ver dónde están almacenadas:
```bash
docker volume inspect ros2_pfe_trajectories
```
