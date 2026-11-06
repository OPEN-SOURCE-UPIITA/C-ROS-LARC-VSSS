# Simulador LARC VSSS 2026 - OPEN SOURCE UPIITA

Entorno de simulación para la categoría Very Small Size Soccer (VSSS) adaptado al reglamento LARC 2026. Construido con **ROS 2 (Jazzy)** y **Gazebo Harmonic**.

## Estructura del Paquete
* `launch/sim_launch.py`: Inyector de robots con formaciones tácticas y puente ROS-Gazebo.
* `models/`: Modelos SDF de la cancha, pelota oficial y plantilla de robot diferencial.
* `worlds/cancha_vsss.world`: Entorno base con cámara cenital rotada en formato horizontal (900x700).
* `scripts/control_teclado.py`: Nodo de teleoperación manual.
* `models/robot/materials/textures/generar_arucos.py`: Generador de uniformes (ArUcos) reglamentarios.

## 1. Limpieza y Compilación
Si Gazebo muestra texturas incorrectas o haces cambios en los archivos `.sdf`, limpia la caché y compila el espacio de trabajo con estos comandos desde la raíz (`vsss_ws`):

```bash
killall -9 ruby gz gz-sim
rm -rf build/ install/ log/ ~/.gz/sim/cache/ ~/.ignition/
colcon build --symlink-install --packages-select vsss_sim
source install/setup.bash
```

## 2. Ejecutar la Simulación (Modos Inteligentes)
La simulación asigna los roles dinámicamente. El tópico `/robotX` siempre será tu equipo (aliado, del lado izquierdo) y `/rivalX` será el enemigo (lado derecho), sin importar el color que elijas.

**Argumentos:**
* `mi_equipo`: Color de tu equipo (`azul` o `amarillo`).
* `modo`: Situación táctica (`normal`, `inicio`, `penalty`, `duelo`).

**Ejemplos:**
```bash
# 1. Partido Completo: Eres el Azul y haces el saque inicial.
ros2 launch vsss_sim sim_launch.py mi_equipo:=azul modo:=inicio

# 2. Partido Completo: Te tocó ser Amarillo. (Tus controles siguen siendo /robotX, pero verás ArUcos amarillos en la izquierda)
ros2 launch vsss_sim sim_launch.py mi_equipo:=amarillo modo:=normal

# 3. Entrenamiento Duelo 1v1 (Tu tirador frente al portero rival)
ros2 launch vsss_sim sim_launch.py mi_equipo:=azul modo:=duelo
```

## 3. Control Manual (Teleoperación)
Controla cualquier robot instanciado publicando velocidades en su tópico.

- **Aliados (Azules):** `/robot1/cmd_vel`, `/robot2/cmd_vel`, `/robot3/cmd_vel`
- **Rivales (Amarillos):** `/rival1/cmd_vel`, `/rival2/cmd_vel`, `/rival3/cmd_vel`

## 4. Regenerar ArUcos (Texturas)
Si requieres ajustar el diccionario de OpenCV o cambiar tonos de color, modifica y ejecuta:

```bash
cd src/vsss_sim/models/robot/materials/textures/
python3 generar_arucos.py
```

*(Nota: Requiere recompilar el paquete para reflejar los cambios en Gazebo; si el simulador mantiene recursos en caché, cierra Gazebo y limpia la caché antes de volver a lanzarlo.)*
