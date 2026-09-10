# Robotnik Isaac MoveIt config

Configuración MoveIt 2 aislada para el UR10e del RB-KAIROS+ ejecutado en
Isaac Sim. Todos los recursos MoveIt están dentro de este paquete; no existe
ninguna dependencia en tiempo de ejecución de `rbkairos_moveit_config` ni de
`robotnik_moveit_configs`.

El launch presupone que Isaac, `ros2_control_node`,
`joint_state_broadcaster`, `robotnik_base_control`, `arm_controller` y el
`robot_state_publisher` del robot compuesto ya están arrancados. No crea un
segundo controller manager, bridge, controlador ni publicador de estado, y no
modifica los launches generales de Robotnik.

## Diseño

El Xacro local compone la base RB-KAIROS y el UR10e a partir de las macros de
`robotnik_description`. Usa nombres fijos `robot_*` y `robot_arm_*`; el
argumento `robot_id` solo establece el namespace ROS. El SRDF conserva el
grupo `arm`, con cadena de `robot_arm_base_link` a `robot_arm_tool0`, y fija la
raíz completa `robot_base_footprint` al frame externo `world`. El link
`robot_base_link` permanece unido a la raíz por el joint fijo del modelo.

MoveIt usa `moveit_simple_controller_manager`, porque el controlador se crea
y activa fuera de este paquete y ya expone una acción estándar
`FollowJointTrajectory`. Con `robot_id:=robot`, el nombre relativo
`arm_controller/follow_joint_trajectory` se resuelve como:

```text
/robot/arm_controller/follow_joint_trajectory
```

`moveit_manage_controllers` está desactivado: el ciclo de vida de los
controladores pertenece al bringup Isaac. `initial_positions.yaml` se conserva
como posición inicial de referencia, pero el launch no usa `FakeSystem`.

## Dependencias ROS 2 Jazzy

El paquete declara dependencias específicas para MoveIt (`move_group`, RViz,
KDL, OMPL, Pilz y el simple controller manager), la descripción/Xacro, la
acción y mensajes de trayectoria, los controladores esperados y
`robot_state_publisher`. No declara el metapaquete `moveit`, porque sus
componentes concretos ya están enumerados. En Jazzy el paquete Pilz se llama
`pilz_industrial_motion_planner`; no existe una clave rosdep llamada
`moveit_planners_pilz_industrial_motion_planner`.

La comprobación del workspace se realiza antes de instalar:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rosdep check --from-paths src/robotnik_isaac_sim --ignore-src -r -y
rosdep install --from-paths src/robotnik_isaac_sim --ignore-src -r -y
```

El `source install/setup.bash` hace visible el paquete local
`robotnik_description`, que vive fuera del subárbol pasado a `--from-paths` y
no es una clave apt/rosdep. En el entorno de desarrollo usado para crear el
paquete, la comprobación indicó que todas las dependencias estaban
satisfechas, por lo que no fue necesario instalar paquetes apt adicionales.

## Reconstrucción

Desde la raíz del workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/robotnik_isaac_sim \
  --packages-select robotnik_isaac_ros2_control \
                    robotnik_isaac_bringup \
                    robotnik_isaac_moveit_config
source install/setup.bash
```

## Ejecución

Con el bringup RB-KAIROS+ y sus tres controladores ya activos:

```bash
ros2 control list_controllers -c /robot/controller_manager
ros2 action info /robot/arm_controller/follow_joint_trajectory
ros2 launch robotnik_isaac_moveit_config isaac_moveit.launch.py robot_id:=robot
```

RViz puede omitirse para una prueba sin interfaz gráfica:

```bash
ros2 launch robotnik_isaac_moveit_config isaac_moveit.launch.py \
  robot_id:=robot launch_rviz:=false
```

Prueba directa del controlador, sin MoveIt:

```bash
ros2 action send_goal \
  /robot/arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [robot_arm_shoulder_pan_joint, robot_arm_shoulder_lift_joint, robot_arm_elbow_joint, robot_arm_wrist_1_joint, robot_arm_wrist_2_joint, robot_arm_wrist_3_joint], points: [{positions: [0.0, -1.2, 1.5, -1.57, -1.57, 0.0], time_from_start: {sec: 5, nanosec: 0}}]}}"
```

## Alcance

Solo se planifica y ejecuta el grupo articular `arm`. No se configura
planificación de la base, pinza, percepción, sensores ni hardware real. El
launch no arranca Isaac y requiere que `/robot/joint_states`, TF y la acción de
`arm_controller` procedan del bringup RB-KAIROS+.

El runtime debe publicar exactamente los seis nombres `robot_arm_*` usados por
este paquete. No se ofrece compatibilidad ni traducción para assets antiguos
que publiquen el brazo como `arm_*`; corregir esa composición pertenece al
bringup/descripción Isaac y queda deliberadamente fuera de este paquete.
