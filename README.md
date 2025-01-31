# PathPlanningRisLab
A continuación se encuentran las instrucciones para poder visualizar el funcionamiento de la implementacion de Path Planning con el Turtlebot3, simulado en Gazebo Classic.

### Requisitos:
1. ROS2 Humble
2. Gazebo Classic
3. Turtlebot3 by ROBOTIS-GIT
4. turtlebot3_simulations by ROBOTIS-GIT
5. slam_toolbox
6. rviz2
7. ROS2-PurePursuitControl-PathPlanning-Tracking by abdulkadrtr

## Instalación ROS2-Humble:

LINK: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

#### Set locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
#### Setup sources

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
#### Ros2 packages

```bash
sudo apt update
sudo apt upgrade

sudo apt install ros-humble-slam-toolbox 

sudo apt install ros-humble-desktop

sudo apt install ros-dev-tools
```

### Herramientas necesarias y Workspace

LINK del tutorial: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

1. Instalar herramientas necesarias:

    ```bash
    sudo apt-get update
    sudo apt-get install -y python3-pip ros-dev-tools stm32flash
    ```

2. Crear el Workspace y clonar los paquetes necesarios:

    ```bash
    mkdir -p ~/turtlebot3/src && cd ~/turtlebot3/src
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone https://github.com/Felipe-SSC/PathPlanningRisLab.git
    cd ~/turtlebot3_ws && colcon build --symlink-install
    ```
## Simulación:
### Modelo del Robot:
Para comenzar a simular, es necesario realizar algunas configuraciones relacionadas al modelo del robot:
  ```bash
      export TURTLEBOT3_MODEL=burger >> ~/.bashrc
      export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models >> ~/.bashrc
      source ~/.bashrc
  ```
### Mundo en Gazebo:
Para la simulación en un entorno básico, destinado a evaluar el comportamiento del script y mapeo en condiciones óptimas se debe realizar lo siguiente:
```bash
  source install/setup.bash
  ros2 launch turtlebot3_gazebo test1_world.launch.py
  ```
Para la simulacion en un "entorno agrícola", destinado a evaluar el comportamiento del script y mapeo frente a obstaculos naturales se debe realizar lo siguiente:
```bash
  source install/setup.bash
  ros2 launch turtlebot3_gazebo trees_test1.launch.py
  ```
### Mapeo (SLAM)
Para realizar un mapeo del entorno haciendo uso del LiDAR se debe realizar lo siguiente:
```bash
  ros2 launch slam_toolbox online_async_launch.py
  ```
### Movimiento del robot:
Para mover el robot en el mundo simulado se puede realizar de la siguiente manera:
#### Paquete teleop_twist_keyboard:
```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
### Path Planning y navegación autónoma:
Para producir un path que seguirá el robot, es necesario lo siguiente:
1. Tener el area de navegación previamente mapeada (Si el destino esta en un sector 'desconocido' o 'ocupado' el robot planeará una trayectoria hasta el ultimo punto conocido)
2. rviz2 abierto con el fixed frame 'map'
3. Ejecutar el script de Path Planning:
```bash
  ros2 run nav_controller euclidean
  ```
o
```bash
  ros2 run nav_controller manhattan
  ```
4. Otorgar Goal Pose y 2D Estimated Pose
   Se puede realizar a través de Rviz2 o ejecutando el siguiente archivo:
   (Entrega la pose estimada del inicio y el goal pose al otro lado del mapa)
   ```bash
    ros2 run nav_controller meta
    ```
5. Visualizar resultados:
   El script permite visualizar en rviz2 el path a seguir, y además escribe en un .csv que contiene datos relevantes respecto al rendimiento del robot en su simulacion.

##### Respecto al paquete nav_controller:
El paquete presente en el repositorio cuenta con cambios específicos realizados al paquete original producido por abdulkadrtr (https://github.com/abdulkadrtr/ROS2-PurePursuitControl-PathPlanning-Tracking).
Los cambios corresponden a modificaciones del script para recopilar información, asi como para implementar dos métodos de calculo de distancia al momento de generar una trayectoria.

   
