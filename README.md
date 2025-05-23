# PathPlanningRisLab
A continuación se encuentran las instrucciones para poder visualizar el funcionamiento de la implementacion de Path Planning con el Turtlebot3, simulado en Gazebo Classic.

### Requisitos:
1. ROS2 Humble
2. Gazebo Classic
3. Turtlebot3 by ROBOTIS-GIT
4. turtlebot3_simulations by ROBOTIS-GIT (utilizar turtlebot3_gazebo de este repositorio)
5. slam_toolbox
6. rviz2
7. nav_controller de este repositorio.

<img src="https://github.com/user-attachments/assets/123f13f1-65b8-402f-8b0d-7e75255ac4a5" width="250"> <img src="https://github.com/user-attachments/assets/8fd84ff0-b1cc-48d2-97b8-e3237de19785" width="180"> <img src="https://github.com/user-attachments/assets/0dd92039-57a7-4a97-81bf-12015330f50e" width="345">


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
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
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
### Path Planning y navegación autónoma:
Para producir un path que seguirá el robot, es necesario lo siguiente:
1. Ejecutar el launch del test deseado a realizar (reemplazar testx por el numero de test que se quiere ejecutar)
  ```bash
  ros2 launch nav_controller test1.launch.py
  ```
2. Ejecutar el nodo de Path Planning que se quiere evaluar:
```bash
  ros2 run nav_controller euclideanA
  ros2 run nav_controller manhattanA
  ros2 run nav_controller octileA
  ros2 run nav_controller dijkstra
  ```
3. Otorgar Goal Pose y 2D Estimated Pose
   Se puede realizar a través de Rviz2 o ejecutando el siguiente archivo:
   (Entrega la pose estimada del inicio y el goal pose al otro lado del mapa)
   ```bash
    ros2 run nav_controller meta
    ```
5. Visualizar resultados:
   El launch file de cada test permite visualizar en rviz2 el path a seguir, el punto de Pure Pursuit y a través de este, el trayecto real que sigue el robot.
   Además escribe en un .csv los datos relevantes respecto al rendimiento de cada algoritmo en la simulacion.

<div align="center">
  <img src="https://github.com/user-attachments/assets/19d32633-7fcb-420b-a797-dada0b60d395" width="340">
</div>

   
> [!WARNING]
 > Si el destino esta en un sector 'desconocido' o 'ocupado' el robot planeará una trayectoria hasta el ultimo punto conocido

> [!NOTE]
> Respecto al paquete nav_controller:
> El paquete presente en el repositorio cuenta con cambios específicos realizados al paquete original producido por abdulkadrtr (https://github.com/abdulkadrtr/ROS2-PurePursuitControl-PathPlanning-Tracking).
> Los cambios corresponden a modificaciones del script para recopilar información, asi como para implementar dos métodos de calculo de distancia al momento de generar una trayectoria. Por otro lado se añadieron archivos y funciones para facilitar la evaluación de los algoritmos.

   
