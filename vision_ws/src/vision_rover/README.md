# Paquete de visión del rover

Este es el paquete de visión que se ejecutará en el rover.
En el archivo ```CMakeLists.txt``` debe estar escrito lo siguiente:

```
find_package( OpenCV REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)
```

## Carpeta scripts

La carpeta scripts contiene todos los nodos necesarios para que se ejecute cada cámara.
Para ejecutarlos es necesario insertar los siguientes comandos en una terminal de bash:

```
cd vision_ws
catkin_make
source devel/setup.bash
```

Y en diferentes terminales ejecutar los siguientes comandos:
```
roscore
rosrun vision_rover interface_cam1_node.py
rosrun vision_rover interface_cam2_node.py
```

Y para comprobar el funcionamiento internamente ejecutar (en terminales distintas):
```
rosrun vision_rover interface_cam1_test.py
rosrun vision_rover interface_cam2_test.py
rostopic pub /cam1_signal std_msgs/Float32 --once 1
```