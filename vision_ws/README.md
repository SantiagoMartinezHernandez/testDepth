# Workspace de visión

Este es el workspace de visión que trabajará robocol.

Para empezar, es necesario clonar el repositorio y ejecutar en el directorio de vision_ws:

```
catkin_make install
```

Este comando creará las carpetas ```build``` y ```devel```.

Posteriormente es necesario ejecutar
```
cd vision_ws
source devel/setup.bash
```

## Paquete vision_rover

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

### Carpeta scripts

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

## Paquetes async_web_server_cpp y web_vision_server

Estos paquetes tienen la función de montar un servidor local (```http://localhost:8080/```) donde se stremean las cámaras (puerto por defecto: 8080)

Para ejecutar el servidor es necesario correr el comando:

```
cd vision_ws
source devel/setup.bash
rosrun video_server web_video_server
```

# Agradecimientos

A continuación se muestra la referencia de los paquetes utilizados externamente:
## Repositorios
async_web_server_cpp: https://github.com/fkie/async_web_server_cpp/tree/ros1-releases

web_video_server: https://github.com/RobotWebTools/web_video_server/tree/master

## Organizaciones

Robot Web Tools: https://github.com/RobotWebTools

Fraunhofer FKIE: https://github.com/fkie
