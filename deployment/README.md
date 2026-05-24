**Usuario de la Jetson NX: ai2FW**  
**Contraseña de la Jetson NX: ai2FW2026**

**Usuario upvnet: upvnet\imp004**  
**Contraseña upvnet: Ya.3stamos-2026**

**Usuario router STRONG blanco: STRONG_p4gM_2.4GHz o STRONG_p4gM_5GHz**  
**Contraseña: 12345678xy**  
**IP: 192.168.188.1**  
**Username: admin**  
**Password: 4ha9b6c4**

## NAV2/EASYNAV EXTERIORES/INTERIORES

En el siguiente archivo se va a explicar cómo poner en marcha el proyecto. Se trata de un proyecto que poner en marcha nav2 tanto en exteriores como en interiores y easynav tanto en exteriores como en interiores, pero teniendo en cuenta que en exteriores no se mueve, lo que hace es republicar el topic de posición que viene del sensor vision-RTK en el topic que se usa para la posición en easynav.

Excepto los paneles de foxglove, todo se ejecuta dentro de la Jetson NX que hay encima del robot FWMINI.

### 1. NAV2 O EASYNAV EN INTERIORES

Para ponerlo en marcha se deben lanzar los siguientes elementos tanto si se usa easynav como si se usa nav2:

**controladores ros2 del robot**

```bash
cd ~/Documents/fwmini_ros2/container/builder 
docker compose up --build
```

**lidar**

En esta apartado se poner en marcha el lidar montado en el robot

```bash
cd ~/Documents/rslidar/container/builder
docker compose up --build
```
Los controladores ros2 del robot y el lidar se deben poner en marcha tanto en easynav como en nav2, y a continuación se debe elegir uno de ellos para la localización/navegación

**Mapeo o navegacion en interiores con nav2**

En este apartado se pone en marcha los siguientes elementos (para ello se usa un mismo docker-compose.yaml, es decir todo se lanza al mismo tiempo):

- #### Descripción del robot: 

  Necesario para que se pueda realizar las transformaciones fijas del robot. Este apartado es el que usa el .urdf. También es el encargado de publicar el RobotModel para posteriormente visualizarlo en rviz2. Para que se visualice correctamente el modelo, es necesario lanzar rviz2 en este mismo contenedor para tener acceso a los meshes. 

- #### Transformación del PointCloud a LaserScan

  Necesario tanto para el mapeo como para la evitación de obstáculo. El nav2, necesita el dato como mensaje del tipo LaserScan. Se suscribe al topic del lidar y publica la información en el topic **/scan** 

- #### Broadcaster odom - base_link

  El topic **/odom** viene por defecto en el robot, pero se necesita publicar una transformación de odom a base_link. Este módulo se encarga precisamente de esto. 

- #### Rviz

​		Para la visualización de los elementos

- #### Mapeo

​		Para la creación del mapa que se usará para la posterior localización/navegación.

- #### Localización + navegación con nav2

  Para la localización/navegación del mapa 

Los primeros cuatro elementos, se ponen en marcha sin depender si se realiza el mapeo o se está en modo localización/navegación. 

En este caso con el mismo docker-compose se puede realizar dos acciones:

**Mapear**

   ```bash
   ~/Documents/nav2_indoor/container/builder
   docker compose --profile map up --build
   ```

   Posteriormente se debe mover con el mando el robot para que se vaya formando el mapa. 

   Una vez finalizado el mapa, este se debe guardar. 

   ```
   docker exec -it nav2_indoor-slam-1 bash
   cd /home/ubuntu/mapas
   ros2 run nav2_map_server map_saver_cli -f nombre_del_mapa
   ```

   Si se ha realizado correctamente los archivos  .pgm y .yaml deben aparecer en **/nav2_indoor/container/mapas**.

   En el archivo **/nav2_indoor/container/mapas/map_server.yaml** modificar el nombre del mapa según el que se ha configurado.

**Localizar/navegar**

   Para lanzar la localización/navegación, se debe tener un mapa previo generado. 

   ```
   ~/Documents/nav2_indoor/container/builder
   docker compose --profile navigation up --build
   ```

   Con la opción de **2D Pose Estimate** de rviz2 se debe configurar una posición inicial. En caso de que la posición inicial no coincida exactamente, se debe mover el robot con el mando para que se pueda localizar correctamente. Se localiza correctamente cuando lo que se ve en LaserScan, coincide con los elementos del mapa. 

   Para mover el robot, se puede usar la opción **2D Goal Pose** de rviz2 para llegar al punto deseado o el panel que se ha creado de foxglove.

   **PARA QUE EL ROBOT SE MUEVA CON NAV2 ES NECESARIO BAJAR EL SWITCH SWA DEL MANDO**

**Mapeo o navegacion en interiores con easynav**

Para el mapeo, se usa el mapa obtenido en el apartado anterior (con nav2). Si se quiere crear un mapa nuevo, se debe seguir los pasos del mapeo con nav2. Una vez obtenido el mapa, se debe guardar en **easynav/src/fwmini/maps**.

Para lanzar easynav en interiores, se debe seguir los siguientes pasos:

   ```bash
   ~/Documents/easynav/container/builder
   docker compose up --build
   ```

De la misma forma que en nav2, para poder navegar el robot se debe localizar de una forma correcta. Con la opción de **2D Pose Estimate** de rviz2 se debe configurar una posición inicial. En caso de que la posición inicial no coincida exactamente, se debe mover el robot con el mando para que se pueda localizar correctamente. Se localiza correctamente cuando lo que se ve en LaserScan, coincide con los elementos del mapa. 

Para mover el robot, se puede usar la opción **2D Goal Pose** de rviz2 para llegar al punto deseado o el panel que se ha creado de foxglove. 

### 2. NAV2 O EASYNAV EN EXTERIORES

Tanto si se usa easynav como si se usa nav2, se deben lanzar los controladores ros2 del robot, el lidar y el visionRTK de Fixposition

**controladores ros2 del robot**

  ```bash
  cd ~/Documents/fwmini_ros2/container/builder 
  docker compose up --build
  ```

**lidar**

En esta apartado se poner en marcha el lidar montado en el robot

  ```bash
  cd ~/Documents/rslidar/container/builder
  docker compose up --build
  ```

**visionRTK de Fixposition**

Antes de poner en marcha el driver ROS2 de este sensor, se debe asegurar de que el sensor tenga internet y que además la fusión esté inicializada. Para ello se deben seguir los siguientes pasos:

Para configurar y ver el estado del sensor, se debe entrar en el navegador y poner la IP que tiene el sensor **192.168.0.156**

1. Asegurarse de que tiene internet:

El visión RTK se debe conectar a internet para coger información de hora y de esta forma poder sincronizar el reloj del resto de sensores, en caso contrario el sistema no funciona.

Para asegurarse de que el sensor tiene internet debe aparecer los 4 iconos arriba derecha tal como se ve en la figura (los dos circulos amarillos pueden ser de cualquier otro color), el resto de simbolos, deben ser iguales. 

(images/init.png)

2. Iniciar la fusión:

A la imagen anterior, se debe pulsar el botón **Load** y aparecerá la siguiente ventana:

(images/load.png)

A continuación se vuelve a pulsado load

(images/load_fusion.png)

Y se elige una de las fusiones existentes (no importa si coincide o no con la realidad, ya que cuando el sensor se conecte a un GNSS fix, esa posición cambiará a la correcta).

(images/yes_fusion.png)

Se acepta pulsando el **yes**

Y así es como debería aparece si se ha iniciado la fusión correctamente 

(images/init_fusion.png)

3. Poner en marcha el driver ROS2:

  ```bash
  cd ~/Documents/fixposition/container/builder
  docker compose up --build
  ```
Los pasos descritos anteriormente se deben realizar tanto si se lanza nav2 como easynav

**Localización y navegación con nav2 en exteriores**

Se pone en marcha de la siguiente forma:

  ```bash
  cd ~/Documents/nav2_tutorial/container/builder
  docker compose up --build
  ```

Posteriormente aparecerá la ventana de rviz2. Para mover el robot se puede hacer pulsando en rviz2 la opción **2D Goal Pose** o pulsado el punto deseado en el panel de foxglove **nav2_outdoor**

**Localización con easynav en exteriores**

En este apartado el sistema unicamente se localiza, no se puede navegar. Para ver la localización se lanza easynav en exteriores y se abre el panel **easynav_outdoor**

Para lanzar easynav en exteriores, se debe cambiar el nombre del archivo **easynav/container/builder/docker-compose-exteriores.yaml** a **docker-compose.yaml** y lanzar:

  ```bash
   ~/Documents/easynav/container/builder
   docker compose up --build
   ```

### 3. ONSERVACIONES

#### Configuraciones del IP forwarding

Para que fixposition pueda conectarse a internet de la upvnet se debe realizar el ip forwarding, de modo que tenga internet aunque no esté conectado a upvnet sino solo al router local:

  ```bash
  PRIV=enP8p1s0
  INT=wlP1p1s0
  sudo iptables -t nat -A POSTROUTING -o $INT -j MASQUERADE
  sudo iptables -A FORWARD  -i $PRIV -o  $INT -j ACCEPT
  sudo iptables -A FORWARD  -i $INT -o $PRIV -m state --state RELATED,ESTABLISHED -j ACCEPT
  iptables -L
  sudo apt update  && sudo apt install iptables-persistent
  ```
