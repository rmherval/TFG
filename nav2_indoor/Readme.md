## ROBOT MÓVIL FW-MINI PARA INTERIORES

En este apartado se explica la puesta en marcha del robot FW-MINI para su navegación en interiores.

#### 1. CONTROLADORES ROS2 DEL ROBOT

En este apartado se ponen en marcha los siguientes elementos:

**foxglove-bridge**

**controladores ros2 del robot**

```bash
cd ~/Documents/fwmini_ros2/container/builder 
docker compose up --build
```

Si se ha lanzado correctamente debe aparecer los siguientes logs

(images/fwminiros2.JPG)

(images/foxglove.JPG)

#### 2. Lidar

En esta apartado se poner en marcha el lidar montado en el robot

```bash
cd ~/Documents/rslidar/container/builder
docker compose up --build
```

Si se pone en marcha correctamente debe aparecer el siguiente mensaje

(images/lidar.JPG)

#### 3. Mapeo o navegación en interiores

En este apartado se pone en marcha los siguientes elementos:

- ###### Descripción del robot: 

  Necesario para que se pueda realizar las transformaciones fijas del robot. Este apartado es el que usa el .urdf. También es el encargado de publicar el RobotModel para posteriormente visualizarlo en rviz2. Para que se visualice correctamente el modelo, es necesario lanzar rviz2 en este mismo contenedor para tener acceso a los meshes. 

- ###### Transformación del PointCloud a LaserScan

  Necesario tanto para el mapeo como para la evitación de obstáculo. El nav2, necesita el dato como mensaje del tipo LaserScan. Se suscribe al topic del lidar y publica la información en el topic **/scan** 

- ###### Broadcaster odom - base_link

  El topic **/odom** viene por defecto en el robot, pero se necesita publicar una transformación de odom a base_link. Este módulo se encarga precisamente de esto. 

- ###### Rviz

​		Para la visualización de los elementos

- ###### Mapeo

​		Para la creación del mapa que se usará para la posterior localización/navegación.

- ###### Localización + navegación 

  Para la localización/navegación del mapa 

Los primeros cuatro elementos, se ponen en marcha sin depender si se realiza el mapeo o se está en modo localización/navegación. 

En este caso con el mismo docker-compose se puede realizar dos acciones:

1. **Mapear**

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

   En el archivo **/nav2_indoor/nav2_indoor/config/map_server.yaml** modificar el nombre del mapa según el que se ha configurado.

2. **Localizar/navegar**

   Para lanzar la localización/navegación, se debe tener un mapa previo generado. 

   ```
   ~/Documents/nav2_indoor/container/builder
   docker compose --profile navigation up --build
   ```

   Con la opción de **2D Pose Estimate** de rviz2 se debe configurar una posición inicial. En caso de que la posición inicial no coincida exactamente, se debe mover el robot con el mando para que se pueda localizar correctamente. Se localiza correctamente cuando lo que se ve en LaserScan, coincide con los elementos del mapa. 

   Para mover el robot, se puede usar la opción **2D Goal Pose** de rviz2 para llegar al punto deseado

   **PARA QUE EL ROBOT SE MUEVA CON NAV2 ES NECESARIO BAJAR EL SWITCH SWA DEL MANDO**

