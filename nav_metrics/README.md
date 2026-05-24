# Comparativa Nav2 vs EasyNav - Guía de Pruebas

Estructura para hacer pruebas reproducibles de navegación en interiores con métricas cuantitativas.

## 📋 Archivos en este directorio

- **`scenarios_indoor.yaml`**: Definición de escenarios de prueba (puntos inicio/final, repeticiones, etc.)
- **`analyze_navigation.py`**: Script para extraer métricas de un único rosbag
- **`compare_results.py`**: Script para comparar resultados entre múltiples bags
- **`bags/`**: Carpeta donde guardarás los rosbags

## 🚀 Flujo de trabajo

### 1️⃣ Preparación previa

```bash
# Crear carpeta para bags
mkdir -p bags

# Editar scenarios_indoor.yaml con tus puntos reales
# Cambiar "your_map_name" por el nombre de tu mapa
# Ajustar start_pose y goal_pose según tu entorno real
```

### 2️⃣ Grabar un rosbag (para cada escenario)

#### **Opción A: Grabar con ROS 2 CLI (más simple)**

```bash
# Terminal 1: Lanza tu sistema (nav2 o easynav + rviz2)
# Por ejemplo para nav2:
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

# Terminal 2: Graba MIENTRAS ejecutas la navegación
# COMANDO EXACTO:
ros2 bag record \
  /tf \
  /tf_static \
  /odom \
  /cmd_vel \
  /goal_pose \
  /local_costmap/costmap \
  /global_costmap/costmap \
  -o bags/pasillo_corto_nav2_run1

# Terminal 3: Envía un goal (con rviz o mediante rosservice)
# En rviz: 2D Goal Pose, o por línea de comandos:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{goal: {pose: {position: {x: 6.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

#### **Opción B: Grabar todo (para mayor detalle)**

Si quieres capturar TODOS los tópicos (aunque sea más volumen):

```bash
ros2 bag record --all -o bags/pasillo_corto_nav2_run1
```

### 3️⃣ Estructura de carpetas esperada

```
bags/
├── pasillo_corto_nav2_run1/
│   ├── metadata.yaml
│   ├── metadata.db
│   └── [archivos de datos]
├── pasillo_corto_easynav_run1/
│   ├── metadata.yaml
│   ├── metadata.db
│   └── [archivos de datos]
└── ... más bags
```

### 4️⃣ Analizar un bag individual

```bash
# Analiza un bag y extrae métricas
python3 analyze_navigation.py \
  bags/pasillo_corto_nav2_run1 \
  pasillo_corto \
  nav2

# Esto genera: bags/pasillo_corto_nav2_run1/report_nav2_pasillo_corto.json
```

### 5️⃣ Comparar todos los bags

```bash
# Crea una tabla comparativa de todos los reportes
python3 compare_results.py bags/ --output comparison_results.csv

# Abre comparison_results.csv en Excel o similar para ver la tabla
```

## 📊 Estructura de un Escenario

En `scenarios_indoor.yaml` define así cada prueba:

```yaml
- name: "pasillo_corto"                    # Nombre único
  description: "Pasillo recto, 5 metros"  # Descripción
  map: "your_map_name"                     # Tu mapa
  start_pose: [1.0, 1.0, 0.0]              # [x, y, theta] en radianes
  goal_pose: [6.0, 1.0, 0.0]               # [x, y, theta] en radianes
  expected_distance: 5.0                   # Para validación
  repetitions: 3                           # Cuántas veces repetir
```

## 🎯 Workflow completo ejemplo

```bash
# 1. Crear carpeta
cd /home/ai2lab/Documents/pruebas/nav_comparison

# 2. Editar escenarios (cambiar puntos start_pose/goal_pose)
nano scenarios_indoor.yaml

# 3. Ejecutar NAV2 con ROS 2
# Terminal 1:
ros2 launch nav2_bringup navigation_launch.py

# Terminal 2: Grabar escenario 1 - Nav2
ros2 bag record /tf /tf_static /odom /cmd_vel /goal_pose -o bags/pasillo_corto_nav2_run1 &
RECORD_PID=$!

# En rviz: 2D Goal Pose → 6.0, 1.0
# Cuando termine → Ctrl+C para parar grabación

# 4. Analizar
python3 analyze_navigation.py bags/pasillo_corto_nav2_run1 pasillo_corto nav2

# 5. Repetir con EasyNav
# (cambiar a EasyNav en terminal 1)
# (repetir pasos 2-4)

# 6. Comparar
python3 compare_results.py bags/
```

## 📦 Topics recomendados para grabar

Para comparar `Nav2` y `EasyNav` de forma justa, intenta grabar siempre el mismo conjunto de topics y con la misma frecuencia de muestreo.

### Imprescindibles

Estos son los mínimos para calcular las métricas principales del proyecto:

- `/odom`: calcular distancia recorrida y velocidad media.
- `/goal_pose` o el topic equivalente de goal que use tu navegador: detectar el inicio de la prueba.
- `/cmd_vel`: detectar el final de la navegación y analizar el comportamiento del controlador.
- `/tf` y `/tf_static`: reconstruir el estado del robot y depurar diferencias entre ejecuciones.

### Muy recomendables

Sirven para análisis más completos y para justificar resultados en el TFG:

- `/local_costmap/costmap`
- `/local_costmap/costmap_raw`
- `/local_costmap/costmap_updates`
- `/global_costmap/costmap`
- `/global_costmap/costmap_updates`
- `/plan_smooth` o `/received_global_plan`
- `/path` o el topic de ruta global si tu stack lo publica
- `/amcl_pose` o `/odom` si quieres comparar localización estimada y odometría
- `/scan` o `/rslidar_points` si quieres relacionar navegación con percepción

### Si quieres comparar rendimiento del sistema

Estos topics ayudan a explicar por qué un sistema tarda más o hace más maniobras:

- `/local_costmap/published_footprint`
- `/local_costmap/voxel_grid`
- `/parameter_events`
- `/rosout`
- `/transition_event` de los servidores de navegación si quieres analizar cambios de estado

### Comando MÍNIMO (solo lo esencial)

Para extraer métricas numéricas y comparar de forma justa:

```bash
ros2 bag record \
  /tf \
  /tf_static \
  /odom \
  /cmd_vel \
  /goal_pose \
  -o bags/recorrido_nav2_run1
```

Si usas **Nav2**, este es el comando exacto. Si usas **EasyNav**, verifica qué tópico usa para goals y reemplaza `/goal_pose` por el equivalente (ej: `/easynav/goal`).

### Comando ampliado (si quieres justificar más en memoria)

Si además quieres explicar por qué un stack tarda más o hace más maniobras:

```bash
ros2 bag record \
  /tf \
  /tf_static \
  /odom \
  /cmd_vel \
  /goal_pose \
  /local_costmap/costmap \
  /global_costmap/costmap \
  /received_global_plan \
  /scan \
  -o bags/recorrido_nav2_run1_completo
```

## 📈 Qué mide cada métrica

| Métrica | Significado | Unidad |
|---------|-----------|--------|
| **total_time_seconds** | Tiempo desde goal hasta que para | segundos |
| **distance_traveled_meters** | Suma de distancias punto a punto de odom | metros |
| **mean_velocity_ms** | Distancia / Tiempo total | m/s |

### Métricas numéricas que te conviene reportar en el TFG

Además de las que ya calcula el script, para una comparativa más sólida puedes añadir:

- `success_rate`: porcentaje de ejecuciones que llegan al goal.
- `time_to_goal`: tiempo desde enviar el goal hasta alcanzarlo.
- `path_length`: distancia real recorrida.
- `path_efficiency`: `distancia_recta / distancia_recorrida`.
- `overshoot`: exceso de recorrido respecto a la distancia esperada.
- `mean_speed`: velocidad media durante la navegación.
- `stops_count`: número de paradas o frenadas fuertes.
- `replan_count`: número de veces que cambia la ruta global.

Si luego quieres hacer la comparación más formal, una tabla por escenario con media y desviación estándar de estas métricas suele quedar muy bien en memoria o tesis.

## ⚙️ Ajustes según tu entorno

### Equivalencia de topics: Nav2 vs EasyNav

Para grabar bags comparables, usa esta tabla como referencia:

| Función | Nav2 | EasyNav | Uso en script |
|---------|------|---------|---------------|
| Localización/Odometría | `/odom` | `/odom` | Calcular distancia y velocidad |
| Goal/destino | `/goal_pose` | Verificar topic real | Detectar inicio de prueba |
| Control de velocidad | `/cmd_vel` | `/cmd_vel` | Detectar fin de prueba |
| Transformadas | `/tf`, `/tf_static` | `/tf`, `/tf_static` | Reconstruir trayectoria |
| Plan global (opcional) | `/received_global_plan` | `/planner_node/simple/path` o similar | Explicar rodeos/replanificaciones |
| Escaneo láser (opcional) | `/scan` | `/rslidar_points` o similar | Relacionar con navegación |

**Pasos:**
1. Ejecuta el stack (Nav2 o EasyNav)
2. En otra terminal, lista los topics: `ros2 topic list | grep -i goal`
3. Anota el topic de goal exacto
4. Usa ese topic en el comando `ros2 bag record`

### Para Nav2:
```bash
ros2 bag record /tf /tf_static /odom /cmd_vel /goal_pose -o bags/recorrido_nav2_run1
```

### Para EasyNav:
```bash
# Primero verifica el topic de goal
ros2 topic list | grep -i goal

# Luego adapta el comando (sustituyendo /goal_pose por el topic real)
ros2 bag record /tf /tf_static /odom /cmd_vel /tu_goal_topic -o bags/recorrido_easynav_run1
```

### Para tu mapa específico:
- Obtén puntos reales usando rviz: herramienta "Publish Point"
- Anota la posición exacta (x, y) de puntos en tu mapa
- Luego ponlos en `scenarios_indoor.yaml`

## 🐛 Troubleshooting

**Error: "No se encontró /tf en el rosbag"**
- Verifica que grabaste `--all` o al menos `/tf`

**Analyzer no detecta goal times**
- Comprueba qué tópico usa Nav2 o EasyNav para goals (puede no ser `/goal_pose`)
- Modifica `analyze_navigation.py` línea ~180 con el tópico correcto

**CSV no genera bien**
- Asegúrate de que los reportes JSON están en `bags/*/report_*.json`
- Ejecuta con `-vv` para debug (si agregas esa opción)

## 📝 Próximas mejoras

- [ ] Automatizar las pruebas con un nodo ROS 2
- [ ] Grabar también `/global_plan` para medir planificador
- [ ] Agregar análisis de replanificaciones
- [ ] Capturar eventos de fallo/éxito explícitos
