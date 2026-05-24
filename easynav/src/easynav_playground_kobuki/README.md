# EasyNav Playground Kobuki

## Installation
```bash
cd <easynav-workspace>/src/
vcs import . < easynav_playground_kobuki/thirdparty.repos
cd ..
colcon build --symlink-install 
```

## Usage
```bash
ros2 launch easynav_playground_kobuki playground_kobuki.launch.py
```

or

```bash
ros2 launch easynav_playground_kobuki playground_kobuki.launch.py world:=<path-to-world>
```