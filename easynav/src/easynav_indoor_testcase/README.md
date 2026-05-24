## Parameter scenarios

| Param file | Robot | Sim/Real | World | Sensors | Maps Managers | Localizer | Planner | Controller | Checked |
|-----------|-------|----------|-------|---------|---------------|-----------|---------|------------|---------|
| [Test1](https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/dummy.params.yaml) | Kobuki | None | None | None | Dummy | Dummy | Dummy | Dummy | ✅ |
| [Test2](https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/dummy.sensor.params.yaml) | Kobuki | Sim | House | Laser | Simple | AMCL/Simple | Dummy | Dummy | ✅ |
| [Test3](https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/simple.params.yaml) | Kobuki | Sim | House | Laser | Simple | AMCL/Simple | A*/Simple | Simple | ✅ |
| [Test4](https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/costmap.serest.params.yaml) | Kobuki | Sim | House | Laser | Costmap (obstacles + inflation) | AMCL/Costmap | A*/Costmap | Serest | ✅ |
| [Test5](https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/costmap.mppi.routed.params.yaml) | Kobuki | Sim | House | Laser | Costmap (obstacles + inflation) + Routes | AMCL/Costmap | A*/Costmap | MPPI | ✅ |
| [Test6](https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/costmap.mpc.params.yaml) | Kobuki | Sim | House | Laser | Costmap (obstacles + inflation) | AMCL/Costmap | A*/Costmap | MPC  + Collision | ✅ |
| [Test7](https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/navmap.ls.params.yaml) | Summit XL | Sim | URJC Dig | Laser 3D | NavMap | LidarSlam | A*/NavMap | MPC | ✅ |
| [Test8](https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/bonxai.amcl.params.urjc.yaml) | Summit XL | Sim | URJC Dig | Laser 3D | Bonxai + NavMap (obstacles + inflation) | AMCL | A*/NavMap | MPC | ✅ |

## Install

First get all the required dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```

https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/bonxai.amcl.params.https://github.com/EasyNavigation/easynav_indoor_testcase/blob/main/robots_params/bonxai.amcl.params.urjc.yaml)