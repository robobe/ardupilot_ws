
```
vcs import src < repositories.yaml
```

## SITL

```bash
/home/user/git/ardupilot/build/sitl/bin/arducopter --model + --speedup 1 --slave 0 --defaults Tools/autotest/default_params/copter.parm --sim-address=127.0.0.1 -I0
```

```bash title="Run SITL without gazebo"
/workspace/src/ardupilot_bringup/bin/arducopter --model + \
--speedup 1 \
--slave 0 \
--defaults /workspace/src/ardupilot_bringup/params/copter_default.param \
--sim-address=127.0.0.1 \
-I0
```


## MAVROS
```bash title="mavros python launch"
 ros2 launch ardupilot_bringup mavros.launch.py fcu_url:=tcp://:5760@127.0.0.1:5760 gcs_url:=udp://@127.0.0.1:14560 
```

```
ros2 launch mavros node.launch \
fcu_url:=tcp://:5760@127.0.0.1:5760 \
gcs_url:=udp://@127.0.0.1:14560 \
tgt_system:=1 \
tgt_component:=1 \
pluginlists_yaml:=/workspace/src/ardupilot_bringup/config/plugins.yaml \
config_yaml:=/workspace/src/ardupilot_bringup/config/config.yaml
```

```
#mavros
 
ros2 launch mavros apm.launch fcu_url:=udp://:14550@ 
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200@
ros2 launch mavros apm.launch fcu_url:=tcp://:5760@ 
pluginlists_yaml:=/workspace/src/ardupilot_bringup/config/plugins.yaml
config_yaml:=/workspace/src/ardupilot_bringup/config/config.yaml

```

```bash
ros2 topic echo /mavros/timesync_status
# (Type: mavros_msgs/msg/TimesyncStatus)
#
header:
  stamp:
    sec: 1751979671
    nanosec: 766953438
  frame_id: ''
remote_timestamp_ns: 116391849000
observed_offset_ns: 1751979555372552567
estimated_offset_ns: 1751979555373389824
round_trip_time_ms: 5.100312232971191
```

| Field                 | Type                      | Description                                           |
| --------------------- | ------------------------- | ----------------------------------------------------- |
| `stamp`               | `builtin_interfaces/Time` | Time when this status was published (ROS time)        |
| `remote_timestamp_ns` | `int64`                   | Timestamp (in nanoseconds) from the FCU               |
| `observed_offset_ns`  | `int64`                   | Raw offset between ROS time and FCU time              |
| `estimated_offset_ns` | `int64`                   | Low-pass filtered estimate of the actual time offset  |
| `round_trip_time_ns`  | `int64`                   | Measured network latency (ping-pong round-trip delay) |


---

## Gazebo (harmonic)

```
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```