
```
vcs import src < repositories.yaml
```

```
./arducopter --model copter

./arducopter --model copter --defaults copter.parm  -I0
```




```
#mavros
udp://:14550@ gcs_url:=udp://:14550@
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