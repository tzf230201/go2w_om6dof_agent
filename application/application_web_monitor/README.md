# application_web_monitor

ROS 2 web dashboard for Go2W and the OM6DOF stack. It displays robot, ROS
graph, controller, and camera status and provides guarded remote actions.

Build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select application_web_monitor
source install/setup.bash
ros2 run application_web_monitor web_monitor
```

Open `http://<robot-ip>:8080` on the trusted robot LAN. The server has no login.

## Optional camera stream

The monitor does not open a camera. Its camera card stays hidden until JPEG
frames are published as `sensor_msgs/msg/CompressedImage` on:

```text
/application_web_monitor/image/compressed
```

The OM6DOF perception node publishes its processed overlay to this input by
default, so it can be started directly:

```bash
ros2 launch om6dof_perception perception.launch.py
```

The card appears automatically while frames arrive and disappears shortly
after the publisher or relay stops. Use the `camera_topic` ROS parameter to
select a different input topic.

The `systemd/` and `sudoers/` directories contain the service unit and the
single-command permission used by the OM6DOF restart button.
