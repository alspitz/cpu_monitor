# cpu_monitor

`monitor.py` is a ROS node that will ask the ROS master for a list of nodes and publish their CPU and memory usage as ROS topics. It will also publish the total system CPU and memory usage.

Only nodes running on the same machine will have their CPU and memory usage published.

## Configuration
The polling period can be configured by setting the `poll_period` argument when launching. The default value if not specified is 1.0 seconds.

Setting `poll_period` to 10 seconds on the command line:
```
roslaunch cpu_monitor cpu_monitor.launch poll_period:=10
```

Setting `poll_period` to 10 seconds in a launch file_
```xml
<!-- mylaunchfile.launch -->
<launch>
  <include file="$(find cpu_monitor)/launch/cpu_monitor.launch">
    <arg name="poll_period" value="10.0"/>
  </include>
</launch>
```
