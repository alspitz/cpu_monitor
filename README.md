# cpu_monitor

[monitor.py](monitor.py) is a ROS node that will ask the ROS master for a list of nodes and publish their CPU and memory usage as ROS topics. It will also publish the total system CPU and memory usage.

Only nodes running on the same machine will have their CPU and memory usage published.

## Dependencies

* [psutil](https://pypi.org/project/psutil) for the Python version you are using (2 or 3).

One of the following should work on most machines.
```
python -m pip install psutil
python3 -m pip install psutil
sudo apt install python-psutil
sudo apt install python3-psutil
```

## Configuration

### Polling Period
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

### Source List
You can specify which ROS nodes you would like to monitor by setting the `source_list` argument when launching. The default is an empty list which then monitors all ROS nodes.

Setting `source_list` in the command line:
```
roslaunch cpu_monitor cpu_monitor.launch source_list:="[<node_name_1>, <node_name_2>]"
```

Or you can set the default values in the included launch file `cpu_monitor.launch`
