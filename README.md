# cpu_monitor

[monitor.py](nodes/monitor.py) is a ROS node that will ask the ROS master for a list of nodes and publish their CPU and memory usage as ROS topics. It will also publish the total system CPU and memory usage.

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

### Saving to CSV
You can save the results to a CSV by setting the `save_to_csv` parameter to true. The results will be saved to the relative path specified in `csv_file` parameter. By default the file is saved in the folder under a file names `cpu_monitor.csv`. If `save_to_csv` is set to true, the node waits for 5 seconds for other nodes to start up before saving the list of nodes it will be monitoring. This is so that the header of the CSV stays constant throughout the tests. If new nodes pop up, they will not be recorded, as this would change the header of the CSV. 

### Source List
You can specify which ROS nodes you would like to save to csv by setting the `source_list` argument when launching. The default is an empty list which then monitors all ROS nodes active at 5 seconds after startup.

Setting `source_list` in the command line:
```
roslaunch cpu_monitor cpu_monitor.launch source_list:="[<node_name_1>, <node_name_2>]"
```

Or you can set the default values in the included launch file `cpu_monitor.launch`

