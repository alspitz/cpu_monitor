# cpu_monitor

`monitor.py` is a ROS node that will ask the ROS master for a list of nodes and publish their CPU and memory usage as ROS topics. It will also publish the total system CPU and memory usage.

Only nodes running on the same machine will have their CPU and memory usage published.
