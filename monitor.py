#!/usr/bin/env python

import rosgraph
import rosnode
import rospy

import psutil

try:
  from xmlrpc.client import ServerProxy
except ImportError:
  from xmlrpclib import ServerProxy

from std_msgs.msg import Float32

POLL_PERIOD = 1.0

class Node:
  def __init__(self, name, pid):
    self.name = name
    self.proc = psutil.Process(pid)
    self.publisher = rospy.Publisher(rosgraph.names.ns_join("cpu_monitor", name[1:]), Float32, queue_size=20)

  def publish(self):
    self.publisher.publish(Float32(self.proc.cpu_percent()))

  def alive(self):
    return self.proc.is_running()

if __name__ == "__main__":
  rospy.init_node("cpu_monitor")
  master = rospy.get_master()

  node_map = {}

  while not rospy.is_shutdown():
    for node in rosnode.get_node_names():
      if node not in node_map:
        rospy.loginfo("[cpu monitor] found new node %s" % node)
        node_api = rosnode.get_api_uri(master, node)
        if node_api[2]:
          try:
            resp = ServerProxy(node_api[2]).getPid('/NODEINFO')
          except:
            rospy.logerr("[cpu monitor] failed to get pid of node %s (api is %s)" % (node, node_api))
          else:
            node_map[node] = Node(node, resp[2])
        else:
          rospy.logerr("[cpu monitor] failed to get api of node %s (%s)" % (node, node_api))

    for node_name, node in list(node_map.items()):
      if node.alive():
        node.publish()
      else:
        rospy.logwarn("[cpu monitor] lost node %s" % node_name)
        del node_map[node_name]

    rospy.sleep(POLL_PERIOD)
