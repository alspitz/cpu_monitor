#!/usr/bin/env python3

import functools
import os
import subprocess

import rosnode
import rospy

import psutil

try:
  from xmlrpc.client import ServerProxy
except ImportError:
  from xmlrpclib import ServerProxy

from std_msgs.msg import Float32, UInt64


def ns_join(*names):
  return functools.reduce(rospy.names.ns_join, names, "")

class Node:
  def __init__(self, name, pid):
    self.name = name
    self.proc = psutil.Process(pid)
    self.cpu_publisher = rospy.Publisher(ns_join("~", name[1:], "cpu"), Float32, queue_size=20)
    self.mem_publisher = rospy.Publisher(ns_join("~", name[1:], "mem"), UInt64, queue_size=20)

  def publish(self):
    self.cpu = self.proc.cpu_percent()
    self.mem = self.proc.memory_info().rss
    self.cpu_publisher.publish(Float32(self.cpu))
    self.mem_publisher.publish(UInt64(self.mem))

  def get_values(self):
    return self.cpu, self.mem

  def alive(self):
    return self.proc.is_running()

class CSVWriter:
  def __init__(self, filename, source_list):
    self.csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    rospy.loginfo("[cpu monitor] saving to file: %s" % self.csv_path)
    self.file = open(self.csv_path, "w")
    self.header_init = False
    if len(source_list) > 0:
      self.init_header(source_list)

  def init_header(self, source_list):
    self.source_list = source_list
    self.file.write('time')
    for source in self.source_list:
      self.file.write(', %s cpu, %s mem' % (source, source))
    self.file.write('\n')
    rospy.loginfo("[cpu monitor] monitoring nodes: %s" % self.source_list)
    self.header_init = True

  def update(self, node_map):
    new_csv_line = str(rospy.get_rostime())
    for source in self.source_list:
      if source in node_map and node_map[source].alive():
        cpu, mem = node_map[source].get_values()
        new_csv_line += ', %f, %f' % (cpu, mem)
      else:
        new_csv_line += ', , '
    new_csv_line += "\n"
    self.file.write(new_csv_line)

if __name__ == "__main__":
  rospy.init_node("cpu_monitor")
  master = rospy.get_master()

  poll_period = rospy.get_param('~poll_period', 1.0)
  save_to_csv = rospy.get_param('~save_to_csv', False)
  csv_file_name = rospy.get_param('~csv_file', 'cpu_monitor.csv')
  source_list = rospy.get_param('~source_list', [])

  if save_to_csv:
    csv_writer = CSVWriter(csv_file_name, source_list)
    node_start_time = rospy.get_rostime()

  this_ip = os.environ.get("ROS_IP")

  node_map = {}
  ignored_nodes = set()

  cpu_publish = rospy.Publisher("~total_cpu", Float32, queue_size=20)

  mem_topics = ["available", "used", "free", "active", "inactive", "buffers", "cached", "shared", "slab"]

  vm = psutil.virtual_memory()
  mem_topics = [topic for topic in mem_topics if topic in dir(vm)]

  mem_publishers = []
  for mem_topic in mem_topics:
    mem_publishers.append(rospy.Publisher("~total_%s_mem" % mem_topic,
                                          UInt64, queue_size=20))

  while not rospy.is_shutdown():
    for node in rosnode.get_node_names():
      if node in node_map or node in ignored_nodes:
        continue

      node_api = rosnode.get_api_uri(master, node, skip_cache=True)[2]
      if not node_api:
        rospy.logerr("[cpu monitor] failed to get api of node %s (%s)" % (node, node_api))
        continue

      ros_ip = node_api[7:] # strip http://
      ros_ip = ros_ip.split(':')[0] # strip :<port>/
      local_node = "localhost" in node_api or \
                  "127.0.0.1" in node_api or \
                  (this_ip is not None and this_ip == ros_ip) or \
                  subprocess.check_output("hostname").decode('utf-8').strip() in node_api
      if not local_node:
        ignored_nodes.add(node)
        rospy.loginfo("[cpu monitor] ignoring node %s with URI %s" % (node, node_api))
        continue

      try:
        resp = ServerProxy(node_api).getPid('/NODEINFO')
      except:
        rospy.logerr("[cpu monitor] failed to get pid of node %s (api is %s)" % (node, node_api))
      else:
        try:
          pid = resp[2]
        except:
          rospy.logerr("[cpu monitor] failed to get pid for node %s from NODEINFO response: %s" % (node, resp))
        else:
          try:
            node_map[node] = Node(name=node, pid=pid)
          except psutil.NoSuchProcess:
            rospy.logwarn("[cpu monitor] psutil can't see %s (pid = %d). Ignoring" % (node, pid))
            ignored_nodes.add(node)
          else:
            rospy.loginfo("[cpu monitor] adding new node %s" % node)

    for node_name, node in list(node_map.items()):
      if node.alive():
        node.publish()
      else:
        rospy.logwarn("[cpu monitor] lost node %s" % node_name)
        del node_map[node_name]

    cpu_publish.publish(Float32(psutil.cpu_percent()))

    vm = psutil.virtual_memory()
    for mem_topic, mem_publisher in zip(mem_topics, mem_publishers):
      mem_publisher.publish(UInt64(getattr(vm, mem_topic)))

    if save_to_csv:
      if csv_writer.header_init:
        csv_writer.update(node_map)
      elif (rospy.get_rostime() - node_start_time) > rospy.Duration(5): # wait 5 seconds before setting header
        rospy.loginfo("[cpu monitor] waited for 5 seconds before initializing csv header")
        csv_writer.init_header(list(node_map.keys()))


    rospy.sleep(poll_period)
