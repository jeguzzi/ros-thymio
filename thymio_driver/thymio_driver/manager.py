# type: ignore
import subprocess

import rospy
from asebaros_msgs.msg import NodeList as AsebaNodeList


class Manager(object):

    _drivers = {}

    def __init__(self):
        self.ros_nodes = {}
        rospy.Subscriber('aseba/nodes', AsebaNodeList, self.on_aseba_node_list)
        self.launch_model = rospy.get_param('~launch_model', "")
        self.model_process = {}

    def on_aseba_node_list(self, msg):
        for node in msg.nodes:
            if node.name in self._drivers and node.id not in self.ros_nodes:
                rospy.loginfo("Connected to new robot %d of kind %s and namespace %s",
                              node.id, node.name, node.name_space)
                self.add(uid=node.id, name=node.name_space, kind=node.name)
        uids = [node.id for node in msg.nodes]
        for uid in self.ros_nodes:
            if uid not in uids:
                rospy.logwarn("Lost connection with robot %s", id)
                self.remove(uid)

    def add(self, uid: int, name, kind):
        _, node = self.ros_nodes[uid] = (name, self._drivers[kind](namespace=name, standalone=False))
        if self.launch_model:
            # TODO(Jerome) ROS2 uses namespace, ROS1 name. I should make them = and
            # fix the differences in description
            self.model_process[uid] = subprocess.Popen(
                (['roslaunch'] + self.launch_model.split(' ') +
                 ['name:={0}'.format(name)]))

    def remove(self, uid):
        _, node = self.ros_nodes.pop(uid)
        # TODO(ROS1): maybe we should deinitialized all subscribers, publishers, ...
        # node.destroy_node()
        if uid in self.model_process:
            process = self.model_process.pop(uid)
            process.terminate()

    def __del__(self):
        for uid in list(self.ros_nodes):
            self.remove(uid)
