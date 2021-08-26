import subprocess
from typing import Dict, Tuple, Type

import rospy
from asebaros_msgs.msg import NodeList as AsebaNodeList

from .base import BaseDriver


class Manager:

    _drivers: Dict[str, Type[BaseDriver]]

    def __init__(self) -> None:
        self.ros_nodes: Dict[int, Tuple[str, BaseDriver]] = {}
        rospy.Subscriber('aseba/nodes', AsebaNodeList, self.on_aseba_node_list)
        self.launch_model = rospy.get_param('~launch_model', "")
        self.model_process: Dict[int, subprocess.Popen[bytes]] = {}

    def on_aseba_node_list(self, msg: AsebaNodeList) -> None:
        for node in msg.nodes:
            if node.name in self._drivers and node.id not in self.ros_nodes:
                rospy.loginfo(f"Connected to new robot {node.id} of kind {node.name} and namespace {node.name_space}")
                self.add(uid=node.id, name=node.name_space, kind=node.name)
        uids = [node.id for node in msg.nodes]
        for uid in self.ros_nodes:
            if uid not in uids:
                rospy.logwarn(f"Lost connection with robot {id}")
                self.remove(uid)

    def add(self, uid: int, name: str, kind: str) -> None:
        _, node = self.ros_nodes[uid] = (name, self._drivers[kind](namespace=name, standalone=False))
        if self.launch_model:
            # TODO(Jerome) ROS2 uses namespace, ROS1 name. I should make them = and
            # fix the differences in description
            self.model_process[uid] = subprocess.Popen(
                (['roslaunch'] + self.launch_model.split(' ') +
                 [f'name:={name}', f'name:={name}']))

    def remove(self, uid: int) -> None:
        _, node = self.ros_nodes.pop(uid)
        # TODO(ROS1): maybe we should deinitialized all subscribers, publishers, ...
        # node.destroy_node()
        process = self.model_process.pop(uid)
        process.terminate()

    def __del__(self) -> None:
        for uid in list(self.ros_nodes):
            self.remove(uid)
