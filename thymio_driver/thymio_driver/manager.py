import subprocess
from typing import Dict, Tuple, Type

import rclpy
import rclpy.executors
from asebaros_msgs.msg import NodeList as AsebaNodeList

from .base import BaseDriver


class Manager(rclpy.node.Node):  # type: ignore

    _drivers: Dict[str, Type[BaseDriver]]

    def __init__(self, executor: rclpy.executors.Executor) -> None:
        super(Manager, self).__init__('manager')
        self.ros_nodes: Dict[int, Tuple[str, BaseDriver]] = {}
        self.create_subscription(AsebaNodeList, 'aseba/nodes', self.on_aseba_node_list, 1)
        self.launch_model = self.declare_parameter('launch_model', "").value
        self.executor = executor
        self.model_process: Dict[int, subprocess.Popen[bytes]] = {}

    def on_aseba_node_list(self, msg: AsebaNodeList) -> None:
        namespaces = [name for name, _ in self.ros_nodes.values()]
        for node in msg.nodes:
            if(node.name in self._drivers and node.id not in self.ros_nodes and
               node.name_space not in namespaces):
                self.get_logger().warn(f"Connected to new robot {node.id} of kind {node.name}")
                self.add(uid=node.id, name=node.name_space, kind=node.name)
        uids = [node.id for node in msg.nodes]
        for uid in self.ros_nodes:
            if uid not in uids:
                self.get_logger().warn(f"Lost connection with robot {id}")
                self.remove(uid)

    def add(self, uid: int, name: str, kind: str) -> None:
        if not self.executor:
            self.get_logger().error("No exector defined: won't add node")
            return
        _, node = self.ros_nodes[uid] = (
            name, self._drivers[kind](namespace=name, manager=self, uid=uid))
        self.executor.add_node(node)
        if self.launch_model:
            self.model_process[uid] = subprocess.Popen(
                (['ros2', 'launch'] + self.launch_model.split(' ') +
                 [f'name:={name}', f'namespace:={name}']))

    def remove(self, uid: int) -> None:
        _, node = self.ros_nodes.pop(uid)
        if self.executor:
            self.executor.remove_node(node)
        node.destroy_node()
        if uid in self.model_process:
            process = self.model_process.pop(uid)
            process.terminate()

    def __del__(self) -> None:
        for uid in list(self.ros_nodes):
            self.remove(uid)
