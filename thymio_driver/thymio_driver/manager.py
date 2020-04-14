import subprocess
from typing import Dict, Tuple

import rclpy
import rclpy.executors
from asebaros_msgs.msg import AsebaNodeList

from .base import BaseDriver


class Manager(rclpy.node.Node):  # type: ignore

    _drivers: Dict[str, BaseDriver]

    def __init__(self, executor: rclpy.executors.Executor) -> None:
        super(Manager, self).__init__('manager')
        self.ros_nodes: Dict[int, Tuple[str, BaseDriver]] = {}
        self.create_subscription(AsebaNodeList, 'aseba/nodes', self.on_aseba_node_list, 1)
        self.launch_model = self.declare_parameter('launch_model', True).value
        self.executor = executor
        self.model_process: Dict[int, subprocess.Popen[bytes]] = {}

    def on_aseba_node_list(self, msg: AsebaNodeList) -> None:
        for node in msg.nodes:
            if node.name in self._drivers and node.id not in self.ros_nodes:
                self.get_logger().warn(f"Connected to new robot {node.id} of kind {node.name}")
                self.add(uid=node.id, name=node.name_space, kind=node.name)
        uids = [node.id for node in msg.nodes]
        for uid in self.ros_nodes:
            if uid not in uids:
                self.get_logger().warn(f"Lost connection with robot {id}")
                self.remove(uid)

    def add(self, uid: int, name: str, kind: str) -> None:
        _, node = self.ros_nodes[uid] = (name, self._drivers[kind](namespace=name, standalone=False))
        self.executor.add_node(node)
        if self.launch_model:
            self.model_process[uid] = subprocess.Popen(
                (['ros2', 'launch'] + self.launch_model.split(' ') +
                 [f'name:={name}', f'namespace:={name}']))

    def remove(self, uid: int) -> None:
        _, node = self.ros_nodes.pop(uid)
        self.executor.remove_node(node)
        node.destroy_node()
        process = self.model_process.pop(uid)
        process.terminate()

    def __del__(self) -> None:
        for uid in list(self.ros_nodes):
            self.remove(uid)
        super(Manager, self).__del__()
