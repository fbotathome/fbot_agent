import rclpy
from fbot_agent.agents.agent import AgentNode
from threading import Thread

import rclpy.executors

def test_navigate():
    rclpy.init()
    node = AgentNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
