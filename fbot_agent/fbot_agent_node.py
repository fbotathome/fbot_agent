import rclpy
from fbot_agent.agents.agent import AgentNode
from threading import Thread

import rclpy.executors

def main():
    rclpy.init()
    node = AgentNode(node_name='fbot_agent_node')
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    main()
