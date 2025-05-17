### Instructions for Setting Up On The Robot

On the Jetson Orin:

- Install ollama from jetson containers
- Pull gemma3 model weights by running "ollama pull gemma3"

On the NUC:

- Run script "install_dependencies.sh"
- Update "config/fbot_agent_config.yaml" with waypoints in the target environment.
- Include the fbot_agent.launch.py from this package into some fbot_behavior launch with required navigation subsystems. The realsense must be publishing messages on the "/camera/camera/color/image_raw" topic.
- Query the agent by calling the "/fbot_agent/execute_command" service.
- The service type is "fbot_agent_msgs.srv.AgentCommand"

### TODO

Some things need further investigation:

- The "fbot_agent_node.py" is not being updated with the llm model and camera topic parameters in "config/fbot_agent_config.py". These must be manually set in "fbot_agent_node.py".