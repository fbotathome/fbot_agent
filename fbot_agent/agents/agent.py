import PIL.Image
from rclpy.node import Node
from sensor_msgs.msg import Image

from smolagents import CodeAgent, tool, LiteLLMModel, ActionStep
import PIL
from ros2_numpy import numpify
from threading import Event
from copy import deepcopy
from fbot_agent_msgs.srv import AgentCommand
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
import yaml
import os
from transformations import quaternion_from_euler
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT

from ..tools import navigate_to_pose, move_forward, rotate, count_objects, detect_object, query_pose_by_name, transform_pose, analyze_scene, detect_and_pick_object, give_object_to_user, search_object, approach_person_by_name, detect_and_approach_object, listen_something, say_something, get_question_answer


class AgentNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.declare_parameters('~named_poses', [])
        self.declare_parameter('~llm_model', 'gemini/gemini-2.5-flash')
        self.declare_parameter('~camera_topic', '/multisense_1/aux/image_color')
        self.llm = LiteLLMModel(model_id=self.get_parameter('~llm_model').get_parameter_value().string_value, flatten_messages_as_text=False)#,api_base="http://127.0.0.1:11434")
        self.agent = CodeAgent(
            tools=self.get_tools(),
            model=self.llm,
            step_callbacks=[]#[self.add_image_to_agent_obs,]
        )
        self.img_event = Event()
        self.img_event.clear()
        service_cb_group = ReentrantCallbackGroup()
        self.log_count = 0
        self.is_egpsr = False
        self.img_sub = self.create_subscription(msg_type=Image, topic=self.get_parameter('~camera_topic').get_parameter_value().string_value, callback=self.on_camera_msg, callback_group=service_cb_group, qos_profile=10)
        self.get_logger().info(f'Subscribed to camera topic: {self.img_sub.topic} and {self.img_sub.topic_name}')
        self.agent_server = self.create_service(srv_type=AgentCommand, srv_name='/fbot_agent/execute_command', callback=self.agent_callback)

    def get_available_parameters(self):
        with open(os.path.join(get_package_share_directory('fbot_agent'), 'config', 'fbot_agent_config.yaml'), 'r') as f:
            yaml_data = yaml.load(f, Loader=yaml.FullLoader)
        named_poses = list(yaml_data['/fbot_agent']['fbot_agent_node']['ros__parameters']['named_poses'].keys())
        object_labels = list(yaml_data['/fbot_agent']['fbot_agent_node']['ros__parameters']['object_labels'])
        return named_poses, object_labels

    def prepare_prompt(self, command: str):
        available_poses, object_labels = self.get_available_parameters()

        prompt = (
            "You are BORIS, a domestic service robot. You will receive a task from a person who is in front of you. "
            "You must speak every action you take, so the person can understand what you are doing.\n"
        )
        if not self.is_egpsr:
            prompt += (
            "The person location is 'start_point'.\n"
            "If the task is \"Tell me ...\", you only have to say the information for the person at the 'start_point' location, "
            "so save the answer for when you are back to the 'start_point' location.\n"
            "If your task does not end at the 'start_point' location, you must return to it after finishing the task.\n"
            )
        prompt += (

            f"The following location names are available: {available_poses}\n"
            f"The following object labels/descriptions are available for detection tools: {object_labels}\n"
            f"Your task is to: {command}\n"
        )
        return prompt
    
    def agent_callback(self, req: AgentCommand.Request, res: AgentCommand.Response):

        self.is_egpsr = req.is_egpsr
        self.get_logger().info(f'Received command: {req.command} and waiting for image...')
        # self.img_event.wait()
        self.get_logger().info('Image received, processing command...')
        final_answer = self.agent.run(
            task=self.prepare_prompt(req.command),
            # images=[PIL.Image.fromarray(numpify(deepcopy(self.image_obs)))]
        )
        res.response = str(final_answer)
        return res

    def on_camera_msg(self, msg: Image):
        self.log_count += 1
        if self.log_count % 20 == 0:
            self.get_logger().info(f'Camera message received {self.log_count} times.')
        self.image_obs = msg
        self.img_event.set()

    def get_tools(self):
        return [
            navigate_to_pose,
            move_forward,
            rotate,
            # pick_object,
            detect_object,
            analyze_scene,
            count_objects,
            # query_pose_by_name,
            # transform_pose,
            search_object,
            detect_and_pick_object,
            give_object_to_user, 
            approach_person_by_name,
            detect_and_approach_object,
            listen_something,
            say_something,
            get_question_answer

        ]
    
    def add_image_to_agent_obs(self, step_log: ActionStep, agent: CodeAgent):
        self.img_event.wait()
        step_log.observations_images = [PIL.Image.fromarray(numpify(deepcopy(self.image_obs)))]
