import PIL.Image
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from state_machine.states.navigation.navigate_to_pose import NavigateToPoseState
from state_machine.states.navigation.move_fixed_value import MoveFixedValueState
from state_machine.states.others.transform_poses import TransformPoses
from nav2_msgs.action import NavigateToPose
from yasmin import StateMachine, Blackboard
from smolagents import CodeAgent, tool, LiteLLMModel, ActionStep
import PIL
from ros2_numpy import numpify
from threading import Event
from copy import deepcopy
from fbot_agent_msgs.srv import AgentCommand
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from transformations import quaternion_from_euler


@tool
def query_pose_by_name(name: str)->PoseStamped:
        """
        Allows you to get the pose associated with a given name. Returns the PoseStamped associated with the name.

        Args:
            name: A string with the name of the pose to query.
        """
        try:
            with open(os.path.join(get_package_share_directory('fbot_agent'), 'config', 'fbot_agent_config.yaml'), 'r') as f:
                yaml_data = yaml.load(f, Loader=yaml.FullLoader)
            ps = yaml_data['/fbot_agent']['fbot_agent_node']['ros__parameters']['named_poses'][name]
            assert len(ps) == 7
        except BaseException as e:
            raise ValueError('Unknown named pose: '+name)
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = ps[0]
        p.pose.position.y = ps[1]
        p.pose.position.z = ps[2]
        p.pose.orientation.x = ps[3]
        p.pose.orientation.y = ps[4]
        p.pose.orientation.z = ps[5]
        p.pose.orientation.w = ps[6]
        return p

@tool
def navigate_to_pose(pose_name: str)->bool:
        """
        Allows you to navigate to a given pose. Returns 'True' if navigation is successfull, 'False' otherwise.

        Args:
            pose_name: The name of the pose to navigate to.
        """
        pose = query_pose_by_name(name=pose_name)
        sm = StateMachine(outcomes=['aborted', 'canceled', 'succeeded', 'timeout'])
        sm.add_state(
            name='NAV_TO_POSE',
            state=NavigateToPoseState(),
            transitions={
                'succeeded': 'succeeded',
                'aborted': 'aborted',
                'canceled': 'canceled',
                'timeout': 'timeout'
            }
        )
        blackboard = Blackboard(init={
            'pose_from_target': NavigateToPose.Goal(pose=pose)
        })
        outcome = sm.execute(blackboard=blackboard)
        return outcome == 'succeeded'

def transform_pose(pose: PoseStamped, target_frame: str)->PoseStamped:
    sm = StateMachine(outcomes=['succeeded', 'canceled', 'aborted'])
    sm.add_state(
        name='TRANSFORM_POSE',
        state=TransformPoses(
            source_frame=pose.header,
            target_frame=target_frame,
            poses_to_transform=[pose.pose]
        )
    )
    blackboard = Blackboard()
    sm.execute(blackboard=blackboard)
    new_ps = PoseStamped()
    new_ps.header.frame_id = target_frame
    new_ps.pose = blackboard['transformed_poses'][0]
    return new_ps

@tool
def move_forward(distance: float)->bool:
     """
     Allows you to move forward by a given distance. Returns 'True' if movement is successfull

     Args:
        distance: A float value with how many meters to move forward
     """
     sm = StateMachine(outcomes=['succeeded', 'canceled', 'aborted'])
     sm.add_state(
          name='MOVE_FIXED_VALUE',
          state=MoveFixedValueState(
               tp=1,
               deviation=distance
          ),
          transitions={
               'succeeded': 'succeeded',
               'canceled': 'canceled',
               'aborted': 'aborted'
          }
     )
     outcome = sm.execute(blackboard=Blackboard())
     return outcome == 'succeeded'

@tool
def rotate(angle: float)->bool:
     """
     Allows you to rotate in-place by a given angle. Returns 'True' if movement is successfull

     Args:
        angle: A float value with how many radians to rotate. Positive values rotate left. Negative values rotate right.
     """
     sm = StateMachine(outcomes=['succeeded', 'canceled', 'aborted'])
     sm.add_state(
          name='MOVE_FIXED_VALUE',
          state=MoveFixedValueState(
               tp=0,
               deviation=angle
          ),
          transitions={
               'succeeded': 'succeeded',
               'canceled': 'canceled',
               'aborted': 'aborted'
          }
     )
     outcome = sm.execute(blackboard=Blackboard())
     return outcome == 'succeeded'

class AgentNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.declare_parameters('~named_poses', [])
        self.declare_parameter('~llm_model', 'gemini/gemini-2.5-flash-preview-04-17')
        self.declare_parameter('~camera_topic', '/camera/camera/color/image_raw')
        self.llm = LiteLLMModel(model_id=self.get_parameter('~llm_model').get_parameter_value().string_value, flatten_messages_as_text=False,) #api_base="http://jetson:11434/v1")
        self.agent = CodeAgent(
            tools=self.get_tools(),
            model=self.llm,
            step_callbacks=[self.add_image_to_agent_obs,]
        )
        self.img_event = Event()
        self.img_event.clear()
        self.img_sub = self.create_subscription(msg_type=Image, topic=self.get_parameter('~camera_topic').get_parameter_value().string_value, callback=self.on_camera_msg, qos_profile=10)
        self.agent_server = self.create_service(srv_type=AgentCommand, srv_name='/fbot_agent/execute_command', callback=self.agent_callback)

    def get_available_named_poses(self):
        with open(os.path.join(get_package_share_directory('fbot_agent'), 'config', 'fbot_agent_config.yaml'), 'r') as f:
            yaml_data = yaml.load(f, Loader=yaml.FullLoader)
        named_poses = list(yaml_data['/fbot_agent']['fbot_agent_node']['ros__parameters']['named_poses'].keys())
        return named_poses

    def prepare_prompt(self, command: str):
        return f"""
        You are a domestic service robot.
        The following location names are available: {self.get_available_named_poses()}
        Your task is to: {command}
        """
    
    def agent_callback(self, req: AgentCommand.Request, res: AgentCommand.Response):
        self.img_event.wait()
        final_answer = self.agent.run(
            task=self.prepare_prompt(req.command),
            images=[PIL.Image.fromarray(numpify(deepcopy(self.image_obs)))]
        )
        res.response = str(final_answer)
        return res

    def on_camera_msg(self, msg: Image):
        self.image_obs = msg
        self.img_event.set()

    def get_tools(self):
        return [
            navigate_to_pose,
            move_forward,
            rotate
        ]
    
    def add_image_to_agent_obs(self, step_log: ActionStep, agent: CodeAgent):
        self.img_event.wait()
        step_log.observations_images = [PIL.Image.fromarray(numpify(deepcopy(self.image_obs)))]
