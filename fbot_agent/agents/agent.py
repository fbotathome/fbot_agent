import PIL.Image
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from state_machine.states.navigation.navigate_to_pose import NavigateToPoseState
from yasmin import StateMachine, Blackboard
from smolagents import CodeAgent, tool, LiteLLMModel, ActionStep
import PIL
from ros2_numpy import numpify
from threading import Event
from copy import deepcopy
from fbot_agent_msgs.srv import AgentCommand

class AgentNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.declare_parameters('~named_poses')
        self.declare_parameter('~llm_model', 'ollama/gemma3')
        self.declare_parameter('~camera_topic', '/camera/color/image_raw')
        self.llm = LiteLLMModel(model_id=self.get_parameter('~llm_model').get_parameter_value().string_value)
        self.agent = CodeAgent(
            tools=self.get_tools(),
            model=self.llm,
            step_callbacks=[self.add_image_to_agent_obs,]
        )
        self.img_event = Event()
        self.img_event.clear()
        self.img_sub = self.create_subscription(msg_type=Image, topic=self.get_parameter('~camera_topic').get_parameter_value().string_value, callback=self.on_camera_msg)
        self.agent_server = self.create_service(srv_type=AgentCommand, srv_name='/fbot_agent/execute_command', callback=self.agent_callback)

    def get_available_named_poses(self):
        return self.get_parameters_by_prefix(prefix='~named_poses')

    def prepare_prompt(self, command: str):
        return f"""
        The following named poses are available: {self.get_available_named_poses()}
        Your task is to: {command}
        """
    
    def agent_callback(self, req: AgentCommand.Request):
        final_answer = self.agent.run(
            task=self.prepare_prompt(req.command)
        ).dict['final_answer']
        return AgentCommand.Response(response=final_answer)

    def on_camera_msg(self, msg: Image):
        self.image_obs = msg
        self.img_event.set()

    def get_tools(self):
        return [
            self.query_pose_by_name,
            self.navigate_to_pose
        ]
    
    def add_image_to_agent_obs(self, step_log: ActionStep, agent: CodeAgent):
        self.img_event.wait()
        step_log.observations_images = [PIL.Image.fromarray(numpify(deepcopy(self.image_obs)))]

    @tool
    def query_pose_by_name(self, name: str)->PoseStamped:
        """
        Allows you to get the pose associated with a given name. Returns the PoseStamped associated with the name.

        Args:
            name: A string with the name of the pose to query.
        """
        try:
            ps = self.get_parameter(f'~named_poses/{name}').get_parameter_value().double_array_value()
            assert len(ps) == 6
        except:
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
    def navigate_to_pose(self, pose: PoseStamped)->bool:
        """
        Allows you to navigate to a given pose. Returns 'True' if navigation is successfull, 'False' otherwise.

        Args:
            pose: A PoseStamped message with the pose to navigate to.
        """
        sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        sm.add_state(
            name='NAV_TO_POSE',
            state=NavigateToPoseState(),
            transitions={
                'succeeded': 'succeeded',
                'aborted': 'aborted',
                'preempted': 'preempted'
            }
        )
        blackboard = Blackboard(init={
            'pose_from_target': pose
        })
        outcome = sm.execute(blackboard=blackboard)
        return outcome == 'succeeded'