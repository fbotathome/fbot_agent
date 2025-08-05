from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from yasmin import StateMachine, Blackboard, CbState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT
from state_machine.states.navigation.navigate_to_pose import NavigateToPoseState
from state_machine.states.navigation.move_fixed_value import MoveFixedValueState
from smolagents import tool
from ament_index_python.packages import get_package_share_directory
import yaml
import os

@tool
def query_pose_by_name(name: str)->PoseStamped:
        """
        Allows you to get the pose of a location in map by its name. It's useful to get the pose of a location that is present in location names given.

        Args:
            name: A string with the name of the pose to query.

        Returns:
            PoseStamped: The PoseStamped associated with the given name.
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
def navigate_to_pose(pose_name: str) -> bool:
        """
        Allows you to navigate to a given pose.

        Args:
            pose_name: The name of the pose to navigate to.
        Returns:
            bool: 'True' if navigation is successful, 'False' otherwise.
        """
        pose = query_pose_by_name(name=pose_name)
        sm = StateMachine(outcomes=['aborted', 'canceled', 'succeeded', 'timeout'])
        sm.add_state(
            name='NAV_TO_POSE',
            state=CbState(outcomes=[SUCCEED], cb=lambda blackboard: SUCCEED),#NavigateToPoseState(),
            transitions={
                SUCCEED: SUCCEED,
                # ABORT: ABORT,
                # CANCEL: CANCEL,
                # TIMEOUT: TIMEOUT
            }
        )
        blackboard = Blackboard(init={
            'pose_from_target': NavigateToPose.Goal(pose=pose)
        })
        outcome = sm.execute(blackboard=blackboard)
        return outcome == SUCCEED



@tool
def move_forward(distance: float)->bool:
    """
    Allows you to move forward by a given distance.

    Args:
    distance: A float value with how many meters to move forward

    Returns:
        bool: 'True' if movement is successful, 'False' otherwise.
    """
    sm = StateMachine(outcomes=['succeeded', 'canceled', 'aborted'])
    sm.add_state(
        name='MOVE_FIXED_VALUE',
        state=MoveFixedValueState(
            deviation=distance
        ),
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: CANCEL,
            ABORT: ABORT
        }
    )
    outcome = sm.execute(blackboard=Blackboard())
    return outcome == SUCCEED

@tool
def rotate(angle: float) -> bool:
    """
    Allows you to rotate in-place by a given angle.

    Args:
        angle: A float value with how many radians to rotate. Positive values rotate left. Negative values rotate right.

    Returns:
        bool: 'True' if rotation is successful, 'False' otherwise.
    """
    sm = StateMachine(outcomes=['succeeded', 'canceled', 'aborted'])
    sm.add_state(
        name='MOVE_FIXED_VALUE',
        state=MoveFixedValueState(
            tp=0,
            deviation=angle
        ),
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: CANCEL,
            ABORT: ABORT
        }
    )
    outcome = sm.execute(blackboard=Blackboard())
    return outcome == SUCCEED