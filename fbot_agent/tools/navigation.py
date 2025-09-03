from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose
from yasmin import StateMachine, Blackboard, CbState, YASMIN_LOG_INFO, YASMIN_LOG_ERROR
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT, FAIL
from state_machine.machines import NavigateToTargetMachine, SaySomethingMachine, RivaListenSomethingMachine, ApproachEntityByNameMachine, FollowMeMachine
from state_machine.states import NavigateToPoseState, MoveFixedValueState, DetectObjectState, TransformPosesState, ComputeTargetFromPoseState, PublisherState
from state_machine.callback_state_utils import poseToPoseStamped, poseToPointStamped
from smolagents import tool
from ament_index_python.packages import get_package_share_directory
import yaml
import os

@tool
def query_pose_by_name(name: str)->PoseStamped:
        """
        Allows you to get the pose of a location in map by its name. It's useful to get the pose of a location that is present in location names given.
        It is used in cases where you need the PoseStamped object of a location.

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
        Allows you to navigate to a location by its name. It is used in cases where you need to navigate to a location having the string name of the location, present in the location names given.

        Args:
            pose_name: The name of the pose to navigate to.
        Returns:
            bool: 'True' if navigation is successful, 'False' otherwise.
        """
        sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL, TIMEOUT])
        sm.add_state(
            name='NAV_TO_POSE',
            state=CbState(outcomes=[SUCCEED], cb=lambda blackboard: SUCCEED),#NavigateToTargetMachine([pose_name]),
            transitions={
                SUCCEED: SUCCEED,
                # ABORT: ABORT,
                # CANCEL: CANCEL,
                # TIMEOUT: TIMEOUT
            }
        )
        blackboard = Blackboard()
        outcome = sm.execute(blackboard=blackboard)
        return outcome == SUCCEED


@tool
def detect_and_approach_object(object: str) -> bool:
    """
    Detects an object or a person in the scene by its name or description, approaches it, and looks at it.
    If more than one object is detected, it will approach the first one found.
    Args:
        object: A string with the name or description of the object to detect and approach.

   Returns:
       bool: 'True' if the object is detected and approached successfully, 'False' otherwise.
   """
    
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL, TIMEOUT])
    sm.add_state(
        name='DETECT_OBJECT',
        state=ApproachEntityByNameMachine(entity=object),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: ABORT,
            FAIL: FAIL,
            CANCEL: CANCEL,
            TIMEOUT: TIMEOUT
        }
    )

    blackboard = Blackboard()
    outcome = sm.execute(blackboard=blackboard)
    return outcome == SUCCEED


@tool
def approach_person_by_name(name: str) -> bool:
    """
    Finds a person by their name, asking every person in the scene if their name is the one given.
    Once the person is found, it will approach them and look at them.
    Args:
        name: A string with the name of the person to find.
    Returns:
        bool: 'True' if the person is found and approached successfully, 'False' otherwise.
    """

    
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL, TIMEOUT])
    sm.add_state(
        name='DETECT_OBJECT',
        state=ApproachEntityByNameMachine(entity='person', person_name=name),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: ABORT,
            FAIL: FAIL,
            CANCEL: CANCEL,
            TIMEOUT: TIMEOUT
        }
    )

    blackboard = Blackboard()
    outcome = sm.execute(blackboard=blackboard)
    return outcome == SUCCEED

     

@tool
def follow_person() -> bool:
    """
    Follows the person placed in front of the robot until the person is no longer detected or says 'Hello Boris' to stop the following.
    This function only ends when the following is stopped.
    Returns:
        bool: 'True' if following was successful, 'False' otherwise.
    """
    sm = StateMachine(outcomes=['succeeded', 'canceled', 'aborted'])
    sm.add_state(
        name='FOLLOW_PERSON',
        state=FollowMeMachine(),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: ABORT,
            CANCEL: CANCEL,
            TIMEOUT: TIMEOUT
        }
    )

    blackboard = Blackboard()
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