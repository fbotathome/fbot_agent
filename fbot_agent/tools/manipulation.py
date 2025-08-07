from smolagents import tool
from geometry_msgs.msg import PoseStamped, Pose
from .vision import detect_object
from .others import transform_pose
from yasmin import YASMIN_LOG_INFO

from yasmin import StateMachine, Blackboard, CbState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
# from state_machine.machines import PickUpClosestObjectMachine

@tool
def pick_object(object_pos: PoseStamped) -> bool:
    """
    Allows you to pick up an object by its Pose, in the 'map' frame. 

    Args:
        object_pos (PoseStamped): A PoseStamped object, which contains the position of the object to pick up, in the 'map' frame.
    Returns:
        bool: 'True' if the object was picked up successfully, 'False' otherwise.
        Raises:
            ValueError: If the object position is invalid or not reachable.
    """
    # This is a placeholder for the actual implementation
    # In a real scenario, you would interact with a robot's gripper or similar mechanism
    if not isinstance(object_pos, PoseStamped):
        raise ValueError("Invalid object position provided. Must be a PoseStamped object.")

    if object_pos.header.frame_id != 'map':
        raise ValueError("Object position must be in the 'map' frame.")
    YASMIN_LOG_INFO(f"Picking up object at position: {object_pos.pose.position.x}, {object_pos.pose.position.y}, {object_pos.pose.position.z}")
    return True

@tool
def detect_and_pick_object(object_name: str) -> bool:
    """
    Detects an object in the scene by its label name and picks it up if detected.

    Args:
        object_name: A string with the name of the object to detect and pick.
    
    Returns:
        bool: True if the object was detected and picked, False otherwise.
    """
    sm = StateMachine(outcomes=['aborted', 'canceled', 'succeeded', 'timeout'])
    sm.add_state(
        name='PICK_OBJECT',
        state=CbState(outcomes=[SUCCEED], cb=lambda blackboard: SUCCEED), #PickUpClosestObjectMachine(announce_object=True, detection_filter_list=[object_name], detection_filter_by_label=True),
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