from geometry_msgs.msg import PoseStamped, Pose
from state_machine.states.others.transform_poses import TransformPosesState
from yasmin import StateMachine, Blackboard, CbState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
from smolagents import tool

@tool
def transform_pose(pose: Pose|PoseStamped, target_frame: str, source_frame: str = '') -> PoseStamped:
    """
    Transforms a given PoseStamped to a specified target frame.
    Args:
        pose (Pose|PoseStamped): The input pose to be transformed. It can be either a Pose or a PoseStamped object. Case a Pose is provided, source_frame has to be provided, because it will be considered as the frame of the pose.
        target_frame (str): The name of the frame to which the pose should be transformed.
        source_frame (str): The frame of the input pose. If the input is a PoseStamped, this parameter is ignored.
    Returns:
        PoseStamped: A new PoseStamped message with the pose transformed to the target frame.
    Raises:
        KeyError: If the transformation does not produce a result in the blackboard.
        Exception: If the state machine execution fails.

    """

    if isinstance(pose, PoseStamped):
        source_frame = pose.header.frame_id
    elif isinstance(pose, Pose):
        if not source_frame:
            raise ValueError("Source frame must be provided when a Pose is given.")
    else:
        raise ValueError("Invalid pose provided. Must be a PoseStamped object.")
    
    if not isinstance(target_frame, str):
        raise ValueError("Invalid target frame provided. Must be a string.")

    new_ps = PoseStamped()
    new_ps.header.frame_id = target_frame
    

    if source_frame == target_frame:
        new_ps.pose = pose.pose if isinstance(pose, PoseStamped) else pose
    else:

        sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
        sm.add_state(
            name='TRANSFORM_POSE',
            state=TransformPosesState(
                source_frame=source_frame,
                target_frame=target_frame,
                poses_to_transform=[pose.pose] if isinstance(pose, PoseStamped) else [pose]
            )
        )
        blackboard = Blackboard()
        try:
            sm.execute(blackboard=blackboard)
        except Exception as e:
            raise RuntimeError('State machine execution failed: ' + str(e))
    
        try:
            new_ps.pose = blackboard['transformed_poses'][0]
        except KeyError:
            raise KeyError('Transformation did not produce a result.')
    return new_ps