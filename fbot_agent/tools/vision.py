from smolagents import tool
from geometry_msgs.msg import PoseStamped, Pose
from fbot_vision_msgs.msg import Detection3D
from state_machine.states import TransformPosesState, ServiceCallerState
from yasmin import StateMachine, Blackboard, CbState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
from yasmin import YASMIN_LOG_INFO
from .others import transform_pose
from fbot_vision_msgs.srv import VLMQuestionAnswering

TRIES = 2
current_try = 0

CAMERA_FRAME = 'camera'

@tool
def detect_object(object_name: str) -> Pose:
    """
    Allows you to detect an object in the scene giving its name or description, in order to return its Pose directly in the 'map' frame.
    The object description can be a person, a person making some gesture, a simple object like a cup, a class of objects such as 'a dish', a description of an object such as "the heavist object in the room", etc.

    Args:
        object_name: A string with the name or description of the object to detect.
    Returns:
        Pose: A Pose message containing the detected object's position, already in the 'map' frame if detection is successful.
        Returns None if the object was not detected in the scene

    """
    # This is a placeholder for the actual implementation
    # In a real scenario, you would interact with a robot's perception system to detect objects

    global current_try
    if current_try < TRIES:
        YASMIN_LOG_INFO(f"Failed to detect object {object_name} after {TRIES} attempts.")
        current_try += 1
        return None
    YASMIN_LOG_INFO(f"Detecting object: {object_name}")
    detection = Detection3D()
    detection.header.frame_id = CAMERA_FRAME
    detection.bbox3d.center.position.x = 1.0
    detection.bbox3d.center.position.y = 0.0
    detection.bbox3d.center.position.z = 0.5
    detection.bbox3d.size.x = 0.1
    detection.bbox3d.size.y = 0.1
    detection.bbox3d.size.z = 0.1
    detection.label = "apple"

    sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    sm.add_state(
        name='TRANSFORM_POSE',
        state=TransformPosesState(
            source_frame=CAMERA_FRAME,
            target_frame='map',
            poses_to_transform=[detection.bbox3d.center]
        )
    )
    blackboard = Blackboard()
    try:
        sm.execute(blackboard=blackboard)
    except Exception as e:
        raise RuntimeError('State machine execution failed: ' + str(e))

    # Transform the detection pose to the 'map' frame
    transformed_pose = blackboard['transformed_poses'][0]
    YASMIN_LOG_INFO(f"Object {object_name} detected at position {transformed_pose.position.x}, {transformed_pose.position.y}, {transformed_pose.position.z}")
    # Return the pose of the detected object

    return transformed_pose

@tool
def search_object(object_name: str) -> Pose:
    """
    Searches for an object in the scene by its name or description, returning its Pose in the 'map' frame. It tries to detect the object in different angles, moving the camera around the scene, and adjusting the view to improve detection chances.
    This function is a wrapper around detect_object to handle retries.
    The object description can be a person, a person making some gesture, a simple object like a cup, a class of objects such as 'a dish', a description of an object such as "the heavist object in the room", etc.

    Args:
        object_name: A string with the name or description of the object to search for.
    Returns:
        Pose: A Pose message containing the detected object's position, already in the 'map' frame if detection is successful.
        Returns None if the object was not found in the scene
    """
    sm = StateMachine(outcomes=['aborted', 'canceled', 'succeeded', 'timeout'])
    sm.add_state(
        name='NAV_TO_POSE',
        state=CbState(outcomes=[SUCCEED], cb=lambda blackboard: SUCCEED),#CRIAR MACHINE QUE MOVE A CAMERA EM DIVERSOS ÂNGULOS E TENTA DETECTAR O OBJETO
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
def analyze_scene(question: str) -> str:
    """
    Analyzes the current camera image to answer questions about the visual scene.
    This tool allows you to ask specific questions about what the robot sees through its camera.
    
    Args:
        question: A string with the question about the scene (e.g., "What objects do you see?", 
                 "How many people are in the room?", "What color is the cup on the table?")
    Returns:
        str: A detailed answer about the visual scene based on the current camera image.
    """

    return 'There are two drinks in the trash bin'

    question_obj =  VLMQuestionAnswering.Request()
    question_obj.question = "You are a domestic robot. Answer the following question based on the image provided. Your answer must be a phrase that can be spoken by the robot. Here is the question: " + question
    question_obj.use_image = True

    sm = StateMachine(outcomes=['aborted', 'canceled', 'succeeded', 'timeout'])
    sm.add_state(
        name='ASK_VLM',
        state=ServiceCallerState(
            service_name='/fbot_vision/vlm/question_answering/query',
            service_type=VLMQuestionAnswering,
            data=question_obj
        ),
        transitions={
            SUCCEED: SUCCEED,
            # ABORT: ABORT,
            # CANCEL: CANCEL,
            # TIMEOUT: TIMEOUT
        }
    )
    blackboard = Blackboard()
    outcome = sm.execute(blackboard=blackboard)
    if outcome == SUCCEED:
        return blackboard['_answer']
    return None


