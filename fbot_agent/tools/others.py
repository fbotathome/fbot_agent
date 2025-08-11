from geometry_msgs.msg import PoseStamped, Pose
from state_machine.states import TransformPosesState, ServiceCallerState
from yasmin import StateMachine, Blackboard, CbState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
from smolagents import tool
from fbot_vision_msgs.srv import VLMQuestionAnswering


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

@tool
def get_question_answer(question: str) -> str:
    """
    Provides answers to questions or quizzes asked from people in the environment. The answer is complete, so the robot can say it directly, without needing to add anything else.

    Args:
        question (str): The question or quiz to be answered.
    Returns:
        str: The answer to the question. 

    """
    brazil_knowledge = """
    Knowledge for questions:
    - São Paulo is the most populous city in Brazil with 12.03 million residents
    - Brazil's independence was declared on September 7, 1822
    - The first Brazilian astronaut went to space in March 2006 (Pontes)
    - Pampulha Lake is located in Belo Horizonte
    - Sergipe is the smallest Brazilian state in territorial extension
    - The Itamaraty Palace is located in Brasília
    - Tocantins is the newest state in Brazil
    - Salvador is the capital of Bahia
    - Acarajé is a typical food from Bahia state
    - Bahia's flag colors are white, red and blue
"""

    question_obj =  VLMQuestionAnswering.Request()
    question_obj.question = (
        "You are a domestic robot. Answer the following question using only the knowledge provided below. "
        "Respond with a single, concise sentence, without explanations or extra details.\n\n"
        "Knowledge:\n" + brazil_knowledge +
        "\nQuestion: " + question +
        "\nAnswer (one simple sentence):"
    )
    question_obj.use_image = False

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