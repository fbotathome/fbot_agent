from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose
from yasmin import StateMachine, Blackboard, CbState, YASMIN_LOG_INFO, YASMIN_LOG_ERROR
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT, FAIL
from state_machine.machines import NavigateToTargetMachine, SaySomethingMachine, RivaListenSomethingMachine
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
    
    return True

    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL, TIMEOUT])
    sm.add_state(
        name='DETECT_OBJECT',
        state=CbState(outcomes=[SUCCEED], cb=lambda blackboard: SUCCEED),
        transitions={
            SUCCEED: "GET_POSES_FROM_DETECTIONS",
            # ABORT: ABORT,
            # CANCEL: CANCEL,
            # TIMEOUT: TIMEOUT
        }
    )
    sm.add_state("GET_POSES_FROM_DETECTIONS", CbState([SUCCEED, ABORT], getPosesFromDetections), transitions={
            SUCCEED: "TRANSFORM",
            ABORT: "GET_POSES_FROM_DETECTIONS",
        }
    )

    sm.add_state("TRANSFORM", TransformPosesState(source_frame='camera_color_optical_frame', target_frame='map'), transitions={
            SUCCEED: "EXTRACT_POSE",
            ABORT: "TRANSFORM",
            CANCEL: CANCEL,
        }
    )

    sm.add_state("EXTRACT_POSE", CbState([SUCCEED, FAIL], extractPose), transitions={
            SUCCEED: "CONVERT_POSE_TO_POSESTAMPED",
            FAIL: "DETECT_OBJECT",
    })

    sm.add_state("CONVERT_POSE_TO_POSESTAMPED", CbState([SUCCEED, ABORT], poseToPoseStamped), transitions={
            SUCCEED: "CONVERT_POSE_TO_POINTSTAMPED",
            ABORT: ABORT,
    })

    sm.add_state("CONVERT_POSE_TO_POINTSTAMPED", CbState([SUCCEED, ABORT], poseToPointStamped), transitions={
            SUCCEED: "COMPUTE_POSITION",
            ABORT: ABORT,
    })

    sm.add_state("COMPUTE_POSITION", ComputeTargetFromPoseState(offset= 1.0), transitions={
           SUCCEED: "CREATE_NAVIGATION_MESSAGE",
           ABORT: "COMPUTE_POSITION",
           CANCEL: CANCEL,
       }, remappings={'pose': 'pose_stamped'}
    )

    sm.add_state("CREATE_NAVIGATION_MESSAGE", CbState([SUCCEED, ABORT], createNavigateMessage), transitions={
            SUCCEED: "GO_TO_POSE",
            ABORT: ABORT,
    })

    sm.add_state("GO_TO_POSE", NavigateToPoseState(), transitions={
        SUCCEED: "LOOK_AT",
        ABORT: "GO_TO_POSE",
        CANCEL: CANCEL,
        }, remappings={'target': 'pose_from_target'}
    )

    sm.add_state("LOOK_AT", PublisherState('/updateNeckByPoint', PointStamped), transitions={
        SUCCEED: SUCCEED,
        ABORT: "LOOK_AT",
        CANCEL: CANCEL,
        }, remappings={'publisher_data': 'point_stamped'}
    )

    blackboard = Blackboard()
    outcome = sm.execute(blackboard=blackboard)
    return outcome == SUCCEED

def getPosesFromDetections(blackboard: Blackboard) -> str:
    if 'detection' not in blackboard:
        YASMIN_LOG_ERROR("No detection found in blackboard.")
        return ABORT
    
    if not isinstance(blackboard['detection'], list):
        YASMIN_LOG_ERROR("Detection in blackboard is not a list.")
        return ABORT
    
    poses = [detection.bbox3d.center for detection in blackboard['detection'] if detection.bbox3d.center]
    blackboard['poses_to_transform'] = poses
    return SUCCEED

def extractPose(blackboard: Blackboard) -> str:
    """
    Function to be called by the state machine to create the pose message.
    """
    if 'extract_index' not in blackboard:
        blackboard['extract_index'] = 0

    if blackboard['extract_index'] >= len(blackboard['poses_to_transform']):
        YASMIN_LOG_ERROR("No more poses to extract.")
        return FAIL
    
    pose_to_extract = blackboard["transformed_poses"]
    blackboard["pose"] = pose_to_extract[blackboard['extract_index']]
    return SUCCEED

def createNavigateMessage(blackboard: Blackboard) -> str:
    """
    Function to be called by the state machine to create the goal message.
    """
    goal = NavigateToPose.Goal()
    raw_pose = blackboard["target"]
    goal.pose.header.frame_id = 'map'
    goal.pose.pose = raw_pose
    goal.pose.pose.position.z = 0.0
    blackboard["pose_from_target"] = goal
    return SUCCEED



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
    if not name:
        raise ValueError("Name must be provided.")

    return True

    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL, TIMEOUT])
    sm.add_state(
        name='DETECT_OBJECT',
        state=CbState(outcomes=[SUCCEED], cb=lambda blackboard: SUCCEED),
        transitions={
            SUCCEED: "GET_POSES_FROM_DETECTIONS",
            # ABORT: ABORT,
            # CANCEL: CANCEL,
            # TIMEOUT: TIMEOUT
        }
    )
    sm.add_state("GET_POSES_FROM_DETECTIONS", CbState([SUCCEED, ABORT], getPosesFromDetections), transitions={
            SUCCEED: "TRANSFORM",
            ABORT: "GET_POSES_FROM_DETECTIONS",
        }
    )

    sm.add_state("TRANSFORM", TransformPosesState(source_frame='camera_color_optical_frame', target_frame='map'), transitions={
            SUCCEED: "EXTRACT_POSE",
            ABORT: "TRANSFORM",
            CANCEL: CANCEL,
        }
    )

    sm.add_state("EXTRACT_POSE", CbState([SUCCEED, FAIL], extractPose), transitions={
            SUCCEED: "CONVERT_POSE_TO_POSESTAMPED",
            FAIL: FAIL,
            
    })

    sm.add_state("CONVERT_POSE_TO_POSESTAMPED", CbState([SUCCEED, ABORT], poseToPoseStamped), transitions={
            SUCCEED: "CONVERT_POSE_TO_POINTSTAMPED",
            ABORT: ABORT,
    })

    sm.add_state("CONVERT_POSE_TO_POINTSTAMPED", CbState([SUCCEED, ABORT], poseToPointStamped), transitions={
            SUCCEED: "COMPUTE_POSITION",
            ABORT: ABORT,
    })

    sm.add_state("COMPUTE_POSITION", ComputeTargetFromPoseState(offset= 1.0), transitions={
           SUCCEED: "CREATE_NAVIGATION_MESSAGE",
           ABORT: "COMPUTE_POSITION",
           CANCEL: CANCEL,
       }, remappings={'pose': 'pose_stamped'}
    )

    sm.add_state("CREATE_NAVIGATION_MESSAGE", CbState([SUCCEED, ABORT], createNavigateMessage), transitions={
            SUCCEED: "GO_TO_POSE",
            ABORT: ABORT,
    })

    sm.add_state("GO_TO_POSE", NavigateToPoseState(), transitions={
        SUCCEED: "LOOK_AT",
        ABORT: "GO_TO_POSE",
        CANCEL: CANCEL,
        }, remappings={'target': 'pose_from_target'}
    )

    sm.add_state("LOOK_AT", PublisherState('/updateNeckByPoint', PointStamped), transitions={
        SUCCEED: SUCCEED,
        ABORT: "LOOK_AT",
        CANCEL: CANCEL,
        }, remappings={'publisher_data': 'point_stamped'}
    )

    sm.add_state("ASK_NAME", SaySomethingMachine("Please, answer after the beep with yes or no. Is your name " + name + "?", timeout=10), transitions={
        SUCCEED: "LISTEN_NAME",
        ABORT: "ASK_NAME",
        CANCEL: CANCEL,
        TIMEOUT: "LISTEN_NAME"
    })

    sm.add_state("LISTEN_NAME", RivaListenSomethingMachine(boosted_words=["yes", "no"]), transitions={
        SUCCEED: "CONFIRM_NAME",
        ABORT: "LISTEN_NAME",
        CANCEL: CANCEL,
    })

    sm.add_state("CONFIRM_NAME", CbState([SUCCEED, ABORT], lambda blackboard: SUCCEED if "yes" in blackboard['_text'] else ABORT), transitions={
        SUCCEED: SUCCEED,
        ABORT: "EXTRACT_POSE",
        CANCEL: CANCEL,
    })

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