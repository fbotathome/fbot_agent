from smolagents import tool
from geometry_msgs.msg import PoseStamped, Pose
from fbot_vision_msgs.msg import Detection3D

@tool
def detect_object(object_name: str) -> Pose:
    """
    Allows you to detect an object in the camera area, in order to return its Pose in the 'camera' frame.

    Args:
        object_name: A string with the name of the object to detect.
    Returns:
        Pose: A Pose message containing the detected object's position, in the 'camera' frame.
        It does not return a PoseStamped, if you need a PoseStamped, you can build it using the returned Pose and the camera's frame_id.
    
    """
    # This is a placeholder for the actual implementation
    # In a real scenario, you would interact with a robot's perception system to detect objects
    print(f"Detecting object: {object_name}")
    detection = Detection3D()
    detection.header.frame_id = "camera"
    detection.bbox3d.center.position.x = 1.0
    detection.bbox3d.center.position.y = 0.0
    detection.bbox3d.center.position.z = 0.5
    detection.bbox3d.size.x = 0.1
    detection.bbox3d.size.y = 0.1
    detection.bbox3d.size.z = 0.1
    detection.label = "apple"

    return detection.bbox3d.center