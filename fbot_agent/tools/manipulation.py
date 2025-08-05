from smolagents import tool
from geometry_msgs.msg import PoseStamped, Pose

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
    print(f"Picking up object at position: {object_pos.pose.position.x}, {object_pos.pose.position.y}, {object_pos.pose.position.z}")
    return True