import yaml
from geometry_msgs.msg import Pose, Point, Quaternion
def load_waypoints_from_yaml(file_path):
    """
    Load waypoints from a YAML file.
    
    :param file_path: Path to the YAML file containing waypoints.
    :return: List of Pose objects representing the waypoints.
    """
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    waypoints = []
    for waypoint in data.get('waypoints', []):
        position = waypoint.get('position', {})
        orientation = waypoint.get('orientation', {})
        
        pose = Pose(
            position=Point(
                x=position.get('x', 0.0),
                y=position.get('y', 0.0),
                z=position.get('z', 0.0)
            ),
            orientation=Quaternion(
                x=orientation.get('x', 0.0),
                y=orientation.get('y', 0.0),
                z=orientation.get('z', 0.0),
                w=orientation.get('w', 1.0)
            )
        )
        waypoints.append(pose)
    
    return waypoints