import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState

from ur10e_mod_interfaces.srv import PlanCartesianPath

class CartesianPlanner(Node):
    def __init__(self):
        super().__init__('cartesian_planner')
        
        self.group_name = "ur10e_mod"
        self.ee_link = "tool0"

        self.latest_joint_state = None

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.compute_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.plan_service = self.create_service(PlanCartesianPath, '/plan_cartesian_path', self.plan_service_cb, callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())

        self.get_logger().info("Waiting for /compute_cartesian_path service...")
        while not self.compute_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("...still waiting")

        self.planned_trajectory = None
        self.get_logger().info("Cartesian planner node ready.")

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    async def plan_service_cb(self, request, response):
        if self.latest_joint_state is None:
            self.get_logger().warn("No joint state received yet.")
            return response

        path_request = GetCartesianPath.Request()
        path_request.group_name = self.group_name
        path_request.link_name = self.ee_link
        path_request.start_state = RobotState(joint_state=self.latest_joint_state)
        path_request.waypoints = request.waypoints
        path_request.max_step = 0.01
        path_request.jump_threshold = 0.0 
        path_request.avoid_collisions = True
        path_request.max_velocity_scaling_factor = 0.1
        path_request.max_acceleration_scaling_factor = 0.1
        path_request.header = Header(frame_id="world")

        result = None
        result = await self.compute_path_client.call_async(path_request)
        self.get_logger().info("Received response from /compute_cartesian_path service.")
        if result:
            self.planned_trajectory = result.solution
            self.get_logger().info(f"Computed path with fraction: {result.fraction * 100:.2f}%")
            self.get_logger().info("Path computed successfully.")
            response.success = True
        else:
            self.get_logger().error("Failed to compute path.")
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    cartesian_planner = CartesianPlanner()
    
    rclpy.spin(cartesian_planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()