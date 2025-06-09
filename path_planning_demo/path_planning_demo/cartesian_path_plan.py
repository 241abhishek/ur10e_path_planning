import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from moveit_msgs.action import ExecuteTrajectory

from ur10e_mod_interfaces.srv import PlanCartesianPath

import os
from path_planning_demo.load_waypoints import load_waypoints_from_yaml

class CartesianPlanner(Node):
    def __init__(self):
        super().__init__('cartesian_planner')
        
        self.group_name = "ur10e_mod"
        self.ee_link = "tool0"

        self.latest_joint_state = None
        self.planned_trajectory = None

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.compute_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.plan_service = self.create_service(PlanCartesianPath, '/plan_cartesian_path', self.plan_service_cb, callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())

        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.execute_service = self.create_service(Trigger, '/execute_planned_trajectory', self.execute_cb, callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())

        self.get_logger().info("Waiting for /compute_cartesian_path service...")
        while not self.compute_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("...still waiting")

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
        path_request.max_step = 0.01
        path_request.jump_threshold = 0.0 
        path_request.avoid_collisions = True
        path_request.max_velocity_scaling_factor = 0.1
        path_request.max_acceleration_scaling_factor = 0.1
        path_request.header = Header(frame_id="world")
        if request.from_yaml:
            self.get_logger().info("Loading waypoints from YAML file...")
            base_dir = os.path.dirname(os.path.abspath(__file__))
            yaml_file_path = os.path.join(base_dir, '..', 'config', 'waypoints.yaml')
            waypoints = load_waypoints_from_yaml(yaml_file_path)
            path_request.waypoints = waypoints
        else:
            path_request.waypoints = request.waypoints

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

    async def execute_cb(self, request, response):
        if self.planned_trajectory is None:
            self.get_logger().warn("No trajectory to execute.")
            response.success = False
            response.message = "No trajectory planned."
            return response

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = self.planned_trajectory

        self.get_logger().info("Waiting for execute_trajectory action server...")
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory action server not available.")
            response.success = False
            response.message = "Action server not available."
            return response

        self.get_logger().info("Sending trajectory for execution...")
        future = self.execute_client.send_goal_async(goal_msg)
        goal_handle = await future

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            response.success = False
            response.message = "Goal rejected."
            return response
        else:
            self.get_logger().info("Goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        result_response = await result_future
        result = result_response.result

        if result.error_code.val == 1:
            self.get_logger().info("Trajectory executed successfully.")
            response.success = True
            response.message = "Execution successful."
        else:
            self.get_logger().error(f"Execution failed with error code: {result.error_code.val}")
            response.success = False
            response.message = "Execution failed."

        return response

def main(args=None):
    rclpy.init(args=args)
    cartesian_planner = CartesianPlanner()
    
    rclpy.spin(cartesian_planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()