import rclpy
import time
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.planning_scene import PlanningScene

def main():
    rclpy.init()

    # Create MoveItPy interface
    panda = MoveItPy(node_name="moveit_py")

    # Select Panda arm planning group
    arm = panda.get_planning_component("panda_arm")

    # Start planning from the current robot configuration
    time.sleep(2.0)
    arm.set_start_state_to_current_state()

    # Set velocity and acceleration scaling
    plan_request_params = PlanRequestParameters(panda, "panda_arm")
    plan_request_params.planning_pipeline = "ompl"
    plan_request_params.max_velocity_scaling_factor = 0.5
    plan_request_params.max_acceleration_scaling_factor = 0.5

    # Create end-effector goal pose
    goal = PoseStamped()

    # Goal is defined relative to Panda base
    goal.header.frame_id = "panda_link0"

    # End-effector orientation
    goal.pose.orientation.w = 1.0

    # End-effector target position
    goal.pose.position.x = 0.5
    goal.pose.position.y = -0.20
    goal.pose.position.z = 0.50

    # panda_link8 should reach this pose
    arm.set_goal_state(
        pose_stamped_msg=goal,
        pose_link="panda_link8",
    )
    print("Planning...")
    # Plan trajectory
    result = arm.plan(single_plan_parameters=plan_request_params)

    if result:
        print("Plan found. Executing...")
        # Execute trajectory
        panda.execute(
            result.trajectory,
            controllers=[]
        )
        print("Finished.")
    else:
        print("Planning failed.")

    rclpy.shutdown()
if __name__ == "__main__":
    main()
