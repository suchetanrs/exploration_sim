import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# Declare global variables
goal_done_flag = True

def main():
    global goal_done_flag
    rclpy.init()
    node = rclpy.create_node('action_client_node')
    posex = 0.0
    posey = 0.0

    # Create an action client for the NavigateToPose action
    action_client = ActionClient(node, NavigateToPose, '/scout_1/navigate_to_pose')

    # Wait for the action server to be available
    if not action_client.wait_for_server(timeout_sec=20.0):
        node.get_logger().info('Action server not available')
        return
    
    def goal_done_callback(future):
        global goal_done_flag
        node.get_logger().info('Goal done :) ')
        goal_done_flag = True
        # print(threading.get_ident())

    # Define a callback function to send poses after receiving responses
    def goal_response_callback(future):
        global goal_done_flag
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().info('Goal rejected :(')
            return
        node.get_logger().info('Goal accepted :)')
        # print(threading.get_ident())
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(goal_done_callback)


    goal_done_flag = True
    # Send a goal to the action server
    while(True):
        if(goal_done_flag == True):
            goal_done_flag = False
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = posex
            goal_msg.pose.pose.position.y = posey
            goal_msg.behavior_tree = "/scout_2"
            future_goal = action_client.send_goal_async(goal_msg, feedback_callback=None)
            # future_goal.add_done_callback(goal_response_callback)
            future_goal.add_done_callback(goal_response_callback)

            # rclpy.spin_until_future_complete(node, future_goal)
            posey = posey + 0.8
            if(posey > 45.0):
                posey = 0.0
                posex = posex + 1.0
            if(posex > 60.0):
                break
        rclpy.spin_once(node)
        

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
