import time
import roslibpy

current_pos = 0

def joint_state_cb(message):
    global current_pos
    current_pos = message['position']

def move_ur_joint_positions(joint_positions):
    global current_pos
    client = roslibpy.Ros(host='192.168.27.1', port=9090)  # Replace with your ROS master IP

    try:
        client.run()

        listener = roslibpy.Topic(client, '/joint_states', 'sensor_msgs/JointState')
        listener.subscribe(joint_state_cb)

        time.sleep(1)
        print(current_pos)

        action_client = roslibpy.actionlib.ActionClient(client, '/follow_joint_trajectory', 'control_msgs/FollowJointTrajectoryAction')

        # Define the goal message
        goal = {
            'trajectory': {
                'joint_names': ['joint1', 'joint2', 'joint3'],  # Replace with your joint names
                'points': [
                    {
                        'positions': joint_positions,  # Replace with your desired positions
                        'time_from_start': {'secs': 5}
                    }
                ]
            }
        }

        goal_handle = action_client.send_goal(goal)

        # Wait for the result
        while not goal_handle.has_result():
            time.sleep(1)
    finally:
        client.terminate()

if __name__ == '__main__':
    try:
        # Example joint positions to move the robot
        target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # You can modify these values

        move_ur_joint_positions(target_joint_positions)
    except KeyboardInterrupt:
        pass