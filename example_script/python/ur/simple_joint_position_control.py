import time
import roslibpy

current_pos = 0


def joint_state_cb(message):
    global current_pos
    current_pos = message['position']


def move_ur_joint_positions(joint_positions):
    global current_pos
    # Replace with your ROS master IP
    client = roslibpy.Ros(host='192.168.27.1', port=9090)

    try:
        client.run()

        listener = roslibpy.Topic(
            client, '/joint_states', 'sensor_msgs/JointState')
        listener.subscribe(joint_state_cb)

        time.sleep(1)
        print(current_pos)

        # Create a JointTrajectory message
        start_joint_trajectory_msg = {
            'joint_names': ['shoulder_1_joint', 'shoulder_2_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            'points': [{
                'positions': current_pos,
                # Time for reaching the desired position
                'time_from_start': {'secs': 0, 'nsecs': 0}
            }]
        }

        end_joint_trajectory_msg = {
            'joint_names': ['shoulder_1_joint', 'shoulder_2_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            'points': [{
                'positions': current_pos,
                # Time for reaching the desired position
                'time_from_start': {'secs': 0, 'nsecs': 0}
            }, {
                'positions': joint_positions,
                # Time for reaching the desired position
                'time_from_start': {'secs': 5, 'nsecs': 0}
            }]
        }

        trajectory = {

        }

        follow_joint_action = {
            'trajectory': end_joint_trajectory_msg
        }

        # Publish the trajectory to the '/arm_controller/command' topic
        topic = roslibpy.Topic(client, '/follow_joint_trajectory/goal',
                               'control_msgs/FollowJointTrajectoryAction')
        topic.advertise()
        topic.publish(roslibpy.Message(follow_joint_action))

        # Wait for the robot to reach the desired position
        time.sleep(6)

        topic.unadvertise()

        while True:
            pass

    finally:
        client.terminate()


if __name__ == '__main__':
    try:
        # Example joint positions to move the robot
        target_joint_positions = [0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0]  # You can modify these values

        move_ur_joint_positions(target_joint_positions)
    except KeyboardInterrupt:
        pass
