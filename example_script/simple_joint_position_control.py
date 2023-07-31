import time
import roslibpy

def move_ur_joint_positions(joint_positions):
    client = roslibpy.Ros(host='150.22.0.40', port=9090)  # Replace with your ROS master IP

    try:
        client.run()

        # Create a JointTrajectory message
        joint_trajectory_msg = {
            'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            'points': [{
                'positions': joint_positions,
                'time_from_start': {'secs': 5, 'nsecs': 0}  # Time for reaching the desired position
            }]
        }

        # Publish the trajectory to the '/arm_controller/command' topic
        topic = roslibpy.Topic(client, '/scaled_pos_joint_traj_controller/command', 'trajectory_msgs/JointTrajectory')
        topic.advertise()
        topic.publish(roslibpy.Message(joint_trajectory_msg))

        # Wait for the robot to reach the desired position
        time.sleep(6)

        topic.unadvertise()

    finally:
        client.terminate()

if __name__ == '__main__':
    try:
        # Example joint positions to move the robot
        target_joint_positions = [1.57, -1.57, 0.0, -1.57, 0.0, 0.0]  # You can modify these values

        move_ur_joint_positions(target_joint_positions)
    except KeyboardInterrupt:
        pass