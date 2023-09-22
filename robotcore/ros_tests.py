import roslibpy
from math import pi

client = roslibpy.Ros(host='192.168.27.1', port=9090)

try:
    client.run()
    subscriber = roslibpy.Topic(client, '/joint_states', 'sensor_msgs/JointState')  # Joint state subscriber
    subscriber.subscribe(lambda message: print(message))  # subscribe to joint_states and print out the message

    control_topic = roslibpy.Topic(client, '/scaled_pos_joint_traj_controller/command',
                                   'trajectory_msgs/JointTrajectory')
    control_topic.advertise()

    # Create a JointTrajectory message
    joint_trajectory_msg = {
        'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
        'points': [{
            'positions': [1.57, -1.57, pi/4, -1.57, 0.0, 0.0],
            'time_from_start': {'secs': 5, 'nsecs': 0}
            # Time for reaching the desired position, which is 5 seconds in this case
        }]
    }
    # -------------------------------------------------------------
    control_topic.publish(roslibpy.Message(joint_trajectory_msg))
    # -------------------------------------------------------------
    while True:
        pass
finally:
    pass
    client.terminate()
