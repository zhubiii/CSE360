import sys
from math import *
import time
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
import time

IP_ADDRESS = '192.168.0.207'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz


if __name__ == "__main__":
    clientAddress = "192.168.0.22"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 7

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    # Bound is max pwm input
    bound = 1500

    # P-Controller gain
    k_v  = 2900
    k_pr = 1200

    # K determines how far we want to be from obstacle
    K = .1

    # Obstacle robot ID
    obstacle_robot_id = 4
    b = None
    
    t = 0.
    while is_running:
        try:
            if obstacle_robot_id in positions:
                b = (positions[obstacle_robot_id][0], positions[obstacle_robot_id][1])
            if robot_id in positions:
                # robots rotation
                theta = rotations[robot_id] * pi/180

                ## Circular Trajectory
                x = 5.43+(1.5*cos(t))
                y = 0.055+(1.5*sin(t))

                if b is not None:
                    # Obstacle avoidance vector
                    errpbx = positions[robot_id][0] - b[0]
                    errpby = positions[robot_id][1] - b[1]
                    # Calculate norm-3
                    pbnorm3 = sqrt(errpbx**2 + errpby**2)**3
                    # Divide by norm 3 and multiply by distance constant K
                    errpbx = (errpbx / pbnorm3) * K
                    errpby = (errpby / pbnorm3) * K

                    # P control for x
                    errx = x - positions[robot_id][0] + errpbx # added term is obstacle avoidance
                    # P control for y
                    erry = y - positions[robot_id][1] + errpby # added term is obstacle avoidance
                else:
                    # P control for x
                    errx = x - positions[robot_id][0]
                    # P control for y
                    erry = y - positions[robot_id][1]

                # P control for rotation
                alpha = atan2(erry, errx)
                errw  = degrees(atan2(sin(alpha-theta), cos(alpha-theta)))
                print(errw)
                omega = k_pr*errw

                v            = k_v*(sqrt(errx**2 + erry**2))
                u           = np.array([v-omega, v+omega])
                # set bound of motor input
                u[u > 1500] = 1500
                u[u < -1500] = -1500

                # Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                print(command)
                s.send(command.encode('utf-8'))

                time.sleep(.1)
                t += .035
        except KeyboardInterrupt:
            # STOP
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))

            # Close the connection
            s.shutdown(2)
            s.close()