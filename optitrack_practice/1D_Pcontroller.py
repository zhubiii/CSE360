import sys
from math import *
import time
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
    clientAddress = "192.168.0.22" # PUT IN YOUR IP HERE
    optitrackServerAddress = "192.168.0.4"
    robot_id = 7 # PUT IN YOUR ROBOT ID HERE

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

    # x desired
    x_des = 2 # PUT IN YOUR DESIRED LOCATION HERE

    # Bound is max pwm input
    bound = 1500

    # P-Controller gain
    kp  = 1000

    # keep track of time
    t = 0.
    while is_running:
        try:
            if robot_id in positions:
                x_des = 2*cos(t) + 5
                # P control for x
                errx = x_des - positions[robot_id][0]
                print(errx)
                u           = kp*errx
                if u > bound: u = bound
                if u < -bound: u = -bound

                # Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u, u, u, u)
                print(command)
                s.send(command.encode('utf-8'))

                time.sleep(.1)
                t += .1
        except KeyboardInterrupt:
            # STOP
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))

            # Close the connection
            s.shutdown(2)
            s.close()