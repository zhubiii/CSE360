from math import *
import socket
import time
import numpy as np
import random
from turtle import position
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1


robot_ip_address = '192.168.0.207'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((robot_ip_address, 5000))
print('Connected')

positions = {}
rotations = {}


"""
Function to get ultrasonic distance

PARAMS:
======
px, py, theta

Returns distance in CM

We also write the pose&distance measurements to file here
"""
def detectObstacle(py, px, theta):
    # Stop motors if we take ultrasonic
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

    print('Request ultrasound')
    # Send command for ultrasonic sensor
    command = 'CMD_SONIC#1\n'
    s.send(command.encode('utf-8'))

    # Receive and print data 1024 bytes at a time, as long as the client is sending something
    data = s.recv(1024).decode('utf-8')
    # print('Data: ', data)
    try:
        distance = int(data.split("#")[1])/100 # Divide by 100 for CM
        print("Distance =", distance)
        # Turn off Ultrasonic
        command = 'CMD_SONIC#0\n'
        s.send(command.encode('utf-8'))
        # Write to file
        with open('data.txt', 'a') as f:
            f.write(str(px)+':'+str(py)+':'+str(theta)+':'+str(distance)+'\n')
        return distance
    except:
        print('Error with data', data)
        # Turn off Ultrasonic
        command = 'CMD_SONIC#0\n'
        s.send(command.encode('utf-8'))
        return None

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
    K = 1.5

    # Radius that determines if we are at waypoint
    radius = .3

    # Work Space
    xMax = 5.5
    xMin = -4.3
    yMax = 3.5
    yMin = -3
    #waypoints = [(5.33,3.58),(-4.27,3.357),(-4.27,-3), (5.458,-3.036)]
    x = random.uniform(xMin, xMax)
    y = random.uniform(yMin, yMax)

    distance = None
    t = 0
    while is_running:
        try:
            if robot_id in positions:
                # eliminate floating point imprecision
                t = round(t,2)
                # Position and rotation of robot
                px = positions[robot_id][0]
                py = positions[robot_id][1]
                theta = rotations[robot_id] * (pi/180) # in radians

                ### DETECT OBSTACLE
                if t%2 == 0:
                    distance = detectObstacle(px, py, theta)
                if distance is not None:
                    # the obstacle will always be detected along the x axis
                    # convert the distance w.r.t. the world frame
                    bx = px + distance*cos(theta)
                    by = py + distance*sin(theta)
                    b  = (bx, by)
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

                # Generate new goal if we get to the old one
                if abs(errx) < radius and abs(erry) < radius:
                    x = random.uniform(xMin, xMax)
                    y = random.uniform(yMin, yMax)

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

                # Write to file in this order:
                # px:py:theta:distance
                # Wait for 1 second
                time.sleep(.1)
                t += .1
        except KeyboardInterrupt:
            # STOP
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))

            # Close the connection
            s.shutdown(2)
            s.close()
    