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
    clientAddress = "192.168.0.13"
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

    # x,y, rotation
    xy_des = (3.8,0)
    waypoints = [(5.33,3.58),(-4.27,3.357),(-4.27,-3), (5.458,-3.036)]
    idx=2
    # radius around waypoint
    radius = 1

    # P-Controller gain
    k_px = 400
    k_py = 400
    k_pr = 10
    k_v  = 1

    # L is length of space between left and right wheels
    L = 1 #meters
    # r is radius of wheel
    t = 0.
    # Bound is max pwm input
    bound = 1500
    while is_running:
        try:
            if robot_id in positions:
                i = idx%4
                ## Lap around the track
                if abs(waypoints[i][0]-positions[robot_id][0])<radius and abs(waypoints[i][1]-positions[robot_id][1])<radius:
                    print('HIT WAYPOINT')
                    idx+=1
                ## P control for x
                errx = waypoints[i][0] - positions[robot_id][0]
                ## P control for y
                erry = waypoints[i][1] - positions[robot_id][1]
                ## P control for rotation
                errr = degrees(atan2(erry, errx))
                print(errr)

                ## Circular Trajectory
                #x = cos(t)
                #y = sin(t)
                #print('(%f,%f)'%(x,y))
                ## P control for x
                #errx = x - positions[robot_id][0]
                ## P control for y
                #erry = y - positions[robot_id][1]
                ## P control for rotation
                #errr = degrees(atan2(erry, errx))


                # Position Control
                ## P control for x
                #errx = xy_des[0] - positions[robot_id][0]
                ## P control for y
                #erry = xy_des[1] - positions[robot_id][1]
                ## P control for rotation
                #errr = degrees(atan2(erry, errx))

                ux = k_px*errx
                uy = k_py*erry
                ur = k_pr*errr

                velocity            = k_v*abs(ux + uy)
                velocity_right      = (velocity + (ur))
                velocity_left       = (velocity - (ur))
                # set bound of motor input
                if velocity_left < 0:
                    velocity_left = max(int(velocity_left), -bound)
                else:
                    velocity_left = min(int(velocity_left), bound)
                if velocity_right < 0:
                    velocity_right = max(int(velocity_right), -bound)
                else:
                    velocity_right = min(int(velocity_right), bound)

                # Send control input to the motors
                command = 'CMD_MOTOR#'+str(velocity_left) \
                            +'#'+str(velocity_left)+'#'   \
                            +str(velocity_right)+'#'      \
                            +str(velocity_right)+'\n'
                print(command)
                s.send(command.encode('utf-8'))

                time.sleep(.5)
                t += .5
        except KeyboardInterrupt:
            # STOP
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))

            # Close the connection
            s.shutdown(2)
            s.close()