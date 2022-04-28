import networkx as nx
from math import *
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
import time

# Define the vertices
vertices = [(1,[5.54,0]), (2,[3.66,0]), (3,[2.07,-1.41]), (4,[2.65,0]),
    (5,[3.3,2.23]), (6,[1.41,0]), (7,[.82,.235]), (8,[1.45,2.42]),
    (9,[.041,-1.34]), (10,[-.32,.214]), (11,[.276,.825]),(12,[.6,2.13]),
    (13,[-.82,-.24]), (14,[-2.32, 2.03]), (15,[-2.52, .45])]

# Define edges and their weights
edges = [(1,2), (1,3), (1,5), (2,3), (2,4), (2,5), (4,3), (4,5), (4,6),
        (6,7), (6,3), (5,8), (3,9), (9,10), (9,13), (7,10), (7,11),
        (8,12), (8,14), (10, 13), (10, 11), (11,14), (11, 12), (12, 14),
        (13, 15), (14, 15)]

G = nx.Graph()

# Add vertices
for v in vertices: G.add_node(v[0])
# Addd edges
G.add_edges_from(edges)

for (u,v) in G.edges:
    ux,uy = vertices[u-1][1]
    vx,vy = vertices[v-1][1]
    w     =  sqrt((ux-vx)**2 + (uy-vy)**2)
    G[u][v]['weight'] = w

# Compute shortest path
path = nx.shortest_path(G, source=6, target=13)
print(path)


'''
Have robot drive the shortest path
'''
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
    clientAddress = "192.168.0.11"
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

    # Radius for waypoints
    radius = .5

    # P-Controller gain
    k_v  = 1700
    k_pr = 1200

    #  idx for waypoints
    idx = 0

    t = 0.
    while is_running:
        try:
            if robot_id in positions:
                if idx == len(path)-1: exit(0)

                theta = rotations[robot_id] * pi/180

                # waypoint idx
                wayidx = path[idx]
                # x,y desired
                x, y  = vertices[wayidx-1][1]
                print(x,y)

                ## Lap around the track
                if abs(x-positions[robot_id][0])<radius and abs(y-positions[robot_id][1])<radius:
                    print('HIT WAYPOINT')
                    idx+=1

                # P control for x
                errx = x - positions[robot_id][0]
                # P control for y
                erry = y - positions[robot_id][1]
                #print('(%f,%f)'%(errx,erry))

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
                t += .02
        except KeyboardInterrupt:
            # STOP
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))

            # Close the connection
            s.shutdown(2)
            s.close()
