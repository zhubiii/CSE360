import math
import random

import matplotlib.pyplot as plt
import numpy as np



class GridMap():
    def __init__(self, dim, div=2, prob=.5):
        # 12x8 sq meters
        self.dim = dim
        # divisions per meter
        self.div = div
        # number of rows and columns respectively.
        self.n, self.m = (dim[0]) * div, (dim[1]) * div

        # Create a matrix to represent the cells of the grid
        self.matrix_map = np.ones((self.n, self.m))*prob

        # Random obstacles for this example.
        # np.random.seed(0)  # You can remove the random seed if you want to test different random grids.
        # self.matrix_map = np.random.rand(self.n, self.m)

    def column_to_x(self, i):
        return -self.dim[1]/2 + self.dim[1] * i/self.m

    def row_to_y(self, j):
        return -self.dim[0]/2 + self.dim[0] * j/self.n

    def point_to_indexes(self, p):
        x, y = p
        i = self.n * (y + self.dim[0]/2) / self.dim[0]
        j = self.m * (x + self.dim[1]/2) / self.dim[1]
        return i, j


def draw_grid(grid):
    fig, ax = plt.subplots()
    # cmap = colors.ListedColormap(['0.9', 'black']) # Colors (0.9 is the almost white in gray scale)
    plt.style.use('grayscale')

    # X labels
    ticks = [i - 0.5 for i in range(grid.m)] + [grid.m-0.5]
    ticks = ticks[::grid.div]
    labels = ['%.1f' % grid.column_to_x(i + 0.5) for i in ticks]
    ax.set_xticks(ticks)
    ax.set_xticklabels(labels)
    # Y labels
    ticks = [i-0.5 for i in range(grid.n)] + [grid.n-.5]
    ticks = ticks[::grid.div]
    labels = ['%.1f' % grid.row_to_y(i+0.5) for i in ticks]
    ax.set_yticks(ticks)
    ax.set_yticklabels(labels)
    ax.imshow(grid.matrix_map, origin='lower')


def draw_measurement(robot_location, robot_orientation, ultrasound_distance):
    # robot location
    i, j = grid.point_to_indexes(robot_location)
    print(i,j)

    ultrasound_point = robot_location[0] + ultrasound_distance * math.cos(robot_orientation), \
                       robot_location[1] + ultrasound_distance * math.sin(robot_orientation)

    ui, uj = grid.point_to_indexes(ultrasound_point)
    #plt.plot([j-.5, uj-.5], [i-.5, ui-.5], 'r--')

    # There is no obstable in the cell where the robot is
    grid.matrix_map[int(i), int(j)] = 1
    # plot robot location
    #plt.plot(j - .5, i - .5, 'yo')


def measurements_to_cells(robot_location, robot_orientation, ultrasound_distance, k_dis=10):
    # discretize the line into k parts
    s = np.linspace(0, ultrasound_distance, int(ultrasound_distance * k_dis), endpoint=True)
    points = [np.array(robot_location) + np.array([si * math.cos(robot_orientation), si * math.sin(robot_orientation)])
              for si in s]

    # Obtain the cells associated with the points
    cells = [tuple(np.array(grid.point_to_indexes(p)).astype(int)) for p in points]
    # remove deleted cells
    return cells


if __name__ == '__main__':
    # Grid representation of the environment. Example, an environment of 4x8 sq meters.
    grid = GridMap((14, 10), 4)

    # Begin: Create random Ultrasound measurements (random)
    measurements = []
    # Read data file for measurements
    with open('data.txt') as f:
        for line in f:
            px,py,theta,distance  = line.split(":",3)
            measurements.append(([float(px),float(py)],float(theta),float(distance)))

    #for _ in range(10):
        ## Each measurement is composed of three parts: robot location, robot orientation, and the ultrasound reading.
        #robot_location = [-grid.dim[1]/2 + random.random() * grid.dim[1], -grid.dim[0]/2 + random.random() * grid.dim[0]]
        #robot_orientation = 2 * math.pi * random.random()
        #ultrasound_distance = grid.dim[0]/2 * random.random()
        #print(ultrasound_distance)
        #measurements.append((robot_location,robot_orientation,ultrasound_distance))
    # END: Create random Ultrasound measurements (random)

    # Process measurements
    for robot_location, robot_orientation, ultrasound_distance in measurements:
        # Convert coordinates to matrix indexes or cells.
        cells = measurements_to_cells(robot_location, robot_orientation, ultrasound_distance, k_dis=10)

        # Update the matrix map
        for i, j in cells:
            # out of boundaries
            if i >= grid.n or j >= grid.m:
                continue
            # Probability of the cell to be free
            grid.matrix_map[i, j] = .6  # FIXME: update the belief based on the log odds method

            # The last cell is occupied
            if (i, j) == cells[-1]:
                grid.matrix_map[i, j] = 0.1  # FIXME: update the belief based on the log odds method

    # Plot map
    draw_grid(grid)
    # Plot measurements
    for robot_location, robot_orientation, ultrasound_distance in measurements:
        draw_measurement(robot_location, robot_orientation, ultrasound_distance)
    plt.show()
