from numpy import *
import matplotlib.pyplot as plt

t 	= ''
times 	= []
inputs 	= []
with open('teleopLog.txt') as f:
	for line in f:
		# extract the time
		for i in range(6,len(line)):
			if line[i] == ':':
				break
			else:
				t = t+line[i]
		times.append(float(t))
		t = ''

		# extract input
		if line[len(line)-4] == '5':
			# check if turning left or not
			if line[len(line)-6] == '-':
				inputs.append(-1)
			else:
				inputs.append(1)
		else:
			# otherwise it is stopped
			inputs.append(0)
		
dt = []
for i in range(1, len(times)):
	dt.append(times[i] - times[i-1])

'''
lets assume that:
- (1500,1500,1500,1500) is .1 m/s
- (1500,1500,-1500,-1500) is pi/4 rad/s CCW
robot starts at (0,0) with orientation facing the positive x axis
'''

x = 0.
y = 0.
pose = array([x,y])
pose_log = [copy(pose)]
theta = 0
w = 7*pi/6
v = .1
for t, i in zip(dt, range(1,len(dt))):
	if inputs[i] == 1:
		x += t*v*cos(theta)
		y += t*v*sin(theta)
	elif inputs[i] == -1:
		theta += t*(w)
	else:
		x = pose[0]
		y = pose[1]
	pose = array([x,y])
	pose_log.append(copy(pose))

pose_log = array(pose_log)
plt.grid()
plt.plot(pose_log[:,0], pose_log[:,1])
plt.show()
