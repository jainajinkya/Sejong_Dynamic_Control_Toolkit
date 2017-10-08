import socket
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
from collections import deque
# import mpl_toolkits.mplot3d.axes3d as p3

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

sim_time = 0.

##=== Simulation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-0.5, 2.5))

ax.grid()
ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_ylabel('Y')

num_line = 7
line_list = []

w = 0.45
h = 0.2
mass_center_offset = [0.15, 0.]

for i in range (0, num_line):
    line, = ax.plot([], [], 'o-', lw=2)
    line_list.append(line)

line_center, = ax.plot([], [],'o-', lw=3)

ax.plot([-1.0, 1.0], [0.0, 0.], 'k--', lw = 3)
trail, = ax.plot([], [], 'r--')
trail_x = deque(maxlen=25)
trail_y = deque(maxlen=25)

time_template = 'time = %.3f s'
time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes)

#body
rec = plt.Rectangle((5, -5), w, h, fc='y')

def init():
    for i in range (0, num_line):
        line_list[i].set_data([], [])

    time_text.set_text('')

    #body
    ax.add_patch(rec)
    return  line_list, time_text, rec

b_updated = False

def leftcorner_angle(joint, center):
    angle = np.arctan2(center[1] - joint[1], center[0] - joint[0])
    angle = angle - np.pi/2.
    leftcorner = center - w * np.array([np.cos(angle), np.sin(angle)]) * 0.5
    return leftcorner, angle

def animate(i):
    global line_list, sim_time, b_updated, trail, trail_x, trail_y

    x_list = []
    y_list = []
    z_list = []

    # Link locations
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    num_string = data.split(', ')

    for j in range(0, num_line):
        x_list.append((float(num_string[4*j]), float(num_string[4*j+1]) ) )
        y_list.append((float(num_string[4*j+2]), float(num_string[4*j+3]) ) )

    # Rectangle Drawing
    joint = np.array([x_list[0][1], y_list[0][1]])
    center = np.array([x_list[0][0], y_list[0][0]])
    leftcorner, angle = leftcorner_angle(joint, center)
    rec.xy = (leftcorner[0], leftcorner[1])
    t = mpl.transforms.Affine2D().rotate_around(leftcorner[0], leftcorner[1], angle) + ax.transData
    rec.set_transform(t)

    # Trail Drawing
    trail_x.append(x_list[-1][1])
    trail_y.append(y_list[-1][1])
    trail.set_data(trail_x,trail_y)

    sim_time = float(num_string[4*num_line])
    time_text.set_text(time_template % sim_time)

    for k in range (0, num_line):
        line_list[k].set_data(x_list[k], y_list[k])

    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-0.5, 1.5)

    b_updated = True
    return line_list, time_text, rec

ani = animation.FuncAnimation(fig, animate, np.arange(1, 800),
                              interval=5, blit=False, init_func=init)

plt.show()
