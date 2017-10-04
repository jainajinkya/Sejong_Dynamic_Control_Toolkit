import socket
import numpy as np
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.pyplot as plt

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))


# Key: (by column)
# data[:,0]    = msg_num
# data[:,1]    = t
# data[:,2:5]  = com_pos
# data[:,5:8]  = com_vel
# data[:,8:11] = pivot

data = np.empty(0)
wait = True
window = 2.

def update_lines(num, all_ax, lines):
    global data, wait, window

    if wait:
        msg, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        temp = np.fromstring(msg,dtype=float,sep=',')
        # print temp

    if data.size == 0:
        if temp[0] == 0:
            data = np.empty(0);
        if temp[0] != 0:
            print 'Warning: Missed first ', str(temp[0]-1), ' message(s)...\n'
        data = np.array([temp])
    elif wait == True:
        # if temp[0] != data[-1,0]+1 and temp[0] != -1:
        #     print 'Warning: Missed ', str(temp[0]-data[-1,0]-1), ' message(s)...\n'
        data = np.vstack((data,temp))

        all_ax[0].set_xlim3d([data[-1,2]-window/2., data[-1,2]+window/2.])

        # Plot COM position on 3D graph
        lines[0].set_data(data[:,2:4].T)
        lines[0].set_3d_properties(data[:,4])

        # Plot Pendulum on 3D graph
        lines[1].set_data(np.vstack((data[-1,8:10],data[-1,2:4])).T)
        lines[1].set_3d_properties(np.array([data[-1,10], data[-1,4]]))
        if data[-1,8] != data[-2,8]:
            if lines[1].get_color() == 'c':
                lines[1].set_color('m')
            else: lines[1].set_color('c')


        lines[2].set_data((data[:,2], data[:,5]))

        lines[3].set_data((data[:,3], data[:,6]))

    if wait==True and temp[0] == -1.: wait = False
    if wait==True and temp[0] == 0.: data = np.empty(0);

    return lines


# Attaching 3D axis to the figure
fig = plt.figure(figsize=(16,9), tight_layout=True)
all_ax = []

all_ax.append( fig.add_subplot(2,3,1, projection='3d') )

# Fifty lines of random 3-D lines
# points = [Gen_RandLine(25, 3) for index in range(50)]
points = np.empty(0)

# Can't pass empty arrays into 3d version of plot()
line_com, = all_ax[0].plot(np.zeros(1),np.zeros(1),np.zeros(1), lw = 2, color='k')


# Setting the axes properties
all_ax[0].set_xlim3d([-0.5, 0.5])
all_ax[0].set_xlabel('X')

all_ax[0].set_ylim3d([-0.5, 0.5])
all_ax[0].set_ylabel('Y')

all_ax[0].set_zlim3d([0.0, 1.1])
all_ax[0].set_zlabel('Z')

all_ax[0].set_title('3D View')

line_pend, = all_ax[0].plot(np.zeros(1),np.zeros(1),np.zeros(1), lw = 2)
line_pend.set_color('c')

all_ax.append( fig.add_subplot(2,3,2) )
all_ax[1].grid(True)
all_ax[1].set_title('X Phase Plot')
all_ax[1].set_xlim(0,5)
all_ax[1].set_ylim(0,0.7)
x_phase, = all_ax[1].plot([], [], 'r-', lw = 2)

all_ax.append( fig.add_subplot(2,3,3) )
all_ax[2].grid(True)
all_ax[2].set_title('Y Phase Plot')
all_ax[2].set_xlim(-0.5,0.5)
all_ax[2].set_ylim(-0.5,0.5)
y_phase, = all_ax[2].plot([], [], 'b-', lw = 2)

lines = [line_com, line_pend, x_phase, y_phase]



# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, 30, fargs=(all_ax, lines),
                                   interval=30., repeat=True, blit=False)

plt.show()
