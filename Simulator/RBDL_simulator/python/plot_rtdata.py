from __future__ import division

import time, random, warnings
import math
from matplotlib import pyplot as plt
from collections import deque
from itertools import izip

warnings.filterwarnings("ignore",".*GUI is implemented.*")

RANDOM = 0
UDP    = 1
# data_source = RANDOM
data_source = UDP
# NUM_JOINTS = 3
NUM_JOINTS = 2

# Time is first. 
# spec format:    ('Figure Title', num_subplots, ['name1','name2','name3'])
#                                                 'name1', 'name2' and 'name3' are plotted on the same subplot
# UDP format:  Q[0, 'actual'], Q[1, 'actual'], ... , Q[NUM_JOINTS, 'actual'],
#              Q[0,'desired'], Q[1,'desired'], ... , Q[NUM_JOINTS,'desired']

# plot_list = [('Q'     , NUM_JOINTS ,['actual','desired'], 'trajectory'),
#              ('Qdot'  , NUM_JOINTS ,['actual','desired'], 'trajectory'),
#              ('EEPos' , 1, ['actual','desired'], 'path')]
plot_list = [('Q_Qdot'     , NUM_JOINTS ,['actual','velocity'], 'trajectory'),
             ('Jeff_JPosCMD'  , NUM_JOINTS ,['eff','jpos_des'], 'trajectory'),
             ('JVelCMD_JEffCMD'  , NUM_JOINTS ,['eff','jpos_des'], 'trajectory'),
             ('current_temp'  , NUM_JOINTS ,['eff','jpos_des'], 'trajectory')]

# plot_list = [('Q'     , NUM_JOINTS ,['actual','desired']),
#              ('Qdot'  , NUM_JOINTS ,['actual','desired'])]




class RealtimePlot:
    def __init__(self, plot_spec, max_entries = 20):
        self.fig_title, self.num_subplots, self.channel_list, self.data_type = plot_spec  # unpack plot_spec

        linespec_list = ['r-','b--','g:']  # set linespec styles for each channel

        if self.data_type == 'trajectory':
            self.fig, self.axes = plt.subplots( self.num_subplots , sharex=True )
            self.axes[0].set_title(self.fig_title)
            self.x_data = deque(maxlen=max_entries)
            self.y_data = []
            self.lines = []
            for ch, _ in enumerate(self.channel_list):
                self.y_data.append( [ deque(maxlen=max_entries)  for _  in range(self.num_subplots) ] )
                self.lines.append(  [ (self.axes[sp].plot([], [], linespec_list[ch % len(linespec_list)] ) )[0] for sp in range(self.num_subplots) ] )
                legend_handles = [ line_ch[0] for line_ch in self.lines ]
                plt.legend(bbox_to_anchor=(0.,-0.42, 1., .102), handles=legend_handles, labels=self.channel_list, loc=3, ncol=len(self.channel_list), mode="expand", borderaxespad=0.)
                (axes.set_autoscaley_on(True) for axes in self.axes)

        elif self.data_type == 'path':
            self.fig = plt.figure()
            self.axes = self.fig.add_subplot(111, autoscale_on=False, xlim=(-0.5, 0.5), ylim=(0.4, 1.4))
            self.axes.grid()
            self.axes.set_aspect('equal')
            self.axes.set_xlabel('X')
            self.axes.set_ylabel('Y')
            self.axes.set_title(self.fig_title)
            self.x_data = []
            self.y_data = []
            self.lines = []

            for ch, _ in enumerate(self.channel_list):
                self.x_data.append( [ deque(maxlen=max_entries) ])
                self.y_data.append( [ deque(maxlen=max_entries) ])
                line, = self.axes.plot([], [], linespec_list[ch % len(linespec_list)])
                self.lines.append([line])

    def add(self, x, y_list):
        self.x_data.append(x)

        y = iter(y_list)
        for y_data_ch, lines_ch in izip(self.y_data, self.lines):
            for y_data, line in izip(y_data_ch, lines_ch):
                y_data.append( y.next() )
                line.set_data(self.x_data, y_data)

        self.axes[0].set_xlim(self.x_data[0], self.x_data[-1] + 1e-15)
        for axes in self.axes: # rescale the y-axis
            axes.relim();  axes.autoscale_view()

    def add_path_data(self, data_list):
        # path data
        data = iter(data_list)

        for x_data_ch, y_data_ch, lines_ch in izip(self.x_data, self.y_data, self.lines):
            for x_data, y_data, line in izip(x_data_ch, y_data_ch, lines_ch):
                x_data.append( data.next() )
                y_data.append( data.next() )
                line.set_data(x_data, y_data)

def main():

    if data_source == UDP:
        import socket
        UDP_IP = "127.0.0.1"
        UDP_PORT = 5007
        # UDP_PORT = 61124
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))

    rt_figs = [ RealtimePlot(plot_spec) for plot_spec in plot_list ]

    # declare time variables
    start = time.time()
    t = 0.0

    # declare data variables
    num_list = []
    st_idx = 0

    while True:
        # print('wait for data')
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        # print('get data')
        # num_list = [ float(x) for x in data.split(', ')]
        num_list = [ float(x) for x in data.split(', ') if x!='']
        # print(num_list)

        t = num_list.pop(0) # time is stored in first element of list
        st_idx = 0

        for fig, plot_spec in izip( rt_figs , plot_list ):
            _, num_subplots, channel_list, data_type = plot_spec

            if data_type == 'trajectory':
                end_idx = st_idx + num_subplots*len(channel_list)
                y_list = num_list[st_idx:end_idx]
                fig.add(t, y_list)
            elif data_type == 'path':
                end_idx = st_idx + 2 * len(channel_list)
                data_list = num_list[st_idx:end_idx]
                fig.add_path_data(data_list)

            st_idx = end_idx

        plt.pause(0.0001)

if __name__ == "__main__": main()
