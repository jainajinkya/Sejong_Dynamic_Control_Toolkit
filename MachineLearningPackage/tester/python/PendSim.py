import data_sender as udp_send
import PendDyn as dyn
import numpy as np
import math

# import mpc_eye_contact as mpc
# import serial
# import time
# import pygame
# import random
# import datetime

z_per = 3.
z_amp = 0.
z_avg = 1.

def z_func(t):
    return z_avg + z_amp * math.sin( t * math.pi / z_per )

def zdd_func(t):
    return -z_amp * pow( math.pi/z_per , 2 ) * math.sin( t * math.pi / z_per )

xd_max = 1.2
yd_max = 0.15

xp_guess = 0.5
yp_guess = 0.15

k_sag = 0.10
k_lat = 0.00


def pick_piv(com_pos,com_vel):
    global xd_max, yd_max, xp_guess, yp_guess, k_sag, k_lat
    temp_pivot = np.zeros(3)

    sag_err = abs(com_vel[0]) - xd_max
    lat_err = abs(com_vel[1]) - yd_max

    xp_guess = xp_guess + k_sag * sag_err
    yp_guess = yp_guess + k_lat * lat_err

    temp_pivot[0] = com_pos[0] + math.copysign(xp_guess,com_vel[0])
    temp_pivot[1] = com_pos[1] + math.copysign(yp_guess,com_vel[1])

    return temp_pivot

com_pos = np.array( [ 0. , 0. , z_func(0.) ] )
com_vel = np.array( [ xd_max,
                      yd_max,
                      (z_func(0.+0.001)-z_func(0.))/0.001 ] )


pivot = pick_piv(com_pos,com_vel)

msg_num = 1
data = np.empty(2 + 3*com_pos.size)

t = 0.0
model = dyn.InvPendModel(t, com_pos, com_vel, pivot, z_func, zdd_func)

while math.sqrt(com_vel.dot(com_vel)) < 8. and t < 5.:

    data[0]    = msg_num
    data[1]    = t
    data[2:5]  = com_pos
    data[5:8]  = com_vel
    data[8:11] = pivot

    udp_send.update(data)
    udp_send.send()
    msg_num += 1
    last_sent = t

    while t - last_sent < 1./30.:  t = model.stepForward()

    temp_pos = model.pos
    com_vel = model.vel

    # When pendulum crosses y axis (y==0) then change pivot location
    if np.sign(com_pos[1]*temp_pos[1]) == -1.0:
        pivot = pick_piv(temp_pos, com_vel)
        model = dyn.InvPendModel(t, temp_pos, model.vel, pivot, z_func, zdd_func)

    com_pos = temp_pos

data[0]    = -1.  # intentional shutdown exit code
data[1]    = t
data[2:5]  = com_pos
data[5:8]  = com_vel
data[8:11] = pivot

udp_send.update(data)
udp_send.send()
