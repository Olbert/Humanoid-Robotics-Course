import numpy as np
from math import sin, cos, sqrt
import matplotlib.pyplot as plt
import math


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y


def cart2pol(x, y):
    rho = np.sqrt(x ** 2 + y ** 2)
    phi = np.arctan2(y, x)
    return rho, phi


# set up constants

G = 9.81
radius = 2.

theta = 0.2
omega = 0

dt = 0.000001
T = 3

print('t = ', 0., '\tangle:', '{:.6f}'.format(theta), '\tspeed:', omega)


def euler(theta, omega, dt, T):
    C = sqrt(10)  # sqrt(G / radius)
    t_arr = []
    o_arr = []
    t_arr.append(theta)
    o_arr.append(omega)
    for t in np.arange(dt, T, dt):
        epsilon = C * sin(theta)
        omega += epsilon * dt
        theta += omega * dt
        t_arr.append(theta)
        o_arr.append(omega)
        # point = angle2point(theta)
        # plt.scatter(point[0], point[1])
        # plt.pause(0.05)
        # print ('t = ', t + dt, '\tangle:', '{:.6f}'.format(theta), '\tspeed:', omega)

    plt.show()
    return t_arr, o_arr


def linerized(theta, omega, dt, T):
    t_arr = []
    o_arr = []
    t_arr.append(theta)
    o_arr.append(omega)

    C = sqrt(10)  # sqrt(G / radius)
    c1 = (theta + omega / C) / 2
    c2 = (theta - omega / C) / 2

    for t in np.arange(dt, T, dt):
        theta = c1 * math.pow(math.e, C * t) + c2 * math.pow(math.e, -C * t)
        omega = c1 * C * math.pow(math.e, C * t) - c2 * C * math.pow(math.e, -C * t)
        t_arr.append(theta)
        o_arr.append(omega)

    return t_arr, o_arr


def local_linerized(theta, omega, dt, T):
    t_arr = []
    o_arr = []
    t_arr.append(theta)
    o_arr.append(omega)

    C = sqrt(10)  # sqrt(G / radius)
    a = C * cos(theta)
    b = C * (sin(theta) - theta * cos(theta))

    c1 = (b / a + theta + omega / sqrt(a)) / 2
    c2 = (b / a + theta - omega / sqrt(a)) / 2

    for t in np.arange(dt, T, dt):
        theta = c1 * math.pow(math.e, sqrt(a) * t) + c2 * math.pow(math.e, -sqrt(a) * t)
        omega = c1 * sqrt(a) * math.pow(math.e, sqrt(a) * t) - c2 * sqrt(a) * math.pow(math.e, -sqrt(a) * t)
        t_arr.append(theta)
        o_arr.append(omega)

    return t_arr, o_arr


def exp(theta, omega):
    t1, o1 = euler(theta, omega, 0.000001, T)

    t2, o2 = euler(theta, omega, 0.1, T)

    t3, o3 = linerized(theta, omega, 0.01, T)

    t4, o4 = local_linerized(theta, omega, 0.01, T)

    # time = np.arange(0., T, 0.000001)

    plt.plot(np.arange(0., T, 0.000001), t1, c="black")
    # plt.show()
    plt.plot(np.arange(0., T, 0.1), t2, c="r")
    # plt.show()
    plt.plot(np.arange(0., T, 0.01), t3, c="g")
    # plt.show()
    plt.plot(np.arange(0., T, 0.01), t4, c="b")
    plt.ylim(0, 7)
    # plt.xlim(0, 7)
    plt.legend(["Euler 10e-6s", "Euler 10e-1s", "LIPM", "LLIPM"])
    plt.xlabel("time")
    plt.ylabel("theta")
    plt.savefig("all_methods.png")
    plt.show()



exp(theta, omega)
#exp(1.2, omega)

l, theta_n = cart2pol(-1, 1)
omega_n = sqrt(1*1 + (-1* -1))

#exp(theta_n, omega_n)

t1, o1 = euler(theta_n, omega_n, 0.000001, T)
plt.plot(np.arange(0., T, 0.000001), t1, c="black")
plt.show()