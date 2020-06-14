import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('../')

from  matrix import T, Rx3, Ry3, Rz3, Rx4, Ry4, Rz4, R3to4, R4to3, elem_vec, \
                mat4_to_posvec_and_rotmat, rot_normal_from_matR, rot_normal_from_radius_vecs


def func_exp(x):
    mat = Rz4(np.radians(30)) @ T([1, 0, 0]) @ Ry4(x) @ T([1.5, 0, 0])
    pos = mat @ np.array([0, 0, 0, 1])
    return np.array(mat @ pos)[:3]
    #return np.exp(-x**2)

def radius_3d(pos):
    return np.sqrt(np.sum(np.square(pos), axis=1))

if __name__ == "__main__":
    L, N = np.radians(360), 100
    x = np.linspace(-L/2, L/2, N)
    # print(x)
    psi = func_exp(x)
    psi = np.array(list(psi)).T
    print(psi)
    psi = radius_3d(psi)
    # psi = psi[2]
    print(psi)

    dx = L/N

    psi_dot_np = np.gradient(psi, dx)
    print(psi_dot_np.shape)
    psi_2dot_np = np.gradient(psi_dot_np, dx)

    # print(dx)

    plt.plot(x, psi, label="f")
    plt.plot(x, psi_dot_np, label="df_dx")
    plt.plot(x, psi_2dot_np, label="d2f_dx2")

    plt.legend()
    plt.show()