import numpy as np
import sys
sys.path.append('../')

from matrix import T, Rx3, Ry3, Rz3, Rx4, Ry4, Rz4, R3to4, elem_vec, mat4_to_posvec_and_rotmat

def normal(R):
    ld, v = np.linalg.eig(R)
    for i, l in enumerate(ld):
        print(l.imag)
        if l.imag == 0.:
            return v[:, i].real

R = Rz3(np.radians(0)) @ Ry3(np.radians(30)) @ Rx3(np.radians(30))
ld, v = np.linalg.eig(R)
print(ld, '\n', v)

print(type(normal(R)))