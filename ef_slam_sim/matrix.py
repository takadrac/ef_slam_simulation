import numpy as np

def T(pos):
    mtrx = np.array([
        [1, 0, 0, pos[0]],
        [0, 1, 0, pos[1]],
        [0, 0, 1, pos[2]],
        [0, 0, 0, 1]
    ])
    return mtrx

def Rx3(t):
    mtrx = np.array([
        [1, 0, 0],
        [0, np.cos(t), -np.sin(t)],
        [0, np.sin(t),  np.cos(t)],
    ])
    return mtrx

def Ry3(t):
    mtrx = np.array([
        [ np.cos(t), 0, np.sin(t)],
        [0, 1, 0],
        [-np.sin(t), 0, np.cos(t)],
    ])
    return mtrx

def Rz3(t):
    mtrx = np.array([
        [np.cos(t), -np.sin(t), 0],
        [np.sin(t),  np.cos(t), 0],
        [0, 0, 1],
    ])
    return mtrx

def R3to4(R3):
    mtrx = np.array([
        np.append(R3[0], 0),
        np.append(R3[1], 0),
        np.append(R3[2], 0),
        [0, 0, 0, 1]
    ])
    return mtrx

def Rx4(t):
    mtrx = np.array([
        [1, 0, 0, 0],
        [0, np.cos(t), -np.sin(t), 0],
        [0, np.sin(t),  np.cos(t), 0],
        [0, 0, 0, 1]
    ])
    return mtrx

def Ry4(t):
    mtrx = np.array([
        [ np.cos(t), 0, np.sin(t), 0],
        [0, 1, 0, 0],
        [-np.sin(t), 0, np.cos(t), 0],
        [0, 0, 0, 1]
    ])
    return mtrx

def Rz4(t):
    mtrx = np.array([
        [np.cos(t), -np.sin(t), 0, 0],
        [np.sin(t),  np.cos(t), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return mtrx

def R4to3(R4):
    mat = np.array([
        R4[0, :3],
        R4[1, :3],
        R4[2, :3]
    ])
    return mat

def mat4_to_posvec_and_rotmat(mat):
    posvec = np.array([mat[0, -1], mat[1, -1], mat[2, -1]])
    matR = np.array([
        mat[0, :3],
        mat[1, :3],
        mat[2, :3]
    ])
    return posvec, matR

def rot_normal_from_matR(matR):
    ld, v = np.linalg.eig(matR)
    for i, l in enumerate(ld):
        # print(v[:, i].real, l.imag)
        if l.imag == 0.:# and l.real == 1.:
            return v[:, i].real

def rot_normal_from_radius_vecs(v1, v2):
    return elem_vec(np.cross(v1, v2))

def elem_vec(vec):
    evec = vec / np.linalg.norm(vec)
    return evec

if __name__ == "__main__":
    print(rot_normal_from_matR(Ry3(np.radians(60))))
