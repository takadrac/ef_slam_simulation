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

def elem_vec(vec):
    evec = vec / np.linalg.norm(vec)
    return evec