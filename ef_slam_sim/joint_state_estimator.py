import matplotlib.pyplot as plt
import numpy as np
import random

from mpl_toolkits.mplot3d import Axes3D

from matrix import T, Rx3, Ry3, Rz3, Rx4, Ry4, Rz4, R3to4, R4to3, elem_vec, \
                mat4_to_posvec_and_rotmat, rot_normal_from_matR, rot_normal_from_radius_vecs

class JointStateEstimator:
    def __init__(self):
        self.traj_memory = []
        self.rot_axis = np.array([0, 0, 0])
        self.rot_center = np.array([0, 0, 0])
        self.rot_radius = 0

    # TODO: 単一関節を動作した時の軌道データを追加
    # TODO: 軌道データから回転軸ベクトルおよび順運動学から軌道中心が一致する関節状態を最適化

    def append_traj(self, pos_qut):
        self.traj_memory.append(pos_qut)
        # self.traj_memory = np.insert(self.traj_memory, len(self.traj_memory), pos_qut, axis=0)

    ###軌道データから回転軸ベクトルを推定###
    def estimate_rot_axis(self, xs, ys, zs):
        r = np.c_[xs, ys, zs]
        # サンプル点群の重心、x, y, zの３つの成分
        c = np.mean(r, axis=0)
        # サンプル点群の重心を原点に移動
        r0 = r - c
        # SVD分解
        u, s, v = np.linalg.svd(r0)
        # sの最小値に対応するベクトルを取り出す
        nv = v[-1, :]
        # サンプル点群の平面と原点との距離
        ds = np.dot(r, nv)
        param = np.r_[nv, -np.mean(ds)]
        # print('planner vector', param)
        return param
        # self.rot_axis = param

    def estimate_rot_center(self, xs, ys, zs):
        sumx = sum(xs)
        sumy = sum(ys)
        sumz = sum(zs)
        sumx2 = sum([ix ** 2 for ix in xs])
        sumy2 = sum([iy ** 2 for iy in ys])
        sumz2 = sum([iz ** 2 for iz in zs])
        sumxy = sum([ix * iy for (ix, iy) in zip(xs, ys)])
        sumyz = sum([iy * iz for (iy, iz) in zip(ys, zs)])
        sumzx = sum([iz * ix for (iz, ix) in zip(zs, xs)])

        # F = np.array([[sumx2, sumxy, sumx],
        #               [sumxy, sumy2, sumy],
        #               [sumx, sumy, len(xs)]])
        F = np.array([[sumx2, sumxy, sumzx, sumx],
                      [sumxy, sumy2, sumyz, sumy],
                      [sumzx, sumyz, sumz2, sumz],
                      [sumx, sumy, sumz, len(xs)]])

        # G = np.array([[-sum([ix ** 3 + ix * iy ** 2 for (ix, iy) in zip(xs, ys)])],
        #               [-sum([ix ** 2 * iy + iy ** 3 for (ix, iy) in zip(xs, ys)])],
        #               [-sum([ix ** 2 + iy ** 2 for (ix, iy) in zip(xs, ys)])]])
        G = np.array([[-sum([ix ** 3 + ix * (iy ** 2 + iz ** 2) for (ix, iy, iz) in zip(xs, ys, zs)])],
                      [-sum([iy ** 3 + iy * (ix ** 2 + iz ** 2) for (ix, iy, iz) in zip(xs, ys, zs)])],
                      [-sum([iz ** 3 + iz * (ix ** 2 + iy ** 2) for (ix, iy, iz) in zip(xs, ys, zs)])],
                      [-sum([ix ** 2 + iy ** 2 + iz ** 2 for (ix, iy, iz) in zip(xs, ys, zs)])]])

        T = np.linalg.inv(F).dot(G)

        cxe = float(T[0] / -2)
        cye = float(T[1] / -2)
        cze = float(T[2] / -2)
        sphere_center = np.array([float(T[0] / -2), float(T[1] / -2), float(T[2] / -2)])
        # re = np.sqrt(cxe ** 2 + cye ** 2 - T[3])
        # print('circle center and radius', cxe, cye, re[0])
        re = np.sqrt(sum(np.square(sphere_center)) - T[3])
        # print('fitted sphere center and radius', sphere_center, re[0])

        k = -(sum(sphere_center * self.rot_axis[:3]) + self.rot_axis[3]) / sum(np.square(self.rot_axis[:3]))
        # self.rot_center = np.array([cxe, cye])
        self.rot_center = sphere_center + k * self.rot_axis[:3]
        self.rot_radius = np.sqrt(re[0]**2 - sum(np.square(k * self.rot_axis[:3])))
        print('rot_center', self.rot_center, '\nrot_radius', self.rot_radius)

    '''
        def estimate_sphere_center(self, xs, ys, zs):
        sumx = sum(xs)
        sumy = sum(ys)
        sumz = sum(zs)
        sumx2 = sum([ix ** 2 for ix in xs])
        sumy2 = sum([iy ** 2 for iy in ys])
        sumz2 = sum([iz ** 2 for iz in zs])
        sumxy = sum([ix * iy for (ix, iy) in zip(xs, ys)])
        sumyz = sum([iy * iz for (iy, iz) in zip(ys, zs)])
        sumzx = sum([iz * ix for (iz, ix) in zip(zs, xs)])

        F = np.array([[sumx2, sumxy, sumzx, sumx],
                      [sumxy, sumy2, sumyz, sumy],
                      [sumzx, sumyz, sumz2, sumz],
                      [sumx, sumy, sumz, len(xs)]])

        G = np.array([[-sum([ix ** 3 + ix * (iy ** 2 + iz ** 2) for (ix, iy, iz) in zip(xs, ys, zs)])],
                      [-sum([iy ** 3 + iy * (ix ** 2 + iz ** 2) for (ix, iy, iz) in zip(xs, ys, zs)])],
                      [-sum([iz ** 3 + iz * (ix ** 2 + iy ** 2) for (ix, iy, iz) in zip(xs, ys, zs)])],
                      [-sum([ix ** 2 + iy ** 2 + iz ** 2 for (ix, iy, iz) in zip(xs, ys, zs)])]])

        T = np.linalg.inv(F).dot(G)

        cxe = float(T[0] / -2)
        cye = float(T[1] / -2)
        cze = float(T[2] / -2)
        re = np.sqrt(cxe ** 2 + cye ** 2 + cze ** 2 - T[3])
        print('sphere center and radius', cxe, cye, cze, re[0])
        self.rot_center = np.array([cxe, cye, cze])
        self.rot_radius = re[0]
        # return (cxe, cye, cze, re)
    '''

    ###順運動学から軌道中心が一致する関節状態を最適化###
    def optimize_joint_state(self):
        xs = [p[0] for p in self.traj_memory]
        ys = [p[1] for p in self.traj_memory]
        zs = [p[2] for p in self.traj_memory]

        self.rot_axis = self.estimate_rot_axis(xs, ys, zs)
        self.estimate_rot_center(xs, ys, zs)


if __name__ == '__main__':
    estimator = JointStateEstimator()
    pos = np.array(
        [[2, 1, 1],
         [1, 1, 1],
         [1, 1, 2]])
    print(
        estimator.estimate_rot_axis(
            pos[:, 0],
            pos[:, 1],
            pos[:, 2]
        )
    )
    print(
        np.cross(pos[0, :] - pos[1, :],
                 pos[1, :] - pos[2, :])
    )
    # input()
    link_1 = np.array([1.0, 0, 0]) # 既知
    link_2 = np.array([1.2, 0, 0]) # 既知
    link_3 = np.array([.8, 0, 0]) # 既知
    theta_i = np.radians(30)
    theta_i_1 = np.radians(45)
    theta_i_2 = np.radians(20)

    axis = []
    true_traj_pos = []
    for i in range(100):
        ef_mat = Rz4(theta_i_2) @ Ry4(theta_i_1+np.radians(i*.6)) @ T(link_2) @ Rz4(theta_i) @ T(link_1)
        pos, matR = mat4_to_posvec_and_rotmat(ef_mat)

        # n = rot_normal_from_matR(
        #     R4to3(Rz4(theta_i_2) @ Ry4(theta_i_1+np.radians(i*.6)) @ Rx4(theta_i))
        # )


        # x = matR @ np.array([1, 0, 0])
        # y = matR @ np.array([0, 1, 0])
        # z = matR @ np.array([0, 0, 1])
        # nx = np.cross(x, np.array([1, 0, 0]))
        # ny = np.cross(y, np.array([0, 1, 0]))
        # nz = np.cross(z, np.array([0, 0, 1]))
        # print("axis", n)
        # axis.append(n)
        # print(x @ np.array([1, 0, 0]), y @ np.array([0, 1, 0]), z @ np.array([0, 0, 1]))
        true_traj_pos.append(pos)
        estimator.append_traj(pos+(random.random()-0.5)*0.01)
        # print(mat4_to_posvec_and_rotmat(ef_mat))
    # print(estimator.traj_memory)
    true_traj_pos = np.array(true_traj_pos, dtype="float32")
    for i in range(len(true_traj_pos)-2):
        # TODO: 3点で円を推定? or
        print('planner vec',
              elem_vec(
                  estimator.estimate_rot_axis(true_traj_pos[i:i + 3, 0], true_traj_pos[i:i + 3, 1],
                                              true_traj_pos[i:i + 3, 2])[:3]
              )
        )
        # print('cross' ,
        #       elem_vec(
        #           np.cross(true_traj_pos[i, :] - true_traj_pos[i + 1, :],
        #                    true_traj_pos[i + 1, :] - true_traj_pos[i + 2, :])
        #       )
        # )

        # print(rot_normal_from_radius_vecs(true_traj_pos[i], true_traj_pos[i+1]))
        pass

    estimator.optimize_joint_state()

    tmp = np.array(estimator.traj_memory)
    # axis = np.array(axis)

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # ax.set_xlim([-0., 2.])
    # ax.set_ylim([-1., 1.])
    # ax.set_zlim([-3., -1.])

    ax.set_xlim([-3., 3.])
    ax.set_ylim([-3., 3.])
    ax.set_zlim([-3., 3.])


    ax.plot(tmp[:, 0], tmp[:, 1], tmp[:, 2], marker="o", linestyle='None')
    # ax.plot(axis[:, 0], axis[:, 1], axis[:, 2], marker="o", linestyle='None')
    plt.show()