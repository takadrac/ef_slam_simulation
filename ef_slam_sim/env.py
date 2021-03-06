import getpass
import numpy as np
import os
import pandas as pd
import pybullet as p
import time

from numpy.linalg import inv

from matrix import T, Rx3, Ry3, Rz3, Rx4, Ry4, Rz4, R3to4, elem_vec

class Robot(object):
    def __init__(self):
        p.connect(p.GUI)
        # p.setGravity(0, 0, -9.8)
        self.user_name = getpass.getuser()
        self.load_robot('/home/{}/git/pybullet-playground/urdf/'.format(self.user_name), 'ur5.urdf')
        '''
        kuka: '/home/{}}/git/bullet3/data/kuka_lwr/', 'kuka.urdf'
        ur5: '/home/{}/git/pybullet-playground/urdf/', 'ur5.urdf'
        ur5: '/home/{}/catkin_ws/src/universal_robot/ur_description/urdf', 'ur5_robot.urdf'
        '''
        # import pdb; pdb.set_trace()
        self.estimated_joint_state = {}
        self.init_pybullet()

    def init_pybullet(self):
        self.enable_torque()
        self.get_joint_info()
        # self.command_ID()
        self.set_marker('/home/{}/git/bullet3/data/'.format(self.user_name), 'cube.urdf', 'markers0.png')

    def set_marker(self, dir_path, file, pic_file):
        tmp = os.getcwd()
        os.chdir(dir_path)
        # c = p.loadURDF(file, (0, 0, 1), useFixedBase=True)
        c = p.loadURDF(file, (0, 1, 0), useFixedBase=True, globalScaling=0.1)
        # for i in range(1, 6):
        #     p.loadURDF(file, (0.2*i, 1, 0), useFixedBase=True, globalScaling=0.1)
        os.chdir(tmp)
        x = p.loadTexture(pic_file)
        p.changeVisualShape(c, -1, textureUniqueId=x)

    def load_robot(self, dir_path, file):
        # TODO: multi-DOF joint robot
        tmp = os.getcwd()
        os.chdir(dir_path)
        self.robot = p.loadURDF(file)
        os.chdir(tmp)

    def enable_torque(self):
        num_joint = p.getNumJoints(self.robot)
        for i in range(num_joint):
            if p.getJointInfo(self.robot, i)[2] != 4:
                p.enableJointForceTorqueSensor(self.robot, i)

    def set_camera(self):
        # TODO: アーム先端部のworld座標

        rot_mtrx = np.array(p.getMatrixFromQuaternion(self.link_info['ee_link'][1])).reshape(3, 3)


        self.view_matrix = p.computeViewMatrix(
            cameraEyePosition=np.array(self.link_info['ee_link'][0]),
            cameraTargetPosition=np.array(self.link_info['ee_link'][0]) + rot_mtrx @ np.array([1, 0, 0]),
            cameraUpVector=rot_mtrx @ np.array([0, 1, 0])
        )

        # self.view_matrix = p.computeViewMatrixFromYawPitchRoll(
        #     cameraTargetPosition=np.array([0.5, -0.5, 1.5]) * 1/1,#np.array([4, -4, 3]) * 1/10, # target focus point
        #     distance=2., # from eye to target
        #     yaw=30, # degree left/right around z axis
        #     pitch=-30, # degree up/down around
        #     roll=90, # degree around forward vector
        #     upAxisIndex=2) # either 1 for Y or 2 for Z axis up

        # 表示のための投影マトリクス計算
        self.proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, # the field of view angle, in degrees, in the y direction
            aspect=320 / 240, # the aspect ratio that determines the field of view in the x direction. The aspect ratio is the ratio of x (width) to y (height).
            nearVal=0.1, # the distance from the viewer to the near clipping plane (always positive)
            farVal=100.0) # the distance from the viewer to the far clipping plane (always positive)

    def get_image(self, img_W=512, img_H=512):
        # 描画計算
        (_, _, rgba_img, dpt_img, sgm_img) = p.getCameraImage(img_W, img_H, viewMatrix=self.view_matrix, projectionMatrix=self.proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)
        print(np.array(rgba_img))
        rgb_img = np.array(rgba_img).reshape(img_W, img_H, -1)[:, :, :3]
        dpt_img = np.array(dpt_img).reshape(img_W, img_H, -1)
        sgm_img = np.array(sgm_img).reshape(img_W, img_H, -1)

    def get_joint_info(self):
        num_joint = p.getNumJoints(self.robot)
        self.driven_joint_info = []
        for i in range(num_joint):
            print(p.getJointInfo(self.robot, i))
            self.driven_joint_info.append(p.getJointInfo(self.robot, i))
        # print(self.driven_joint_info)
        # print(num_joint)

    # TODO: 全関節状態を取得
    def get_all_joint_state(self):
        self.joint_state = {}
        for i in range(len(self.driven_joint_info)):
            if p.getJointInfo(self.robot, i)[2] != 4:
                self.joint_state[self.driven_joint_info[i][1].decode('utf-8')] = p.getJointState(self.robot, i)
                ###### Debug ######
                self.estimated_joint_state[self.driven_joint_info[i][1].decode('utf-8')] = \
                    p.getJointState(self.robot, i)
                ###################

        # df = pd.DataFrame(self.joint_state.values(), index=self.joint_state.keys())
        # print(df)
        pass

    def get_all_link_info(self):
        self.link_info = {}
        # print(self.driven_joint_info)
        for i in range(len(self.driven_joint_info)):
            link_index = self.driven_joint_info[i][-1] + 1
            if link_index != -1:
                self.link_info[self.driven_joint_info[i][-5].decode('utf-8')] = p.getLinkState(self.robot, link_index,
                                                            computeLinkVelocity=1, computeForwardKinematics=1)

        df = pd.DataFrame(self.link_info.values(), index=self.link_info.keys())
        # print(df.index)
        print(df.loc[df.index[-1]][0]) # linkWorldPosition
        # print(df.loc[df.index[-1]][1]) # linkWorldOrientation

        # import ipdb; ipdb.set_trace()

        # print(np.array(p.getMatrixFromQuaternion(df.loc[df.index[-1]][1])).reshape(3, 3))


        # print(np.array(p.getMatrixFromQuaternion(df.loc[df.index[-1]][1])).reshape(3, 3) @ np.array([1, 0, 0]))

    def command_ID(self):
        self.position_Ids = {}
        self.torque_Ids = {}
        for i in range(len(self.driven_joint_info)):
            if p.getJointInfo(self.robot, i)[2] != 4:
                self.position_Ids[self.driven_joint_info[i][1].decode('utf-8')] = \
                    p.addUserDebugParameter("postion_of_{}".format(self.driven_joint_info[i][1].decode('utf-8')), 0, 360, 0)
                # self.torque_Ids[self.driven_joint_info[i][1].decode('utf-8')] = \
                #     p.addUserDebugParameter("torque_of_{}".format(self.driven_joint_info[i][1].decode('utf-8')), 0, 20, 5)

    def control_joint_state(self):
        for i in range(len(self.driven_joint_info)):
            if p.getJointInfo(self.robot, i)[2] != 4:
                p.setJointMotorControl2(self.robot, self.driven_joint_info[i][0], p.POSITION_CONTROL,
                    targetPosition=np.radians(p.readUserDebugParameter(self.position_Ids[self.driven_joint_info[i][1].decode('utf-8')])))

    # TODO: アーム手先位置座標出力
    def get_global_ef_pos(self):
        pass

    # TODO: 目標位置
    def set_target_pos(self, target_pos):
        pass

    # TODO: アーム手先を対象物位置にインチング動作
    def drive_joint(self):
        idx, direc = self.get_drive_joint()
        joint_idx = idx + 1
        # print(self.estimated_joint_state[self.driven_joint_info[joint_idx][1].decode('utf-8')])
        # print(direc)
        print(self.driven_joint_info[joint_idx][1].decode('utf-8'))
        target_pos = self.estimated_joint_state[self.driven_joint_info[joint_idx][1].decode('utf-8')][0] + np.radians(direc*5)
        p.setJointMotorControl2(self.robot,
                                self.driven_joint_info[joint_idx][0],
                                p.POSITION_CONTROL,
                                targetPosition=target_pos)

    def joint_axis_vec(self, ax):
        if ax == 'x':
            return np.array([1, 0, 0])
        elif ax == 'y':
            return np.array([0, 1, 0])
        elif ax == 'z':
            return np.array([0, 0, 1])

    # TODO: 駆動関節を判定
    def get_drive_joint(self, target_pos=(0.1, 0.15, -0.3), target_qut=None):
        # 手先と目標位置のベクトル
        '''
        getLinkStateの引数
        linkWorldPosition: [0] リンク位置(desirable)
        worldLinkFramePosition: [4] 関節位置
        '''
        err_pos = np.array(target_pos) - np.array(self.link_info['ee_link'][0])
        # 手先座標系でのerr_pos(xyz方向との内積)
        rot_mtrx = np.array(p.getMatrixFromQuaternion(self.link_info['ee_link'][1])).reshape(3, 3)
        ee_x = rot_mtrx @ np.array([1, 0, 0])
        ee_y = rot_mtrx @ np.array([0, 1, 0])
        ee_z = rot_mtrx @ np.array([0, 0, 1])
        ee_err_pos = np.array([np.dot(err_pos, ee_x), np.dot(err_pos, ee_y), np.dot(err_pos, ee_z)])
        # print(ee_err_pos)

        # 関節ごとの座標系での ee_err_pos(平行移動)
        '''
        平行移動： self.driven_joint_info[-1][-3]
        回転移動: p.getMatrixFromQuaternion(self.driven_joint_info[-1][-2])
        
        axis: z, y, y, y, z, y
        '''
        ee_err_pos_6 = inv(Ry3(self.estimated_joint_state[self.driven_joint_info[-2][1].decode('utf-8')][0])) \
              @ inv(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-1][-2])).reshape(3, 3)) @ ee_err_pos
        ee_err_pos_5 = inv(Rz3(self.estimated_joint_state[self.driven_joint_info[-3][1].decode('utf-8')][0])) \
                    @ inv(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-2][-2])).reshape(3, 3)) @ ee_err_pos_6
        ee_err_pos_4 = inv(Ry3(self.estimated_joint_state[self.driven_joint_info[-4][1].decode('utf-8')][0])) \
                @ inv(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-3][-2])).reshape(3, 3)) @ ee_err_pos_5
        ee_err_pos_3 = inv(Ry3(self.estimated_joint_state[self.driven_joint_info[-5][1].decode('utf-8')][0])) \
                @ inv(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-4][-2])).reshape(3, 3)) @ ee_err_pos_4
        ee_err_pos_2 = inv(Ry3(self.estimated_joint_state[self.driven_joint_info[-6][1].decode('utf-8')][0])) \
                @ inv(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-5][-2])).reshape(3, 3)) @ ee_err_pos_3
        ee_err_pos_1 = inv(Rz3(self.estimated_joint_state[self.driven_joint_info[-7][1].decode('utf-8')][0])) \
                @ inv(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-6][-2])).reshape(3, 3)) @ ee_err_pos_2
        # print(ee_err_pos_6)
        # print(ee_err_pos_5)
        # print(ee_err_pos_4)
        # print(ee_err_pos_3)
        # print(ee_err_pos_2)
        # print(ee_err_pos_1)

        # 半径方向 axis: z, y, y, y, z, y
        r6 = T(self.driven_joint_info[-1][-3]) \
             @ R3to4(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-1][-2])).reshape(3, 3)) \
             @ np.array([0, 0, 0, 1])
        r5 = T(self.driven_joint_info[-2][-3]) \
             @ R3to4(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-2][-2])).reshape(3, 3)) \
             @ Ry4(self.estimated_joint_state[self.driven_joint_info[-2][1].decode('utf-8')][0]) @ r6
        r4 = T(self.driven_joint_info[-3][-3]) \
             @ R3to4(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-3][-2])).reshape(3, 3)) \
             @ Rz4(self.estimated_joint_state[self.driven_joint_info[-3][1].decode('utf-8')][0]) @ r5
        r3 = T(self.driven_joint_info[-4][-3]) \
             @ R3to4(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-4][-2])).reshape(3, 3)) \
             @ Ry4(self.estimated_joint_state[self.driven_joint_info[-4][1].decode('utf-8')][0]) @ r4
        r2 = T(self.driven_joint_info[-5][-3]) \
             @ R3to4(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-5][-2])).reshape(3, 3)) \
             @ Ry4(self.estimated_joint_state[self.driven_joint_info[-5][1].decode('utf-8')][0]) @ r3
        r1 = T(self.driven_joint_info[-6][-3]) \
             @ R3to4(np.array(p.getMatrixFromQuaternion(self.driven_joint_info[-6][-2])).reshape(3, 3)) \
             @ Ry4(self.estimated_joint_state[self.driven_joint_info[-6][1].decode('utf-8')][0]) @ r2

        # 推定関節状態量から手先動作方向を導出(軸方向x半径方向=>外積を計算)
        ax_x_r6 = np.cross(self.joint_axis_vec('y'), r6[:3])
        ax_x_r5 = np.cross(self.joint_axis_vec('z'), r5[:3])
        ax_x_r4 = np.cross(self.joint_axis_vec('y'), r4[:3])
        ax_x_r3 = np.cross(self.joint_axis_vec('y'), r3[:3])
        ax_x_r2 = np.cross(self.joint_axis_vec('y'), r2[:3])
        ax_x_r1 = np.cross(self.joint_axis_vec('z'), r1[:3])
        # print(ax_x_r6)
        # print(ax_x_r5)
        # print(ax_x_r4)
        # print(ax_x_r3)
        # print(ax_x_r2)
        # print(ax_x_r1)
        # TODO: 手先ベクトルと外積の内積の最大値
        dots = [
            np.dot(elem_vec(ax_x_r1), elem_vec(ee_err_pos_1)),
            np.dot(elem_vec(ax_x_r2), elem_vec(ee_err_pos_2)),
            np.dot(elem_vec(ax_x_r3), elem_vec(ee_err_pos_3)),
            np.dot(elem_vec(ax_x_r4), elem_vec(ee_err_pos_4)),
            np.dot(elem_vec(ax_x_r5), elem_vec(ee_err_pos_5)),
            np.dot(elem_vec(ax_x_r6), elem_vec(ee_err_pos_6))
        ]
        # print(dots)
        dots_idx = list(np.abs(dots)).index(max(np.abs(dots)))
        return dots_idx, np.sign(dots[dots_idx])




    # TODO: 動作起動から関節状態量を推定
    def estimate_joint_state(self):
        # TODO: 手先軌道を格納
        # TODO: 半径、回転中心軸を導出
        # TODO: 駆動関節から手先までのIKから推定関節状態量を更新
        pass


if __name__ == '__main__':
    robot = Robot()
    # robot.set_camera()
    # robot.get_image()

    try:
        while 1:

            robot.get_all_joint_state()
            # robot.control_joint_state()
            robot.get_all_link_info()
            # robot.get_drive_joint()
            robot.drive_joint()

            p.stepSimulation()
            robot.set_camera()
            robot.get_image()
            # time.sleep(0.1)
    except KeyboardInterrupt:
        pass