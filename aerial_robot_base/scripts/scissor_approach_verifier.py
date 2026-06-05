#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Scissor Approach Verifier for Gimbalrotor
Author: Antigravity & Kamada

RViz上の「Publish Point」ツール (/clicked_point トピック) を通じて点群をクリックし，
その選択された3D座標をターゲット目標 (target) として動的TF配信します．
その上で，ハサミ先端をターゲットに一致させるための目標機体位置・姿勢を逆計算し，
現在の自己位置との 6自由度 (6DOF) 偏差をリアルタイムにターミナル表示・検証するノードです．
"""

import sys
import os

# C++のROS_WARN警告を黙らせる
os.environ['ROSCONSOLE_MIN_SEVERITY'] = 'ERROR'

# 標準エラー出力のTF重複警告をフィルタリング
class StderrFilter:
    def __init__(self, original_stderr):
        self.original_stderr = original_stderr
    def write(self, message):
        if "TF_REPEATED_DATA" in message or "redundant timestamp" in message:
            return
        self.original_stderr.write(message)
    def flush(self):
        self.original_stderr.flush()

sys.stderr = StderrFilter(sys.stderr)

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped, TransformStamped
from aerial_robot_msgs.msg import FlightNav

def transform_to_matrix(translation, rotation):
    """
    位置 [x, y, z] とクォータニオン [x, y, z, w] から 4x4 同次変換行列を作成する
    """
    T = tft.translation_matrix(translation)
    R = tft.quaternion_matrix(rotation)
    return tft.concatenate_matrices(T, R)

def matrix_to_transform(matrix):
    """
    4x4 同次変換行列から位置 [x, y, z] とクォータニオン [x, y, z, w] を抽出する
    """
    translation = tft.translation_from_matrix(matrix)
    quaternion = tft.quaternion_from_matrix(matrix)
    return translation, quaternion

class ScissorApproachVerifier:
    def __init__(self):
        rospy.init_node('scissor_approach_verifier', anonymous=True)

        # パラメータの取得（デフォルト値は gimbalrotor1 プレフィックスを想定）
        self.tf_prefix = rospy.get_param('~tf_prefix', 'gimbalrotor1')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.target_frame = rospy.get_param('~target_frame', 'target')
        
        # ベースフレームとハサミフレーム（プレフィックス付き）
        self.base_frame = rospy.get_param('~base_frame', f'{self.tf_prefix}/base_link')
        self.scissor_frame = rospy.get_param('~scissor_frame', f'{self.tf_prefix}/scissor_center')
        
        self.rate_hz = rospy.get_param('~rate', 10.0)
        self.publish_nav = rospy.get_param('~publish_nav', True)

        # ターゲット位置の初期化
        self.target_position_world = [1.0, 0.0, 1.0] # 初期デフォルト位置 (1m四方の仮目標)
        self.target_quaternion_world = [0.0, 0.0, 0.0, 1.0]
        self.has_target = False

        # TFバッファ，リスナー，ブロードキャスターのセットアップ
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # サブスクライバ: RVizの Publish Point トピック
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)

        # UAV NAVトピックのパブリッシャ
        self.nav_pub = rospy.Publisher('uav/nav', FlightNav, queue_size=1)

        rospy.loginfo("==============================================")
        rospy.loginfo(" Scissor Approach Verifier Initialized")
        rospy.loginfo(f" tf_prefix    : {self.tf_prefix}")
        rospy.loginfo(f" world_frame  : {self.world_frame}")
        rospy.loginfo(f" target_frame : {self.target_frame}")
        rospy.loginfo(f" base_frame   : {self.base_frame}")
        rospy.loginfo(f" scissor_frame: {self.scissor_frame}")
        rospy.loginfo(" --------------------------------------------  ")
        rospy.loginfo(" RVizの 'Publish Point' ツールをクリックして， ")
        rospy.loginfo(" 点群上に目標ターゲット(target)を設定してください． ")
        rospy.loginfo("==============================================")

    def clicked_point_callback(self, msg):
        """
        RViz上で点群がクリックされた際のコールバック関数．
        クリック位置を world 座標系に変換し，ターゲット座標を更新します．
        """
        try:
            # クリックされた点が含まれるフレームから world_frame へのTFを取得
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
            )
            
            t_trans = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
            q_trans = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            M_world_clickframe = transform_to_matrix(t_trans, q_trans)
            
            # 同次座標系でクリック点を定義し，world 座標に変換
            P_click = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0])
            P_world = np.dot(M_world_clickframe, P_click)
            
            # ターゲット座標を更新
            self.target_position_world = P_world[:3].tolist()
            # ターゲット姿勢は基準世界座標系と平行 (No rotation) とする
            self.target_quaternion_world = [0.0, 0.0, 0.0, 1.0]
            self.has_target = True
            
            rospy.loginfo(f"Target updated via RViz click: {self.target_position_world}")
            
        except Exception as e:
            rospy.logerr(f"Failed to transform clicked point: {e}")

    def publish_target_tf(self):
        """
        選択されたターゲット座標を 'target' フレームとしてTFブロードキャストする
        """
        t_msg = TransformStamped()
        t_msg.header.stamp = rospy.Time.now()
        t_msg.header.frame_id = self.world_frame
        t_msg.child_frame_id = self.target_frame
        
        t_msg.transform.translation.x = self.target_position_world[0]
        t_msg.transform.translation.y = self.target_position_world[1]
        t_msg.transform.translation.z = self.target_position_world[2]
        
        t_msg.transform.rotation.x = self.target_quaternion_world[0]
        t_msg.transform.rotation.y = self.target_quaternion_world[1]
        t_msg.transform.rotation.z = self.target_quaternion_world[2]
        t_msg.transform.rotation.w = self.target_quaternion_world[3]
        
        self.tf_broadcaster.sendTransform(t_msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        
        while not rospy.is_shutdown():
            # ターゲット位置をTFとして常時配信（未選択の場合は初期デフォルト座標をパブリッシュ）
            self.publish_target_tf()

            try:
                # 1. world -> target (アプローチ目標) のTFルックアップ
                target_tf = self.tf_buffer.lookup_transform(
                    self.world_frame, self.target_frame, rospy.Time(0), rospy.Duration(0.5)
                )
                t_target = [target_tf.transform.translation.x, target_tf.transform.translation.y, target_tf.transform.translation.z]
                q_target = [target_tf.transform.rotation.x, target_tf.transform.rotation.y, target_tf.transform.rotation.z, target_tf.transform.rotation.w]
                M_world_target = transform_to_matrix(t_target, q_target)

                # 2. base_link -> scissor_center (ハサミの相対変換) のTFルックアップ
                scissor_tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.scissor_frame, rospy.Time(0), rospy.Duration(0.5)
                )
                t_scissor = [scissor_tf.transform.translation.x, scissor_tf.transform.translation.y, scissor_tf.transform.translation.z]
                q_scissor = [scissor_tf.transform.rotation.x, scissor_tf.transform.rotation.y, scissor_tf.transform.rotation.z, scissor_tf.transform.rotation.w]
                M_base_scissor = transform_to_matrix(t_scissor, q_scissor)

                # 3. 目標ベースリンク座標の逆計算
                # M_world_base_target = M_world_target * (M_base_scissor)^-1
                M_scissor_base = tft.inverse_matrix(M_base_scissor)
                M_world_base_target = tft.concatenate_matrices(M_world_target, M_scissor_base)

                t_base_target, q_base_target = matrix_to_transform(M_world_base_target)
                euler_base_target = tft.euler_from_quaternion(q_base_target, axes='sxyz')
                rpy_base_target_deg = [np.degrees(a) for a in euler_base_target]

                # 4. 現在の base_link の世界座標を取得
                current_tf = self.tf_buffer.lookup_transform(
                    self.world_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5)
                )
                t_curr = [current_tf.transform.translation.x, current_tf.transform.translation.y, current_tf.transform.translation.z]
                q_curr = [current_tf.transform.rotation.x, current_tf.transform.rotation.y, current_tf.transform.rotation.z, current_tf.transform.rotation.w]
                M_world_curr = transform_to_matrix(t_curr, q_curr)
                
                euler_curr = tft.euler_from_quaternion(q_curr, axes='sxyz')
                rpy_curr_deg = [np.degrees(a) for a in euler_curr]

                # 5. 6DOF 偏差 (ERROR) の計算
                # 位置偏差 (XYZ)
                dx = t_base_target[0] - t_curr[0]
                dy = t_base_target[1] - t_curr[1]
                dz = t_base_target[2] - t_curr[2]
                dist_err = np.sqrt(dx**2 + dy**2 + dz**2)

                # 姿勢偏差 (R_err = R_goal * R_curr^-1)
                M_curr_inv = tft.inverse_matrix(M_world_curr)
                M_err = tft.concatenate_matrices(M_world_base_target, M_curr_inv)
                q_err = tft.quaternion_from_matrix(M_err)
                euler_err = tft.euler_from_quaternion(q_err, axes='sxyz')
                rpy_err_deg = [np.degrees(a) for a in euler_err]

                # 6. ターミナル表示のリフレッシュと表示
                os.system('clear')
                print("=================================================================")
                print("                 SCISSOR APPROACH 6DOF VERIFIER                  ")
                print("=================================================================")
                print(f"Target Frame  : {self.world_frame} -> {self.target_frame}")
                print(f"Base Frame    : {self.base_frame}")
                print(f"Scissor Frame : {self.base_frame} -> {self.scissor_frame}")
                print("-----------------------------------------------------------------")
                if self.has_target:
                    print(" Target Source: RViz CLICKED POINT [ ACTIVE ]")
                else:
                    print(" Target Source: DEFAULT COORDINATE [ WAITING FOR RViz CLICK ]")
                print(f"Target Pos  (World): X: {t_target[0]:6.3f}, Y: {t_target[1]:6.3f}, Z: {t_target[2]:6.3f} m")
                t_rpy = [np.degrees(a) for a in tft.euler_from_quaternion(q_target)]
                print(f"Target Att  (World): R: {t_rpy[0]:6.1f}, P: {t_rpy[1]:6.1f}, Y: {t_rpy[2]:6.1f} deg")
                print("-----------------------------------------------------------------")
                print(f"Goal Base Pos (World): X: {t_base_target[0]:6.3f}, Y: {t_base_target[1]:6.3f}, Z: {t_base_target[2]:6.3f} m")
                print(f"Goal Base Att (World): R: {rpy_base_target_deg[0]:6.1f}, P: {rpy_base_target_deg[1]:6.1f}, Y: {rpy_base_target_deg[2]:6.1f} deg")
                print("-----------------------------------------------------------------")
                print(f"Curr Base Pos (World): X: {t_curr[0]:6.3f}, Y: {t_curr[1]:6.3f}, Z: {t_curr[2]:6.3f} m")
                print(f"Curr Base Att (World): R: {rpy_curr_deg[0]:6.1f}, P: {rpy_curr_deg[1]:6.1f}, Y: {rpy_curr_deg[2]:6.1f} deg")
                print("-----------------------------------------------------------------")
                print("ERROR (Goal - Current):")
                print(f"  POSITION:  dX: {dx:6.3f} m, dY: {dy:6.3f} m, dZ: {dz:6.3f} m  --> Dist Error: {dist_err:6.3f} m")
                print(f"  ATTITUDE:  dR: {rpy_err_deg[0]:6.1f} deg, dP: {rpy_err_deg[1]:6.1f} deg, dY: {rpy_err_deg[2]:6.1f} deg")
                print("=================================================================")
                print(" ※ RViz上の 'Publish Point' ボタンを押し，点群をクリックすると，")
                print("    その位置へターゲットが自動更新され，偏差が即座に再計算されます．")
                print("    ERROR値を 0 に合わせ，ハサミの挟み込み状態を目視してください．")

                # 7. 飛行指令値 (FlightNav) のパブリッシュ
                if self.publish_nav and self.has_target:
                    nav_msg = FlightNav()
                    nav_msg.header.stamp = rospy.Time.now()
                    nav_msg.header.frame_id = self.world_frame
                    nav_msg.control_frame = FlightNav.WORLD_FRAME
                    nav_msg.target = FlightNav.BASELINK
                    nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                    nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                    nav_msg.yaw_nav_mode = FlightNav.POS_MODE

                    # 目標位置・Yawを代入
                    nav_msg.target_pos_x = t_base_target[0]
                    nav_msg.target_pos_y = t_base_target[1]
                    nav_msg.target_pos_z = t_base_target[2]
                    nav_msg.target_yaw = euler_base_target[2] # ラジアン

                    self.nav_pub.publish(nav_msg)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(2.0, f"Waiting for TF frames... Error: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        verifier = ScissorApproachVerifier()
        verifier.spin()
    except rospy.ROSInterruptException:
        pass
