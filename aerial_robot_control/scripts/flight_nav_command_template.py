#!/usr/bin/env python

import rospy
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Header

def publish_flight_nav():
    rospy.init_node("flight_nav_publisher", anonymous=True)
    
    # ロボットの名前空間を取得（デフォルトは gimbalrotor1）
    robot_ns = rospy.get_param("~robot_ns", "/gimbalrotor1")
    topic_name = f"{robot_ns}/uav/nav"
    
    pub = rospy.Publisher(topic_name, FlightNav, queue_size=10)
    
    rospy.sleep(1)  

    msg = FlightNav()
    
    # ヘッダー設定
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world" 

    # --- ナビゲーション設定 ---
    
    # 座標系の設定 (control_frame)
    # WORLD_FRAME (0): ワールド座標系で制御
    # LOCAL_FRAME (1): ロボットの現在位置を基準とした相対座標系で制御
    msg.control_frame = FlightNav.WORLD_FRAME
    
    # 制御対象の設定 (target)
    # BASELINK (0): 機体の中心（BaseLink）を制御対象とする
    # COG (1): 重心（Center of Gravity）を制御対象とする
    msg.target = FlightNav.BASELINK

    # --- XY位置制御 (Position Control) ---
    # モード (pos_xy_nav_mode):
    # NO_NAVIGATION (0): 制御しない
    # VEL_MODE (1): 速度制御モード (target_vel_x, target_vel_y を使用)
    # POS_MODE (2): 位置制御モード (target_pos_x, target_pos_y を使用)
    # ACC_MODE (3): 加速度制御モード (target_acc_x, target_acc_y を使用)
    # POS_VEL_MODE (4): 位置と速度を同時に指定
    # GPS_WAYPOINT_MODE (5): GPSウェイポイントを使用
    msg.pos_xy_nav_mode = FlightNav.POS_MODE
    msg.target_pos_x = 1.0  # 目標X座標 [m]
    msg.target_pos_y = 0.5  # 目標Y座標 [m]
    # VEL_MODEの場合に使用
    msg.target_vel_x = 0.0
    msg.target_vel_y = 0.0
    # ACC_MODEの場合に使用
    msg.target_acc_x = 0.0
    msg.target_acc_y = 0.0

    # --- Z高さ制御 (Position Control) ---
    # モード (pos_z_nav_mode):
    # NO_NAVIGATION (0), VEL_MODE (1), POS_MODE (2), ACC_MODE (3) ... (XYと同じ)
    msg.pos_z_nav_mode = FlightNav.POS_MODE
    msg.target_pos_z = 1.5  # 目標高さ [m]
    msg.target_vel_z = 0.0
    msg.target_pos_diff_z = 0.0 # 現在の目標値からの差分で指定する場合に使用

    # --- 姿勢制御 (Yaw) ---
    # モード (yaw_nav_mode):
    # NO_NAVIGATION (0), VEL_MODE (1), POS_MODE (2) ...
    msg.yaw_nav_mode = FlightNav.POS_MODE
    msg.target_yaw = 0.0    # 目標ヨー角 [rad]
    msg.target_omega_z = 0.0 # VEL_MODEの場合の角速度 [rad/s]

    # --- 姿勢制御 (Roll/Pitch) ---
    msg.roll_nav_mode = FlightNav.POS_MODE
    msg.target_roll = 0.0
    msg.target_omega_x = 0.0
    
    msg.pitch_nav_mode = FlightNav.POS_MODE
    msg.target_pitch = 0.0
    msg.target_omega_y = 0.0

    rospy.loginfo(f"Publishing FlightNav message to {topic_name}")
    rospy.loginfo(f"Target: X={msg.target_pos_x}, Y={msg.target_pos_y}, Z={msg.target_pos_z}, Yaw={msg.target_yaw}")
    
    pub.publish(msg)

if __name__ == "__main__":
    try:
        publish_flight_nav()
    except rospy.ROSInterruptException:
        pass
