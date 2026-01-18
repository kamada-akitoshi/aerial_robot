#!/usr/bin/env python

import rospy
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Header

def publish_flight_nav():
    rospy.init_node("flight_nav_publisher", anonymous=True)
    pub = rospy.Publisher("/gimbalrotor1/uav/nav", FlightNav, queue_size=10)
    
    rospy.sleep(1)  # Publisherが準備できるまで待つ

    msg = FlightNav()
    
    # ヘッダー設定
    msg.header = Header()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ""

    # 各種ナビゲーションパラメータ
    msg.control_frame = 0
    msg.target = 0
    msg.pos_xy_nav_mode = 2
    msg.target_pos_x = 0.3
    msg.target_vel_x = 0.0
    msg.target_acc_x = 0.0
    msg.target_pos_y = 0.0
    msg.target_vel_y = 0.0
    msg.target_acc_y = 0.0
    msg.roll_nav_mode = 0
    msg.target_omega_x = 0.0
    msg.target_roll = 0.0
    msg.pitch_nav_mode = 0
    msg.target_omega_y = 0.0
    msg.target_pitch = 0.0
    msg.yaw_nav_mode = 0
    msg.target_omega_z = 0.0
    msg.target_yaw = 0.0
    msg.pos_z_nav_mode = 0
    msg.target_pos_z = 0.0
    msg.target_vel_z = 0.0
    msg.target_pos_diff_z = 0.0

    rospy.loginfo("Publishing FlightNav message")
    pub.publish(msg)

if __name__ == "__main__":
    publish_flight_nav()
