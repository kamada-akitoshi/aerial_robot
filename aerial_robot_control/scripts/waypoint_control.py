#!/usr/bin/env python

import rospy
import tf
import numpy as np
import sys, select, termios, tty
import os
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Header, Empty
from collections import deque

msg = """
---------------------------------------------------------
Waypoint Control (Velocity Based)
---------------------------------------------------------
Target:  [{x:.2f}, {y:.2f}, {z:.2f}]
Yaw:     {yaw:.2f} rad
Velocity:{vel:.2f} m/s

Commands:
  p: Start Flight (Go to Target)
  o: Return to Origin (0, 0, 0.3)
  r: Arming (Start Motor)
  t: Takeoff
  l: Land
  f: Force Landing
  h: Halt (Emergency Stop)
  
CTRL+c to quit
---------------------------------------------------------
"""

def get_current_pose(listener, target_frame="uav/cog", source_frame="world"):
    try:
        listener.waitForTransform(source_frame, target_frame, rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        return np.array(trans), euler[2] # x, y, z, yaw
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Could not get current pose")
        return None, None

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def printMsg(command_msg, msg_len=60):
    """Print message with carriage return to update in place"""
    print(command_msg.ljust(msg_len) + "\r", end="")

def main():
    rospy.init_node("waypoint_control_velocity_based", anonymous=True)
    
    robot_ns = rospy.get_param("~robot_ns", "/gimbalrotor1")
    topic_name = f"{robot_ns}/uav/nav"
    pub = rospy.Publisher(topic_name, FlightNav, queue_size=10)
    
    # Teleop Publishers
    ns = robot_ns + "/teleop_command"
    land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
    halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
    start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
    takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
    force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)

    listener = tf.TransformListener()
    settings = termios.tcgetattr(sys.stdin)

    # Parameters
    nav_rate_hz = 20.0
    dt = 1.0 / nav_rate_hz
    
    # Target Configuration (Edit here)
    target_pos = np.array([3.0, 2.5, 2.5]) # x, y, z
    target_yaw = 1.0 # rad
    
    # Desired Velocity [m/s]
    velocity = 0.1 

    # UI Initialization - print menu without clearing screen
    print(msg.format(x=target_pos[0], y=target_pos[1], z=target_pos[2], yaw=target_yaw, vel=velocity))

    try:
        while not rospy.is_shutdown():
            key = getKey(settings)
            
            if key == 'p' or key == 'o':
                current_pos, current_yaw = get_current_pose(listener, target_frame=f"{robot_ns[1:]}/root", source_frame="world")
                
                # Determine target based on key
                if key == 'p':
                    final_target_pos = target_pos
                    final_target_yaw = target_yaw
                    printMsg("Go to Target")
                else: # key == 'o'
                    final_target_pos = np.array([0.0, 0.0, 0.3])
                    final_target_yaw = 0.0
                    printMsg("Return to Origin (0, 0, 0.3)")

                if current_pos is not None:
                    # Calculate distance and duration
                    dist_vec = final_target_pos - current_pos
                    distance = np.linalg.norm(dist_vec)
                    
                    if distance < 0.01:
                        printMsg("Already at target position")
                        continue

                    duration = distance / velocity
                    steps = int(duration * nav_rate_hz)
                    
                    printMsg(f"Starting flight: Dist={distance:.2f}m, Time={duration:.2f}s")
                    
                    # Velocity vector for feedforward
                    vel_vec = (dist_vec / distance) * velocity
                    yaw_diff = final_target_yaw - current_yaw
                    omega_z = yaw_diff / duration

                    r = rospy.Rate(nav_rate_hz)
                    
                    # Enter Raw mode for loop input
                    tty.setraw(sys.stdin.fileno())
                    
                    flight_aborted = False

                    try:
                        for i in range(steps + 1):
                            # Emergency Check
                            if select.select([sys.stdin], [], [], 0)[0]:
                                check_key = sys.stdin.read(1)
                                if check_key == 'h':
                                    halt_pub.publish(Empty())
                                    printMsg("Emergency Halt!")
                                    flight_aborted = True
                                    break
                                elif check_key == 'f':
                                    force_landing_pub.publish(Empty())
                                    printMsg("Force Landing!")
                                    flight_aborted = True
                                    break
                                elif check_key == 'l':
                                    land_pub.publish(Empty())
                                    printMsg("Landing initiated!")
                                    flight_aborted = True
                                    break

                            # Calculate current target point (Linear Interpolation)
                            alpha = float(i) / steps
                            curr_target_pos = (1 - alpha) * current_pos + alpha * final_target_pos
                            curr_target_yaw = (1 - alpha) * current_yaw + alpha * final_target_yaw

                            nav_msg = FlightNav()
                            nav_msg.header.stamp = rospy.Time.now()
                            nav_msg.header.frame_id = "world"
                            nav_msg.control_frame = FlightNav.WORLD_FRAME
                            nav_msg.target = FlightNav.BASELINK
                            
                            # Use POS_VEL_MODE for smoother control (Feedforward velocity)
                            nav_msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
                            nav_msg.target_pos_x = curr_target_pos[0]
                            nav_msg.target_pos_y = curr_target_pos[1]
                            nav_msg.target_vel_x = vel_vec[0]
                            nav_msg.target_vel_y = vel_vec[1]
                            
                            nav_msg.pos_z_nav_mode = FlightNav.POS_VEL_MODE
                            nav_msg.target_pos_z = curr_target_pos[2]
                            nav_msg.target_vel_z = vel_vec[2]
                            
                            nav_msg.yaw_nav_mode = FlightNav.POS_VEL_MODE
                            nav_msg.target_yaw = curr_target_yaw
                            nav_msg.target_omega_z = omega_z
                            
                            pub.publish(nav_msg)
                            r.sleep()
                    finally:
                        # Restore terminal settings
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                    
                    if not flight_aborted:
                        # Stop at the end
                        stop_msg = FlightNav()
                        stop_msg.header.stamp = rospy.Time.now()
                        stop_msg.header.frame_id = "world"
                        stop_msg.control_frame = FlightNav.WORLD_FRAME
                        stop_msg.target = FlightNav.BASELINK
                        stop_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                        stop_msg.target_pos_x = final_target_pos[0]
                        stop_msg.target_pos_y = final_target_pos[1]
                        stop_msg.pos_z_nav_mode = FlightNav.POS_MODE
                        stop_msg.target_pos_z = final_target_pos[2]
                        stop_msg.yaw_nav_mode = FlightNav.POS_MODE
                        stop_msg.target_yaw = final_target_yaw
                        pub.publish(stop_msg)
                        
                        printMsg("Target Reached")
                    else:
                        printMsg("Flight aborted")

                else:
                    rospy.logerr("Cannot get current pose")

            elif key == 'r':
                start_pub.publish(Empty())
                printMsg("ARM")
            elif key == 't':
                takeoff_pub.publish(Empty())
                printMsg("TAKEOFF")
            elif key == 'l':
                land_pub.publish(Empty())
                printMsg("LAND")
            elif key == 'h':
                halt_pub.publish(Empty())
                printMsg("HALT")
            elif key == 'f':
                force_landing_pub.publish(Empty())
                printMsg("FORCE LANDING")
            elif key == '\x03':
                break

    except Exception as e:
        rospy.logerr(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    main()
