#!/usr/bin/env python
import sys
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry

class ClickPointFlight(object):
    def __init__(self):
        print("DEBUG: [0/4] Initializing Node...")
        sys.stdout.flush()
        rospy.init_node('click_point_flight', anonymous=True)

        print("DEBUG: fetching parameters...")
        sys.stdout.flush()
        self.scissor_frame = rospy.get_param('~scissor_frame', 'gimbalrotor1/scissor_center')
        self.cog_frame = rospy.get_param('~cog_frame', 'gimbalrotor1/cog')
        self.nav_topic = rospy.get_param('~nav_topic', '/gimbalrotor1/uav/nav')
        self.odom_topic = rospy.get_param('~odom_topic', '/gimbalrotor1/uav/cog/odom')
        self.nav_frame = rospy.get_param('~nav_frame', 'world')
        self.publish_rate = rospy.get_param('~publish_rate', 10)
        self.timeout = rospy.get_param('~tf_timeout', 1.0)

        self.curr_odom = None
        self.points_queue = []
        
        print("DEBUG: [1/4] Starting TransformListener initialization...")
        sys.stdout.flush()
        self.listener = tf.TransformListener()
        print("DEBUG: [2/4] TransformListener initialized successfully.")
        sys.stdout.flush()
        
        rospy.sleep(0.5)

        print("DEBUG: [3/4] Setting up Publishers and Subscribers...")
        sys.stdout.flush()
        self.pub = rospy.Publisher(self.nav_topic, FlightNav, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_cb, queue_size=10)
        
        print("DEBUG: [4/4] All Subscribers registered. Entering main loop.")
        sys.stdout.flush()

        rospy.loginfo('click_point_flight: listening /clicked_point, publishing FlightNav to %s', self.nav_topic)
        
        # Start processing loop
        self.main_loop()

    def odom_cb(self, msg):
        self.curr_odom = msg

    def clicked_cb(self, msg):
        # 1. Coordinate Transform and Target Calculation
        target_frame = (self.curr_odom.header.frame_id if self.curr_odom else self.nav_frame).strip("/")
        source_frame = msg.header.frame_id.strip("/")
        msg.header.frame_id = source_frame

        try:
            msg.header.stamp = rospy.Time(0) 
            if self.listener.canTransform(target_frame, source_frame, rospy.Time(0)):
                point_nav_frame = self.listener.transformPoint(target_frame, msg)
            else:
                point_nav_frame = msg
                target_frame = source_frame
        except Exception as e:
            point_nav_frame = msg
            target_frame = source_frame

        tx, ty, tz = point_nav_frame.point.x, point_nav_frame.point.y, point_nav_frame.point.z

        if self.curr_odom is None:
            rospy.logwarn("Odometry not received yet. Dropping point.")
            return

        # Get orientation details at the time of click
        q_curr = self.curr_odom.pose.pose.orientation
        euler_curr = tf.transformations.euler_from_quaternion([q_curr.x, q_curr.y, q_curr.z, q_curr.w])
        current_yaw = euler_curr[2]

        # Calculate COG target
        R_ideal_target = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, 0, current_yaw))[:3, :3]
        
        try:
            self.listener.waitForTransform(self.cog_frame, self.scissor_frame, rospy.Time(0), rospy.Duration(self.timeout))
            (t_cog_scissor_body, _) = self.listener.lookupTransform(self.cog_frame, self.scissor_frame, rospy.Time(0))
            offset_vec_world_ideal = np.dot(R_ideal_target, np.array(t_cog_scissor_body))
            desired_cog_pos = np.array([tx, ty, tz]) - offset_vec_world_ideal
            
            # Add to queue
            target_data = {
                'click_point': [tx, ty, tz],
                'desired_cog_pos': desired_cog_pos,
                'yaw': current_yaw,
                'source_frame': source_frame,
                'target_frame': target_frame,
                'euler_curr': euler_curr,
                't_cog_scissor_body': t_cog_scissor_body,
                'offset_vec_world_ideal': offset_vec_world_ideal
            }
            self.points_queue.append(target_data)
            
            print("\n" + "-"*30)
            print("[Callback] Click received! Point added to queue (Total: %d)" % len(self.points_queue))
            print("Target: x=%.3f, y=%.3f, z=%.3f" % (tx, ty, tz))
            print("-" * 30)
            sys.stdout.flush()

        except Exception as e:
            rospy.logerr("Calculation failed: %s" % str(e))

    def main_loop(self):
        try:
            get_input = raw_input  # Python2
        except NameError:
            get_input = input      # Python3

        while not rospy.is_shutdown():
            if not self.points_queue:
                rospy.sleep(0.2)
                continue
            
            # Peek at the next target
            target = self.points_queue[0]
            
            print("\n" + "="*60)
            print("COMMAND PREVIEW (Queue Size: %d)" % len(self.points_queue))
            print("-" * 60)
            print("Scissor Target (World): x=%.5f, y=%.5f, z=%.5f" % tuple(target['click_point']))
            print("Preserved Robot Yaw:    %.4f rad (%.1f deg)" % (target['yaw'], np.degrees(target['yaw'])))
            print("Calculated COG Target:  x=%.5f, y=%.5f, z=%.5f" % tuple(target['desired_cog_pos']))
            print("="*60)
            sys.stdout.flush()

            user = get_input("Press [Enter] to FLY to this point (type 'c' to cancel/skip, 'q' to quit): ")
            
            if user.lower().strip() == 'c':
                self.points_queue.pop(0)
                rospy.loginfo('Skipped one point. Remaining: %d', len(self.points_queue))
                continue
            elif user.lower().strip() == 'q':
                rospy.signal_shutdown('User requested quit.')
                break

            # Send flight command
            data = self.points_queue.pop(0)
            self.execute_flight(data)

    def execute_flight(self, data):
        if self.curr_odom is None:
            rospy.logerr("Odometry lost. Cannot execute flight.")
            return

        start_pos = np.array([self.curr_odom.pose.pose.position.x, 
                             self.curr_odom.pose.pose.position.y, 
                             self.curr_odom.pose.pose.position.z])
        # Force minimum altitude for safety
        if start_pos[2] < 0.5: start_pos[2] = 0.5
        
        goal_pos = data['desired_cog_pos']
        target_frame = data['target_frame']
        current_yaw = data['yaw']
        
        diff_vec = goal_pos - start_pos
        distance = np.linalg.norm(diff_vec)
        velocity_limit = 0.1
        duration = distance / velocity_limit
        if duration < 0.1: duration = 0.1
            
        steps = int(duration * self.publish_rate)
        vel_vec = diff_vec / duration
        
        rospy.loginfo('Flight started: Dist=%.3fm, Duration=%.1fs', distance, duration)

        rate = rospy.Rate(self.publish_rate)
        for i in range(steps + 1):
            if rospy.is_shutdown(): break
            
            alpha = float(i) / steps if steps > 0 else 1.0
            curr_target = (1.0 - alpha) * start_pos + alpha * goal_pos
            
            nav = FlightNav()
            nav.header = Header(stamp=rospy.Time.now(), frame_id=target_frame)
            nav.control_frame = 0  # WORLD_FRAME
            nav.target = 1         # COG
            
            nav.pos_xy_nav_mode = 4 # POS_VEL_MODE
            nav.target_pos_x = float(curr_target[0])
            nav.target_pos_y = float(curr_target[1])
            nav.target_vel_x = float(vel_vec[0])
            nav.target_vel_y = float(vel_vec[1])
            
            nav.pos_z_nav_mode = 4
            nav.target_pos_z = float(curr_target[2])
            nav.target_vel_z = float(vel_vec[2])
            
            nav.yaw_nav_mode = 2 # POS_MODE
            nav.target_yaw = float(current_yaw)
            
            self.pub.publish(nav)
            try: rate.sleep()
            except rospy.ROSInterruptException: break

        # Stop command
        nav.pos_xy_nav_mode = 2 
        nav.pos_z_nav_mode = 2
        for _ in range(5):
            nav.header.stamp = rospy.Time.now()
            self.pub.publish(nav)
            rospy.sleep(0.05)

        rospy.loginfo('Trajectory completed. At target position.')

if __name__ == '__main__':
    try:
        ClickPointFlight()
    except rospy.ROSInterruptException:
        pass