#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from aerial_robot_msgs.msg import FlightNav

def quaternion_to_matrix(q):
    return tf.transformations.quaternion_matrix([q[0], q[1], q[2], q[3]])

def make_transform_matrix(translation, quat):
    T = quaternion_to_matrix(quat)
    T[0:3, 3] = translation
    return T

class ClickPointFlight(object):
    def __init__(self):
        rospy.init_node('click_point_flight')

        self.scissor_frame = rospy.get_param('~scissor_frame', 'gimbalrotor1/scissor_center')
        self.cog_frame = rospy.get_param('~cog_frame', 'gimbalrotor1/cog')
        self.nav_topic = rospy.get_param('~nav_topic', '/gimbalrotor1/uav/nav')
        self.odom_topic = rospy.get_param('~odom_topic', '/gimbalrotor1/uav/cog/odom')
        self.publish_rate = rospy.get_param('~publish_rate', 10)
        self.timeout = rospy.get_param('~tf_timeout', 1.0)

        self.current_odom_yaw = None

        self.listener = tf.TransformListener()
        rospy.sleep(0.5)

        self.pub = rospy.Publisher(self.nav_topic, FlightNav, queue_size=1)
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)

        rospy.loginfo('click_point_flight: listening /clicked_point, publishing FlightNav to %s', self.nav_topic)
        rospy.spin()

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_odom_yaw = euler[2]

    def clicked_cb(self, msg):
        # クリック点を表示して、端末で Enter 押下を待つ（'c' + Enter でキャンセル）
        frame = msg.header.frame_id if msg.header.frame_id else 'world'
        try:
            point_disp = self.listener.transformPoint(frame, msg) if msg.header.frame_id else msg
        except Exception:
            point_disp = msg
        rospy.loginfo('Clicked point received in frame %s: x=%.3f y=%.3f z=%.3f', frame,
                      point_disp.point.x, point_disp.point.y, point_disp.point.z)

        try:
            get_input = raw_input  # Python2
        except NameError:
            get_input = input      # Python3

        try:
            user = get_input("Press Enter to send FlightNav to move scissor_center there (type 'c' + Enter to cancel): ")
        except EOFError:
            rospy.logwarn('No terminal input available; cancelling command')
            return

        if isinstance(user, str) and user.lower().strip() == 'c':
            rospy.loginfo('User cancelled the FlightNav command')
            return

        world_frame = msg.header.frame_id if msg.header.frame_id else 'world'

        try:
            self.listener.waitForTransform(self.cog_frame, self.scissor_frame, rospy.Time(0), rospy.Duration(self.timeout))
            (t_cog_scissor, q_cog_scissor) = self.listener.lookupTransform(self.cog_frame, self.scissor_frame, rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('TF lookup failed between %s and %s: %s', self.cog_frame, self.scissor_frame, str(e))
            return

        T_cog_scissor = make_transform_matrix(t_cog_scissor, q_cog_scissor)
        T_cog_scissor_inv = np.linalg.inv(T_cog_scissor)

        try:
            point_world = self.listener.transformPoint(world_frame, msg) if msg.header.frame_id != world_frame else msg
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn('Could not transform clicked point to %s: %s', world_frame, str(e))
            return

        tx = point_world.point.x
        ty = point_world.point.y
        tz = point_world.point.z

        # Determine current robot yaw for maintaining orientation
        # Prioritize Odometry as it has better timestamp synchronization in this environment
        if self.current_odom_yaw is not None:
            current_yaw = self.current_odom_yaw
            rospy.loginfo("Using current Yaw from Odometry: %.3f rad", current_yaw)
        else:
            # Fallback to TF if Odometry hasn't been received yet
            try:
                (t_world_cog, q_world_cog) = self.listener.lookupTransform(world_frame, self.cog_frame, rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(q_world_cog)
                current_yaw = euler[2]
                rospy.loginfo("Using current Yaw from TF (Fallback): %.3f rad", current_yaw)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                current_yaw = 0.0
                rospy.logwarn("Could not get current yaw from Odom or TF. Using 0.0 rad. Error: %s", str(e))

        q_target = tf.transformations.quaternion_from_euler(0, 0, current_yaw)
        T_world_scissor = make_transform_matrix([tx, ty, tz], q_target)

        T_world_cog = np.dot(T_world_scissor, T_cog_scissor_inv)
        desired_cog_pos = T_world_cog[0:3, 3]

        nav = FlightNav()
        nav.header = Header()
        nav.header.stamp = rospy.Time.now()
        nav.header.frame_id = world_frame

        nav.control_frame = 0  # WORLD_FRAME
        nav.target = 1         # COG

        nav.pos_xy_nav_mode = 2  # POS_MODE
        nav.target_pos_x = float(desired_cog_pos[0])
        nav.target_pos_y = float(desired_cog_pos[1])

        nav.pos_z_nav_mode = 2
        nav.target_pos_z = float(desired_cog_pos[2])

        nav.target_vel_x = 0.0
        nav.target_vel_y = 0.0
        nav.target_vel_z = 0.0

        nav.yaw_nav_mode = 2 # POS_MODE
        nav.target_yaw = current_yaw
        nav.target_roll = 0.0
        nav.target_pitch = 0.0

        rate = rospy.Rate(self.publish_rate)
        for i in range(int(self.publish_rate * 1.0)):
            self.pub.publish(nav)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

        rospy.loginfo('Published FlightNav to %s', self.nav_topic)
        rospy.loginfo(' - Clicked Point (Target Scissor): x=%.3f, y=%.3f, z=%.3f', tx, ty, tz)
        rospy.loginfo(' - Calculated COG Target:          x=%.3f, y=%.3f, z=%.3f',
                      desired_cog_pos[0], desired_cog_pos[1], desired_cog_pos[2])

if __name__ == '__main__':
    try:
        ClickPointFlight()
    except rospy.ROSInterruptException:
        pass