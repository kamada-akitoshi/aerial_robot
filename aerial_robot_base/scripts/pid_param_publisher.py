#!/usr/bin/env python

"""
PID Parameter Publisher Node

Reads PID parameters from a YAML config file and publishes them as individual 
Float64 topics for easy monitoring with PlotJuggler.

Usage:
    rosrun aerial_robot_base pid_param_publisher.py _config_file:=/path/to/config.yaml
"""

import rospy
import yaml
from std_msgs.msg import Float64


class PIDParamPublisher:
    def __init__(self):
        rospy.init_node('pid_param_publisher', anonymous=False)
        
        # Get config file path from parameter
        config_file = rospy.get_param('~config_file', '')
        if not config_file:
            rospy.logerr("No config file specified. Use _config_file:=<path>")
            rospy.signal_shutdown("Missing config file parameter")
            return
        
        # Load YAML config
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load config file: {e}")
            rospy.signal_shutdown("Config load failed")
            return
        
        # Create publishers
        self.publishers = {}
        self._create_publishers()
        
        # Publish rate (1Hz is sufficient for parameters)
        self.rate = rospy.Rate(rospy.get_param('~publish_rate', 1.0))
        
        rospy.loginfo(f"PID Parameter Publisher started with {len(self.publishers)} topics")
        rospy.loginfo(f"Config loaded from: {config_file}")
    
    def _create_publishers(self):
        """Create publishers for all PID parameters"""
        controller = self.config.get('controller', {})
        
        # XY Position Control
        xy = controller.get('xy', {})
        self._add_pub('xy/pos/p', xy.get('p_gain'))
        self._add_pub('xy/pos/i', xy.get('i_gain'))
        self._add_pub('xy/pos/d', xy.get('d_gain'))
        self._add_pub('xy/limit_sum', xy.get('limit_sum'))
        self._add_pub('xy/limit_p', xy.get('limit_p'))
        self._add_pub('xy/limit_i', xy.get('limit_i'))
        self._add_pub('xy/limit_d', xy.get('limit_d'))
        
        # Z Position Control
        z = controller.get('z', {})
        self._add_pub('z/pos/p', z.get('p_gain'))
        self._add_pub('z/pos/i', z.get('i_gain'))
        self._add_pub('z/pos/d', z.get('d_gain'))
        self._add_pub('z/limit_sum', z.get('limit_sum'))
        self._add_pub('z/limit_p', z.get('limit_p'))
        self._add_pub('z/limit_i', z.get('limit_i'))
        self._add_pub('z/limit_d', z.get('limit_d'))
        self._add_pub('z/limit_err_p', z.get('limit_err_p'))
        
        # Roll Attitude Control
        roll = controller.get('roll', {})
        self._add_pub('roll/p', roll.get('p_gain'))
        self._add_pub('roll/i', roll.get('i_gain'))
        self._add_pub('roll/d', roll.get('d_gain'))
        self._add_pub('roll/limit_sum', roll.get('limit_sum'))
        self._add_pub('roll/limit_p', roll.get('limit_p'))
        self._add_pub('roll/limit_i', roll.get('limit_i'))
        self._add_pub('roll/limit_d', roll.get('limit_d'))
        
        # Pitch Attitude Control
        pitch = controller.get('pitch', {})
        self._add_pub('pitch/p', pitch.get('p_gain'))
        self._add_pub('pitch/i', pitch.get('i_gain'))
        self._add_pub('pitch/d', pitch.get('d_gain'))
        self._add_pub('pitch/limit_sum', pitch.get('limit_sum'))
        self._add_pub('pitch/limit_p', pitch.get('limit_p'))
        self._add_pub('pitch/limit_i', pitch.get('limit_i'))
        self._add_pub('pitch/limit_d', pitch.get('limit_d'))
        
        # Yaw Attitude Control
        yaw = controller.get('yaw', {})
        self._add_pub('yaw/p', yaw.get('p_gain'))
        self._add_pub('yaw/i', yaw.get('i_gain'))
        self._add_pub('yaw/d', yaw.get('d_gain'))
        self._add_pub('yaw/limit_sum', yaw.get('limit_sum'))
        self._add_pub('yaw/limit_p', yaw.get('limit_p'))
        self._add_pub('yaw/limit_i', yaw.get('limit_i'))
        self._add_pub('yaw/limit_d', yaw.get('limit_d'))
        self._add_pub('yaw/limit_err_p', yaw.get('limit_err_p'))
    
    def _add_pub(self, topic_suffix, value):
        """Add a publisher for a parameter if value is not None"""
        if value is not None:
            topic_name = f'/pid_params/{topic_suffix}'
            pub = rospy.Publisher(topic_name, Float64, queue_size=1, latch=True)
            self.publishers[topic_name] = (pub, float(value))
    
    def run(self):
        """Main loop: publish all parameters periodically"""
        while not rospy.is_shutdown():
            for topic_name, (pub, value) in self.publishers.items():
                msg = Float64()
                msg.data = value
                pub.publish(msg)
            
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = PIDParamPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
