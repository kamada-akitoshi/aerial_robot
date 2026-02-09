#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('prune_servo_test_loop', anonymous=True)
    pub = rospy.Publisher('/dynamixel/cmd_angle', Float64, queue_size=1)
    
    rospy.sleep(1.0) # Wait for connection
    
    angle_min = -160.0
    angle_max = 170.0
    interval = 3.0 # seconds
    
    count = 1
    
    print("--------------------------------------------------")
    print(" Prune Servo Durability Test Loop")
    print(" Angles: {} deg <-> {} deg".format(angle_min, angle_max))
    print(" Interval: {} sec".format(interval))
    print("--------------------------------------------------")

    while not rospy.is_shutdown():
        # Move to MIN
        msg = Float64()
        msg.data = angle_min
        pub.publish(msg)
        rospy.loginfo("[Iter {}] Command: {:.1f} deg".format(count, angle_min))
        
        rospy.sleep(interval)
        
        if rospy.is_shutdown(): break

        # Move to MAX
        msg.data = angle_max
        pub.publish(msg)
        rospy.loginfo("[Iter {}] Command: {:.1f} deg".format(count, angle_max))
        
        rospy.sleep(interval)
        
        count += 1

if __name__ == '__main__':
    main()
