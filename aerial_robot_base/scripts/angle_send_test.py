#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import sys

# Python 2/3 compatibility for input
if sys.version_info[0] < 3:
    input = raw_input

def main():
    rospy.init_node('prune_servo_keyboard_commander', anonymous=True)
    pub = rospy.Publisher('/dynamixel/cmd_angle', Float64, queue_size=1)
    
    # Wait for connection
    rospy.sleep(1.0)
    
    print("--------------------------------------------------")
    print(" Prune Servo Keyboard Commander (Input Angle Mode)")
    print("--------------------------------------------------")
    print(" Enter target angle in degrees (e.g., 0, 90, -45)")
    print(" To quit, type 'q' or press Ctrl+C")
    print("--------------------------------------------------")

    while not rospy.is_shutdown():
        try:
            # Wait for user input
            user_input = input("Enter Angle [deg]: ")
            
            if user_input.lower() == 'q':
                print("Quitting...")
                break
            
            # Convert to float and publish
            angle = float(user_input)
            
            # Safety range check (-180 to 180)
            if -180.0 <= angle <= 180.0:
                msg = Float64()
                msg.data = angle
                pub.publish(msg)
                rospy.loginfo("Published Command: {:.2f} deg".format(angle))
            else:
                rospy.logwarn("Angle {:.2f} is out of range (-180 to 180). Ignored.".format(angle))
                
        except ValueError:
            print("Invalid input! Please enter a number.")
        except EOFError:
            # Handle Ctrl+D
            print("\nQuitting...")
            break
        except KeyboardInterrupt:
            print("\nShutting down...")
            break
        except Exception as e:
            rospy.logerr("Error: {}".format(e))

if __name__ == '__main__':
    main()
