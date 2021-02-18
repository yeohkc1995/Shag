#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('mission_control', String, queue_size=10)
    rospy.init_node('MCS', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rospy.loginfo("Either input 'end' to end mission, or 'E0' to 'E20' to set safety distance between 0 and 20, or 'E0' to 'E359' to set offset bearing\n\n")
    
    while not rospy.is_shutdown():
        hello_str = raw_input()

        try:
            if ((hello_str[0].upper()) == 'N') and ( 0 <= int(hello_str[1:]) <= 4):
                rospy.loginfo(hello_str)
                pub.publish(hello_str)

            elif ((hello_str[0].upper()) == 'E'):
                rospy.loginfo(hello_str)
                pub.publish(hello_str)

            else:
                rospy.loginfo("Incorrect input! Either input 'end' to end mission, or 'E0' to 'E20' to set safety distance between 0 and 20, or 'E0' to 'E359' to set offset bearing\n\n")
        
        except:
            rospy.loginfo("Incorrect input! Either input 'end' to end mission, or 'E0' to 'E20' to set safety distance between 0 and 20, or 'E0' to 'E359' to set offset bearing\n\n")
            
        rate.sleep()

        
            

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass