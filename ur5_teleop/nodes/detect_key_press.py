#!/usr/bin/env python

import getch
import rospy

def listen_key_dirs():
    rospy.init_node("detect_key_press",anonymous=True)
    rate = rospy.Rate(5) #5hz

    while not rospy.is_shutdown():
        char = getch.getch() # getch.getche(): also displayed on the screen
        print("presed: "+char)
        if (char == 'x'):
            print("exitting .....")
            break

if __name__ == "__main__":
    try:
         listen_key_dirs()
    except rospy.ROSInterruptException:
        pass
