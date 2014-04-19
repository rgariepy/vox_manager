#!/usr/bin/env python

"""
vox_sequencer.py searches for a single command continuously using pocketsphinx
Once found, it acknowledges and then switches to a more complex speech
recognition engine (Google API)

Right now, pocketsphinx is only looking for the word "computer"

TODO: Switch to a more complex speech recognition engine
"""

import roslib; roslib.load_manifest('vox_manager')
import rospy
import math

from std_msgs.msg import String

class vox_sequencer:
    """
    State Machine:
    0 - Waiting for trigger
        Transition: Trigger spotted (via topic)
        Action: Disable pocketsphinx, acknowledge, go to 1
    1 - Triggered, recognizing full command
        Transition: Command retrieved | N s timeout
        Action: Enable pocketsphinx, go to 0
    """

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.state = 0 
        self.ack_ = String()

        # Subscribe to pocketsphinx output, publish to vox synthesis
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        self.pub_ack_ = rospy.Publisher('speech', String)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)

        if msg.data.find("computer") > -1:
            self.ack_.data = "Online"
            self.pub_ack_.publish(self.ack_)

    def cleanup(self):
        pass

if __name__=="__main__":
    rospy.init_node('vox_sequencer')
    try:
        vox_sequencer()
    except:
        pass

