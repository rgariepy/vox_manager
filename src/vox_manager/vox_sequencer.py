#!/usr/bin/env python

"""
vox_sequencer.py searches for a single command continuously using pocketsphinx
Once found, it acknowledges and then switches to a more complex speech
recognition engine (Google API)

Right now, pocketsphinx is only looking for the word "computer"

TODO: Switch to a more complex speech recognition engine
publications:
    speech (std_msgs/String) - Output to vox synthesis
subscriptions:
    monitor/output (std_msgs/String) - Output from pocketsphinx
    recognizer/output (std_msgs/String) - Output from gspeech
"""

import roslib; roslib.load_manifest('vox_manager')
import rospy
import math

from std_msgs.msg import String
from std_srvs.srv import Empty

class vox_sequencer:
    """
    State Machine:
    0 - Waiting for trigger
        Transition: Trigger spotted (via topic)
        Action: Disable pocketsphinx, acknowledge, go to 1
    1 - Triggered, recognizing full command with gspeech
        Transition: Command retrieved | N s timeout
        Action: Disable gspeech if it's still going, enable pocketsphinx, go to 0
    """

    def __init__(self):
        rospy.init_node('vox_sequencer')
        rospy.on_shutdown(self.cleanup)
        self.state_ = 0
        self.ack_ = String()
        self.timeout_ = rospy.get_param("timeout",5)    # [s] Time before we switch back to monitoring mode

        # Wait for services
        rospy.wait_for_service('monitor/start')
        rospy.wait_for_service('monitor/stop')
        rospy.wait_for_service('recognizer/start')
        rospy.wait_for_service('recognizer/stop')
        self.mon_start_svc_ = rospy.ServiceProxy('monitor/start', Empty)
        self.mon_stop_svc_ = rospy.ServiceProxy('monitor/stop', Empty)
        self.rec_start_svc_ = rospy.ServiceProxy('recognizer/start', Empty)
        self.rec_stop_svc_ = rospy.ServiceProxy('recognizer/stop', Empty)

        # Disable recog and enable monitoring
        self.mon_start_svc_()
        self.rec_stop_svc_()

        # Subscribe to speech rec output, publish to vox synthesis
        rospy.Subscriber('monitor/output', String, self.speechMonitor)
        rospy.Subscriber('recognizer/output', String, self.speechRecognizer)
        self.pub_ack_ = rospy.Publisher('speech', String)
        rospy.loginfo("VOX Manager initialized")

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            r.sleep()
        
    def speechMonitor(self, msg):
        """
        Handles input from pocketsphinx (continuous monitor)
        """
        rospy.loginfo("Monitor: %s",msg.data)
        if msg.data.find("computer") > -1:
            self.ack_.data = "Online"
            self.pub_ack_.publish(self.ack_)
            # Disable monitoring and enable recog
            self.mon_stop_svc_()
            self.rec_start_svc_()

            # Start timeout timer
            self.timeout_timer_ = rospy.Timer(rospy.Duration(self.timeout_), self.timerCallback, oneshot=True)
            self.state_ = 1

    def speechRecognizer(self, msg):
        """
        Handles input from gspeech (high-precision recognition)
        """
        rospy.loginfo("Recognition: %s",msg.data)
        # Stop timer from firing later, we've succeeded
        # RYANNOTE: Nope, leave recognition going - time it out only
        # This will allow for continuous recognition
        #self.timeout_timer_.shutdown()
        # Disable recog and enable monitoring
        #self.mon_start_svc_()
        #self.rec_stop_svc_()
        #self.state_ = 0;

        #self.ack_.data = "Success"
        #self.pub_ack_.publish(self.ack_)

    def timerCallback(self, event):
        """
        Talking too long or no response 
        """
        # Disable recog and enable monitoring
        self.mon_start_svc_()
        self.rec_stop_svc_()

        self.state_ = 0;
        rospy.loginfo("Recognition timeout")

    def cleanup(self):
        pass

if __name__=="__main__":
    try:
        vox_sequencer()
    except:
        pass

