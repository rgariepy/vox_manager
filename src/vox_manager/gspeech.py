#!/usr/bin/env python

# Based on "gspeech" by Achu Wilson (achu@achuwilson.in, github.com/achuwilson)

# When enabled by a service, it will record voice and recognize it
# Starts disabled

import roslib; roslib.load_manifest('vox_manager') 
import rospy

from std_msgs.msg import String, Int8
from std_srvs.srv import *

import shlex,subprocess,os

# Recording command
# 16 kHz sample rate
# ALSA format
# Trim 0.1 s silence from the beginning of the sample, where silence is <1% of the max value
# Wait until 1.5 s of silence is observed before stopping recording, where silence is <1% of the max value
#
cmd1='sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'

# GSpeech command
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=16000" -O - "http://www.google.com/speech-api/v1/recognize?lang=en-us&client=chromium"'

class gspeech:

    def __init__(self):
        rospy.init_node('gspeech')
        pubs = rospy.Publisher('speech_rec', String)
        pubc = rospy.Publisher('confidence', Int8)
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)
        self.enable_ = False

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable_:
                self.speech()
            r.sleep()

    def speech(self):
        # TODO: Add functionality to kill processes in the middle of exec
        # when we get a "stop" call
        rospy.loginfo("Recording")
        os.system(cmd1)
        args2 = shlex.split(cmd2)
        rospy.loginfo("Parsing")
        output,error = subprocess.Popen(args2,stdout = subprocess.PIPE, stderr= subprocess.PIPE).communicate()
        rospy.loginfo("Handling")
        print output
        if not error and len(output)>16:
            a = eval(output)
            status = int(a['status'])
            if status == 0:
                confidence= a['hypotheses'][0]['confidence']
                confidence= confidence*100
                data=a['hypotheses'][0]['utterance']
                pubs.publish(String(data))
                pubc.publish(confidence)
                rospy.loginfo("%s %d",String(data),confidence)

    def start(self, req):
        """
        Enables GSpeech
        """
        rospy.loginfo("GSpeech enabled")
        self.enable_ = True
        return EmptyResponse()

    def stop(self, req):
        """
        Disables GSpeech
        """
        rospy.loginfo("GSpeech disabled")
        self.enable_ = False
        return EmptyResponse()

if __name__ == '__main__':
    try:
        gspeech()
    except:
        pass
